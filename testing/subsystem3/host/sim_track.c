/*
 *	sim_track.c -- host simulation: the REAL s3_line.c (v25, both sensors
 *	on the line) driving a simple model of our car around test tracks.
 *
 *	Normally run by ../run_host_tests.sh, or on any PC/WSL with gcc:
 *	  gcc -std=c99 -O2 -Wall -Wextra -I../../../app_program/subsystem3 \
 *	      sim_track.c ../../../app_program/subsystem3/s3_line.c \
 *	      -lm -o sim && ./sim            (./sim sweep = gain sweep only)
 *
 *	What is modelled (and what is NOT):
 *	- Differential drive, track 112 mm (S2). S2 FOLLOW: wheels = base -/+
 *	  half of S3's dv (S2_S3_STEER_PERCENT 50), with S2's corner
 *	  slow-down (base - (|corr| - 10), at least 70 % of base, as
 *	  corner_speed() in motion_task.c) and set-point ramp 30 mm/s per
 *	  20 ms. Optional wheel lag (first order). No slip, no stall.
 *	  S3 steps every 10 ms, S2 reads every 20 ms.
 *	- Sensors: two, SPACING mm apart centre to centre, the pair centred on
 *	  the car, LOOKAHEAD mm in front of the axle. Each reads the fraction
 *	  of a 5 mm spot over the 18 mm black line. Optional uniform noise
 *	  (fixed-seed LCG), 4-sample averaging and 8-bit readings, as
 *	  drv_adc.c.
 *	- UNVERIFIED on our car: lookahead (swept 40/60/80 mm), spot size,
 *	  noise, wheel lag. So this shows whether the RULES steer the right
 *	  way and settle; it does not prove the car will.
 */
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "s3_line.h"

#define W_RAW       (200.0)
#define B_RAW       (3000.0)
#define LINE_HALF   (9.0)        /* mm */
#define SPOT        (5.0)        /* mm, sensor spot width */
#define TRACK       (112.0)      /* mm, wheel centre to centre */
#define RAMP_MMS    (30.0)       /* S2 set-point ramp per 20 ms */
#define CORNER_DEAD (10.0)       /* = S2_CORNER_DEADBAND_MMS */
#define CORNER_MIN  (0.70)       /* = S2_CORNER_MIN_PERCENT / 100 */
#define MAX_PTS     (4000)
#define PI          (3.14159265358979)
#define KICK_MS     (300.0)      /* Disturbance spread over this. */
#define S3_IR_SPACING_TEST_MM (15.0)   /* Our car, centre to centre.   */

static int32_t  g_kp = S3_LN_KP_X100;   /* gains used by drive()        */
static int32_t  g_kd = S3_LN_KD_X100;
static int32_t  g_noise = 0;            /* +- raw counts on each read    */
static uint32_t g_seed  = 12345u;
static double   g_half  = 6.0;          /* half the sensor spacing, mm   */
static int32_t  g_l_side = S3_LN_L_ON_RIGHT;   /* GP26 mounting side     */
static double   g_lag_ms     = 0.0;     /* 0 = wheels follow at once     */
static int      g_quant8     = 0;       /* 1 = 8-bit readings            */
static int      g_oversample = 1;       /* ADC samples per reading       */
static double   g_kick_at_mm = -1.0;    /* < 0 = no disturbance          */
static double   g_kick_deg   = 0.0;     /* heading kick, + = left        */
static double   g_kick_mm    = 0.0;     /* sideways slide, + = left      */

static double g_px[MAX_PTS], g_py[MAX_PTS];
static int    g_n;
static int    g_fail = 0;

static void check (char const * p_id, int ok, char const * p_what)
{
    printf("[TEST] %-8s %s  %s\n", p_id, ok ? "PASS" : "FAIL", p_what);
    g_fail += !ok;
}

/* ---- track building: a polyline of points 1 mm apart ---- */
static double g_hx, g_hy, g_hh;  /* pen position and heading (rad) */
static void pen_start (void) { g_n = 0; g_hx = 0.0; g_hy = 0.0; g_hh = 0.0; g_px[g_n] = 0.0; g_py[g_n] = 0.0; g_n++; }
static void pen_add (void) { if (g_n < MAX_PTS) { g_px[g_n] = g_hx; g_py[g_n] = g_hy; g_n++; } }
static void straight (double mm) { int i; for (i = 0; i < (int)mm; i++) { g_hx += cos(g_hh); g_hy += sin(g_hh); pen_add(); } }
static void arc (double r, double deg)          /* deg > 0 = left */
{
    int    i, n = (int)(fabs(deg) * PI / 180.0 * r);
    double dh = (deg * PI / 180.0) / n;
    for (i = 0; i < n; i++) { g_hh += dh; g_hx += cos(g_hh); g_hy += sin(g_hh); pen_add(); }
}
static void corner (double deg) { g_hh += deg * PI / 180.0; }   /* sharp: turn on the spot */

/* distance from (x,y) to the polyline and index of the nearest point */
static double line_dist (double x, double y, int * p_idx)
{
    double best = 1e9; int i, bi = 0;
    for (i = 0; (i + 1) < g_n; i++)
    {
        double ax = g_px[i], ay = g_py[i], bx = g_px[i + 1], by = g_py[i + 1];
        double vx = bx - ax, vy = by - ay, l2 = vx * vx + vy * vy;
        double t  = (l2 > 0.0) ? (((x - ax) * vx + (y - ay) * vy) / l2) : 0.0;
        double dx, dy, d;
        t  = (t < 0.0) ? 0.0 : ((t > 1.0) ? 1.0 : t);
        dx = ax + t * vx - x; dy = ay + t * vy - y; d = sqrt(dx * dx + dy * dy);
        if (d < best) { best = d; bi = i; }
    }
    if (NULL != p_idx) { *p_idx = bi; }
    return best;
}

static uint16_t sensor (double x, double y)
{
    double d   = line_dist(x, y, NULL);
    double cov = (LINE_HALF + SPOT / 2.0 - d) / SPOT;
    double v;
    cov = (cov < 0.0) ? 0.0 : ((cov > 1.0) ? 1.0 : cov);
    v   = W_RAW + (B_RAW - W_RAW) * cov;
    if (g_noise > 0)
    {
        g_seed = (g_seed * 1103515245u) + 12345u;          /* LCG */
        v += (double)((int32_t)((g_seed >> 16) % (uint32_t)(2 * g_noise + 1)) - g_noise);
    }
    v = (v < 0.0) ? 0.0 : ((v > 4095.0) ? 4095.0 : v);
    return (uint16_t)v;
}

/* One reading as task_line gets it (drv_adc.c): averaged, 8-bit. */
static uint16_t ir_read (double x, double y)
{
    uint32_t sum = 0u;
    uint32_t avg;
    int      k;

    for (k = 0; k < g_oversample; k++) { sum += sensor(x, y); }
    avg = (sum + ((uint32_t)g_oversample / 2u)) / (uint32_t)g_oversample;
    if (0 != g_quant8)
    {
        uint32_t v8 = (avg + 8u) >> 4;
        avg = ((v8 > 255u) ? 255u : v8) << 4;
    }
    return (uint16_t)avg;
}

static double clamp_step (double diff, double max_step)
{
    return (diff > max_step) ? max_step : ((diff < -max_step) ? -max_step : diff);
}

/* flip_pct: % of S3 steps where dv changed sign (left <-> right).
 * rms_err / max_err: axle centre's distance from the line centre. */
typedef struct { double progress; double max_err; double rms_err; int losts; int flip_pct; } run_t;

/* Drive the car along the current track from the start, offset_mm to the
 * left of the line (+ = left). */
static run_t drive (double base, int32_t steer_max, double lookahead, double offset_mm)
{
    s3_line_t     ln;
    s3_line_out_t o;
    run_t         r = {0.0, 0.0, 0.0, 0, 0};
    double        err_sq = 0.0, total = (double)(g_n - 1);
    double        x = 0.0, y = offset_mm, h = 0.0, vl = 0.0, vr = 0.0, dist = 0.0;
    double        sl = 0.0, sr = 0.0, t_ms, kick_ms = 0.0;
    int           i, idx = 0, steps = 0, flips = 0, err_n = 0, best_idx = 0;
    int32_t       dv = 0, dv_prev = 0;

    s3_line_init(&ln, g_l_side);
    s3_line_set_gains(&ln, g_kp, g_kd);
    g_seed = 12345u;
    for (i = 0; i < 20; i++) { s3_line_step(&ln, 200U, 200U, 0U, 10000U, &o); }
    for (i = 0; i < 20; i++) { s3_line_step(&ln, 3000U, 3000U, 0U, 10000U, &o); }

    for (t_ms = 0.0; t_ms < 120000.0; t_ms += 1.0)
    {
        if (0 == ((int)t_ms % 10))                    /* S3 line step */
        {
            double   sx = x + lookahead * cos(h), sy = y + lookahead * sin(h);
            double   nx = -sin(h), ny = cos(h);       /* left normal */
            uint16_t r_left  = ir_read(sx + g_half * nx, sy + g_half * ny);
            uint16_t r_right = ir_read(sx - g_half * nx, sy - g_half * ny);
            uint16_t r_gp26  = (S3_LN_L_ON_RIGHT == g_l_side) ? r_right : r_left;
            uint16_t r_gp27  = (S3_LN_L_ON_RIGHT == g_l_side) ? r_left : r_right;

            s3_line_step(&ln, r_gp26, r_gp27, (uint32_t)(((vl + vr) / 2.0) * 10.0), 10000U, &o);
            dv = s3_line_dv(&o, steer_max);           /* same as task_line */
            if (((dv > 0) && (dv_prev < 0)) || ((dv < 0) && (dv_prev > 0))) { flips++; }
            if (0 != dv) { dv_prev = dv; }
            steps++;
            r.losts += o.lost_event;
        }
        if (0 == ((int)t_ms % 20))                    /* S2 FOLLOW */
        {
            double corr = -(double)((dv * 50) / 100);   /* S2 sign: + = left */
            double bas  = base;
            double exc  = fabs(corr) - CORNER_DEAD;
            if (exc > 0.0) { bas = base - exc; }
            if (bas < (CORNER_MIN * base)) { bas = CORNER_MIN * base; }
            sl += clamp_step((bas - corr) - sl, RAMP_MMS);
            sr += clamp_step((bas + corr) - sr, RAMP_MMS);
        }
        if (g_lag_ms > 0.0) { vl += (sl - vl) / g_lag_ms; vr += (sr - vr) / g_lag_ms; }
        else                { vl = sl; vr = sr; }
        if ((g_kick_at_mm >= 0.0) && (dist >= g_kick_at_mm) && (kick_ms < KICK_MS))
        {
            kick_ms += 1.0;
            h += (g_kick_deg * PI / 180.0) / KICK_MS;
            x -= (g_kick_mm / KICK_MS) * sin(h);
            y += (g_kick_mm / KICK_MS) * cos(h);
        }
        h += ((vr - vl) / TRACK) * 0.001;
        x += ((vl + vr) / 2.0) * cos(h) * 0.001;
        y += ((vl + vr) / 2.0) * sin(h) * 0.001;
        dist += fabs((vl + vr) / 2.0) * 0.001;
        if (0 == ((int)t_ms % 50))
        {
            double e = line_dist(x, y, &idx);         /* axle centre vs line */
            if (idx > best_idx) { best_idx = idx; }
            if (best_idx > 30)
            {
                if (e > r.max_err) { r.max_err = e; }
                err_sq += e * e; err_n++;
            }
            if (e > 150.0) { break; }                 /* lost for good */
            /* Done when the SENSORS reach the end (past it they see white). */
            if (best_idx >= (g_n - 3 - (int)lookahead)) { best_idx = g_n - 1; break; }
        }
        if (dist > 3.0 * total) { break; }            /* circling */
    }
    r.progress = (double)best_idx / total;
    r.rms_err  = (err_n > 0) ? sqrt(err_sq / err_n) : 0.0;
    r.flip_pct = (steps > 0) ? (100 * flips) / steps : 0;
    return r;
}

/* ---- the six tracks ---- */
#define N_TRACKS    (6)
static char const * const g_trk_name[N_TRACKS] =
{
    "straight 2 m", "left curve r200", "right curve r200",
    "sharp RIGHT 90", "sharp LEFT 90", "S-bend r150"
};
static double const g_trk_err[N_TRACKS] = {10.0, 25.0, 25.0, 60.0, 60.0, 30.0};

static void track_build (int t)
{
    pen_start();
    switch (t)
    {
        case 0: straight(2000.0); break;
        case 1: straight(300.0); arc(200.0, 90.0); straight(400.0); break;
        case 2: straight(300.0); arc(200.0, -90.0); straight(400.0); break;
        case 3: straight(300.0); corner(-90.0); straight(500.0); break;
        case 4: straight(300.0); corner(90.0); straight(500.0); break;
        default: straight(300.0); arc(150.0, -90.0); arc(150.0, 90.0); straight(400.0); break;
    }
}

/* Every track x 3 lookaheads x 2 speeds (40 = drive test, 80). */
static void all_tracks (char const * p_tag)
{
    static double const looks[] = {40.0, 60.0, 80.0};
    static double const speeds[] = {40.0, 80.0};
    int  t, k, s;
    char id[16], line[160];

    for (t = 0; t < N_TRACKS; t++)
    {
        int ok = 1;
        track_build(t);
        for (s = 0; s < 2; s++)
        {
            for (k = 0; k < 3; k++)
            {
                run_t r = drive(speeds[s], (int32_t)speeds[s], looks[k], 0.0);
                printf("  %-17s %2.0f mm/s lookahead %2.0f: %3.0f %% of track, max err %5.1f mm, "
                       "RMS %4.1f mm, LOST %d\n", g_trk_name[t], speeds[s], looks[k],
                       100.0 * r.progress, r.max_err, r.rms_err, r.losts);
                ok = ok && (r.progress >= 0.98) && (r.max_err <= g_trk_err[t]);
            }
        }
        (void)snprintf(id, sizeof(id), "S3-%s%d", p_tag, t + 1);
        (void)snprintf(line, sizeof(line), "%s: >= 98 %% of the track, axle within %.0f mm",
                       g_trk_name[t], g_trk_err[t]);
        check(id, ok, line);
    }
}

static void layout_set (char const * p_name, double spacing, int32_t l_side)
{
    g_half   = spacing / 2.0;
    g_l_side = l_side;
    printf("== layout: %s\n", p_name);
}

/* S3-S80: start off the line (car 15 mm left / right of it): the side
 * memory set by the search mode is not used; the sensor that meets the
 * line first must pull the car on. */
static void offset_starts (void)
{
    static double const offs[] = {-15.0, -8.0, 8.0, 15.0};
    int  k, ok = 1;

    track_build(0);
    for (k = 0; k < 4; k++)
    {
        run_t r = drive(40.0, 40, 60.0, offs[k]);
        printf("  start %+3.0f mm from the line: %3.0f %% of track, max err %5.1f mm\n",
               offs[k], 100.0 * r.progress, r.max_err);
        ok = ok && (r.progress >= 0.98);
    }
    check("S3-S80", ok, "start 8-15 mm off the line (one sensor on it) -> pulls on, finishes");
}

/* S3-S91: realism (wheel lag 40-120 ms, noise, 8-bit, 4-sample
 * averaging) on the straight and the S-bend at 40 and 80 mm/s. */
static void realism (void)
{
    static double  const lags[]  = {40.0, 80.0, 120.0};
    static int32_t const noise[] = {40, 80};
    int    trk, l, n, s, ok = 1;
    char   line[160];

    g_quant8 = 1; g_oversample = 4;
    for (trk = 0; trk < 2; trk++)
    {
        track_build((0 == trk) ? 0 : 5);
        printf("  -- %s\n", (0 == trk) ? "straight 2 m" : "S-bend r150");
        for (s = 0; s < 2; s++)
        {
            double base = (0 == s) ? 40.0 : 80.0;
            for (l = 0; l < 3; l++)
            {
                for (n = 0; n < 2; n++)
                {
                    run_t r;
                    g_lag_ms = lags[l]; g_noise = noise[n];
                    r = drive(base, (int32_t)base, 60.0, 0.0);
                    printf("  %2.0f mm/s lag %3.0f ms noise +-%2d: %3.0f %% of track, max err %5.1f mm, "
                           "RMS %4.1f mm, dv flips %2d %%\n", base, g_lag_ms, (int)g_noise,
                           100.0 * r.progress, r.max_err, r.rms_err, r.flip_pct);
                    ok = ok && (r.progress >= 0.98) && (r.max_err <= 30.0);
                }
            }
        }
    }
    g_quant8 = 0; g_oversample = 1; g_lag_ms = 0.0; g_noise = 0;
    (void)snprintf(line, sizeof(line), "lag 40-120 ms, noise 40-80, 8-bit, 40 and 80 mm/s: "
                   "on track, axle within 30 mm");
    check("S3-S91", ok, line);
}

/* S3-S92: one bump or slip per run (heading +-15-35 deg or slide +-15-30
 * mm, spread over 0.3 s), straight and two S-bends, 3 lookaheads, wheel
 * lag 60 / 120 ms, noise, averaging, 8-bit: 216 runs. Pass = every run
 * finishes (no 180, no circle). */
static void disturb (void)
{
    static double const kick_deg[] = {-35.0, -25.0, -15.0, 15.0, 25.0, 35.0,
                                      0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    static double const kick_mm[]  = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                      -30.0, -22.0, -15.0, 15.0, 22.0, 30.0};
    static double const lags[]     = {60.0, 120.0};
    static double const looks[]    = {40.0, 60.0, 80.0};
    int  trk, l, k, a, runs = 0, fails = 0;
    char line[160];

    g_quant8 = 1; g_oversample = 4; g_noise = 60;
    for (trk = 0; trk < 3; trk++)
    {
        pen_start();
        if (0 == trk) { straight(1200.0); }
        else if (1 == trk) { straight(300.0); arc(200.0, -90.0); arc(200.0, 90.0); straight(300.0); }
        else { straight(300.0); arc(150.0, 90.0); arc(150.0, -90.0); straight(300.0); }
        for (l = 0; l < 2; l++)
        {
            g_lag_ms = lags[l];
            for (k = 0; k < (int)(sizeof(kick_deg) / sizeof(kick_deg[0])); k++)
            {
                for (a = 0; a < 3; a++)
                {
                    run_t r;
                    g_kick_at_mm = 400.0; g_kick_deg = kick_deg[k]; g_kick_mm = kick_mm[k];
                    r = drive(40.0, 40, looks[a], 0.0);
                    runs++;
                    if (r.progress < 0.98)
                    {
                        fails++;
                        printf("    track %d kick %+4.0f deg %+4.0f mm lookahead %2.0f lag %3.0f: "
                               "%3.0f %% of track  <-- did not finish\n", trk, g_kick_deg,
                               g_kick_mm, looks[a], g_lag_ms, 100.0 * r.progress);
                    }
                }
            }
        }
    }
    g_kick_at_mm = -1.0; g_quant8 = 0; g_oversample = 1; g_lag_ms = 0.0; g_noise = 0;
    printf("  disturbed runs finished: %d of %d\n", runs - fails, runs);
    (void)snprintf(line, sizeof(line), "after a bump or slip the car finishes the track "
                   "(%d of %d runs)", runs - fails, runs);
    check("S3-S92", 0 == fails, line);
}

/* "./sim sweep": gain sweep on all tracks with realism on (not a test). */
static void sweep (void)
{
    static int32_t const kps[] = {50, 75, 100, 150, 200, 300};
    static int32_t const kds[] = {0, 25, 50, 100, 200};
    int a, b, t, s;

    g_quant8 = 1; g_oversample = 4; g_noise = 60; g_lag_ms = 80.0;
    for (a = 0; a < 6; a++)
    {
        for (b = 0; b < 5; b++)
        {
            double worst_p = 1.0, worst_e = 0.0, rms = 0.0, flips = 0.0;
            g_kp = kps[a]; g_kd = kds[b];
            for (t = 0; t < N_TRACKS; t++)
            {
                track_build(t);
                for (s = 0; s < 2; s++)
                {
                    double base = (0 == s) ? 40.0 : 80.0;
                    run_t  r = drive(base, (int32_t)base, 60.0, 0.0);
                    worst_p = (r.progress < worst_p) ? r.progress : worst_p;
                    if (t < 3) { worst_e = (r.max_err > worst_e) ? r.max_err : worst_e; }
                    if (0 == t) { rms += r.rms_err / 2.0; flips += r.flip_pct / 2.0; }
                }
            }
            printf("Kp %3d Kd %3d: worst progress %3.0f %%, worst curve err %5.1f mm, "
                   "straight RMS %4.1f mm, flips %4.1f %%\n", (int)g_kp, (int)g_kd,
                   100.0 * worst_p, worst_e, rms, flips);
        }
    }
}

int main (int argc, char ** argv)
{
    if ((argc > 1) && (0 == strcmp(argv[1], "sweep")))
    {
        sweep();
        return 0;
    }
    if ((argc > 2) && (0 == strcmp(argv[1], "gap")))
    {
        /* "./sim gap 22": every check at another centre-to-centre spacing. */
        double mm = atof(argv[2]);

        layout_set("gap", mm, S3_LN_L_ON_RIGHT);
        all_tracks("G");
        offset_starts();
        realism();
        disturb();
        printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
        return g_fail;
    }

    /* OUR CAR (v25): both sensors on the line, 12 mm apart, GP26 on the
     * robot's RIGHT (S3_IR_L_SIDE in s3_config.h). */
    layout_set("OUR CAR: 15 mm (measured 28 Oct), GP26 RIGHT", S3_IR_SPACING_TEST_MM,
               S3_LN_L_ON_RIGHT);
    all_tracks("S0");
    /* The same mounted the other way round, with S3_IR_L_SIDE set to
     * match: the result must be the same. */
    layout_set("mirror mount, GP26 LEFT", S3_IR_SPACING_TEST_MM, S3_LN_L_ON_LEFT);
    all_tracks("S1");
    /* Mounting tolerance: 12 and 18 mm instead of 15. */
    layout_set("12 mm", 12.0, S3_LN_L_ON_RIGHT);
    all_tracks("S2");
    layout_set("18 mm", 18.0, S3_LN_L_ON_RIGHT);
    all_tracks("S3");

    layout_set("OUR CAR: start off the line", S3_IR_SPACING_TEST_MM, S3_LN_L_ON_RIGHT);
    offset_starts();
    layout_set("OUR CAR: wheel lag + noise + 8-bit + averaging", S3_IR_SPACING_TEST_MM,
               S3_LN_L_ON_RIGHT);
    realism();
    layout_set("OUR CAR: disturbed once", S3_IR_SPACING_TEST_MM, S3_LN_L_ON_RIGHT);
    disturb();

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
