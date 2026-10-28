/*
 *	sim_track.c -- host simulation: the REAL s3_line.c driving a simple
 *	model of our car around test tracks (straight, curves, sharp corners).
 *
 *	Build and run on any PC/WSL with gcc, no Pico needed:
 *	  gcc -std=c99 -O2 -Wall -Wextra -I../../../app_program/subsystem3 \
 *	      sim_track.c ../../../app_program/subsystem3/s3_line.c \
 *	      ../../../app_program/subsystem3/s3_ir_cal.c -lm -o sim && ./sim
 *
 *	What is modelled (and what is NOT):
 *	- Differential drive, track 112 mm (S2). S2 FOLLOW: wheels = base -/+
 *	  half of S3's dv (S2_S3_STEER_PERCENT 50), set-point ramp 30 mm/s
 *	  per 20 ms, wheels reach the ramped set-point at once (no PI lag,
 *	  no slip, no stall). S3 steps every 10 ms, S2 reads every 20 ms.
 *	- Sensors: Centre 9 mm LEFT of the car centre (on the line's left edge
 *	  when the car is centred), outer SPACING mm to the right of the
 *	  Centre, both LOOKAHEAD mm in front of the axle. Each reads the
 *	  fraction of a 5 mm spot over the 18 mm black line. Noise: none in
 *	  the pass/fail runs; S3-S9x adds uniform +-g_noise raw counts
 *	  (fixed-seed LCG, so every run is the same) to compare gain sets.
 *	- UNVERIFIED on our car: lookahead (swept 40/60/80 mm), spot size,
 *	  spacing (21 mm assumed). So this shows whether the RULES steer the
 *	  right way and settle; it does not prove the car will.
 */
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "s3_line.h"

#define W_RAW       (200.0)
#define B_RAW       (3000.0)
#define LINE_HALF   (9.0)        /* mm */
#define SPOT        (5.0)        /* mm, sensor spot width */
#define TRACK       (112.0)      /* mm, wheel centre to centre */
/* Sensor layout, set per run by layout_set(). */
static int32_t  g_kp = S3_LN_KP_X100;   /* gains used by drive()        */
static int32_t  g_kd = S3_LN_KD_X100;
static int32_t  g_noise = 0;            /* +- raw counts on each read    */
static uint32_t g_seed  = 12345u;
static double  g_c_left;         /* Centre this far LEFT of the car centre, mm */
static double  g_spacing;        /* Centre -> outer, mm; + = outer to the RIGHT */
static int32_t g_edge;           /* S3_LN_EDGE_LEFT / RIGHT */
static double  g_offset;         /* car centre's steady offset from the line */
#define RAMP_MMS    (30.0)       /* per 20 ms */
#define MAX_PTS     (4000)
#define PI          (3.14159265358979)

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
    cov = (cov < 0.0) ? 0.0 : ((cov > 1.0) ? 1.0 : cov);
    double v   = W_RAW + (B_RAW - W_RAW) * cov;
    if (g_noise > 0)
    {
        g_seed = (g_seed * 1103515245u) + 12345u;          /* LCG */
        v += (double)((int32_t)((g_seed >> 16) % (uint32_t)(2 * g_noise + 1)) - g_noise);
    }
    v = (v < 0.0) ? 0.0 : ((v > 4095.0) ? 4095.0 : v);
    return (uint16_t)v;
}

/* flip_pct: % of S3 steps where dv changed sign (left <-> right): the
 * "shaky" measure from the car log (27 % on v19 at 20 mm, 28 Oct). */
typedef struct { double progress; double max_err; int edge_pct; int losts; int flip_pct; } run_t;

/* Drive the car along the current track. start offset: car centre lateral
 * offset in mm (+ = left). Returns how far along it got (fraction). */
static run_t drive (double base, int32_t steer_max, double lookahead, double offset_mm)
{
    s3_line_t     ln;
    s3_line_out_t o;
    run_t         r = {0.0, 0.0, 0, 0, 0};
    int           flips = 0;
    int32_t       dv_prev = 0;
    double x = 0.0, y = offset_mm + g_offset, h = 0.0, vl = 0.0, vr = 0.0, dist = 0.0;
    double t_ms, total = (double)(g_n - 1);
    int    i, idx = 0, steps = 0, edge = 0, best_idx = 0;
    int32_t dv = 0;

    s3_line_init(&ln, g_edge);
    s3_line_set_gains(&ln, g_kp, g_kd);
    g_seed = 12345u;
    for (i = 0; i < 20; i++) s3_line_step(&ln, 200U, 200U, 0U, 10000U, &o);
    for (i = 0; i < 20; i++) s3_line_step(&ln, 3000U, 3000U, 0U, 10000U, &o);

    for (t_ms = 0.0; t_ms < 60000.0; t_ms += 1.0)
    {
        if (0 == ((int)t_ms % 10))                    /* S3 line step */
        {
            double sx = x + lookahead * cos(h), sy = y + lookahead * sin(h);
            double cx = sx - g_c_left * sin(h),       cy = sy + g_c_left * cos(h);  /* left normal */
            double ox = cx + g_spacing * sin(h),      oy = cy - g_spacing * cos(h); /* + = right */
            uint16_t rc = sensor(cx, cy), ro = sensor(ox, oy);
            s3_line_step(&ln, ro, rc, (uint32_t)(((vl + vr) / 2.0) * 10.0), 10000U, &o);
            dv = s3_line_dv(&o, steer_max);           /* same as task_line */
            if (((dv > 0) && (dv_prev < 0)) || ((dv < 0) && (dv_prev > 0))) { flips++; }
            if (0 != dv) { dv_prev = dv; }
            steps++;
            edge  += ((o.n_c >= 250) && (o.n_c <= 750)) ? 1 : 0;
            r.losts += o.lost_event;
        }
        if (0 == ((int)t_ms % 20))                    /* S2 FOLLOW */
        {
            double corr = -(double)((dv * 50) / 100);   /* S2 sign: + = left */
            double tl = base - corr, tr = base + corr;
            vl += (tl > vl) ? fmin(RAMP_MMS, tl - vl) : -fmin(RAMP_MMS, vl - tl);
            vr += (tr > vr) ? fmin(RAMP_MMS, tr - vr) : -fmin(RAMP_MMS, vr - tr);
        }
        h += ((vr - vl) / TRACK) * 0.001;
        x += ((vl + vr) / 2.0) * cos(h) * 0.001;
        y += ((vl + vr) / 2.0) * sin(h) * 0.001;
        dist += fabs((vl + vr) / 2.0) * 0.001;
        if (0 == ((int)t_ms % 50))
        {
            double e = line_dist(x, y, &idx);         /* axle centre vs line */
            if (idx > best_idx) { best_idx = idx; }
            e = fabs(e - fabs(g_offset));             /* vs the steady offset */
            if ((best_idx > 30) && (e > r.max_err)) { r.max_err = e; }
            if (e > 150.0) { break; }                 /* lost for good */
            /* Done when the SENSORS (lookahead in front) reach the end:
             * past it they see white and would report a false LOST. */
            if (best_idx >= (g_n - 3 - (int)lookahead)) { best_idx = g_n - 1; break; }
        }
        if (dist > 3.0 * total) { break; }            /* circling */
    }
    r.progress = (double)best_idx / total;
    r.edge_pct = (steps > 0) ? (100 * edge) / steps : 0;
    r.flip_pct = (steps > 0) ? (100 * flips) / steps : 0;
    return r;
}

static void run_case (char const * p_id, char const * p_name, double base, int32_t smax,
                      double min_progress, double max_err)
{
    static double const looks[] = {40.0, 60.0, 80.0};
    int  k, ok = 1;
    char line[160];

    for (k = 0; k < 3; k++)
    {
        run_t r = drive(base, smax, looks[k], 0.0);
        printf("  %-26s %3.0f mm/s steer %3d lookahead %2.0f: %3.0f %% of track, max err %5.1f mm, "
               "Centre on edge %3d %%, LOST %d\n", p_name, base, (int)smax, looks[k],
               100.0 * r.progress, r.max_err, r.edge_pct, r.losts);
        ok = ok && (r.progress >= min_progress) && (r.max_err <= max_err);
    }
    (void)snprintf(line, sizeof(line), "%s: >= %.0f %% of the track, axle within %.0f mm of the line",
                   p_name, 100.0 * min_progress, max_err);
    check(p_id, ok, line);
}

static void layout_set (char const * p_name, double c_left, double spacing, int32_t edge)
{
    g_c_left  = c_left;
    g_spacing = spacing;
    g_edge    = edge;
    /* Steady state: the Centre on its edge. Riding the RIGHT edge puts the
     * Centre 9 mm right of the line centre, so the car centre sits at
     * 9 + c_left right of it (y negative = right). */
    g_offset  = (S3_LN_EDGE_RIGHT == edge) ? -(LINE_HALF + c_left) : (LINE_HALF - c_left);
    printf("== layout: %s (car centre %+.0f mm from the line centre when on track)\n",
           p_name, g_offset);
}

static void all_tracks (char const * p_tag)
{
    char id[16];

    (void)snprintf(id, sizeof(id), "S3-%s1", p_tag);
    pen_start(); straight(2000.0);
    run_case(id, "straight 2 m", 80.0, 80, 0.98, 25.0);
    (void)snprintf(id, sizeof(id), "S3-%s2", p_tag);
    pen_start(); straight(300.0); arc(200.0, 90.0); straight(400.0);
    run_case(id, "left curve r200", 80.0, 80, 0.98, 30.0);
    (void)snprintf(id, sizeof(id), "S3-%s3", p_tag);
    pen_start(); straight(300.0); arc(200.0, -90.0); straight(400.0);
    run_case(id, "right curve r200", 80.0, 80, 0.98, 30.0);
    (void)snprintf(id, sizeof(id), "S3-%s4", p_tag);
    pen_start(); straight(300.0); corner(-90.0); straight(500.0);
    run_case(id, "sharp RIGHT 90", 80.0, 80, 0.95, 60.0);
    (void)snprintf(id, sizeof(id), "S3-%s5", p_tag);
    pen_start(); straight(300.0); corner(90.0); straight(500.0);
    run_case(id, "sharp LEFT 90", 80.0, 80, 0.95, 60.0);
    (void)snprintf(id, sizeof(id), "S3-%s6", p_tag);
    pen_start(); straight(300.0); arc(150.0, -90.0); arc(150.0, 90.0); straight(400.0);
    run_case(id, "S-bend r150", 80.0, 80, 0.98, 35.0);
}

/* v20 gain check: the same track with sensor noise, v19 gains (150 / 400)
 * against the v20 defaults. Pass = v20 stays on track and flips sign
 * less often. The noise level and its shape are guesses (UNVERIFIED). */
static void noise_compare (void)
{
    static struct { char const * p_name; int32_t kp; int32_t kd; } const sets[] =
    {
        {"v19 Kp 150 Kd 400", 150, 400},
        {"v20 Kp 250 Kd  50", S3_LN_KP_X100, S3_LN_KD_X100},
    };
    static int32_t const noise[] = {0, 40, 80};
    run_t r[2][3];
    int   n, k, ok = 1;
    char  line[160];

    layout_set("OUR CAR 20 mm, LEFT edge, sensor noise", 5.0, 20.0, S3_LN_EDGE_LEFT);
    pen_start(); straight(300.0); arc(200.0, -90.0); arc(200.0, 90.0); straight(400.0);
    for (n = 0; n < 3; n++)
    {
        g_noise = noise[n];
        for (k = 0; k < 2; k++)
        {
            g_kp = sets[k].kp; g_kd = sets[k].kd;
            r[k][n] = drive(80.0, 80, 60.0, 0.0);
            printf("  noise +-%2d raw  %s: %3.0f %% of track, max err %5.1f mm, "
                   "Centre on edge %3d %%, dv sign flips %2d %% of steps\n",
                   (int)g_noise, sets[k].p_name, 100.0 * r[k][n].progress,
                   r[k][n].max_err, r[k][n].edge_pct, r[k][n].flip_pct);
        }
        ok = ok && (r[1][n].progress >= 0.98) && (r[1][n].max_err <= 35.0) &&
             (r[1][n].flip_pct <= r[0][n].flip_pct);
    }
    g_noise = 0; g_kp = S3_LN_KP_X100; g_kd = S3_LN_KD_X100;
    (void)snprintf(line, sizeof(line), "S-bend r200 with noise 0/40/80: v20 gains on track "
                   "and flip sign no more often than v19");
    check("S3-S90", ok, line);
}

int main (void)
{
    /* OUR CAR (Sean, photo and 20 mm test, 28 Oct): outer sensor on the robot's RIGHT,
     * Centre about 5 mm left of the car centre, Centre rides the LEFT edge
     * (S3_EDGE_SIDE in s3_config.h). Spacing: 20 mm (Sean's best on the car), and the
     * ~33 mm estimated from the photo (UNVERIFIED). */
    layout_set("OUR CAR: outer RIGHT 20 mm, Centre 5 mm left, LEFT edge",
               5.0, 20.0, S3_LN_EDGE_LEFT);
    all_tracks("S0");
    layout_set("OUR CAR as in the photo: outer RIGHT ~33 mm, LEFT edge",
               5.0, 33.0, S3_LN_EDGE_LEFT);
    all_tracks("S1");

    /* Mirror image (outer on the LEFT, RIGHT edge): the same rules must
     * work the other way round, for the day the sensor is moved. */
    layout_set("mirror: outer LEFT 21 mm, Centre 5 mm left, RIGHT edge",
               5.0, -21.0, S3_LN_EDGE_RIGHT);
    all_tracks("S2");

    noise_compare();

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
