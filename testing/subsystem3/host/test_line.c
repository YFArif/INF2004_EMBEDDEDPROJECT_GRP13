/*
 *	test_line.c -- host (laptop) unit test for s3_line.c (v29, one sensor
 *	on each edge) and the nav map in s3_code39.c. Normally run by
 *	../run_host_tests.sh, or:
 *
 *	  gcc -std=c99 -Wall -Wextra -I../../../app_program/subsystem3 test_line.c \
 *	      ../../../app_program/subsystem3/s3_line.c \
 *	      ../../../app_program/subsystem3/s3_code39.c \
 *	      -o tline && ./tline
 *
 *	Raw levels below are like the robot: white ~200, black ~3000 counts.
 *	Layout as on our car: GP26 on the robot's RIGHT (S3_LN_L_ON_RIGHT),
 *	GP27 on its LEFT. lr() feeds (left, right) and maps them to the
 *	channels, so each case reads in physical terms. Centred on the line,
 *	each sensor sits on an edge and reads MID (500 permille).
 */
#include <stdio.h>
#include "s3_line.h"
#include "s3_code39.h"

#define W     200U
#define B     3000U
#define MID   1600U          /* half black: sensor on an edge (500)      */
#define DARK  2300U          /* sensor moved onto the line   (750)       */
#define LIGHT 900U           /* sensor moved off the line    (250)       */
#define STEP  3000U          /* 3 mm per step = 0.30 m/s at 10 ms       */
#define DT    10000U         /* 10 ms                                    */
#define KP    S3_LN_KP_X100

static int g_fail = 0;

static void check (char const * p_id, int ok, char const * p_what)
{
    printf("[TEST] %-8s %s  %s\n", p_id, ok ? "PASS" : "FAIL", p_what);
    g_fail += !ok;
}

static int near (int32_t v, int32_t want)
{
    return (v >= (want - 20)) && (v <= (want + 20));
}

/* Feed n identical steps, LEFT and RIGHT sensor raw values (our car:
 * GP26 = right, GP27 = left). Returns junction + 100 x lost events. */
static int lr (s3_line_t * p_ln, uint16_t left, uint16_t right, uint32_t mv, int n,
               s3_line_out_t * p_o)
{
    int i, ev = 0;
    for (i = 0; i < n; i++)
    {
        s3_line_step(p_ln, right, left, mv, DT, p_o);
        ev += p_o->junction_event + 100 * p_o->lost_event;
    }
    return ev;
}

/* Parked start-up sweep (white, then black), then centred on the line. */
static void calibrate (s3_line_t * p_ln, s3_line_out_t * p_o)
{
    (void)lr(p_ln, W, W, 0U, 20, p_o);
    (void)lr(p_ln, B, B, 0U, 20, p_o);
    (void)lr(p_ln, MID, MID, 0U, 5, p_o);
}

static void fresh (s3_line_t * p_ln, s3_line_out_t * p_o)
{
    s3_line_init(p_ln, S3_LN_L_ON_RIGHT);
    calibrate(p_ln, p_o);
}

int main (void)
{
    s3_line_t     ln;
    s3_line_out_t o;
    s3_ir_cal_t   c;
    int           i, ev, ok;
    int32_t       s_p, s_pd;

    /* L01: never steer before both sensors have seen white and black */
    s3_line_init(&ln, S3_LN_L_ON_RIGHT);
    (void)lr(&ln, DARK, LIGHT, STEP, 1, &o);
    check("S3-L01", (0U == o.calibrated) && (0 == o.steer), "uncalibrated -> steer 0");

    /* L02: each sensor on its edge (both grey) -> straight */
    fresh(&ln, &o);
    (void)lr(&ln, MID, MID, STEP, 5, &o);
    check("S3-L02", (1U == o.calibrated) && (o.steer > -20) && (o.steer < 20) &&
          (S3_LN_FOLLOW == o.state) && (0U == o.overshoot) && (0U == o.off_line),
          "both grey (centred) -> steer ~0, FOLLOW");

    /* L03 / L04: drift: one sensor darker, the other lighter -> steer back */
    fresh(&ln, &o);
    (void)lr(&ln, DARK, LIGHT, STEP, 1, &o);
    check("S3-L03", (o.steer < 0) && near(o.steer, -(KP * 500) / 100),
          "car drifted right (left darker, right lighter) -> steer LEFT, Kp x e");
    fresh(&ln, &o);
    (void)lr(&ln, LIGHT, DARK, STEP, 1, &o);
    check("S3-L04", (o.steer > 0) && near(o.steer, (KP * 500) / 100),
          "car drifted left (right darker) -> steer RIGHT, same size");

    /* L05: GP26 mounted on the LEFT (S3_LN_L_ON_LEFT): the same physical
            case steers the same way */
    s3_line_init(&ln, S3_LN_L_ON_LEFT);
    for (i = 0; i < 20; i++) { s3_line_step(&ln, W, W, 0U, DT, &o); }
    for (i = 0; i < 20; i++) { s3_line_step(&ln, B, B, 0U, DT, &o); }
    s3_line_step(&ln, LIGHT, DARK, STEP, DT, &o);   /* GP26 = LEFT, light */
    check("S3-L05", o.steer > 0, "mirror mount: left sensor lighter -> steer RIGHT");

    /* L06: proportional: half drift -> Kp x 500 / 100; one sensor black,
            the other white (e = -1000) -> Kp x 1000 / 100, flagged */
    fresh(&ln, &o);
    (void)lr(&ln, DARK, LIGHT, STEP, 1, &o);
    s_p = o.steer;
    fresh(&ln, &o);
    (void)lr(&ln, B, W, STEP, 1, &o);
    printf("  half drift -> steer %d, black/white -> steer %d (Kp %d)\n", (int)s_p,
           (int)o.steer, (int)KP);
    check("S3-L06", near(s_p, -(KP * 500) / 100) &&
          near(o.steer, (KP >= 100) ? -1000 : -(KP * 1000) / 100) &&
          (1U == o.overshoot),
          "steer proportional to the difference; one sensor white -> flagged");

    /* L07: drift right until both white: the line was last seen on the
            LEFT -> turn LEFT at once (no hold in the edge pair) */
    fresh(&ln, &o);
    (void)lr(&ln, DARK, LIGHT, STEP, 2, &o);
    (void)lr(&ln, B, W, STEP, 2, &o);
    (void)lr(&ln, MID, W, STEP, 1, &o);
    (void)lr(&ln, W, W, STEP, 1, &o);
    check("S3-L07", (1U == o.off_line) && (o.steer <= -S3_LN_SOFT_PERMILLE) &&
          (o.steer > -1000) && (S3_LN_FOLLOW == o.state),
          "both white: soft LEFT at once, not yet full lock");
    s_p = o.steer;
    (void)lr(&ln, W, W, STEP, 4, &o);
    check("S3-L07b", (o.steer < s_p) && (o.steer > -1000) && (S3_LN_FOLLOW == o.state),
          "both white 15 mm -> LEFT rising, not yet LOST");

    /* L08: both white for 40 mm -> one LOST event, still full LEFT; a
            sensor on the line again -> FOLLOW with PD */
    ev = lr(&ln, W, W, STEP, 20, &o);
    ok = (100 == ev) && (S3_LN_LOST == o.state) && (-1000 == o.steer);
    (void)lr(&ln, MID, W, STEP, 1, &o);
    check("S3-L08", ok && (S3_LN_FOLLOW == o.state) && (0U == o.off_line) && (o.steer < 0),
          "40 mm both white -> 1 LOST event, keeps LEFT; line back -> FOLLOW");

    /* L09: a 30 mm white gap is not LOST */
    fresh(&ln, &o);
    (void)lr(&ln, LIGHT, DARK, STEP, 2, &o);
    ev = lr(&ln, W, W, STEP, 10, &o);
    ev += lr(&ln, MID, MID, STEP, 5, &o);
    check("S3-L09", (0 == ev) && (S3_LN_FOLLOW == o.state), "30 mm white gap -> not lost");

    /* L10: a parked car (no odometry) never goes LOST */
    fresh(&ln, &o);
    ev = lr(&ln, W, W, 0U, 200, &o);
    check("S3-L10", 0 == ev, "stopped on white for 2 s -> no LOST");

    /* L11: junction: both black for 5 mm -> ONE event, JUNCTION, steer 0;
            back on the edges -> FOLLOW; the next junction is reported too;
            3 mm of both black (noise, a speck) is not a junction */
    fresh(&ln, &o);
    ev = lr(&ln, B, B, STEP, 1, &o);
    ok = (0 == ev) && (S3_LN_FOLLOW == o.state) && (0 == o.steer);
    (void)lr(&ln, MID, MID, STEP, 2, &o);
    ev = lr(&ln, B, B, STEP, 33, &o);
    ok = ok && (1 == ev) && (S3_LN_JUNCTION == o.state) && (0 == o.steer);
    (void)lr(&ln, MID, MID, STEP, 3, &o);
    ok = ok && (S3_LN_FOLLOW == o.state);
    ev = lr(&ln, B, B, STEP, 6, &o);
    check("S3-L11", ok && (1 == ev),
          "both black 100 mm -> 1 junction event, steer 0; 3 mm -> none; next one re-arms");

    /* L12: relax never drops below the trusted range */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 300U); s3_ir_cal_learn(&c, 1000U);
    for (i = 0; i < 200; i++) { s3_ir_cal_learn(&c, 300U); s3_ir_cal_learn(&c, 700U); s3_ir_cal_relax(&c); }
    check("S3-L12", (1U == c.valid) && ((uint32_t)(c.black - c.white) >= S3_LN_MIN_RANGE),
          "calibration relax keeps >= 600 counts range");

    /* L13: a high extreme fades once normal black returns */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 3600U);
    for (i = 0; i < 30; i++) { s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 3000U); s3_ir_cal_relax(&c); }
    check("S3-L13", (c.black < 3100U) && (c.black >= 3000U), "one-off extreme fades to real black in 30 s");

    /* L14: a sensor that sees only white keeps its black level */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 3000U);
    for (i = 0; i < 120; i++) { s3_ir_cal_learn(&c, 210U); s3_ir_cal_relax(&c); }
    check("S3-L14", 3000U == c.black, "2 min of white only -> black level unchanged");

    /* L15: disabled: steer 0, no events */
    fresh(&ln, &o);
    s3_line_set_mode(&ln, S3_LN_MODE_DISABLED);
    ev = lr(&ln, W, W, STEP, 30, &o);
    check("S3-L15", (0 == ev) && (0 == o.steer) && (S3_LN_DISABLED == o.state),
          "disabled: steer 0, no events");

    /* L16 / L17: search: steer 0 until a sensor is on the line, then
            follow; the search side presets OFF LINE (from LEFT -> RIGHT) */
    fresh(&ln, &o);
    s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);
    ev = lr(&ln, W, W, STEP, 30, &o);
    ok = (0 == ev) && (S3_LN_SEARCH == o.state) && (0 == o.steer);
    (void)lr(&ln, DARK, DARK, STEP, 1, &o);  /* e = 0: side from the search */
    ok = ok && (1U == o.line_found_event) && (S3_LN_FOLLOW == o.state);
    s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);   /* flag still set */
    (void)lr(&ln, W, W, STEP, 1, &o);
    ok = ok && (o.steer >= S3_LN_SOFT_PERMILLE);
    (void)lr(&ln, W, W, STEP, 14, &o);
    check("S3-L16", ok && (1000 == o.steer),
          "search from LEFT: steer 0, found -> FOLLOW (not restarted), off line -> RIGHT");
    fresh(&ln, &o);
    s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_RIGHT);
    (void)lr(&ln, W, W, STEP, 5, &o);
    (void)lr(&ln, DARK, DARK, STEP, 1, &o);
    ok = (1U == o.line_found_event);
    (void)lr(&ln, W, W, STEP, 15, &o);
    check("S3-L17", ok && (-1000 == o.steer), "search from RIGHT: found, off line -> LEFT");

    /* L18: D adds to P on a sudden change; no D kick after LOST */
    fresh(&ln, &o);
    s3_line_set_gains(&ln, 100, 100);
    (void)lr(&ln, MID, MID, STEP, 3, &o);
    (void)lr(&ln, DARK, LIGHT, STEP, 1, &o);
    s_pd = o.steer;
    (void)lr(&ln, W, W, STEP, 20, &o);              /* LOST            */
    (void)lr(&ln, MID, W, STEP, 1, &o);             /* line back       */
    check("S3-L18", (s_pd < -550) && (o.steer >= -505) && (o.steer <= -495),
          "D adds on a sudden change; first step after LOST is P only");

    /* L19: late wake-up (dt 1 us) with the largest Kd: no overflow, the D
            term stays within its limit */
    fresh(&ln, &o);
    s3_line_set_gains(&ln, 0, S3_LN_KD_MAX_X100);
    (void)lr(&ln, MID, MID, STEP, 2, &o);
    s3_line_step(&ln, LIGHT, DARK, STEP, 1U, &o);   /* GP26 = right */
    check("S3-L19", o.steer == -S3_LN_D_LIMIT, "dt 1 us, Kd max: D clamped to its limit");

    /* L20: stuck extreme (black=3600) fades with real 500/2500 readings */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 500U); s3_ir_cal_learn(&c, 3600U);
    for (i = 0; i < 60; i++) { s3_ir_cal_learn(&c, 500U); s3_ir_cal_learn(&c, 2500U); s3_ir_cal_relax(&c); }
    check("S3-L20", (c.black < 2600U) && (s3_ir_cal_norm(&c, 2500U) > S3_LN_BLACK),
          "black stuck at 3600 recovers within 60 s; real black reads black again");

    /* L21: dv: follow = steer x steer_max; OFF LINE / LOST x 5/4; clamp */
    {
        s3_line_out_t t = {0};
        int32_t dv_follow, dv_search, dv_clamp;

        t.state = S3_LN_FOLLOW; t.steer = 1000;
        dv_follow = s3_line_dv(&t, 40);
        t.off_line = 1U;
        dv_search = s3_line_dv(&t, 40);
        dv_clamp  = s3_line_dv(&t, 511);
        t.off_line = 0U; t.state = S3_LN_LOST; t.steer = -1000;
        check("S3-L21", (40 == dv_follow) && (50 == dv_search) && (511 == dv_clamp) &&
              (-50 == s3_line_dv(&t, 40)),
              "dv: follow 40, OFF LINE / LOST x 5/4 = 50, clamp 511");
    }

    /* L22: curve boost adds steering only past BOOST_FROM */
    fresh(&ln, &o);
    s3_line_set_gains(&ln, 100, 0);
    s3_line_set_shape(&ln, 300);
    (void)lr(&ln, DARK, LIGHT, STEP, 1, &o);
    s_p = o.steer;                                  /* |e| = 500: no boost */
    (void)lr(&ln, B, 1040U, STEP, 1, &o);           /* e = -700           */
    check("S3-L22", (s_p >= -510) && (s_p <= -490) && (o.steer <= -1000 + 10),
          "boost 300: none at |e| 500, -700 + 300 x -200 / 100 -> full lock");

    /* L23: small differences (noise, |e| ~ 50 < SIDE_MIN) do not change the
            remembered side; a real drift does */
    fresh(&ln, &o);
    (void)lr(&ln, LIGHT, DARK, STEP, 1, &o);        /* line RIGHT          */
    for (i = 0; i < 20; i++)
    {
        (void)lr(&ln, MID, (uint16_t)(MID - ((i % 2) * 150U)), STEP, 1, &o);   /* |e| ~ 50 */
    }
    (void)lr(&ln, W, W, STEP, 15, &o);
    check("S3-L23", (1U == o.off_line) && (1000 == o.steer), "noise (|e| < 60) keeps the side -> RIGHT");

    /* L42: log 28 Oct 224 mm: left sensor white, right sensor 200 / 1000
            (a weak reading of the line): the side is RIGHT, not the older LEFT */
    fresh(&ln, &o);
    (void)lr(&ln, DARK, LIGHT, STEP, 2, &o);        /* line LEFT (e < 0)   */
    (void)lr(&ln, 150U, 560U, STEP, 1, &o);         /* L ~ 0, R ~ 130      */
    (void)lr(&ln, W, W, STEP, 15, &o);
    check("S3-L42", (1U == o.off_line) && (o.steer > 0), "weak reading names the side");

    /* L41: car lifted during the sweep (4050, no floor) is not learned:
            the tape (2700) still reads BLACK, not ~60 % grey (log 28 Oct) */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 150U); s3_ir_cal_learn(&c, 4050U);
    s3_ir_cal_learn(&c, 2700U);
    check("S3-L41", (2700U == c.black) && (s3_ir_cal_norm(&c, 2700U) > S3_LN_BLACK) &&
          (s3_ir_cal_norm(&c, 2500U) > 900),
          "car lifted (4050) not learned: tape 2700 reads black");

    /* L26: Change Requirement 1 -- packed sizes */
    printf("  sizeof(s3_ir_cal_t)=%u (was 14)  sizeof(s3_line_t)=%u (was 80)\n",
           (unsigned)sizeof(s3_ir_cal_t), (unsigned)sizeof(s3_line_t));
    check("S3-L26", (8U == sizeof(s3_ir_cal_t)) && (sizeof(s3_line_t) <= 48U),
          "calibration 14 -> 8 B, line state 80 -> <= 48 B");

    /* N01: barcode letter -> command */
    check("S3-N01", (S3_NAV_LEFT == s3_nav_from_char('A')) && (S3_NAV_RIGHT == s3_nav_from_char('B')) &&
                    (S3_NAV_STRAIGHT == s3_nav_from_char('C')) && (S3_NAV_UTURN == s3_nav_from_char('D')) &&
                    (S3_NAV_NONE == s3_nav_from_char('Z')), "A/B/C/D -> L/R/S/U, others NONE");

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
