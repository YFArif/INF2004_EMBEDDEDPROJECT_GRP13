/*
 *	test_line.c -- host (laptop) unit test for s3_line.c and s3_nav.c
 *
 *	  gcc -std=c99 -Wall -Wextra -I../../../app_program/subsystem3 test_line.c \
 *	      ../../../app_program/subsystem3/s3_line.c ../../../app_program/subsystem3/s3_ir_cal.c ../../../app_program/subsystem3/s3_nav.c \
 *	      ../../../app_program/subsystem3/s3_code39.c \
 *	      -o tline && ./tline
 *
 *	Raw levels below are like the robot: white ~200, black ~3000 counts.
 *	Readings are filtered (4-sample IIR), so each case feeds several steps.
 */
#include <stdio.h>
#include "s3_line.h"
#include "s3_nav.h"

#define W     200U
#define B     3000U
#define MID   1600U          /* half black: Centre exactly on the edge */
#define STEP  3000U          /* 3 mm per step = 0.30 m/s at 10 ms       */
#define DT    10000U         /* 10 ms                                    */

static int g_fail = 0;

static void check (char const * p_id, int ok, char const * p_what)
{
    printf("[TEST] %-8s %s  %s\n", p_id, ok ? "PASS" : "FAIL", p_what);
    g_fail += !ok;
}

/* feed n identical steps; returns number of junction (+ lost*100) events */
static int feed (s3_line_t * p_ln, uint16_t l, uint16_t c, uint32_t mv, int n, s3_line_out_t * p_o)
{
    int i, ev = 0;
    for (i = 0; i < n; i++)
    {
        s3_line_step(p_ln, l, c, mv, DT, p_o);
        ev += p_o->junction_event + 100 * p_o->lost_event;
    }
    return ev;
}

static void calibrate (s3_line_t * p_ln, s3_line_out_t * p_o)
{
    (void)feed(p_ln, W, W, 0U, 20, p_o);     /* parked start-up sweep: white ... */
    (void)feed(p_ln, B, B, 0U, 20, p_o);     /* ... black ...                    */
    (void)feed(p_ln, W, MID, 0U, 20, p_o);   /* ... then put on the edge         */
}

int main (void)
{
    s3_line_t     ln;
    s3_line_out_t o;
    s3_ir_cal_t      c;
    int           i, ev, ok;
    int32_t       s10, s20;

    /* L01: never steer before both sensors have seen white and black */
    s3_line_init(&ln, S3_LN_EDGE_LEFT);
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    check("S3-L01", (0U == o.calibrated) && (0 == o.steer), "uncalibrated -> steer 0");

    /* L02: on the edge -> steer about 0, and no D kick right after calibration */
    s3_line_init(&ln, S3_LN_EDGE_LEFT);
    calibrate(&ln, &o);
    (void)feed(&ln, W, MID, STEP, 1, &o);
    check("S3-L02", (1U == o.calibrated) && (o.steer > -50) && (o.steer < 50) &&
                    (S3_LN_FOLLOW == o.state), "Centre on edge -> steer ~0, FOLLOW, no D kick");

    /* L03/L04: steering direction (Centre follows the LEFT edge) */
    (void)feed(&ln, W, 2600U, STEP, 10, &o);
    check("S3-L03", o.steer < -100, "drifted onto line (C darker) -> steer LEFT (negative)");
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    (void)feed(&ln, W, MID, STEP, 5, &o);     /* v19: drive on the edge first */
    (void)feed(&ln, W, 600U, STEP, 10, &o);
    check("S3-L04", o.steer > 100, "drifted off line (C whiter) -> steer RIGHT (positive)");

    /* L05: right-edge mounting flips the sign */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    (void)feed(&ln, W, 2600U, STEP, 10, &o);
    check("S3-L05", o.steer > 100, "right-edge follow: C darker -> steer RIGHT");

    /* L06: a junction (L and C go black together) -> exactly 1 event */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev = feed(&ln, B, B, STEP, 10, &o);
    check("S3-L06", (1 == ev) && (S3_LN_JUNCTION == o.state), "30 mm of junction -> 1 event, JUNCTION, steer 0");

    /* L07: a 6 mm blip is NOT a junction */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, B, B, STEP, 2, &o);
    ev += feed(&ln, W, MID, STEP, 5, &o);
    check("S3-L07", 0 == ev, "6 mm of black -> no junction");

    /* L08: parked on a junction (moved 0) -> no event */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev = feed(&ln, B, B, 0U, 100, &o);
    check("S3-L08", 0 == ev, "stopped on black -> no junction");

    /* L09: second junction only after 30 mm of normal line */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, B, B, STEP, 6, &o);
    ev += feed(&ln, W, MID, STEP, 5, &o);     /* ~15 mm: not re-armed */
    ev += feed(&ln, B, B, STEP, 6, &o);
    ok  = (1 == ev);
    ev += feed(&ln, W, MID, STEP, 15, &o);    /* ~45 mm: re-armed */
    ev += feed(&ln, B, B, STEP, 6, &o);
    check("S3-L09", ok && (2 == ev), "re-arm only after 30 mm off the junction");

    /* L10: lost after 40 mm of all-white, once, keeps steering back */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    (void)feed(&ln, W, MID, STEP, 5, &o);     /* v19: drive on the edge first */
    (void)feed(&ln, W, 900U, STEP, 10, &o);   /* last correction: right */
    ev = feed(&ln, W, W, STEP, 25, &o);
    check("S3-L10", (100 == ev) && (S3_LN_LOST == o.state) && (o.steer > 0),
          "75 mm white -> LOST once, still steering back (right)");

    /* L11: a 30 mm white gap is not lost; line found again -> FOLLOW */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, W, W, STEP, 10, &o);
    ev += feed(&ln, W, MID, STEP, 5, &o);
    check("S3-L11", (0 == ev) && (S3_LN_FOLLOW == o.state), "30 mm white gap -> not lost");

    /* L12: relax never drops below the trusted range */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 300U); s3_ir_cal_learn(&c, 1000U);
    for (i = 0; i < 200; i++) { s3_ir_cal_learn(&c, 300U); s3_ir_cal_learn(&c, 700U); s3_ir_cal_relax(&c); }
    check("S3-L12", (1U == c.valid) && ((uint32_t)(c.black - c.white) >= S3_LN_MIN_RANGE),
          "calibration relax keeps >= 600 counts range");

    /* L13: a one-off extreme (car lifted: 4000) fades once normal black returns */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 4000U);
    for (i = 0; i < 30; i++) { s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 3000U); s3_ir_cal_relax(&c); }
    check("S3-L13", (c.black < 3100U) && (c.black >= 3000U), "lifted-car extreme fades to real black in 30 s");

    /* L14: a sensor that sees only white keeps its black level (Left sensor) */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 200U); s3_ir_cal_learn(&c, 3000U);
    for (i = 0; i < 120; i++) { s3_ir_cal_learn(&c, 210U); s3_ir_cal_relax(&c); }
    check("S3-L14", 3000U == c.black, "2 min of white only -> black level unchanged");

    /* L15: drift onto the line / hard left curve: Centre black long before Left
            -> NOT a junction, steering stays hard left */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, W, B, STEP, 10, &o);      /* 30 mm: Centre black, Left white */
    ev += feed(&ln, B, B, STEP, 10, &o);      /* then Left black too             */
    check("S3-L15", (0 == ev) && (o.steer < -500) && (S3_LN_FOLLOW == o.state),
          "curve/drift (C black 30 mm before L) -> no junction, steer hard left");

    /* L16 (v19 straddle): outer black + Centre white = drifted towards the
           outer side (right here) -> steer RIGHT, at least OUTER_STEER */
    (void)feed(&ln, B, W, STEP, 10, &o);
    check("S3-L16", (1U == o.overshoot) && (o.steer >= S3_LN_OUTER_STEER), "outer on line -> steer RIGHT >= 600");

    /* L17: D term uses real time: same jump over 20 ms gives a smaller D than over 10 ms */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    s3_line_set_gains(&ln, 0, 400);
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    s3_line_step(&ln, W, 2000U, STEP, DT, &o);       s10 = o.steer;
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    s3_line_set_gains(&ln, 0, 400);
    s3_line_step(&ln, W, MID, STEP, 2U * DT, &o);
    s3_line_step(&ln, W, 2000U, STEP, 2U * DT, &o);  s20 = o.steer;
    check("S3-L17", (s10 < 0) && (s20 < 0) && (s10 <= 2 * s20 + 8) && (s10 >= 2 * s20 - 8),
          "D per 10 ms: 20 ms step gives half the D of a 10 ms step");

    /* L18: crossing a real 18 mm line: 0.30 m/s (6 steps) and 0.40 m/s (4.5 steps) */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, B, B, 3000U, 6, &o);
    ev += feed(&ln, W, MID, 3000U, 15, &o);
    ev += feed(&ln, B, B, 4000U, 4, &o);
    check("S3-L18", 2 == ev, "18 mm crossing line detected at 0.30 and 0.40 m/s");

    /* L19: overshoot, then recovering back over the line: NOT a junction */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, W, B, STEP, 3, &o);       /* drifted onto the line      */
    ev += feed(&ln, W, W, STEP, 3, &o);       /* Centre past it (far white) */
    ev += feed(&ln, B, W, STEP, 5, &o);       /* Left on the line: overshoot */
    ev += feed(&ln, B, B, STEP, 10, &o);      /* steering back: both black   */
    check("S3-L19", 0 == ev, "overshoot recovery (Left black first while Centre white) -> no junction");

    /* L20: stuck extreme (car lifted, black=3999) fades with real 500/2500 readings */
    s3_ir_cal_init(&c); s3_ir_cal_learn(&c, 500U); s3_ir_cal_learn(&c, 3999U);
    for (i = 0; i < 60; i++) { s3_ir_cal_learn(&c, 500U); s3_ir_cal_learn(&c, 2500U); s3_ir_cal_relax(&c); }
    check("S3-L20", (c.black < 2600U) && (s3_ir_cal_norm(&c, 2500U) > S3_LN_BLACK),
          "black stuck at 3999 recovers within 60 s; real black reads black again");

    /* L21: coming back from LOST gives no D kick (steer = P only) */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    (void)feed(&ln, W, W, STEP, 25, &o);      /* LOST */
    s3_line_step(&ln, W, 1885U, STEP, DT, &o);   /* line found, e ~ +100 */
    check("S3-L21", (S3_LN_FOLLOW == o.state) && (o.steer > -300) && (o.steer < 0),
          "LOST -> FOLLOW: steer from P only (no D kick)");

    /* L22: right-edge mounting (v19: outer sensor on the robot's LEFT):
            outer on the line steers LEFT, junction still detected */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    ev = feed(&ln, B, B, STEP, 10, &o);
    ok = (1 == ev);
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    (void)feed(&ln, B, W, STEP, 10, &o);
    check("S3-L22", ok && (1U == o.overshoot) && (o.steer <= -S3_LN_OUTER_STEER),
          "right edge: junction 1 event; outer (on the left) on line -> steer LEFT");

    /* L23: LF_ENABLE clear (turning/bypassing): steer 0, no junction or lost events */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    s3_line_set_mode(&ln, S3_LN_MODE_DISABLED);
    ev  = feed(&ln, B, B, STEP, 10, &o);
    ev += feed(&ln, W, W, STEP, 30, &o);
    check("S3-L23", (0 == ev) && (0 == o.steer) && (S3_LN_DISABLED == o.state), "disabled: steer 0, no events");

    /* L24: search from the edge's own side (right edge, car right of line): found at the first edge */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    s3_line_set_mode(&ln, S3_LN_MODE_SEARCH_FROM_RIGHT);
    (void)feed(&ln, W, W, STEP, 5, &o);
    ok = (S3_LN_SEARCH == o.state) && (0 == o.steer);
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    check("S3-L24", ok && (1U == o.line_found_event) && (S3_LN_FOLLOW == o.state),
          "search from right (right edge): LINE FOUND at the first (correct) edge");

    /* L25: search from the other side: the first edge is the WRONG one -> keep searching,
            found only after Centre crossed the black and reached the far (correct) edge */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    s3_line_set_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);
    s3_line_step(&ln, W, MID, STEP, DT, &o);  ok = (0U == o.line_found_event);   /* left edge: wrong one */
    s3_line_step(&ln, W, B, STEP, DT, &o);    ok = ok && (0U == o.line_found_event); /* on the line */
    s3_line_step(&ln, W, MID, STEP, DT, &o);                                      /* right edge */
    check("S3-L25", ok && (1U == o.line_found_event) && (S3_LN_FOLLOW == o.state),
          "search from left (right edge): ignores the wrong edge, LINE FOUND after crossing");

    /* L27: v14 review defect -- after LINE FOUND the Vehicle Controller still has
            LF_SEARCH_L set (and may add LF_ENABLE): the same REQUEST must not restart
            the search.  Then clearing LF_SEARCH_L (-> FOLLOW) must not reset, and a
            new search request must start a new search. */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    s3_line_step(&ln, W, B,   STEP, DT, &o);
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    ok = (1U == o.line_found_event);
    ev = 0;
    for (i = 0; i < 5; i++) {                 /* flags now SEARCH_L | ENABLE: same mode */
        s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);
        s3_line_step(&ln, W, MID, STEP, DT, &o);
        ev += o.line_found_event;
        ok = ok && (S3_LN_FOLLOW == o.state);
    }
    s3_line_request_mode(&ln, S3_LN_MODE_FOLLOW);   /* VC clears LF_SEARCH_L */
    s3_line_step(&ln, W, MID, STEP, DT, &o);
    ok = ok && (S3_LN_FOLLOW == o.state) && (0 == ev);
    s3_line_request_mode(&ln, S3_LN_MODE_SEARCH_FROM_LEFT);   /* a new search */
    s3_line_step(&ln, W, W, STEP, DT, &o);
    check("S3-L27", ok && (S3_LN_SEARCH == o.state),
          "found line stays FOLLOW while LF_SEARCH_x still set (5/5 steps, 0 extra FOUND); new request re-searches");

    /* L28: curve boost -- Centre fully white gives (near) full lock; on the
            edge the steering stays gentle (straight line unchanged) */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    s3_line_set_gains(&ln, 100, 300);
    s3_line_set_shape(&ln, 300);              /* boost is off by default (v18) */
    (void)feed(&ln, W, MID, STEP, 5, &o);
    {
        int32_t edge_steer = o.steer;
        (void)feed(&ln, W, W, STEP, 3, &o);   /* 9 mm of white: not lost yet */
        check("S3-L28", (S3_LN_FOLLOW == o.state) && (edge_steer > -300) && (edge_steer < 300) &&
                        ((o.steer >= 950) || (o.steer <= -950)),
              "curve boost: Centre all white -> >= 95 % lock; on the edge |steer| < 300");
    }

    /* L29: late wake-up -- the step after an overrun can come only a few
            us after the previous one. Then d_err is huge (here 1e6) and
            Kd * d_err used to overflow 32 bits, flipping the D term's sign
            (steer right instead of left). Must saturate the right way. */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    s3_line_set_gains(&ln, 0, S3_LN_KD_MAX_X100);       /* D term only */
    (void)feed(&ln, W, MID, STEP, 3, &o);               /* on the edge  */
    s3_line_step(&ln, W, (uint16_t)(W + ((B - W) * 7U) / 10U), 0U, 1U, &o);
    /* C at 700 permille (inside the EDGE band, so no curve boost), 1 us
       later: d_err = 200 * 10000 / 1 = 2e6, and Kd * d_err = 8e9. */
    check("S3-L29", (S3_LN_FOLLOW == o.state) && (o.steer == -S3_LN_D_LIMIT),
          "dt 1 us, Kd max: D term saturates to steer -600 (no 32-bit overflow)");

    /* L30-L35: v19 STRADDLE rules on our car: Centre on the LEFT edge, outer
       sensor just outside the RIGHT edge (S3_LN_EDGE_LEFT). Steer + = right. */
    {
        /* L30: outer black, Centre part white (drifted LEFT) -> steer RIGHT
                at least S3_LN_OUTER_STEER, flagged, no junction. */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 5, &o);
        ev = feed(&ln, B, 1000U, STEP, 5, &o);
        check("S3-L30", (0 == ev) && (1U == o.overshoot) && (o.steer >= S3_LN_OUTER_STEER),
              "outer on the line (drift left) -> steer RIGHT >= 600, no junction");

        /* L31: drifted far LEFT: outer saw the line last, now both white
                -> full RIGHT, and LOST keeps steering RIGHT. */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 5, &o);
        (void)feed(&ln, W, W, STEP, 1, &o);        /* Centre goes white     */
        (void)feed(&ln, B, W, STEP, 4, &o);        /* line passes the outer */
        s3_line_step(&ln, W, W, STEP, DT, &o);
        ok = (1U == o.off_line) && (1000 == o.steer);
        ev = feed(&ln, W, W, STEP, 20, &o);
        check("S3-L31", ok && (100 == ev) && (S3_LN_LOST == o.state) && (1000 == o.steer),
              "far left (outer saw line last) -> full RIGHT, LOST keeps it");

        /* L32: drifted far RIGHT: Centre crossed the line, outer never black
                -> full LEFT, LOST keeps it. */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 5, &o);
        (void)feed(&ln, W, B, STEP, 6, &o);        /* Centre over the line  */
        s3_line_step(&ln, W, W, STEP, DT, &o);     /* past the right edge   */
        ok = (1U == o.off_line) && (-1000 == o.steer);
        ev = feed(&ln, W, W, STEP, 20, &o);
        check("S3-L32", ok && (100 == ev) && (S3_LN_LOST == o.state) && (-1000 == o.steer),
              "far right (Centre saw line last) -> full LEFT, LOST keeps it");

        /* L33: sharp LEFT turn: the line swings left under the Centre (dark),
                then out past it (white); the outer never sees it -> LEFT.
                L33b: a slight drift LEFT (Centre fades to white before the
                outer reaches the line) -> RIGHT, not left (v19 review). */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 10, &o);   /* 30 mm on the edge */
        (void)feed(&ln, W, B, STEP, 2, &o);
        s3_line_step(&ln, W, W, STEP, DT, &o);
        ok = (1U == o.off_line) && (-1000 == o.steer);
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 10, &o);   /* 30 mm on the edge */
        (void)feed(&ln, W, 1000U, STEP, 1, &o);   /* fading: light side */
        s3_line_step(&ln, W, W, STEP, DT, &o);
        check("S3-L33", ok && (1U == o.off_line) && (1000 == o.steer),
              "sharp left turn -> full LEFT; slight drift left -> RIGHT");

        /* L34: sharp RIGHT turn: the line swings under the outer sensor
                (Centre white), then leaves both -> RIGHT. This is the case
                v15b/v18 steered LEFT (28 Oct logs). */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        (void)feed(&ln, W, MID, STEP, 5, &o);
        s3_line_step(&ln, B, W, STEP, DT, &o);     /* outer on the line     */
        ok = (o.steer >= S3_LN_OUTER_STEER);
        s3_line_step(&ln, W, W, STEP, DT, &o);     /* line gone right       */
        check("S3-L34", ok && (1U == o.off_line) && (1000 == o.steer),
              "sharp right turn (outer saw it last) -> RIGHT, not left");

        /* L35: normal tracking near the edge never triggers OUTER / OFF LINE,
                and a cross junction (both black) is still 1 event. */
        s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
        ok = 1;
        for (i = 0; i < 40; i++)
        {
            uint16_t c_raw = (uint16_t)(MID + ((i % 4) * 300U) - 450U);   /* 1150..2050 */
            s3_line_step(&ln, W, c_raw, STEP, DT, &o);
            ok = ok && (0U == o.overshoot) && (0U == o.off_line) && (S3_LN_FOLLOW == o.state);
        }
        ev = feed(&ln, B, B, STEP, 10, &o);
        check("S3-L35", ok && (1 == ev), "edge wobble: no OUTER/OFF LINE; junction still 1 event");

        /* L36 (v20): the OUTER push grows with the outer reading instead of
                jumping to 600: Centre on its edge (PD ~ 0), outer half on the
                line -> about 600 * (500 - 250) / 750 = 200 right; outer at
                300 permille -> about 40; outer white -> no push, not flagged. */
        {
            int32_t s_half, s_low;
            uint16_t o_half = (uint16_t)(W + ((B - W) * 5U) / 10U);
            uint16_t o_low  = (uint16_t)(W + ((B - W) * 3U) / 10U);

            s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
            (void)feed(&ln, o_half, MID, STEP, 5, &o);
            s_half = o.steer; ok = (1U == o.overshoot);
            (void)feed(&ln, o_low, MID, STEP, 5, &o);
            s_low = o.steer; ok = ok && (1U == o.overshoot);
            (void)feed(&ln, W, MID, STEP, 5, &o);
            ok = ok && (0U == o.overshoot) && (o.steer > -20) && (o.steer < 20);
            printf("  OUTER push: outer 500 -> steer %d, outer 300 -> steer %d\n", (int)s_half, (int)s_low);
            check("S3-L36", ok && (s_half >= 180) && (s_half <= 220) && (s_low >= 30) && (s_low <= 60),
                  "OUTER push proportional: outer 500 -> ~200, 300 -> ~40, white -> 0");
        }

        /* L37 (v20): search full lock is steer_max x 5/4 (was x 2), tracking
                full lock is steer_max, both clamped to the 10-bit field. */
        {
            s3_line_out_t t = {0};
            int32_t dv_follow, dv_search, dv_clamp;

            t.state = S3_LN_FOLLOW; t.steer = 1000;
            dv_follow = s3_line_dv(&t, 80);
            t.off_line = 1U;
            dv_search = s3_line_dv(&t, 80);
            dv_clamp  = s3_line_dv(&t, 511);
            t.off_line = 0U; t.state = S3_LN_LOST; t.steer = -1000;
            check("S3-L37", (80 == dv_follow) && (100 == dv_search) && (511 == dv_clamp) &&
                  (-100 == s3_line_dv(&t, 80)),
                  "dv: follow 80, OFF LINE / LOST x 5/4 = 100, clamp 511");
        }
    }

    /* L26: Change Requirement 1 -- packed sizes */
    printf("  sizeof(s3_ir_cal_t)=%u (was 14)  sizeof(s3_line_t)=%u (was 80)\n",
           (unsigned)sizeof(s3_ir_cal_t), (unsigned)sizeof(s3_line_t));
    check("S3-L26", (8U == sizeof(s3_ir_cal_t)) && (sizeof(s3_line_t) <= 48U), "calibration 14 -> 8 B, line state 80 -> <= 48 B");

    /* N01: barcode letter -> command */
    check("S3-N01", (S3_NAV_LEFT == s3_nav_from_char('A')) && (S3_NAV_RIGHT == s3_nav_from_char('B')) &&
                    (S3_NAV_STRAIGHT == s3_nav_from_char('C')) && (S3_NAV_UTURN == s3_nav_from_char('D')) &&
                    (S3_NAV_NONE == s3_nav_from_char('Z')), "A/B/C/D -> L/R/S/U, others NONE");

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
