/*
 *	test_line.c -- host (laptop) unit test for s3_line.c and s3_nav.c
 *
 *	  gcc -std=c99 -Wall -Wextra -I../../../app_program/subsystem3 test_line.c \
 *	      ../../../app_program/subsystem3/s3_line.c ../../../app_program/subsystem3/s3_nav.c \
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
    s3_cal_t      c;
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
    s3_cal_init(&c); s3_cal_learn(&c, 300U); s3_cal_learn(&c, 1000U);
    for (i = 0; i < 200; i++) { s3_cal_learn(&c, 300U); s3_cal_learn(&c, 700U); s3_cal_relax(&c); }
    check("S3-L12", (1U == c.valid) && ((uint32_t)(c.black - c.white) >= S3_LN_MIN_RANGE),
          "calibration relax keeps >= 600 counts range");

    /* L13: a one-off extreme (car lifted: 4000) fades once normal black returns */
    s3_cal_init(&c); s3_cal_learn(&c, 200U); s3_cal_learn(&c, 4000U);
    for (i = 0; i < 30; i++) { s3_cal_learn(&c, 200U); s3_cal_learn(&c, 3000U); s3_cal_relax(&c); }
    check("S3-L13", (c.black < 3100U) && (c.black >= 3000U), "lifted-car extreme fades to real black in 30 s");

    /* L14: a sensor that sees only white keeps its black level (Left sensor) */
    s3_cal_init(&c); s3_cal_learn(&c, 200U); s3_cal_learn(&c, 3000U);
    for (i = 0; i < 120; i++) { s3_cal_learn(&c, 210U); s3_cal_relax(&c); }
    check("S3-L14", 3000U == c.black, "2 min of white only -> black level unchanged");

    /* L15: drift onto the line / hard left curve: Centre black long before Left
            -> NOT a junction, steering stays hard left */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    ev  = feed(&ln, W, B, STEP, 10, &o);      /* 30 mm: Centre black, Left white */
    ev += feed(&ln, B, B, STEP, 10, &o);      /* then Left black too             */
    check("S3-L15", (0 == ev) && (o.steer < -500) && (S3_LN_FOLLOW == o.state),
          "curve/drift (C black 30 mm before L) -> no junction, steer hard left");

    /* L16: overshoot: Left black + Centre white -> steer hard left */
    (void)feed(&ln, B, W, STEP, 10, &o);
    check("S3-L16", (1U == o.overshoot) && (-1000 == o.steer), "overshoot -> steer -1000");

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
    s3_cal_init(&c); s3_cal_learn(&c, 500U); s3_cal_learn(&c, 3999U);
    for (i = 0; i < 60; i++) { s3_cal_learn(&c, 500U); s3_cal_learn(&c, 2500U); s3_cal_relax(&c); }
    check("S3-L20", (c.black < 2600U) && (s3_cal_norm(&c, 2500U) > S3_LN_BLACK),
          "black stuck at 3999 recovers within 60 s; real black reads black again");

    /* L21: coming back from LOST gives no D kick (steer = P only) */
    s3_line_init(&ln, S3_LN_EDGE_LEFT); calibrate(&ln, &o);
    (void)feed(&ln, W, W, STEP, 25, &o);      /* LOST */
    s3_line_step(&ln, W, 1885U, STEP, DT, &o);   /* line found, e ~ +100 */
    check("S3-L21", (S3_LN_FOLLOW == o.state) && (o.steer > -300) && (o.steer < 0),
          "LOST -> FOLLOW: steer from P only (no D kick)");

    /* L22: right-edge mounting (outer sensor on the robot's RIGHT, the real car):
            overshoot steers hard RIGHT, junction still detected */
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    ev = feed(&ln, B, B, STEP, 10, &o);
    ok = (1 == ev);
    s3_line_init(&ln, S3_LN_EDGE_RIGHT); calibrate(&ln, &o);
    (void)feed(&ln, B, W, STEP, 10, &o);
    check("S3-L22", ok && (1U == o.overshoot) && (1000 == o.steer), "right edge: junction 1 event; overshoot -> steer +1000");

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

    /* L26: Change Requirement 1 -- packed sizes */
    printf("  sizeof(s3_cal_t)=%u (was 14)  sizeof(s3_line_t)=%u (was 80)\n",
           (unsigned)sizeof(s3_cal_t), (unsigned)sizeof(s3_line_t));
    check("S3-L26", (8U == sizeof(s3_cal_t)) && (sizeof(s3_line_t) <= 44U), "calibration 14 -> 8 B, line state 80 -> <= 44 B");

    /* N01: barcode letter -> command */
    check("S3-N01", (S3_NAV_LEFT == s3_nav_from_char('A')) && (S3_NAV_RIGHT == s3_nav_from_char('B')) &&
                    (S3_NAV_STRAIGHT == s3_nav_from_char('C')) && (S3_NAV_UTURN == s3_nav_from_char('D')) &&
                    (S3_NAV_NONE == s3_nav_from_char('Z')), "A/B/C/D -> L/R/S/U, others NONE");

    printf("%s: %d failure(s)\n", g_fail ? "FAILED" : "ALL PASSED", g_fail);
    return g_fail;
}
