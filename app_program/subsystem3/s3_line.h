/** @file s3_line.h
 *
 * @brief Subsystem 3 -- line-following combiner: uses the Left IR and the
 *        Centre sensor together for PD steering, junction, line lost and
 *        search (pure C, no kernel calls, unit-tested on a laptop:
 *        testing/subsystem3/host/test_line.c).
 *
 * @par Geometry: STRADDLE (v19, 28 Oct; design report 6.1, A19)
 *
 *          outer side ->
 *     C |##########| O          line 18 mm wide
 *     ^ Centre on the edge AWAY from the outer sensor
 *                    ^ outer sensor just OUTSIDE the other edge
 *
 * - The Centre rides one edge, so half black (n_C = 500 permille) is "on
 *   track". On our car the outer sensor ("L", GP26) is on the robot's
 *   RIGHT, so the Centre rides the LEFT edge, as drawn above
 *   (S3_EDGE_SIDE in s3_config.h).
 * - The outer sensor is a real steering input: it turns black when the
 *   car drifts towards the outer side, which the Centre alone cannot
 *   tell from "line gone".
 * - Until v18 the Centre rode the edge NEXT TO the outer sensor, the
 *   outer sensor was almost always white, and "Centre white" could not
 *   tell "drifted off" from "line turned away" (right turns failed).
 * - Mounting: Centre to outer = line width + 2 ... 6 mm (20 ... 24 mm;
 *   20 worked best on the car, 28 Oct), same height. Less than 18 mm and
 *   the outer sensor sits on the line.
 * - LIMIT: a junction branch is seen only if it reaches the outer
 *   sensor (both black); a short stub on the Centre's side is not.
 *
 * @par Units
 * Normalised readings are permille, 0 = white, 1000 = black. Steering
 * output is permille too: -1000 = full left, +1000 = full right.
 * Distances inside s3_line_t are SATURATING 16-bit micrometres: every
 * threshold is <= 40 mm, so 65 535 um is enough.
 *
 * @par Rules (each one has a host test in test_line.c)
 * - PD on e = n_C - 500 (optional curve boost outside the EDGE band, off
 *   by default). D is "change per 10 ms", so Kd holds if the period
 *   varies.
 * - OUTER: outer sensor darker than WHITE (and the Centre not black) ->
 *   the car is drifting towards the outer side: steer towards the outer
 *   side, at least S3_LN_OUTER_STEER * (n_L - WHITE) / (1000 - WHITE),
 *   so the push grows with how far the outer sensor is on the line
 *   (v20; v19 jumped to a fixed 600 once it was black).
 * - OFF LINE: both white -> full lock towards where the line must be,
 *   decided ONCE when both go white and held until a sensor sees the line
 *   again (the 16-bit distances saturate at 65 mm):
 *   If the OUTER sensor saw the line within the last 65 mm:
 *   1. the sensor that saw it LAST: the outer sensor -> the outer side;
 *      the Centre -> the Centre's side (a turn towards the Centre's side
 *      makes the outer sensor leave first);
 *   2. a tie (both left within TIE_MM: they crossed it together, a sharp
 *      turn towards the outer side) -> the outer side.
 *   Otherwise (only the Centre has seen it):
 *   3. whether the Centre went DARK
 *      (>= SIDE_DARK) since it last sat SIDE_SETTLE_MM on the edge: DARK
 *      (it crossed the whole line) -> the Centre's side; LIGHT (it slid
 *      off its own edge) -> the outer side.
 * - Junction: L and C both black for S3_LN_JUNC_MM, L black no later than
 *   S3_LN_JUNC_LEAD_MM after C, and not while C is white.
 * - Lost: L and C both white for S3_LN_LOST_MM. The car keeps turning in
 *   the OFF LINE direction until the Centre reaches the CORRECT edge
 *   (coming from the far side it must cross the line first), as in the
 *   Vehicle Controller's search.
 * - Calibration learns on every step; relaxing is the caller's job.
 */

#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdint.h>

#include "s3_ir_cal.h"

/* --------------------------------------------------------------- tuning */
#define S3_LN_BLACK         (750)   /* n > 750 = black (design 6.1).      */
#define S3_LN_WHITE         (250)   /* n < 250 = white.                   */
#define S3_LN_EDGE_MID      (500)   /* Centre on the edge.                */
#define S3_LN_MIN_RANGE     (S3_IR_CAL_MIN_RANGE)
#define S3_LN_JUNC_MM       (8u)    /* L and C both black this far.       */
#define S3_LN_JUNC_LEAD_MM  (15u)   /* C black at most this far before L  */
                                    /* (else it is drift or a curve).     */
#define S3_LN_REARM_MM      (30u)   /* Junction re-armed after this much  */
                                    /* non-junction travel.               */
#define S3_LN_LOST_MM       (40u)   /* L and C both white this far.       */
#define S3_LN_KP_X100       (250)   /* Default gains: steer =             */
#define S3_LN_KD_X100       (50)    /*   -(Kp * e + Kd * de) / 100.       */
                                    /* v20: was 150 / 400. Car log 28 Oct */
                                    /* (20 mm): P too weak in the band    */
                                    /* (mean |dv| 20 mm/s), D made 27 %   */
                                    /* of steps flip sign.                */
#define S3_LN_KP_MAX_X100   (2000)
#define S3_LN_KD_MAX_X100   (4000)
#define S3_LN_BOOST_X100    (0)     /* Curve boost: extra steer per       */
                                    /* permille outside the EDGE band.    */
                                    /* 0 = v15b steering (the version     */
                                    /* that tracked well on the car);     */
                                    /* 300 (v15c ... v17) gave full lock  */
                                    /* outside 250-750 and the car spun   */
                                    /* (28 Oct). Settable at run time.    */
#define S3_LN_BOOST_MAX_X100 (600)
#define S3_LN_SEARCH_NUM    (5)     /* OFF LINE / LOST: full lock is      */
#define S3_LN_SEARCH_DEN    (4)     /* steer_max x 5/4 (v20; v19 used 2 = */
                                    /* pivot, which swept past the line   */
                                    /* on the car). Inner wheel still     */
                                    /* rolls forward at 3/8 of speed.     */
#define S3_LN_DV_LIMIT      (511)   /* 10-bit signed steer_sp field.      */
#define S3_LN_OUTER_STEER   (600)   /* OUTER rule: steer towards the      */
                                    /* outer side of at least this x the  */
                                    /* outer sensor's darkness above      */
                                    /* WHITE (600 at fully black, 400 at  */
                                    /* BLACK, 0 at WHITE).                */
#define S3_LN_SEEN_C        (S3_LN_WHITE)   /* Centre "sees the line"     */
                                    /* when darker than WHITE.            */
#define S3_LN_TIE_MM        (3u)    /* OFF LINE: both sensors left the    */
                                    /* line within this distance of each  */
                                    /* other (they crossed it together,   */
                                    /* a sharp turn towards the outer     */
                                    /* side) -> the outer side. A turn    */
                                    /* towards the Centre's side makes    */
                                    /* the outer sensor leave first.      */
#define S3_LN_SIDE_DARK     (600)   /* Centre at or above this: DARK side.*/
#define S3_LN_SIDE_SETTLE_MM (5u)   /* Back to LIGHT only after this much */
                                    /* travel settled on the edge (an     */
                                    /* exit through the line's far edge   */
                                    /* passes light values in 1 ... 3 mm  */
                                    /* and must stay DARK).               */
#define S3_LN_D_LIMIT       (600)   /* The D term alone never exceeds it. */
#define S3_LN_REF_DT_US     (10000u)/* Gains are defined for 10 ms.       */
#define S3_LN_STEER_MAX     (1000)  /* Full lock, permille.               */
#define S3_LN_FOUND_DIRECT  (400)   /* Search: C above this = found.      */
#define S3_LN_FOUND_CROSSED (600)   /* Search from the far side: C black  */
                                    /* once, then below this = found.     */

/* Which edge the Centre sensor follows: +1 = left edge of the line (line
 * to the sensor's right), -1 = right edge. The outer sensor is always on
 * the OTHER side, just outside the line (straddle). */
#define S3_LN_EDGE_LEFT     (1)
#define S3_LN_EDGE_RIGHT    (-1)

/** Line state: 5 values -> 3 bits (Change Requirement 1). */
typedef enum
{
    S3_LN_FOLLOW = 0,   /* On the edge, PD steering valid.               */
    S3_LN_JUNCTION,     /* Junction under the car: steer 0.              */
    S3_LN_LOST,         /* L and C both white for S3_LN_LOST_MM.         */
    S3_LN_DISABLED,     /* LF_ENABLE clear (turn / bypass): steer 0.     */
    S3_LN_SEARCH        /* Reacquiring after a bypass: steer 0.          */
} s3_ln_state_t;

/** What the Vehicle Controller asks for (LF_ENABLE / LF_SEARCH_x).
 *  SEARCH_FROM_LEFT = the car is on the LEFT of the line, coming back. */
typedef enum
{
    S3_LN_MODE_FOLLOW = 0,
    S3_LN_MODE_DISABLED,
    S3_LN_MODE_SEARCH_FROM_LEFT,
    S3_LN_MODE_SEARCH_FROM_RIGHT
} s3_ln_mode_t;

/** Working state of the line task (one owner, never shared). */
typedef struct
{
    s3_ir_cal_t  cal_l;         /* Left IR (outer sensor).               */
    s3_ir_cal_t  cal_c;         /* Centre sensor.                        */
    int16_t      kp_x100;       /* 0 ... 2000.                           */
    int16_t      kd_x100;       /* 0 ... 4000.                           */
    int16_t      e_prev;        /* -500 ... +500 permille.               */
    int16_t      last_steer;    /* -1000 ... +1000.                      */
    uint16_t     both_black_um; /* Distance with L and C both black.     */
    uint16_t     c_black_um;    /* Distance with Centre black.           */
    uint16_t     c_black_at_l;  /* c_black_um when L turned black;       */
                                /* 0xFFFF = Centre was white.            */
    uint16_t     not_junc_um;   /* Distance since the last junction.     */
    uint16_t     both_white_um; /* Distance with L and C both white.     */
    int16_t      boost_x100;    /* Curve boost, 0 ... 600 (0 = off).     */
    uint16_t     since_o_um;    /* Distance since the outer was black.   */
    uint16_t     since_c_um;    /* Distance since the Centre saw line.   */
    uint16_t     c_band_um;     /* Distance settled on the edge (Centre  */
                                /* between SEEN_C and SIDE_DARK).        */
    unsigned int state        : 3;  /* s3_ln_state_t.                    */
    unsigned int mode         : 2;  /* s3_ln_mode_t.                     */
    unsigned int edge_right   : 1;  /* 1 = Centre rides the RIGHT edge.  */
    unsigned int e_prev_valid : 1;
    unsigned int l_was_black  : 1;
    unsigned int junc_armed   : 1;
    unsigned int seen_black   : 1;  /* Search: Centre crossed the line.  */
    unsigned int req_mode     : 2;  /* Last mode REQUESTED by the VC.    */
    unsigned int c_dark_side  : 1;  /* Centre's last side: 1 = DARK.     */
    unsigned int off_latched  : 1;  /* OFF LINE direction decided.       */
    unsigned int off_right    : 1;  /* ... and it is: 1 = right.         */
    unsigned int recovering   : 1;  /* LOST: turning back to the line.   */
} s3_line_t;

/** Result of one step. */
typedef struct
{
    int32_t       n_l;              /* Normalised Left IR, permille.     */
    int32_t       n_c;              /* Normalised Centre, permille.      */
    int32_t       pos;              /* Line position: -500 ... +500.     */
    int32_t       steer;            /* -1000 ... +1000.                  */
    s3_ln_state_t state;
    uint8_t       junction_event;   /* 1 on the step a junction starts.  */
    uint8_t       lost_event;       /* 1 on the step the line is lost.   */
    uint8_t       overshoot;        /* 1 while the OUTER rule steers.    */
    uint8_t       calibrated;       /* Both sensors have a trusted range. */
    uint8_t       line_found_event; /* 1 on the step a search ends.      */
    uint8_t       off_line;         /* 1 while both white (OFF LINE).    */
} s3_line_out_t;

void s3_line_init(s3_line_t * p_ln, int32_t edge_side);
void s3_line_set_gains(s3_line_t * p_ln, int32_t kp_x100, int32_t kd_x100);
void s3_line_set_shape(s3_line_t * p_ln, int32_t boost_x100);
int32_t s3_line_dv(s3_line_out_t const * p_out, int32_t steer_max_mms);
void s3_line_recalibrate(s3_line_t * p_ln);
void s3_line_set_mode(s3_line_t * p_ln, s3_ln_mode_t mode);
void s3_line_request_mode(s3_line_t * p_ln, s3_ln_mode_t mode);
void s3_line_step(s3_line_t * p_ln, uint16_t raw_l, uint16_t raw_c,
                  uint32_t moved_um, uint32_t dt_us, s3_line_out_t * p_out);

#endif /* S3_LINE_H */

/*** end of file ***/
