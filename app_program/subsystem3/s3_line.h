/** @file s3_line.h
 *
 * @brief Subsystem 3 -- line-following combiner: uses the Left IR and the
 *        Centre sensor together for PD steering, junction, line lost and
 *        search (pure C, no kernel calls, unit-tested on a laptop:
 *        testing/subsystem3/host/test_line.c).
 *
 * @par Geometry (design report 6.1, assumption A19)
 * - The Centre sensor rides ONE EDGE of the 18 mm line, so half black /
 *   half white (n_C = 500 permille) is "on track".
 * - The Left IR ("L", the outer sensor) sits further out, beyond that
 *   edge. It sees black only at a junction on that side, or when the car
 *   has drifted past the line (overshoot). On our car it is mounted on
 *   the robot's RIGHT, so the Centre rides the RIGHT edge.
 * - LIMIT: a branch on the OTHER side of the line is not seen as a
 *   junction.
 *
 * @par Units
 * Normalised readings are permille, 0 = white, 1000 = black. Steering
 * output is permille too: -1000 = full left, +1000 = full right.
 * Distances inside s3_line_t are SATURATING 16-bit micrometres: every
 * threshold is <= 40 mm, so 65 535 um is enough.
 *
 * @par Rules (each one has a host test, L01 ... L28)
 * - PD on e = n_C - 500, with a curve boost outside the EDGE band. D is
 *   "change per 10 ms", so Kd holds if the period varies.
 * - Overshoot: L black + Centre white -> full lock back.
 * - Junction: L and C both black for S3_LN_JUNC_MM, L black no later than
 *   S3_LN_JUNC_LEAD_MM after C, and not while C is white.
 * - Lost: L and C both white for S3_LN_LOST_MM; steer keeps the last
 *   correction.
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
#define S3_LN_KP_X100       (150)   /* Default gains: steer =             */
#define S3_LN_KD_X100       (400)   /*   -(Kp * e + Kd * de) / 100.       */
#define S3_LN_KP_MAX_X100   (2000)
#define S3_LN_KD_MAX_X100   (4000)
#define S3_LN_BOOST_X100    (300)   /* Extra steer per permille outside   */
                                    /* the EDGE band (curves).            */
#define S3_LN_D_LIMIT       (600)   /* The D term alone never exceeds it. */
#define S3_LN_REF_DT_US     (10000u)/* Gains are defined for 10 ms.       */
#define S3_LN_STEER_MAX     (1000)  /* Full lock, permille.               */
#define S3_LN_FOUND_DIRECT  (400)   /* Search: C above this = found.      */
#define S3_LN_FOUND_CROSSED (600)   /* Search from the far side: C black  */
                                    /* once, then below this = found.     */

/* Which edge the Centre sensor follows: +1 = left edge of the line (line
 * to the sensor's right), -1 = right edge. */
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
    unsigned int state        : 3;  /* s3_ln_state_t.                    */
    unsigned int mode         : 2;  /* s3_ln_mode_t.                     */
    unsigned int edge_right   : 1;  /* 1 = Centre rides the RIGHT edge.  */
    unsigned int e_prev_valid : 1;
    unsigned int l_was_black  : 1;
    unsigned int junc_armed   : 1;
    unsigned int seen_black   : 1;  /* Search: Centre crossed the line.  */
    unsigned int req_mode     : 2;  /* Last mode REQUESTED by the VC.    */
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
    uint8_t       overshoot;        /* 1 while L black + Centre white.   */
    uint8_t       calibrated;       /* Both sensors have a trusted range. */
    uint8_t       line_found_event; /* 1 on the step a search ends.      */
} s3_line_out_t;

void s3_line_init(s3_line_t * p_ln, int32_t edge_side);
void s3_line_set_gains(s3_line_t * p_ln, int32_t kp_x100, int32_t kd_x100);
void s3_line_recalibrate(s3_line_t * p_ln);
void s3_line_set_mode(s3_line_t * p_ln, s3_ln_mode_t mode);
void s3_line_request_mode(s3_line_t * p_ln, s3_ln_mode_t mode);
void s3_line_step(s3_line_t * p_ln, uint16_t raw_l, uint16_t raw_c,
                  uint32_t moved_um, uint32_t dt_us, s3_line_out_t * p_out);

#endif /* S3_LINE_H */

/*** end of file ***/
