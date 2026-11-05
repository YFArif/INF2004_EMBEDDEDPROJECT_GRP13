/** @file s3_line.h
 *
 * @brief Subsystem 3 -- line following: per-sensor calibration and the
 *        PD controller (pure C, no kernel calls, unit-tested on a laptop:
 *        testing/subsystem3/host/test_line.c, sim_track.c).
 *
 * @par Geometry now: SINGLE EDGE (v30, 5 Nov)
 * GP27 ("Centre IR") sits on the line's LEFT edge with the car centred
 * on the line; GP26 is a guard further out on the right, on white, for
 * junctions and big drifts. s3_line_step() turns these into the
 * edge-pair readings the rules below use (far side = 1000 - edge, or the
 * guard if darker), so every rule below still applies. edge_black: after
 * the edge sensor went DEEP black (S3_LN_EDGE_DEEP), only the guard
 * counts as the far side until it has been grey S3_LN_EDGE_SETTLE_MM or
 * the guard sees the line (sharp inside corners). UNVERIFIED on the car.
 *
 * @par Units
 * Normalised readings are permille, 0 = white, 1000 = black. Steering
 * output is permille too: -1000 = full left, +1000 = full right.
 * Distances inside s3_line_t are SATURATING 16-bit micrometres.
 *
 * @par Rules (each one has a host test in test_line.c)
 * - PD on e = n_RIGHT - n_LEFT (-1000 ... +1000), optional curve boost
 *   once |e| > S3_LN_BOOST_FROM. D is "change per 10 ms", so Kd holds if
 *   the period varies.
 * - Last side: while |e| > S3_LN_SIDE_MIN, remember which side the line
 *   is on (the darker sensor).
 * - Junction: n_LEFT + n_RIGHT > S3_LN_JUNC_SUM -> steer 0 (straight
 *   across); after S3_LN_JUNC_MM one junction_event and state JUNCTION.
 * - OFF LINE / Lost: both white (only after the edge sensor crossed
 *   the line) -> keep the last PD steer (S3_LN_HOLD_MM), then
 *   S3_LN_SOFT_PERMILLE rising to full lock at S3_LN_LOST_MM (state
 *   LOST, one lost_event).
 * - Search (Vehicle Controller): steer 0 until either sensor is above
 *   S3_LN_FOUND, then follow.
 * - Calibration learns on every step; relaxing is the caller's job.
 */

#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdint.h>

/* ---------------------------------------------------------- calibration
 * FR2: each line sensor has its own s3_ir_cal_t. A sensor starts
 * UNcalibrated (white = 4095, black = 0) and must see real white AND real
 * black once (the start-up sweep) before its reading is trusted.
 * It learns the lowest (white) and highest (black) FILTERED value
 * (4-sample IIR, so one glitch cannot set an extreme). About once a
 * second s3_ir_cal_relax() moves each end 1/8 of the way towards what the
 * sensor really saw in that second, but only if it saw something on that
 * side: a one-off extreme fades, an unused end does not decay, and the
 * range never shrinks below S3_IR_CAL_MIN_RANGE. Inputs are 12-bit ADC
 * counts, higher = darker; s3_ir_cal_norm() returns permille, 0 = white,
 * 1000 = black. */
#define S3_IR_CAL_MIN_RANGE (600u)  /* white..black range needed to trust. */
#define S3_IR_CAL_ADC_MAX   (4095u)
#define S3_IR_CAL_LIFT_RAW  (3700u) /* At or above: no floor (car lifted, */
                                    /* about 4050 on 28 Oct), never       */
                                    /* learned. Real tape read 2500-2900, */
                                    /* so a lift made tape read ~60 %     */
                                    /* (grey) and "black" never happened. */

/** One sensor's calibration: 54 bits in 8 bytes (was 14 bytes). */
typedef struct
{
    unsigned int white   : 12;  /* Lowest filtered value.                 */
    unsigned int black   : 12;  /* Highest filtered value.                */
    unsigned int valid   : 1;   /* 1 once black - white >= MIN_RANGE.     */
    unsigned int started : 1;   /* The filter has its first sample.       */
    unsigned int filt    : 12;  /* 4-sample IIR of the raw value.         */
    unsigned int win_lo  : 8;   /* Lowest filtered value since the last   */
                                /* relax, >> 4 (16-count steps are enough */
                                /* for a 1/8 relax step).                 */
    unsigned int win_hi  : 8;   /* Highest, >> 4.                         */
} s3_ir_cal_t;

void     s3_ir_cal_init(s3_ir_cal_t * p_cal);
uint16_t s3_ir_cal_filter(s3_ir_cal_t * p_cal, uint16_t raw);
void     s3_ir_cal_learn(s3_ir_cal_t * p_cal, uint16_t filt);
void     s3_ir_cal_relax(s3_ir_cal_t * p_cal);
int32_t  s3_ir_cal_norm(s3_ir_cal_t const * p_cal, uint16_t value);

/* --------------------------------------------------------------- tuning */
#define S3_LN_BLACK         (750)   /* n > 750 = black (design 6.1).      */
#define S3_LN_WHITE         (250)   /* n < 250 = white.                   */
#define S3_LN_MIN_RANGE     (S3_IR_CAL_MIN_RANGE)
#define S3_LN_LOST_MM       (40u)   /* Both white this far = LOST.        */
#define S3_LN_KP_X100       (60)    /* Default gains: steer =             */
#define S3_LN_KD_X100       (0)     /*   (Kp * e + Kd * de) / 100.        */
                                    /* v29 host sweep (sim_track.c sweep, */
                                    /* 40 mm/s, steer_max 40, lag 80/120  */
                                    /* ms, spot 5/8 mm, noise, 8-bit):    */
                                    /* every Kp 20-100 / Kd 0-200         */
                                    /* finishes every track. Kp <= 50:    */
                                    /* r200 curves need more steer than   */
                                    /* e can give, worst error 18 mm (at  */
                                    /* the edge). Kp 60: 6.3 mm. Kd >= 50 */
                                    /* only added sign flips (1-27 %).    */
                                    /* UNVERIFIED on the car.             */
#define S3_LN_KP_MAX_X100   (2000)
#define S3_LN_KD_MAX_X100   (4000)
#define S3_LN_BOOST_X100    (0)     /* Curve boost: extra steer per       */
                                    /* permille of e beyond BOOST_FROM.   */
                                    /* 0 = off. Settable at run time.     */
#define S3_LN_BOOST_MAX_X100 (600)
#define S3_LN_BOOST_FROM    (500)   /* |e| above this: one sensor is off  */
                                    /* the line (a curve or a drift).     */
#define S3_LN_SIDE_MIN      (60)    /* |e| above this updates the side    */
                                    /* the line was last seen on, also    */
                                    /* while both read "white" (a reading */
                                    /* of 200 / 1000 is the line, not     */
                                    /* noise: white floor was 3-66).      */
#define S3_LN_HOLD_MM       (15u)   /* Both white: keep the last PD steer */
                                    /* this far. Car log 29 Oct: centred, */
                                    /* the line passed BETWEEN the spots  */
                                    /* (hard steer there = weave), and on */
                                    /* curves steer 0 there lost the turn.*/
#define S3_LN_SOFT_PERMILLE (350)   /* ... then this much steer towards   */
                                    /* the remembered side, rising        */
                                    /* linearly to full lock at LOST_MM.  */
#define S3_LN_JUNC_SUM      (1300)  /* n_L + n_R above this = junction  */
                                    /* (plain line: both grey, ~1000;   */
                                    /* car junction 807 + 674 = 1481).  */
#define S3_LN_JUNC_MM       (5u)   /* Both black this far = junction     */
                                    /* (one sample of noise is 0.4 mm at  */
                                    /* 40 mm/s; a crossing line is 18).   */
#define S3_LN_EDGE_FULL     (850)   /* Edge reading on the line's own   */
                                    /* black (car 5 Nov: <= 909) = 1000. */
#define S3_LN_EDGE_DEEP     (950)   /* Edge sensor this dark = it is    */
                                    /* crossing the line (car straights */
                                    /* peak ~850).                      */
#define S3_LN_EDGE_SETTLE_MM (10u)  /* Edge sensor grey this far after  */
                                    /* black = back on its own edge. Out */
                                    /* to white sooner = it crossed the  */
                                    /* line (sharp inside corner): the   */
                                    /* line is on the edge side.         */
#define S3_LN_FOUND         (500)   /* Search: a sensor above this =      */
                                    /* line found.                        */
#define S3_LN_SEARCH_NUM    (5)     /* OFF LINE / LOST: full lock is      */
#define S3_LN_SEARCH_DEN    (4)     /* steer_max x 5/4: inner wheel still */
                                    /* rolls forward at 3/8 of speed.     */
#define S3_LN_DV_LIMIT      (511)   /* 10-bit signed steer_sp field.      */
#define S3_LN_D_LIMIT       (600)   /* The D term alone never exceeds it. */
#define S3_LN_REF_DT_US     (10000u)/* Gains are defined for 10 ms.       */
#define S3_LN_STEER_MAX     (1000)  /* Full lock, permille.               */

/* Side of the robot the "Left IR" channel (GP26, raw_l) is mounted on.
 * The "Centre IR" channel (GP27, raw_c) is on the other side. */
#define S3_LN_L_ON_LEFT     (1)
#define S3_LN_L_ON_RIGHT    (-1)

/** Line state: 5 values -> 3 bits (Change Requirement 1). */
typedef enum
{
    S3_LN_FOLLOW = 0,   /* On the line, PD steering valid.               */
    S3_LN_JUNCTION,     /* Both black for S3_LN_JUNC_MM: steer 0.        */
    S3_LN_LOST,         /* Both white for S3_LN_LOST_MM.                 */
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
    s3_ir_cal_t  cal_l;         /* "Left IR" channel, GP26.              */
    s3_ir_cal_t  cal_c;         /* "Centre IR" channel, GP27.            */
    int16_t      kp_x100;       /* 0 ... 2000.                           */
    int16_t      kd_x100;       /* 0 ... 4000.                           */
    int16_t      e_prev;        /* -1000 ... +1000 permille.             */
    int16_t      boost_x100;    /* Curve boost, 0 ... 600 (0 = off).     */
    int16_t      steer_prev;    /* Last PD steering, permille.           */
    uint16_t     both_white_um; /* Distance with both sensors white.     */
    uint16_t     both_black_um; /* Distance with both sensors black.     */
    uint16_t     edge_grey_um;  /* Edge sensor out of black this far.    */
    unsigned int state        : 3;  /* s3_ln_state_t.                    */
    unsigned int mode         : 2;  /* s3_ln_mode_t.                     */
    unsigned int req_mode     : 2;  /* Last mode REQUESTED by the VC.    */
    unsigned int l_right      : 1;  /* 1 = GP26 on the robot's RIGHT.    */
    unsigned int e_prev_valid : 1;
    unsigned int last_right   : 1;  /* Line last seen: 1 = to the right. */
    unsigned int seen_black   : 1;  /* Black since the line changed side.*/
    unsigned int edge_black   : 1;  /* Edge sensor black, not settled.   */
} s3_line_t;

/** Result of one step. */
typedef struct
{
    int32_t       n_l;              /* Normalised GP26, permille.        */
    int32_t       n_c;              /* Normalised GP27, permille.        */
    int32_t       pos;              /* Line position: e / 2, -500 ...    */
                                    /* +500, + = line to the right.      */
    int32_t       steer;            /* -1000 ... +1000.                  */
    s3_ln_state_t state;
    uint8_t       junction_event;   /* 1 on the step a junction starts.  */
    uint8_t       lost_event;       /* 1 on the step the line is lost.   */
    uint8_t       overshoot;        /* 1 while one sensor is white.      */
    uint8_t       calibrated;       /* Both sensors have a trusted range. */
    uint8_t       line_found_event; /* 1 on the step a search ends.      */
    uint8_t       off_line;         /* 1 while both white (OFF LINE).    */
} s3_line_out_t;

void s3_line_init(s3_line_t * p_ln, int32_t l_side);
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
