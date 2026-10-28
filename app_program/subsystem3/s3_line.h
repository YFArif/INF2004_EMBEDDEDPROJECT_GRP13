/** @file s3_line.h
 *
 * @brief Subsystem 3 -- line following: per-sensor calibration and the
 *        PD controller that keeps BOTH line sensors on the black line
 *        (pure C, no kernel calls, unit-tested on a laptop:
 *        testing/subsystem3/host/test_line.c, sim_track.c).
 *
 * @par Geometry: CENTRED PAIR (v25, 28 Oct; replaces the v19 straddle)
 *
 *              <-- 12 mm -->
 *             L            R        both sensor centres ON the line
 *       white |####################| white      line 18 mm wide
 *
 * - Both sensors sit over the line, 12 mm apart centre to centre, the
 *   pair centred on the car and on the line. On the line both read
 *   black (about 1000 permille).
 * - When the car drifts, the sensor on the drifting side reaches the
 *   edge first and turns grey, then white; the other stays black. The
 *   difference e = n_RIGHT - n_LEFT says where the line is: e > 0, the
 *   right sensor is darker, the line is to the right -> steer right.
 * - The rule is the same for left and right, so left and right corners
 *   behave the same, and there is no "which edge am I on" to get wrong
 *   (the v19 ... v22 straddle could, and then spun).
 * - Which ADC channel is on which side: s3_line_init() l_side (the "Left
 *   IR" channel GP26 on the robot's LEFT or RIGHT; S3_IR_L_SIDE in
 *   s3_config.h). The other channel (GP27, "Centre IR") is on the other
 *   side.
 * - Spacing: closer than about 6 mm and both stay black until the car is
 *   well off the line (late, hard corrections); wider than about 16 mm
 *   and a sensor sits near the edge all the time (noisy). 12 mm leaves
 *   about 3 mm of black between each sensor and its edge.
 * - LIMIT: both sensors are black on every part of the line, so a
 *   junction looks like plain line. This layout does NOT detect
 *   junctions (junction_event is always 0; FR3 junctions need another
 *   method, e.g. the barcode before each junction).
 *
 * @par Units
 * Normalised readings are permille, 0 = white, 1000 = black. Steering
 * output is permille too: -1000 = full left, +1000 = full right.
 * Distances inside s3_line_t are SATURATING 16-bit micrometres.
 *
 * @par Rules (each one has a host test in test_line.c)
 * - PD on e = n_RIGHT - n_LEFT (-1000 ... +1000), optional curve boost
 *   once |e| > S3_LN_BOOST_FROM (one sensor off the line). D is "change
 *   per 10 ms", so Kd holds if the period varies.
 * - Last side: while |e| > S3_LN_SIDE_MIN, remember which side the line
 *   is on (the darker sensor).
 * - OFF LINE: both white -> full lock towards the last side, until
 *   either sensor is darker than WHITE again (then PD).
 * - Lost: both white for S3_LN_LOST_MM -> one lost_event, state LOST,
 *   still full lock towards the last side until a sensor sees the line.
 * - Search (Vehicle Controller): steer 0 until either sensor is above
 *   S3_LN_FOUND, then follow. SEARCH_FROM_LEFT (car left of the line)
 *   presets the last side to RIGHT, FROM_RIGHT to LEFT.
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
#define S3_LN_KP_X100       (100)   /* Default gains: steer =             */
#define S3_LN_KD_X100       (0)     /*   (Kp * e + Kd * de) / 100.        */
                                    /* v25 host sweep (sim_track.c sweep, */
                                    /* lag 80 ms, noise, 8-bit): every Kp */
                                    /* 50-300 / Kd 0-200 finishes every   */
                                    /* track; Kp 100 / Kd 0: worst curve  */
                                    /* error 5.7 mm, 2 % sign flips. D    */
                                    /* only added flips. UNVERIFIED on    */
                                    /* the car.                           */
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
#define S3_LN_HOLD_MM       (8u)    /* Both white: steer 0 this far, so a */
                                    /* sensor pair straddling the line,   */
                                    /* or a one-sample dropout, does not  */
                                    /* throw the car into full lock (log  */
                                    /* 28 Oct: every WW sample was +-50). */
#define S3_LN_SOFT_PERMILLE (350)   /* ... then this much steer towards   */
                                    /* the remembered side, rising        */
                                    /* linearly to full lock at LOST_MM.  */
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
    S3_LN_JUNCTION,     /* Not used by the centred pair (kept for the    */
                        /* IF-07 state field).                           */
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
    uint16_t     both_white_um; /* Distance with both sensors white.     */
    unsigned int state        : 3;  /* s3_ln_state_t.                    */
    unsigned int mode         : 2;  /* s3_ln_mode_t.                     */
    unsigned int req_mode     : 2;  /* Last mode REQUESTED by the VC.    */
    unsigned int l_right      : 1;  /* 1 = GP26 on the robot's RIGHT.    */
    unsigned int e_prev_valid : 1;
    unsigned int last_right   : 1;  /* Line last seen: 1 = to the right. */
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
    uint8_t       junction_event;   /* Always 0 (see LIMIT above).       */
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
