/*
 *	s3_line.h
 *	Subsystem 3 -- line sensing logic (pure C, no kernel calls,
 *	unit-tested on a laptop: testing/subsystem3/host/test_line.c).
 *
 *	Covers the write-up tasks "sensor calibration", "line position
 *	estimation", "line-following algorithm" and "junction detection".
 *
 *	Geometry (design report 6.1, assumption A19):
 *	  - Centre sensor rides ONE EDGE of the 18 mm line, so half black /
 *	    half white is "on track" (n_C = 500 permille).
 *	  - Left sensor sits further out on the white side of that edge.
 *	    It sees black only at a junction from that side, or when the car
 *	    has drifted right past the line (overshoot).
 *	  - LIMIT: a branch on the OTHER side of the line is not seen as a
 *	    junction.
 *
 *	Units: normalised readings are permille, 0 = white, 1000 = black.
 *	Steering output is permille too: -1000 = full left, +1000 = full right.
 *
 *	v12 changes (code review 20 Oct): D term scaled by the real period and
 *	reset after calibration / lost / junction; junction only when Left goes
 *	black before (or with) Centre, so a hard curve or drift is not a
 *	junction; overshoot rule (Left black + Centre white -> steer hard back);
 *	calibration (only) learns from a 4-sample filtered value and relaxes towards
 *	what the sensor recently saw, never towards nothing; gains settable at
 *	run time.
 */
#ifndef S3_LINE_H
#define S3_LINE_H

#include <stdint.h>

/* ---------------------------------------------------------------- tuning */
#define S3_LN_BLACK          750    /* n > 750  = black (design 6.1)                 */
#define S3_LN_WHITE          250    /* n < 250  = white                              */
#define S3_LN_MIN_RANGE      600U   /* need white..black range >= this to trust      */
#define S3_LN_JUNC_MM        8U     /* L and C both black this far = junction        */
#define S3_LN_JUNC_LEAD_MM   15U    /* Centre may be black at most this far before
                                       Left turns black (else it is drift/curve)     */
#define S3_LN_REARM_MM       30U    /* junction re-armed after this much non-junction */
#define S3_LN_LOST_MM        40U    /* L and C both white this far = line lost       */
#define S3_LN_KP_X100        150    /* default gains: steer = -(Kp*e + Kd*de) / 100  */
#define S3_LN_KD_X100        400    /* de is per 10 ms, whatever the real period     */
#define S3_LN_BOOST_X100     300    /* extra steer per permille outside the EDGE band (curves) */
#define S3_LN_D_LIMIT        600    /* the D term alone never exceeds this           */
#define S3_LN_REF_DT_US      10000U /* gains are defined for a 10 ms step            */

/* Which edge the Centre sensor follows: +1 = left edge of the line
   (line is to the sensor's right), -1 = right edge. */
#define S3_LN_EDGE_LEFT      1
#define S3_LN_EDGE_RIGHT     (-1)

typedef enum               /* 5 states -> 3 bits (Change Requirement 1) */
{
    S3_LN_FOLLOW = 0,      /* on the edge, PD steering valid               */
    S3_LN_JUNCTION,        /* junction under the car: steer 0              */
    S3_LN_LOST,            /* L and C both white for S3_LN_LOST_MM         */
    S3_LN_DISABLED,        /* LF_ENABLE clear (turn/bypass): steer 0, no events */
    S3_LN_SEARCH           /* reacquiring after a bypass: steer 0, wait for line */
} s3_ln_state_t;

/* What the Vehicle Controller asks for (LF_ENABLE / LF_SEARCH_L / LF_SEARCH_R).
   SEARCH_FROM_LEFT = the car is on the LEFT of the line, coming back to it. */
typedef enum
{
    S3_LN_MODE_FOLLOW = 0,
    S3_LN_MODE_DISABLED,
    S3_LN_MODE_SEARCH_FROM_LEFT,
    S3_LN_MODE_SEARCH_FROM_RIGHT
} s3_ln_mode_t;

/* One sensor's calibration.  Learns the lowest (white) and highest (black)
   FILTERED value; once a second it relaxes each end towards what the sensor
   actually saw in that second (only if it saw something white-ish/black-ish),
   so a one-off extreme fades and an unused end does not decay. */
typedef struct            /* 54 bits in 8 bytes (was 14 bytes): ADC values are 12-bit */
{
    unsigned int white   : 12;
    unsigned int black   : 12;
    unsigned int valid   : 1;   /* 1 once black - white >= S3_LN_MIN_RANGE */
    unsigned int started : 1;
    unsigned int filt    : 12;  /* 4-sample IIR of the raw value           */
    unsigned int win_lo  : 8;   /* lowest filtered value since last relax, >> 4
                                   (relax moves 1/8 of the gap: 16-count steps are enough) */
    unsigned int win_hi  : 8;   /* highest filtered value since last relax, >> 4 */
} s3_cal_t;

/* Working state of the line task (one owner, never shared).  Distances are
   SATURATING 16-bit micrometres: every threshold is <= 40 mm = 40 000 um, so
   65 535 um is enough and "more than 65 mm" never changes a decision. */
typedef struct
{
    s3_cal_t       cal_l;         /* "L" = the OUTER sensor (on the robot's right on the real car) */
    s3_cal_t       cal_c;
    int16_t        kp_x100;       /* 0..2000  */
    int16_t        kd_x100;       /* 0..4000  */
    int16_t        e_prev;        /* -500..+500 permille       */
    int16_t        last_steer;    /* -1000..+1000              */
    uint16_t       both_black_um; /* distance with L and C both black    */
    uint16_t       c_black_um;    /* distance with Centre black          */
    uint16_t       c_black_at_l;  /* c_black_um when L turned black; 0xFFFF = Centre was white */
    uint16_t       not_junc_um;   /* distance since the last junction    */
    uint16_t       both_white_um; /* distance with L and C both white    */
    unsigned int   state        : 3;  /* s3_ln_state_t */
    unsigned int   mode         : 2;  /* s3_ln_mode_t  */
    unsigned int   edge_right   : 1;  /* 1 = Centre follows the RIGHT edge */
    unsigned int   e_prev_valid : 1;
    unsigned int   l_was_black  : 1;
    unsigned int   junc_armed   : 1;
    unsigned int   seen_black   : 1;  /* search: Centre has crossed the line */
    unsigned int   req_mode     : 2;  /* last mode REQUESTED by the Vehicle Controller */
} s3_line_t;

typedef struct
{
    int32_t        n_l;           /* normalised Left, permille           */
    int32_t        n_c;           /* normalised Centre, permille         */
    int32_t        pos;           /* line position estimate: -500..+500  */
    int32_t        steer;         /* -1000..+1000                         */
    s3_ln_state_t  state;
    uint8_t        junction_event;/* 1 on the step a junction is entered */
    uint8_t        lost_event;    /* 1 on the step the line is lost      */
    uint8_t        overshoot;     /* 1 while Left black + Centre white   */
    uint8_t        calibrated;    /* both sensors have a trusted range   */
    uint8_t        line_found_event; /* 1 on the step a search finds the line */
} s3_line_out_t;

/* Starts UNcalibrated (white=4095, black=0): the sensor must see real
   white AND real black once (start-up sweep) before n is trusted. */
void     s3_cal_init(s3_cal_t * p_cal);
uint16_t s3_cal_filter(s3_cal_t * p_cal, uint16_t raw);  /* returns filtered */
void     s3_cal_learn(s3_cal_t * p_cal, uint16_t filt);
void     s3_cal_relax(s3_cal_t * p_cal);                 /* call about 1 / s */
int32_t  s3_cal_norm(s3_cal_t const * p_cal, uint16_t value);

void     s3_line_init(s3_line_t * p_ln, int32_t edge_side);
void     s3_line_set_gains(s3_line_t * p_ln, int32_t kp_x100, int32_t kd_x100);
void     s3_line_recalibrate(s3_line_t * p_ln);           /* forget both ranges */
void     s3_line_set_mode(s3_line_t * p_ln, s3_ln_mode_t mode);
/* Call every step with the mode the flags ask for.  Acts only when the
   REQUEST changes, so a search that has already found the line (internal
   mode FOLLOW) is not restarted while LF_SEARCH_x is still set. */
void     s3_line_request_mode(s3_line_t * p_ln, s3_ln_mode_t mode);

/* One control step.  raw_l / raw_c: ADC counts (higher = darker).
   moved_um: distance since the previous step in MICROMETRES (odometry;
   0 while stopped, so a parked car never raises junction/lost -- only true
   with real odometry; the bench fallback assumes the car always moves).
   dt_us: real time since the previous step (scales the D term). */
void     s3_line_step(s3_line_t * p_ln, uint16_t raw_l, uint16_t raw_c,
                      uint32_t moved_um, uint32_t dt_us, s3_line_out_t * p_out);

#endif /* S3_LINE_H */
