/*
 *	drv_encoder.h
 *	Subsystem 2 -- wheel encoder edge counter (GP16 left, GP17 right).
 *
 *	Owned by subsystem 2.  Only motion_task (and the S2-U01 test) may read it.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include "../app.h"

#define S2_ENC_LEFT_PIN 16
#define S2_ENC_RIGHT_PIN 17

#define S2_ENC_LEFT 0
#define S2_ENC_RIGHT 1

#define S2_ENC_GLITCH_US 100 /* real edges are at least ~350 us apart at 0.4 m/s */

typedef struct
{
    UW rise;      /* rising edges since the last clear */
    UW fall;      /* falling edges since the last clear */
    UW t_rise_us; /* 1 MHz timer value at the last rising edge */
    UW t_last_us; /* timer value at the last edge of either kind */
    UW period_us; /* time between the last two rising edges */
    UW min_dt_us; /* shortest gap between any two edges (bounce check) */
    UW glitches;  /* gaps under S2_ENC_GLITCH_US since the last clear */
} S2_ENC;

/* Set up GP16/GP17 and the edge interrupt on the core that calls this. */
IMPORT ER s2_enc_init(void);

/* Zero both wheels' counters. */
IMPORT void s2_enc_clear(void);

/* Copy one wheel's counters (S2_ENC_LEFT or S2_ENC_RIGHT). */
IMPORT void s2_enc_get(INT wheel, S2_ENC *out);

#endif /* DRV_ENCODER_H */
