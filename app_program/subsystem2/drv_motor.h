/*
 *	drv_motor.h
 *	Subsystem 2 -- DC motor driver for the Robo Pico.
 *	Left motor M1 on GP8/GP9 (PWM slice 4), right motor M2 on GP10/GP11 (slice 5), 20 kHz.
 *
 *	Robo Pico truth table (inputs A/B): high/low = forward, low/high = backward,
 *	low/low = brake, high/high = coast.  Only motion_task and the S2 tests call this.
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include "../app.h"

#define S2_MOTOR_LEFT 0
#define S2_MOTOR_RIGHT 1

/* Set to 1 if that wheel turns backward for a positive duty (mirrored mounting). */
#define S2_MOTOR_INVERT_L 0
#define S2_MOTOR_INVERT_R 0

#define S2_DUTY_MAX 1000 /* permille */

/* Both motors braked, 20 kHz PWM running. */
IMPORT ER s2_motor_init(void);

/* duty -1000..+1000 permille, positive = forward, 0 = brake. */
IMPORT void s2_motor_set(INT motor, INT duty);

IMPORT void s2_motor_brake(void); /* both motors */
IMPORT void s2_motor_coast(void); /* both motors, no braking */

#endif /* DRV_MOTOR_H */
