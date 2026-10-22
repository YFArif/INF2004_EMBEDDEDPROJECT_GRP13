/** @file drv_motor.h
 *
 * @brief Subsystem 2 -- DC motor driver for the Cytron Robo Pico.
 *
 * @par
 * Left motor M1 on GP8 (M1A) / GP9 (M1B), PWM slice 4. Right motor M2 on
 * GP10 (M2A) / GP11 (M2B), PWM slice 5. Both run at 20 kHz, the driver's
 * maximum PWM frequency.
 *
 * @par Robo Pico truth table (inputs A / B)
 * high / low = forward, low / high = backward, low / low = brake,
 * high / high = coast.
 *
 * @par
 * Only motion_task, and the S2 bench tests while motion_task is idle, call
 * this driver (TEAM.md section 3: one owning task per peripheral).
 */

#ifndef DRV_MOTOR_H
#define DRV_MOTOR_H

#include <stdint.h>

/** The two motor channels. */
typedef enum
{
    DRV_MOTOR_LEFT = 0,     /* M1, GP8 / GP9.                            */
    DRV_MOTOR_RIGHT,        /* M2, GP10 / GP11.                          */
    DRV_MOTOR_COUNT
} drv_motor_t;

#define DRV_MOTOR_DUTY_MAX      (1000)  /* Permille, in either direction. */

/* Set to 1 if that wheel turns backward for a positive duty (S2-U06). */
#define DRV_MOTOR_INVERT_LEFT   (0)
#define DRV_MOTOR_INVERT_RIGHT  (0)

void drv_motor_init(void);
void drv_motor_set(drv_motor_t motor, int32_t duty_permille);
void drv_motor_brake(void);
void drv_motor_coast(void);

#endif /* DRV_MOTOR_H */

/*** end of file ***/
