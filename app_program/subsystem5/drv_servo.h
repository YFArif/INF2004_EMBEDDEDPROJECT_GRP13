/** @file drv_servo.h
 *
 * @brief SG90 servo that turns the ultrasonic sensor: Robo Pico servo
 *        port 1, GP12, PWM slice 6 channel A at 50 Hz.
 *
 * @par Angles
 * 90 degrees looks straight ahead, 0 degrees to the robot's right and
 * 180 degrees to its left (Week 6 report 6.4, checked in S5-U03).
 *
 * @par
 * Only obstacle_task calls this driver (TEAM.md section 3).
 */

#ifndef DRV_SERVO_H
#define DRV_SERVO_H

#include <stdint.h>

#define DRV_SERVO_PIN           (12u)
#define DRV_SERVO_RIGHT_DEG     (0)
#define DRV_SERVO_AHEAD_DEG     (90)
#define DRV_SERVO_LEFT_DEG      (180)

uint32_t drv_servo_init(void);
uint32_t drv_servo_move(int32_t angle_deg);
int32_t  drv_servo_angle(void);

#endif /* DRV_SERVO_H */

/*** end of file ***/
