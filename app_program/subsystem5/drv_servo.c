/** @file drv_servo.c
 *
 * @brief SG90 servo driver.
 *
 * @par
 * The PWM slice counts at 1 MHz and wraps every 20 000 counts (50 Hz),
 * so the compare level is the pulse width in microseconds.
 *
 * WARNING: The pulse limits and settle times below are typical SG90
 * values, not measured on this servo. Calibrate them with S5-U03 (angle
 * with a protractor, settle time against a flat wall, at full and low
 * battery, assumption A9).
 */

#include <stdint.h>

#include "drv_s5_hw.h"
#include "drv_servo.h"

#define PERIOD_US           (20000u)    /* 50 Hz servo frame.            */
#define PULSE_RIGHT_US      (500)       /* Pulse at 0 degrees.           */
#define PULSE_LEFT_US       (2500)      /* Pulse at 180 degrees.         */
#define TRIM_DEG            (0)         /* Mounting offset; S5-U03.      */
#define SETTLE_BASE_MS      (40u)       /* Ringing after any move.       */
#define SETTLE_MS_PER_DEG   (4u)        /* Travel time per degree.       */

static int32_t g_angle_deg = DRV_SERVO_AHEAD_DEG;

static int32_t clamp_angle(int32_t angle_deg);

/*!
 * @brief Start the 50 Hz servo signal pointing straight ahead.
 *
 * @return Milliseconds to wait for the servo to reach straight ahead.
 *
 * NOTE: Call it from obstacle_task, after every subsystemN_init() has
 * run. Subsystem 2's motor init resets the whole PWM block.
 */
uint32_t
drv_servo_init (void)
{
    uint32_t slice = drv_s5_hw_pwm_slice(DRV_SERVO_PIN);

    drv_s5_hw_pwm_release();
    drv_s5_hw_pwm_setup(slice, DRV_S5_HW_PWM_DIV_1MHZ, PERIOD_US - 1u,
                        DRV_S5_HW_PWM_FREE_RUN);

    // The start angle is unknown. From either end the servo is 90 degrees
    // from straight ahead, so assuming an end gives the longest settle.
    // The pulse is set before the pin is connected, so the servo never
    // sees a zero-width pulse.
    g_angle_deg = DRV_SERVO_LEFT_DEG;

    uint32_t settle_ms = drv_servo_move(DRV_SERVO_AHEAD_DEG);

    drv_s5_hw_pin_to_pwm(DRV_SERVO_PIN, false);

    return (settle_ms);
}

/*!
 * @brief Command a new angle.
 *
 * @param[in] angle_deg Target angle; clamped to 0 ... 180 degrees.
 *
 * @return Milliseconds to wait before the servo has settled and a ping
 *         at the new angle can be trusted.
 */
uint32_t
drv_servo_move (int32_t angle_deg)
{
    int32_t target  = clamp_angle(angle_deg);
    int32_t travel  = target - g_angle_deg;
    int32_t trimmed = clamp_angle(target + TRIM_DEG);

    if (travel < 0)
    {
        travel = -travel;
    }

    // Pulse width grows linearly from the right end to the left end.
    int32_t pulse_us = PULSE_RIGHT_US
                       + (((PULSE_LEFT_US - PULSE_RIGHT_US) * trimmed)
                          / DRV_SERVO_LEFT_DEG);

    // NOTE: Both casts are safe: pulse_us is 500 ... 2500 and travel is
    // 0 ... 180, so neither is negative.
    drv_s5_hw_pwm_set_a(drv_s5_hw_pwm_slice(DRV_SERVO_PIN),
                        (uint32_t) pulse_us);
    g_angle_deg = target;

    return (SETTLE_BASE_MS + (SETTLE_MS_PER_DEG * (uint32_t) travel));
}

/*!
 * @brief Last commanded angle.
 *
 * @return Angle in degrees, 0 ... 180.
 */
int32_t
drv_servo_angle (void)
{
    return (g_angle_deg);
}

/*!
 * @brief Limit an angle to the servo's range.
 *
 * @param[in] angle_deg Any angle.
 *
 * @return The angle limited to 0 ... 180 degrees.
 */
static int32_t
clamp_angle (int32_t angle_deg)
{
    int32_t limited = angle_deg;

    if (limited < DRV_SERVO_RIGHT_DEG)
    {
        limited = DRV_SERVO_RIGHT_DEG;
    }
    else if (limited > DRV_SERVO_LEFT_DEG)
    {
        limited = DRV_SERVO_LEFT_DEG;
    }
    else
    {
        // Already inside the range.
    }

    return (limited);
}

/*** end of file ***/
