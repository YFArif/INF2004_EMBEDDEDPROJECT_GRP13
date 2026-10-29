/** @file drv_ultrasonic.c
 *
 * @brief HC-SR04 ultrasonic ranger driver.
 *
 * @par
 * HC-SR04 datasheet (Elecfreaks): a TRIG pulse of at least 10 us starts
 * a ping; ECHO then goes high for the round-trip time of the sound.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_s5_hw.h"
#include "drv_ultrasonic.h"

#define TRIG_PULSE_US       (12u)       /* Datasheet minimum 10 us.      */
#define ECHO_COUNT_TOP      (0xFFFFu)   /* Counter wraps after 65.5 ms.  */

#if (1u != (DRV_ULTRASONIC_ECHO_PIN % 2u))
# error "ECHO must be on an odd pin: only PWM channel B can gate a count."
#endif

/*!
 * @brief Set up TRIG as a low output and ECHO as a gated PWM input.
 *
 * NOTE: Call it from obstacle_task, after every subsystemN_init() has
 * run. Subsystem 2's motor init resets the whole PWM block.
 */
void
drv_ultrasonic_init (void)
{
    uint32_t slice = drv_s5_hw_pwm_slice(DRV_ULTRASONIC_ECHO_PIN);

    drv_s5_hw_pwm_release();
    drv_s5_hw_pin_output(DRV_ULTRASONIC_TRIG_PIN);
    drv_s5_hw_pwm_setup(slice, DRV_S5_HW_PWM_DIV_1MHZ, ECHO_COUNT_TOP,
                        DRV_S5_HW_PWM_GATED_B);
    drv_s5_hw_pin_to_pwm(DRV_ULTRASONIC_ECHO_PIN, true);
}

/*!
 * @brief Clear the echo counter and send one TRIG pulse.
 *
 * NOTE: The caller must check that ECHO is low first; the HC-SR04 ignores
 * a trigger while it is still timing the previous ping.
 */
void
drv_ultrasonic_trigger (void)
{
    drv_s5_hw_pwm_clear_count(drv_s5_hw_pwm_slice(DRV_ULTRASONIC_ECHO_PIN));
    drv_s5_hw_pin_write(DRV_ULTRASONIC_TRIG_PIN, true);
    drv_s5_hw_wait_us(TRIG_PULSE_US);
    drv_s5_hw_pin_write(DRV_ULTRASONIC_TRIG_PIN, false);
}

/*!
 * @brief Tell whether the ECHO line is high (a ping is in progress).
 *
 * @return true while ECHO is high.
 */
bool
drv_ultrasonic_echo_is_high (void)
{
    return (drv_s5_hw_pin_read(DRV_ULTRASONIC_ECHO_PIN));
}

/*!
 * @brief Width of the echo pulse since the last trigger.
 *
 * @return Microseconds ECHO was high, 0 if it never went high. Only
 *         meaningful once ECHO is low again.
 */
uint32_t
drv_ultrasonic_echo_us (void)
{
    return (drv_s5_hw_pwm_count(drv_s5_hw_pwm_slice(DRV_ULTRASONIC_ECHO_PIN)));
}

/*** end of file ***/
