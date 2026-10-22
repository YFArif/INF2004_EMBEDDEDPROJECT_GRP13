/** @file drv_s5_hw.h
 *
 * @brief RP2040 register access for the subsystem 5 drivers: GPIO pins,
 *        the 1 MHz timer and PWM slices.
 *
 * @par
 * Only drv_ultrasonic.c and drv_servo.c call this module, and only from
 * obstacle_task, the owning task of S5's pins and PWM slices 1 and 6
 * (TEAM.md sections 2 and 3).
 */

#ifndef DRV_S5_HW_H
#define DRV_S5_HW_H

#include <stdbool.h>
#include <stdint.h>

/* PWM counter modes, PWM CSR.DIVMODE (RP2040 datasheet 4.5.2.4). */
#define DRV_S5_HW_PWM_FREE_RUN  (0x00u)     /* Count all the time.       */
#define DRV_S5_HW_PWM_GATED_B   (0x10u)     /* Count while pin B is high.*/

/* Divider giving a 1 MHz PWM count from the 125 MHz system clock. */
#define DRV_S5_HW_PWM_DIV_1MHZ  (125u)

void     drv_s5_hw_pwm_release(void);
uint32_t drv_s5_hw_pwm_slice(uint32_t pin);
void     drv_s5_hw_pwm_setup(uint32_t slice, uint32_t div_int, uint32_t top,
                             uint32_t mode);
void     drv_s5_hw_pwm_set_a(uint32_t slice, uint32_t level);
uint32_t drv_s5_hw_pwm_count(uint32_t slice);
void     drv_s5_hw_pwm_clear_count(uint32_t slice);
void     drv_s5_hw_pin_to_pwm(uint32_t pin, bool b_input);
void     drv_s5_hw_pin_output(uint32_t pin);
void     drv_s5_hw_pin_input_pullup(uint32_t pin);
void     drv_s5_hw_pin_write(uint32_t pin, bool b_high);
bool     drv_s5_hw_pin_read(uint32_t pin);
uint32_t drv_s5_hw_now_us(void);
void     drv_s5_hw_wait_us(uint32_t delay_us);

#endif /* DRV_S5_HW_H */

/*** end of file ***/
