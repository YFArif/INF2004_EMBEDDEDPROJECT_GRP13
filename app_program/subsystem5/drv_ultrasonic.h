/** @file drv_ultrasonic.h
 *
 * @brief HC-SR04 ultrasonic ranger: TRIG on GP2, ECHO on GP3 (Grove 2).
 *
 * @par How the echo is timed
 * GP3 is channel B of PWM slice 1. The slice runs in gated mode at
 * 1 MHz, so its counter advances only while ECHO is high: after a ping
 * the counter holds the echo pulse width in microseconds. No interrupt
 * is needed, so the HC-SR04 does not use the shared IO_IRQ_BANK0
 * handler (IF-11).
 *
 * WARNING: ECHO is a 5 V signal on a 5 V HC-SR04. It must reach GP3
 * through a divider to 3.3 V or less (assumption A8, test S5-U02).
 *
 * @par
 * Only obstacle_task calls this driver (TEAM.md section 3).
 */

#ifndef DRV_ULTRASONIC_H
#define DRV_ULTRASONIC_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_ULTRASONIC_TRIG_PIN     (2u)
#define DRV_ULTRASONIC_ECHO_PIN     (3u)

void     drv_ultrasonic_init(void);
void     drv_ultrasonic_trigger(void);
bool     drv_ultrasonic_echo_is_high(void);
uint32_t drv_ultrasonic_echo_us(void);

#endif /* DRV_ULTRASONIC_H */

/*** end of file ***/
