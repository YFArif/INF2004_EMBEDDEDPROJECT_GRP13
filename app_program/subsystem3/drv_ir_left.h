/** @file drv_ir_left.h
 *
 * @brief Subsystem 3 -- "Left IR" line sensor, analogue output on GP26
 *        (ADC channel 2).
 *
 * @par Where it is mounted
 * The module is labelled Left IR, but on our car it sits on the robot's
 * RIGHT, about 20-21 mm outside the Centre sensor, beyond the edge the
 * Centre sensor rides (README_S3.md). It sees black only at a junction
 * on that side or when the car has drifted past the line (overshoot).
 * s3_line.c calls it the "L" (outer) sensor.
 *
 * @par Data
 * The sampler interrupt (drv_adc.c) calls drv_ir_left_sample_isr() every
 * 10 ms. The value is kept as 8 bits (Change Requirement 1) in one byte
 * with one writer, so a read is never torn. Higher = darker.
 */

#ifndef DRV_IR_LEFT_H
#define DRV_IR_LEFT_H

#include <stdint.h>

#define DRV_IR_LEFT_PIN     (26u)   /* Rewired 28 Oct (was GP28).       */
#define DRV_IR_LEFT_ADC_CH  (0u)

/* ADC channel n is on GP(26 + n): pin and channel must agree. */
#if (DRV_IR_LEFT_PIN != (26u + DRV_IR_LEFT_ADC_CH))
#error "drv_ir_left: pin and ADC channel do not match"
#endif

void     drv_ir_left_init(void);
void     drv_ir_left_sample_isr(void);
uint16_t drv_ir_left_raw(void);

#endif /* DRV_IR_LEFT_H */

/*** end of file ***/
