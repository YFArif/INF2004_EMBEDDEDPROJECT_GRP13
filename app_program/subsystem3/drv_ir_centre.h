/** @file drv_ir_centre.h
 *
 * @brief Subsystem 3 -- Centre line sensor, analogue output on GP27
 *        (ADC channel 1).
 *
 * @par Where it is mounted
 * On the robot's centre line, riding the RIGHT edge of the 18 mm line:
 * half black / half white under the sensor (about 500 permille) is "on
 * track". s3_line.c steers to keep it there.
 *
 * @par Data
 * The sampler interrupt (drv_adc.c) calls drv_ir_centre_sample_isr()
 * every 10 ms, straight after the Left IR sample. The value is kept as
 * 8 bits (Change Requirement 1) in one byte with one writer, so a read
 * is never torn. Higher = darker.
 */

#ifndef DRV_IR_CENTRE_H
#define DRV_IR_CENTRE_H

#include <stdint.h>

#define DRV_IR_CENTRE_PIN       (27u)
#define DRV_IR_CENTRE_ADC_CH    (1u)

#if (DRV_IR_CENTRE_PIN != (26u + DRV_IR_CENTRE_ADC_CH))
#error "drv_ir_centre: pin and ADC channel do not match"
#endif

void     drv_ir_centre_init(void);
void     drv_ir_centre_sample_isr(void);
uint16_t drv_ir_centre_raw(void);

#endif /* DRV_IR_CENTRE_H */

/*** end of file ***/
