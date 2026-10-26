/** @file drv_ir_left.c
 *
 * @brief Subsystem 3 -- "Left IR" line sensor driver (GP28, ADC2).
 */

#include <stdint.h>

#include "drv_adc.h"
#include "drv_ir_left.h"

static volatile uint8_t g_raw8;     /* Latest sample, 8-bit. ISR writes. */

/*!
 * @brief Make GP28 an analogue input. Sampling starts with drv_adc_init().
 */
void
drv_ir_left_init (void)
{
    drv_adc_pin_analog(DRV_IR_LEFT_PIN);
}

/*!
 * @brief Take one sample. Called only by the sampler interrupt.
 */
void
drv_ir_left_sample_isr (void)
{
    g_raw8 = drv_adc_to8(drv_adc_convert(DRV_IR_LEFT_ADC_CH));
}

/*!
 * @brief Latest reading on the 12-bit scale (0 ... 4080), higher = darker.
 */
uint16_t
drv_ir_left_raw (void)
{
    return (uint16_t) ((uint16_t) g_raw8 << 4);
}

/*** end of file ***/
