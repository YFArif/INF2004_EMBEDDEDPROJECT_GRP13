/** @file drv_ir_centre.c
 *
 * @brief Subsystem 3 -- Centre line sensor driver (GP27, ADC1).
 */

#include <stdint.h>

#include "drv_adc.h"
#include "drv_ir_centre.h"

static volatile uint8_t g_raw8;     /* Latest sample, 8-bit. ISR writes. */

/*!
 * @brief Make GP27 an analogue input. Sampling starts with drv_adc_init().
 */
void
drv_ir_centre_init (void)
{
    drv_adc_pin_analog(DRV_IR_CENTRE_PIN);
}

/*!
 * @brief Take one sample. Called only by the sampler interrupt.
 */
void
drv_ir_centre_sample_isr (void)
{
    g_raw8 = drv_adc_to8(drv_adc_convert(DRV_IR_CENTRE_ADC_CH));
}

/*!
 * @brief Latest reading on the 12-bit scale (0 ... 4080), higher = darker.
 */
uint16_t
drv_ir_centre_raw (void)
{
    return (uint16_t) ((uint16_t) g_raw8 << 4);
}

/*** end of file ***/
