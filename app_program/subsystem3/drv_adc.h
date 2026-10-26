/** @file drv_adc.h
 *
 * @brief Subsystem 3 -- ADC owner and the 2 kHz sampler interrupt
 *        (timer alarm 1).
 *
 * @par
 * S3 drives the ADC registers itself and does not use the kernel ADC
 * driver ("adca"), so the ADC has exactly one owner. One sampler
 * interrupt runs every DRV_ADC_SAMPLE_US. Each time it hands the
 * barcode driver one analogue sample; every DRV_ADC_IR_EVERY samples
 * (10 ms) it also lets the two line-sensor drivers take one sample.
 * Each driver converts its own ADC channel with drv_adc_convert().
 *
 * NOTE: Call drv_adc_init() from a task on core 1: RP2040 interrupt
 * enables are per core, and task_line reads the line sensors with
 * interrupts disabled on that same core.
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include <stdint.h>

#include "../app.h"         /* ER for the kernel error code.            */

#define DRV_ADC_SAMPLE_US   (500u)  /* 2 kHz: 0.15 mm per sample at      */
                                    /* 0.30 m/s.                         */
#define DRV_ADC_IR_EVERY    (10000u / DRV_ADC_SAMPLE_US)  /* 10 ms.     */
#define DRV_ADC_FIRST_PIN   (26u)   /* ADC channel n is on GP(26 + n).  */

/*!
 * @brief 12-bit ADC count to 8 bits, rounded to nearest (Change
 *        Requirement 1; maximum error 8 counts = 0.2 %).
 */
static inline uint8_t
drv_adc_to8 (uint16_t adc)
{
    uint32_t value = ((uint32_t) adc + 8u) >> 4;

    return (uint8_t) ((value > 255u) ? 255u : value);
}

void     drv_adc_pin_analog(uint32_t pin);
uint16_t drv_adc_convert(uint32_t channel);
uint32_t drv_adc_sample_index(void);
ER       drv_adc_init(void);

#endif /* DRV_ADC_H */

/*** end of file ***/
