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

/* ---------------------------------------------------------------------
 *  Interrupt code in RAM (Tribunal 1, Version C).
 *
 *  S3_RAM_FUNC marks every function that runs inside S3's two interrupts
 *  (sampler and barcode DOUT edge), so a flash cache miss (WiFi and USB
 *  code also run from flash) cannot stall them. The section names follow
 *  the Pico SDK (".time_critical.<name>").
 *
 *  OFF by default: it needs ONE team change in the shared linker script
 *  etc/linker/pico_rp2040/tkernel_map.ld, inside ".data : { ... }",
 *  right after *(.data.*):
 *      *(.time_critical*)
 *  Reset_Handler then copies the code to RAM with .data. Checked in a
 *  build (28 Oct): all 8 S3 functions link at 0x2000xxxx, 0 warnings.
 *  Without that line the sections stay in flash (harmless, no gain).
 *  Team effects: the same line also moves TinyUSB's own .time_critical
 *  functions to RAM (as the Pico SDK intends) and serves Buddy 2's
 *  gpio_isr (Version C). Not yet run on the car.
 *  ".data.*" would need no linker change, but the assembler then warns
 *  "setting incorrect section attributes" for every function.
 * --------------------------------------------------------------------- */
#ifndef S3_ISR_IN_RAM
#define S3_ISR_IN_RAM       (0)
#endif

#if (1 == S3_ISR_IN_RAM)
#define S3_RAM_FUNC(name)   __attribute__((section(".time_critical.s3_" \
                                                   #name), noinline))
#else
#define S3_RAM_FUNC(name)
#endif

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
uint32_t drv_adc_isr_max_us_take(void);     /* Only with S3_TIMING 1.   */

#endif /* DRV_ADC_H */

/*** end of file ***/
