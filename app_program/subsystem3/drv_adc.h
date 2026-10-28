/** @file drv_adc.h
 *
 * @brief Subsystem 3 -- ADC owner, the 2 kHz sampler interrupt (timer
 *        alarm 1) and the two analogue line IR sensors.
 *
 * @par
 * S3 drives the ADC registers itself and does not use the kernel ADC
 * driver ("adca"), so the ADC has exactly one owner (NFR12). One sampler
 * interrupt runs every DRV_ADC_SAMPLE_US. Each time it hands the
 * barcode driver one analogue sample. Every DRV_ADC_IR_STRIDE samples it
 * also samples both line IR sensors, and every DRV_ADC_IR_SAMPLES of
 * those (10 ms, NFR1) it publishes their average: averaging cuts the
 * sensor noise that made the steering flip left/right, at no extra
 * delay for task_line, which reads once per 10 ms anyway.
 *
 * @par Line IR sensors
 * - "Left IR" (FR1's name): GP26 / ADC0. On our car it is mounted on the
 *   robot's RIGHT, just outside the line edge the Centre does not ride
 *   (the OUTER sensor; s3_line.h). s3_line.c calls it "L".
 * - Centre: GP27 / ADC1, rides one edge of the line.
 * Each reading is kept as 8 bits (Change Requirement 1) in one byte with
 * one writer; drv_adc_ir_read() returns both from the same 10 ms sample.
 * Higher = darker.
 *
 * NOTE: Call drv_adc_init() from a task on core 1: RP2040 interrupt
 * enables are per core, and drv_adc_ir_read() disables interrupts on
 * that same core to read both sensors together.
 */

#ifndef DRV_ADC_H
#define DRV_ADC_H

#include <stdint.h>

#include "../app.h"         /* ER for the kernel error code.            */

#define DRV_ADC_SAMPLE_US   (500u)  /* 2 kHz: 0.15 mm per sample at      */
                                    /* 0.30 m/s.                         */
#define DRV_ADC_IR_EVERY    (10000u / DRV_ADC_SAMPLE_US)  /* 10 ms.     */
#define DRV_ADC_IR_SAMPLES  (4u)    /* ADC samples averaged per line IR  */
                                    /* reading (one every 2.5 ms). v21:  */
                                    /* host sim S3-S91, steering sign    */
                                    /* flips 26 % -> 15 %. A power of 2, */
                                    /* so the average is a shift.        */
#define DRV_ADC_IR_SHIFT    (2u)    /* log2(DRV_ADC_IR_SAMPLES).         */
#define DRV_ADC_IR_STRIDE   (DRV_ADC_IR_EVERY / DRV_ADC_IR_SAMPLES)
#define DRV_ADC_FIRST_PIN   (26u)   /* ADC channel n is on GP(26 + n).  */

#define DRV_IR_LEFT_PIN         (26u)   /* Rewired 28 Oct (was GP28).   */
#define DRV_IR_LEFT_ADC_CH      (0u)
#define DRV_IR_CENTRE_PIN       (27u)
#define DRV_IR_CENTRE_ADC_CH    (1u)

#if ((DRV_IR_LEFT_PIN != (DRV_ADC_FIRST_PIN + DRV_IR_LEFT_ADC_CH)) || \
     (DRV_IR_CENTRE_PIN != (DRV_ADC_FIRST_PIN + DRV_IR_CENTRE_ADC_CH)))
#error "drv_adc: a line IR pin and its ADC channel do not match"
#endif

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
void     drv_adc_ir_init(void);
void     drv_adc_ir_read(uint16_t * p_left, uint16_t * p_centre);
ER       drv_adc_init(void);
uint32_t drv_adc_isr_max_us_take(void);     /* Only with S3_TIMING 1.   */

#endif /* DRV_ADC_H */

/*** end of file ***/
