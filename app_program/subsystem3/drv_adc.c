/** @file drv_adc.c
 *
 * @brief Subsystem 3 -- ADC owner and the 2 kHz sampler interrupt.
 *
 * @par
 * Register addresses are from the RP2040 datasheet: ADC (4.9) and
 * TIMER (4.6). See drv_adc.h for who calls what.
 */

#include <stdint.h>

#include "../app.h"

#include "drv_adc.h"
#include "drv_barcode.h"
#include "drv_ir_centre.h"
#include "drv_ir_left.h"
#include "s3_config.h"

/* ADC registers (datasheet 4.9.6). */
#define ADC_BASE_ADDR       (0x4004C000u)
#define ADC_CS_REG          (ADC_BASE_ADDR + 0x00u)
#define ADC_RESULT_REG      (ADC_BASE_ADDR + 0x04u)
#define ADC_CS_EN_BIT       (1u << 0)
#define ADC_CS_START_BIT    (1u << 2)
#define ADC_CS_READY_BIT    (1u << 8)
#define ADC_AINSEL_POS      (12u)
#define ADC_RESULT_MASK     (0x0FFFu)

/* Timer alarm 1 = RP2040 IRQ 1 (TIMER_IRQ_1). Level 2 is the level the
 * kernel's own drivers use. */
#define INTNO_TIMER1        (1u)
#define TIMER_ALARM1_BIT    (1u << 1)
#define SAMPLER_INT_LEVEL   (2)
#define ALARM_MIN_LEAD_US   (20)    /* Closer than this = fell behind.  */

/* One ADC channel per sensor (rewired 28 Oct: outer IR ADC0, Centre
 * ADC1, barcode AOUT ADC2). */
#if ((DRV_IR_LEFT_ADC_CH == DRV_IR_CENTRE_ADC_CH) || \
     (DRV_IR_LEFT_ADC_CH == DRV_BARCODE_ADC_CH) || \
     (DRV_IR_CENTRE_ADC_CH == DRV_BARCODE_ADC_CH))
#error "two S3 sensors on the same ADC channel"
#endif

/* g_ir_countdown is 8 bits. */
typedef char drv_adc_chk_ir_every_t[((DRV_ADC_IR_EVERY >= 1u) &&
                                     (DRV_ADC_IR_EVERY <= 256u)) ? 1 : -1];

static volatile uint32_t g_sample_idx;  /* Samples taken (monotonic).   */
static volatile uint32_t g_alarm_next;  /* Next alarm time, us.         */
static uint8_t           g_ir_countdown; /* Samples until the next line */
                                         /* sensor reading (ISR only).  */

#if (1 == S3_TIMING)
static volatile uint16_t g_isr_max_us;  /* Longest sampler run, us.     */
#endif

static void sampler_isr(UINT intno);

/*!
 * @brief Make a pin a pure analogue input (no digital input, no pulls).
 */
void
drv_adc_pin_analog (uint32_t pin)
{
    clr_w(GPIO(pin), GPIO_IE | GPIO_PUE | GPIO_PDE);
    out_w(GPIO_CTRL(pin), GPIO_CTRL_FUNCSEL_NULL);
}

/*!
 * @brief One conversion on one channel, about 2 us.
 *
 * NOTE: Call only from the sampler interrupt (through the drivers'
 * _sample_isr functions): the ADC has one owner.
 */
S3_RAM_FUNC(drv_adc_convert) uint16_t
drv_adc_convert (uint32_t channel)
{
    while (0u == (in_w(ADC_CS_REG) & ADC_CS_READY_BIT))
    {
        ;
    }
    out_w(ADC_CS_REG, (channel << ADC_AINSEL_POS) | ADC_CS_EN_BIT |
                      ADC_CS_START_BIT);
    while (0u == (in_w(ADC_CS_REG) & ADC_CS_READY_BIT))
    {
        ;
    }
    return (uint16_t) (in_w(ADC_RESULT_REG) & ADC_RESULT_MASK);
}

/*!
 * @brief Number of samples taken since drv_adc_init(). Compare values
 *        by difference: the counter wraps after 24 days.
 */
S3_RAM_FUNC(drv_adc_sample_index) uint32_t
drv_adc_sample_index (void)
{
    return g_sample_idx;
}

#if (1 == S3_TIMING)
/*!
 * @brief S3_TIMING: longest sampler interrupt since the last call, in us
 *        (1 us timer, so a 3 us reading means 3 to 4 us). Resets the max.
 */
uint32_t
drv_adc_isr_max_us_take (void)
{
    uint32_t max_us = g_isr_max_us;

    g_isr_max_us = 0u;      /* A max lost to a race only misses one run. */
    return max_us;
}
#endif

/*!
 * @brief Enable the ADC and start the sampler on the calling core.
 *
 * @return E_OK, or the kernel error from tk_def_int().
 */
ER
drv_adc_init (void)
{
    T_DINT dint = {0};
    ER     er;

    set_w(ADC_CS_REG, ADC_CS_EN_BIT);   /* Harmless if already enabled. */

    dint.intatr = TA_HLNG;
    dint.inthdr = (FP) sampler_isr;
    er = tk_def_int(INTNO_TIMER1, &dint);
    if (er < E_OK)
    {
        return er;
    }
    out_w(TIMER_INTR, TIMER_ALARM1_BIT);
    set_w(TIMER_INTE, TIMER_ALARM1_BIT);
    EnableInt(INTNO_TIMER1, SAMPLER_INT_LEVEL);
    g_alarm_next = in_w(TIMER_TIMERAWL) + DRV_ADC_SAMPLE_US;
    out_w(TIMER_ALARM1, g_alarm_next);  /* Start sampling.              */
    return E_OK;
}

/*!
 * @brief Timer alarm 1: re-arm, then sample the barcode every time and
 *        the two line sensors every DRV_ADC_IR_EVERY samples (10 ms).
 *
 * A countdown replaces "idx % DRV_ADC_IR_EVERY": the Cortex-M0+ has no
 * divide instruction, so % called the library divide 2000 times a
 * second. The first sample (idx 0) still reads the line sensors.
 */
S3_RAM_FUNC(sampler_isr) static void
sampler_isr (UINT intno)
{
    uint32_t idx    = g_sample_idx;
    uint32_t now_us = in_w(TIMER_TIMERAWL);

    (void) intno;
    out_w(TIMER_INTR, TIMER_ALARM1_BIT);        /* Acknowledge.         */
    g_alarm_next += DRV_ADC_SAMPLE_US;
    if ((int32_t) (g_alarm_next - now_us) < ALARM_MIN_LEAD_US)
    {
        g_alarm_next = now_us + DRV_ADC_SAMPLE_US;
    }
    out_w(TIMER_ALARM1, g_alarm_next);          /* Re-arm.              */

    drv_barcode_sample_isr(idx);
    if (0u == g_ir_countdown)
    {
        g_ir_countdown = (uint8_t) (DRV_ADC_IR_EVERY - 1u);
        drv_ir_left_sample_isr();
        drv_ir_centre_sample_isr();
    }
    else
    {
        g_ir_countdown--;
    }
    g_sample_idx = idx + 1u;

#if (1 == S3_TIMING)
    {
        uint32_t run_us = in_w(TIMER_TIMERAWL) - now_us;

        if (run_us > g_isr_max_us)
        {
            g_isr_max_us = (uint16_t) run_us;
        }
    }
#endif
}

/*** end of file ***/
