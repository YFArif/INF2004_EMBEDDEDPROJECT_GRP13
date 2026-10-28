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

/* g_ir_countdown is 8 bits; the IR samples must divide 10 ms evenly, and
 * the 12-bit sums must fit 16 bits. */
typedef char drv_adc_chk_ir_every_t[((DRV_ADC_IR_STRIDE >= 1u) &&
                                     (DRV_ADC_IR_STRIDE <= 256u) &&
                                     ((DRV_ADC_IR_STRIDE *
                                       DRV_ADC_IR_SAMPLES) ==
                                      DRV_ADC_IR_EVERY) &&
                                     ((1u << DRV_ADC_IR_SHIFT) ==
                                      DRV_ADC_IR_SAMPLES) &&
                                     ((DRV_ADC_IR_SAMPLES * 4095u) <=
                                      0xFFFFu)) ? 1 : -1];

static volatile uint32_t g_sample_idx;  /* Samples taken (monotonic).   */
static volatile uint32_t g_alarm_next;  /* Next alarm time, us.         */
static uint8_t           g_ir_countdown; /* Samples until the next line */
                                         /* sensor sample (ISR only).   */
static uint8_t           g_ir_count;    /* IR samples in the sums.      */
static uint16_t          g_ir_left_sum; /* 12-bit sums (ISR only).      */
static uint16_t          g_ir_centre_sum;
static volatile uint8_t  g_ir_left8;    /* Latest averaged readings,    */
static volatile uint8_t  g_ir_centre8;  /* 8-bit. Only the ISR writes.  */

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
 * @brief Make both line IR pins analogue inputs. Sampling starts with
 *        drv_adc_init().
 */
void
drv_adc_ir_init (void)
{
    drv_adc_pin_analog(DRV_IR_LEFT_PIN);
    drv_adc_pin_analog(DRV_IR_CENTRE_PIN);
}

/*!
 * @brief Both line IR readings from the SAME 10 ms sample, on the 12-bit
 *        scale (0 ... 4080), higher = darker.
 *
 * NOTE: Call on core 1 only: the sampler interrupt runs there, so
 * disabling interrupts keeps it from running between the two reads.
 */
void
drv_adc_ir_read (uint16_t * p_left, uint16_t * p_centre)
{
    UINT imask;

    DI(imask);
    *p_left   = (uint16_t) ((uint16_t) g_ir_left8 << 4);
    *p_centre = (uint16_t) ((uint16_t) g_ir_centre8 << 4);
    EI(imask);
}

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
 *        the two line sensors every DRV_ADC_IR_STRIDE samples; every
 *        DRV_ADC_IR_SAMPLES line sensor samples (10 ms) publish their
 *        averages.
 *
 * A countdown replaces "idx % DRV_ADC_IR_STRIDE", and the average is a
 * shift: the Cortex-M0+ has no divide instruction, so % or / would call
 * the library divide in this 2 kHz interrupt. The first sample (idx 0)
 * still samples the line sensors.
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
        g_ir_countdown   = (uint8_t) (DRV_ADC_IR_STRIDE - 1u);
        g_ir_left_sum    = (uint16_t) (g_ir_left_sum +
                                       drv_adc_convert(DRV_IR_LEFT_ADC_CH));
        g_ir_centre_sum  = (uint16_t) (g_ir_centre_sum +
                                       drv_adc_convert(DRV_IR_CENTRE_ADC_CH));
        g_ir_count++;
        if (g_ir_count >= DRV_ADC_IR_SAMPLES)
        {
            // Rounded to nearest, then 8 bits like a single sample.
            g_ir_left8      = drv_adc_to8((uint16_t)
                                  ((g_ir_left_sum +
                                    (DRV_ADC_IR_SAMPLES / 2u)) >>
                                   DRV_ADC_IR_SHIFT));
            g_ir_centre8    = drv_adc_to8((uint16_t)
                                  ((g_ir_centre_sum +
                                    (DRV_ADC_IR_SAMPLES / 2u)) >>
                                   DRV_ADC_IR_SHIFT));
            g_ir_left_sum   = 0u;
            g_ir_centre_sum = 0u;
            g_ir_count      = 0u;
        }
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
