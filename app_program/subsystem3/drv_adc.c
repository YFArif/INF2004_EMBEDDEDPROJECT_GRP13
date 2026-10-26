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

static volatile uint32_t g_sample_idx;  /* Samples taken (monotonic).   */
static volatile uint32_t g_alarm_next;  /* Next alarm time, us.         */

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
uint16_t
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
uint32_t
drv_adc_sample_index (void)
{
    return g_sample_idx;
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
 *        the two line sensors every 10 ms.
 */
static void
sampler_isr (UINT intno)
{
    uint32_t idx = g_sample_idx;

    (void) intno;
    out_w(TIMER_INTR, TIMER_ALARM1_BIT);        /* Acknowledge.         */
    g_alarm_next += DRV_ADC_SAMPLE_US;
    if ((int32_t) (g_alarm_next - in_w(TIMER_TIMERAWL)) < ALARM_MIN_LEAD_US)
    {
        g_alarm_next = in_w(TIMER_TIMERAWL) + DRV_ADC_SAMPLE_US;
    }
    out_w(TIMER_ALARM1, g_alarm_next);          /* Re-arm.              */

    drv_barcode_sample_isr(idx);
    if (0u == (idx % DRV_ADC_IR_EVERY))
    {
        drv_ir_left_sample_isr();
        drv_ir_centre_sample_isr();
    }
    g_sample_idx = idx + 1u;
}

/*** end of file ***/
