/** @file drv_barcode.c
 *
 * @brief Subsystem 3 -- barcode sensor driver (DOUT GP7, AOUT GP28).
 *
 * @par
 * Register addresses are from the RP2040 datasheet: IO_BANK0 (2.19.6).
 * See drv_barcode.h for the two paths and how the data is shared.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem2/subsystem2.h"   /* subsystem2_gpio_isr_register() */

#include "drv_adc.h"
#include "drv_barcode.h"

#if (DRV_BARCODE_DOUT_PIN == (DRV_ADC_FIRST_PIN + DRV_BARCODE_ADC_CH))
#error "barcode DOUT and AOUT cannot be on the same pin"
#endif

/* IO_BANK0 interrupt registers of the DOUT pin (8 pins per register). */
#define BC_REG_OFS          (4u * (DRV_BARCODE_DOUT_PIN / 8u))
#define BC_IO_INTR_REG      (IO_BANK0_BASE + 0x0F0u + BC_REG_OFS)
#define BC_PROC1_INTE_REG   (IO_BANK0_BASE + 0x130u + BC_REG_OFS)
#define BC_PROC1_INTS_REG   (IO_BANK0_BASE + 0x150u + BC_REG_OFS)
#define BC_EDGE_BITS        (0xCu << (4u * (DRV_BARCODE_DOUT_PIN % 8u)))

#define DIG_MASK            (DRV_BARCODE_DIG_RING - 1u)
#define GAP_SHIFT           (4u)        /* Gaps in 16 us units: 16 bits */
                                        /* hold up to 1.05 s.           */
#define GAP_MAX             (0xFFFFu)
#define RING_MASK           (DRV_BARCODE_RING - 1u)
#define COPY_GUARD          (64u)       /* Window starts at least this  */
                                        /* many samples (32 ms) ahead   */
                                        /* of the sampler.              */
#define PRE_SAMPLES         (200000u / DRV_ADC_SAMPLE_US)  /* 200 ms    */
                                        /* kept before and after.       */
#define QUIET_SAMPLES       (DRV_BARCODE_QUIET_US / DRV_ADC_SAMPLE_US)

/* Activity window state, g_act bits. */
#define ACT_OPEN_BIT        (1u)
#define ACT_AOUT_BIT        (DRV_BARCODE_SRC_AOUT << 1)
#define ACT_DOUT_BIT        (DRV_BARCODE_SRC_DOUT << 1)

/* Digital path: written by the GPIO interrupt, except g_gap_rd (task). */
static volatile uint16_t g_gap[DRV_BARCODE_DIG_RING];
static volatile uint8_t  g_gap_wr;
static volatile uint8_t  g_gap_rd;
static volatile uint8_t  g_glitches;    /* Free-running; task takes     */
static volatile uint8_t  g_gap_lost;    /* differences.                 */
static volatile uint32_t g_last_us;     /* Time of the last kept edge.  */

/* Analogue path: written by the interrupts only. */
static volatile uint8_t  g_ring[DRV_BARCODE_RING];
static volatile uint32_t g_act_first;
static volatile uint32_t g_act_last;
static volatile uint8_t  g_act;         /* ACT_xxx_BIT.                 */
static volatile uint8_t  g_act_ref;     /* Sample at the last activity. */

/* Task-side copies of one pass (static: keeps the task stack small). */
static uint32_t g_dur[DRV_BARCODE_DIG_RING];
static uint8_t  g_win[DRV_BARCODE_RING];

static void gpio_isr(UINT intno);
static void mark_active(uint32_t idx, uint8_t src_bit);

/*!
 * @brief Set up DOUT and attach the edge handler to S2's shared
 *        IO_IRQ_BANK0 handler.
 *
 * @return E_OK, or the error from subsystem2_gpio_isr_register().
 *
 * NOTE: Call from a task on core 1. Returns once S2's handler is
 * installed; our pin interrupt is enabled only after that.
 */
ER
drv_barcode_init (void)
{
    ER er;

    drv_adc_pin_analog(DRV_ADC_FIRST_PIN + DRV_BARCODE_ADC_CH);
    (void) gpio_set_pin(DRV_BARCODE_DOUT_PIN, GPIO_MODE_IN);
    clr_w(GPIO(DRV_BARCODE_DOUT_PIN), GPIO_PUE | GPIO_PDE);
    set_w(GPIO(DRV_BARCODE_DOUT_PIN), GPIO_SHEMITT);

    er = subsystem2_gpio_isr_register(gpio_isr);
    if (er < E_OK)
    {
        return er;
    }
    out_w(BC_IO_INTR_REG, BC_EDGE_BITS);
    set_w(BC_PROC1_INTE_REG, BC_EDGE_BITS);
    return E_OK;
}

/*!
 * @brief Store one AOUT sample. Called only by the sampler interrupt.
 *
 * @param[in] idx  Sample index from drv_adc (monotonic).
 */
S3_RAM_FUNC(drv_barcode_sample_isr) void
drv_barcode_sample_isr (uint32_t idx)
{
    uint8_t value = drv_adc_to8(drv_adc_convert(DRV_BARCODE_ADC_CH));
    int32_t diff;

    g_ring[idx & RING_MASK] = value;
    if (0u == idx)
    {
        g_act_ref = value;
    }
    diff = (int32_t) value - (int32_t) g_act_ref;
    if ((diff > DRV_BARCODE_ACT_STEP) || (diff < -DRV_BARCODE_ACT_STEP))
    {
        g_act_ref = value;
        mark_active(idx, ACT_AOUT_BIT);
    }
}

/*!
 * @brief True when a pass has started and both paths have been quiet for
 *        DRV_BARCODE_QUIET_US.
 */
bool
drv_barcode_pass_done (void)
{
    uint32_t now_us     = in_w(TIMER_TIMERAWL);
    bool     b_dig_open = (g_gap_wr != g_gap_rd);
    bool     b_ana_open = (0u != (g_act & ACT_OPEN_BIT));
    bool     b_dig_busy = b_dig_open &&
                          ((now_us - g_last_us) <= DRV_BARCODE_QUIET_US);
    bool     b_ana_busy = b_ana_open &&
                          ((drv_adc_sample_index() - g_act_last) <=
                           QUIET_SAMPLES);

    return (b_dig_open || b_ana_open) && (!b_dig_busy) && (!b_ana_busy);
}

/*!
 * @brief Take the finished pass and re-arm both paths.
 *
 * @param[out] p_pass  Durations, analogue window and diagnostics.
 */
void
drv_barcode_take_pass (drv_barcode_pass_t * p_pass)
{
    static uint8_t glitch_seen;     /* Counter values at the last pass. */
    static uint8_t lost_seen;
    uint8_t        rd    = g_gap_rd;
    uint8_t        wr    = g_gap_wr;
    uint32_t       first = 1u;
    uint32_t       last  = 0u;
    uint32_t       n_win = 0u;
    uint32_t       now_idx;
    uint32_t       idx;
    UINT           imask;

    /* Digital: drain the gap ring. Gap 0 is the idle time before the
     * first edge, so element k is gap k + 1. */
    p_pass->n_edges = (uint32_t) ((uint8_t) (wr - rd) & DIG_MASK);
    for (idx = 1u; idx < p_pass->n_edges; idx++)
    {
        g_dur[idx - 1u] = (uint32_t) g_gap[(rd + idx) & DIG_MASK] << GAP_SHIFT;
    }
    g_gap_rd          = wr;             /* Hand the slots back.         */
    p_pass->glitches  = (uint8_t) (g_glitches - glitch_seen);
    glitch_seen       = g_glitches;
    p_pass->lost      = (uint8_t) (g_gap_lost - lost_seen);
    lost_seen         = g_gap_lost;
    p_pass->p_dur     = g_dur;
    p_pass->b_clipped = false;
    p_pass->src       = 0u;

    /* Analogue: snapshot the window indices with interrupts off. */
    DI(imask);
    now_idx = drv_adc_sample_index();
    if (0u != (g_act & ACT_OPEN_BIT))
    {
        first = (g_act_first > PRE_SAMPLES) ? (g_act_first - PRE_SAMPLES) : 0u;
        last  = g_act_last + PRE_SAMPLES;
        if (last >= now_idx)
        {
            last = now_idx - 1u;
        }
        if ((now_idx - first) > (DRV_BARCODE_RING - COPY_GUARD))
        {
            first             = now_idx - DRV_BARCODE_RING + COPY_GUARD;
            p_pass->b_clipped = true;   /* Oldest part overwritten.     */
        }
        p_pass->src = (uint32_t) (g_act >> 1);
        g_act       = 0u;
    }
    EI(imask);

    /* Copy with interrupts on: the sampler keeps writing NEW samples,
     * at least COPY_GUARD samples behind the start of this window. */
    for (idx = first; (idx <= last) && (n_win < DRV_BARCODE_RING); idx++)
    {
        g_win[n_win] = g_ring[idx & RING_MASK];
        n_win++;
    }
    p_pass->p_win = g_win;
    p_pass->n_win = n_win;
}

/*!
 * @brief DOUT edge, called from S2's IO_IRQ_BANK0 handler. Microseconds
 *        only, never waits.
 */
S3_RAM_FUNC(gpio_isr) static void
gpio_isr (UINT intno)
{
    uint32_t now_us = in_w(TIMER_TIMERAWL);
    uint32_t status = in_w(BC_PROC1_INTS_REG) & BC_EDGE_BITS;
    uint8_t  wr;
    uint8_t  next;
    uint32_t gap;

    (void) intno;
    if (0u == status)
    {
        return;                         /* Not our pin.                 */
    }
    out_w(BC_IO_INTR_REG, status);

    mark_active(drv_adc_sample_index(), ACT_DOUT_BIT);

    wr   = g_gap_wr;
    next = (uint8_t) ((wr + 1u) & DIG_MASK);
    gap  = (now_us - g_last_us) >> GAP_SHIFT;
    if ((wr != g_gap_rd) && ((now_us - g_last_us) < DRV_BARCODE_GLITCH_US))
    {
        g_glitches = (uint8_t) (g_glitches + 1u);   /* Chatter.         */
        return;
    }
    if (next == g_gap_rd)
    {
        g_gap_lost = (uint8_t) (g_gap_lost + 1u);   /* Full: keep older. */
    }
    else
    {
        g_gap[wr] = (uint16_t) ((gap > GAP_MAX) ? GAP_MAX : gap);
        g_gap_wr  = next;               /* Publish AFTER the data.      */
    }
    g_last_us = now_us;
}

/*!
 * @brief Mark sample idx as "something is passing". Called from both
 *        interrupts, which run on the same core at the same level and so
 *        never nest.
 */
S3_RAM_FUNC(mark_active) static void
mark_active (uint32_t idx, uint8_t src_bit)
{
    if (0u == (g_act & ACT_OPEN_BIT))
    {
        g_act       = ACT_OPEN_BIT;
        g_act_first = idx;
    }
    g_act_last = idx;
    g_act      = (uint8_t) (g_act | src_bit);
}

/*** end of file ***/
