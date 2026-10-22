/** @file drv_encoder.c
 *
 * @brief Subsystem 2 -- wheel encoder interrupt handler and tick counter.
 *
 * @par
 * One IO_IRQ_BANK0 handler takes the edge events of GP16 and GP17 on the
 * core that calls drv_encoder_init() (motion_task: core 1, as the Week 6
 * report section 3 places it). Register addresses and bits are from the
 * RP2040 datasheet: IO_BANK0 and PADS_BANK0 (2.19), SIO (2.3.1) and
 * TIMER (4.6).
 *
 * @par Debouncing
 * The S2-U01 runs showed edge gaps as short as 11 us, while real edges
 * are hundreds of microseconds apart. The handler first clears the
 * latched events, then reads the pin level. A level change is accepted
 * only if it differs from the last accepted level and at least
 * DRV_ENCODER_MIN_GAP_US have passed since that change; a change to high
 * counts one tick. Every edge after the read latches a new event, so the
 * final level of a burst of glitches is always seen.
 *
 * @par
 * TODO: Merge into the team's shared gpio_isr (Week 6 report IF-11) once
 * S3's barcode edges on GP6 need the same interrupt.
 */

#include <stdbool.h>
#include <stdint.h>

#include <tk/tkernel.h>     /* tk_def_int(), in_w(), out_w().           */
#include <tm/tmonitor.h>    /* tm_printf() console output.              */
#include <bsp/libbsp.h>     /* gpio_set_pin().                           */

#include "drv_encoder.h"

/* Atomic register aliases for APB peripherals (datasheet 2.1.2). */
#define ALIAS_SET           (0x2000u)
#define ALIAS_CLR           (0x3000u)

/* IO_BANK0 interrupt registers; register 2 covers GPIO16 ... GPIO23. */
#define IO_BANK0_ADDR       (0x40014000u)
#define IO_INTR2_REG        (IO_BANK0_ADDR + 0x0F8u)
#define IO_PROC0_INTE2_REG  (IO_BANK0_ADDR + 0x108u)
#define IO_PROC0_INTS2_REG  (IO_BANK0_ADDR + 0x128u)
#define IO_PROC1_INTE2_REG  (IO_BANK0_ADDR + 0x138u)
#define IO_PROC1_INTS2_REG  (IO_BANK0_ADDR + 0x158u)
#define IO_BITS_PER_PIN     (4u)
#define IO_FIRST_PIN        (16u)       /* First pin of register 2.      */
#define IO_EDGE_LOW_BIT     (0x4u)      /* Falling edge, per pin.        */
#define IO_EDGE_HIGH_BIT    (0x8u)      /* Rising edge, per pin.         */
#define IO_EDGE_BITS        (IO_EDGE_LOW_BIT | IO_EDGE_HIGH_BIT)

/* PADS_BANK0: one word per pin after the voltage-select word. */
#define PADS_BANK0_ADDR     (0x4001C000u)
#define PADS_STRIDE         (4u)
#define PADS_OFFSET         (4u)
#define PAD_PUE_BIT         (0x08u)     /* Pull-up enable.               */
#define PAD_PDE_BIT         (0x04u)     /* Pull-down enable.             */

/* SIO input levels and the raw low word of the 1 MHz timer. */
#define SIO_GPIO_IN_REG     (0xD0000004u)
#define TIMER_RAWL_REG      (0x40054028u)

/* WATCHDOG TICK (datasheet 4.7.6): the tick generator that clocks the
 * TIMER. The kernel does not set it; on this board it was already 12
 * cycles at boot (S2-U08 read 0x360c). It is checked and enforced here
 * because every encoder time depends on it: clk_ref is the 12 MHz
 * crystal (kernel cpu_clock.c), and 12 cycles per tick give 1 MHz. */
#define WATCHDOG_TICK_REG   (0x4005802Cu)
#define TICK_ENABLE_BIT     (0x00000200u)
#define TICK_CYCLES_MASK    (0x000001FFu)
#define TICK_CYCLES_1MHZ    (12u)

/* Interrupt number of IO_IRQ_BANK0, and the level the kernel's own
 * UART / ADC / I2C drivers use. */
#define INTNO_IO_BANK0      (13u)
#define ENCODER_INT_LEVEL   (2)

/** Interrupt-side state of one wheel. */
typedef struct
{
    drv_encoder_count_t count;
    uint32_t            pin;
    uint32_t            last_edge_us;   /* Time of the last raw event.   */
    uint32_t            change_us;      /* Time of the last accepted     */
                                        /* level change.                 */
    bool                b_level;        /* Last accepted level.          */
    bool                b_any_edge;     /* min_gap_us is meaningful.     */
} encoder_state_t;

static volatile encoder_state_t g_encoder[DRV_ENCODER_COUNT] =
{
    { .pin = DRV_ENCODER_LEFT_PIN  },
    { .pin = DRV_ENCODER_RIGHT_PIN }
};

static uint32_t g_reg_ints  = IO_PROC0_INTS2_REG;
static uint32_t g_all_edges = 0u;
static uint32_t g_core      = UINT32_MAX;   /* Not set up yet.          */
static uint32_t g_tick_was  = 0u;           /* TICK before the fix.     */

static void     timer_tick_setup(void);
static void     encoder_isr(UINT intno);
static void     encoder_event(volatile encoder_state_t * p_state,
                              uint32_t events, uint32_t levels,
                              uint32_t now_us);
static uint32_t edge_bits(uint32_t pin);
static uint32_t pad_reg(uint32_t pin);

/*!
 * @brief Set up GP16 / GP17 and the edge interrupt on the calling core.
 *
 * @return E_OK, or the kernel error from tk_def_int().
 *
 * NOTE: Call once, from motion_task, so the interrupt runs on core 1.
 */
ER
drv_encoder_init (void)
{
    // NOTE: The casts are safe: the handler has the kernel's interrupt
    // handler signature, void (UINT), which FP stands for.
    T_DINT const dint =
    {
        .intatr = TA_HLNG,
        .inthdr = (FP) encoder_isr,
    };
    uint32_t core     = 0u;
    uint32_t reg_inte = IO_PROC0_INTE2_REG;
    uint32_t levels   = 0u;
    uint32_t now_us   = 0u;
    uint32_t idx      = 0u;
    ER       result   = E_OK;

#if TK_SUPPORT_SMP
    // NOTE: The cast is safe: tk_get_prc() returns 1 or 2 here.
    core = (uint32_t) tk_get_prc() - 1u;
#endif

    timer_tick_setup();
    if (0u != core)
    {
        reg_inte   = IO_PROC1_INTE2_REG;
        g_reg_ints = IO_PROC1_INTS2_REG;
    }

    g_all_edges = edge_bits(DRV_ENCODER_LEFT_PIN) |
                  edge_bits(DRV_ENCODER_RIGHT_PIN);

    // NOTE: The cast is safe: DRV_ENCODER_COUNT is 2.
    for (idx = 0u; idx < (uint32_t) DRV_ENCODER_COUNT; idx++)
    {
        (void) gpio_set_pin(g_encoder[idx].pin, GPIO_MODE_IN);

        // The pad resets with a pull-down; the encoder output may be
        // open-collector, so use a pull-up instead.
        out_w(pad_reg(g_encoder[idx].pin) + ALIAS_CLR, PAD_PDE_BIT);
        out_w(pad_reg(g_encoder[idx].pin) + ALIAS_SET, PAD_PUE_BIT);
    }

    out_w(reg_inte + ALIAS_CLR, g_all_edges);
    out_w(IO_INTR2_REG, g_all_edges);

    // Start from the real level, so the first change is judged right.
    levels = in_w(SIO_GPIO_IN_REG);
    now_us = in_w(TIMER_RAWL_REG);

    // NOTE: The cast is safe: DRV_ENCODER_COUNT is 2.
    for (idx = 0u; idx < (uint32_t) DRV_ENCODER_COUNT; idx++)
    {
        g_encoder[idx].b_level   =
            (0u != (levels & (1u << g_encoder[idx].pin)));
        g_encoder[idx].change_us = now_us;
        g_encoder[idx].count.min_gap_us = UINT32_MAX;
    }

    result = tk_def_int(INTNO_IO_BANK0, &dint);

    if (E_OK == result)
    {
        ClearInt(INTNO_IO_BANK0);
        out_w(reg_inte + ALIAS_SET, g_all_edges);
        EnableInt(INTNO_IO_BANK0, ENCODER_INT_LEVEL);  /* This core.     */
        g_core = core;

        // NOTE: The cast is safe: format strings are ASCII, and every
        // value is below 32, which an int always holds.
        tm_printf((UB *) "encoder: GP%d/GP%d edge interrupt on core %d\n",
                  (int) DRV_ENCODER_LEFT_PIN, (int) DRV_ENCODER_RIGHT_PIN,
                  (int) core);
    }

    return (result);
}

/*!
 * @brief Copy one wheel's counters.
 *
 * @param[in]  wheel   DRV_ENCODER_LEFT or DRV_ENCODER_RIGHT.
 * @param[out] p_count The counters; ticks and tick_time_us always match.
 *
 * NOTE: Safe from any task on either core: the copy is repeated if a tick
 * arrived while it was taken.
 */
void
drv_encoder_get (drv_encoder_t wheel, drv_encoder_count_t * p_count)
{
    volatile encoder_state_t const * p_state = &g_encoder[DRV_ENCODER_LEFT];
    uint32_t                         before  = 0u;

    if (DRV_ENCODER_RIGHT == wheel)
    {
        p_state = &g_encoder[DRV_ENCODER_RIGHT];
    }

    do
    {
        before                = p_state->count.ticks;
        p_count->ticks        = before;
        p_count->tick_time_us = p_state->count.tick_time_us;
        p_count->raw_rise     = p_state->count.raw_rise;
        p_count->raw_fall     = p_state->count.raw_fall;
        p_count->rejected     = p_state->count.rejected;
        p_count->min_gap_us   = p_state->count.min_gap_us;
    } while (before != p_state->count.ticks);
}

/*!
 * @brief Read the RP2040's free-running 1 MHz timer (low 32 bits).
 *
 * @return Microseconds; wraps every 71.6 minutes, so use differences.
 */
uint32_t
drv_encoder_now_us (void)
{
    return (in_w(TIMER_RAWL_REG));
}

/*!
 * @brief Core that takes the encoder interrupt (evidence for A4).
 *
 * @return 0 or 1, or UINT32_MAX if drv_encoder_init() has not succeeded.
 */
uint32_t
drv_encoder_core (void)
{
    return (g_core);
}

/*!
 * @brief WATCHDOG TICK register as it was before drv_encoder_init() set
 *        it (evidence for the S2-U08 finding).
 *
 * @return The register value.
 */
uint32_t
drv_encoder_tick_was (void)
{
    return (g_tick_was);
}

/*!
 * @brief Make sure the TIMER counts at 1 MHz.
 *
 * NOTE: Writes only if the tick is not already 12 cycles and enabled.
 * Only the tick generator is touched; the watchdog itself stays off.
 */
static void
timer_tick_setup (void)
{
    g_tick_was = in_w(WATCHDOG_TICK_REG);

    if ((TICK_CYCLES_1MHZ != (g_tick_was & TICK_CYCLES_MASK)) ||
        (0u == (g_tick_was & TICK_ENABLE_BIT)))
    {
        out_w(WATCHDOG_TICK_REG, TICK_ENABLE_BIT | TICK_CYCLES_1MHZ);
    }
}

/*!
 * @brief IO_IRQ_BANK0 handler. Interrupt context: no printing, no waits.
 *
 * @param[in] intno Interrupt number (unused).
 */
static void
encoder_isr (UINT intno)
{
    uint32_t events = in_w(g_reg_ints) & g_all_edges;

    (void) intno;

    while (0u != events)
    {
        uint32_t levels = 0u;
        uint32_t now_us = 0u;

        // Clear first, then read the level: an edge after the read
        // latches a new event and comes round this loop again.
        out_w(IO_INTR2_REG, events);
        now_us = in_w(TIMER_RAWL_REG);
        levels = in_w(SIO_GPIO_IN_REG);

        encoder_event(&g_encoder[DRV_ENCODER_LEFT], events, levels, now_us);
        encoder_event(&g_encoder[DRV_ENCODER_RIGHT], events, levels,
                      now_us);

        events = in_w(g_reg_ints) & g_all_edges;
    }
}

/*!
 * @brief Handle the latched edge events of one wheel.
 *
 * @param[in,out] p_state The wheel.
 * @param[in]     events  Latched INTR2 edge bits of both wheels.
 * @param[in]     levels  SIO input levels read after clearing them.
 * @param[in]     now_us  Timer value at the read.
 */
static void
encoder_event (volatile encoder_state_t * p_state, uint32_t events,
               uint32_t levels, uint32_t now_us)
{
    uint32_t shift  = (p_state->pin - IO_FIRST_PIN) * IO_BITS_PER_PIN;
    uint32_t mine   = (events >> shift) & IO_EDGE_BITS;
    bool     b_high = (0u != (levels & (1u << p_state->pin)));

    if (0u != mine)
    {
        if (0u != (mine & IO_EDGE_HIGH_BIT))
        {
            p_state->count.raw_rise++;
        }

        if (0u != (mine & IO_EDGE_LOW_BIT))
        {
            p_state->count.raw_fall++;
        }

        if (p_state->b_any_edge &&
            ((now_us - p_state->last_edge_us) < p_state->count.min_gap_us))
        {
            p_state->count.min_gap_us = now_us - p_state->last_edge_us;
        }

        p_state->b_any_edge   = true;
        p_state->last_edge_us = now_us;

        if ((b_high != p_state->b_level) &&
            ((now_us - p_state->change_us) >= DRV_ENCODER_MIN_GAP_US))
        {
            p_state->b_level   = b_high;
            p_state->change_us = now_us;

            if (b_high)
            {
                p_state->count.tick_time_us = now_us;
                p_state->count.ticks++;
            }
        }
        else
        {
            // Back at the old level already, or too soon after the last
            // change: a glitch.
            p_state->count.rejected++;
        }
    }
}

/*!
 * @brief INTR2 edge bits of one pin (GPIO16 ... GPIO23).
 *
 * @param[in] pin GPIO number, 16 ... 23.
 *
 * @return Its rising and falling edge bits.
 */
static uint32_t
edge_bits (uint32_t pin)
{
    return (IO_EDGE_BITS << ((pin - IO_FIRST_PIN) * IO_BITS_PER_PIN));
}

/*!
 * @brief Pad control register of a pin.
 *
 * @param[in] pin GPIO number, 0 ... 29.
 *
 * @return The register address.
 */
static uint32_t
pad_reg (uint32_t pin)
{
    return (PADS_BANK0_ADDR + PADS_OFFSET + (PADS_STRIDE * pin));
}

/*** end of file ***/
