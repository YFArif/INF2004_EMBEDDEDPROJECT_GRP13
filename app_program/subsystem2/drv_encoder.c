/*
 *	drv_encoder.c
 *	Subsystem 2 -- wheel encoder edge counter on GP16 (left) and GP17 (right).
 *
 *	One IO_IRQ_BANK0 handler counts rising and falling edges and stamps them
 *	with the RP2040 1 MHz timer.  The shared gpio_isr from the design report
 *	will replace this once the barcode and echo handlers join it.
 *
 *	APIs shown: tk_def_int, EnableInt, ClearInt, tk_get_prc
 */

#include "drv_encoder.h"

#define INTNO_IO_BANK0 13 /* IO_IRQ_BANK0 */
#define ENC_INT_LEVEL 2   /* same level as the kernel's UART/ADC/I2C drivers */

/* IO_BANK0 interrupt registers.  Register 2 covers GPIO16-23, 4 bits per pin. */
#define IO_INTR2 (IO_BANK0_BASE + 0x0F8)
#define IO_PROC0_INTE2 (IO_BANK0_BASE + 0x108)
#define IO_PROC0_INTS2 (IO_BANK0_BASE + 0x128)
#define IO_PROC1_INTE2 (IO_BANK0_BASE + 0x138)
#define IO_PROC1_INTS2 (IO_BANK0_BASE + 0x158)

#define EV_FALL(pin) (0x4U << (((pin) - 16) * 4))
#define EV_RISE(pin) (0x8U << (((pin) - 16) * 4))
#define ENC_EVENTS (EV_FALL(S2_ENC_LEFT_PIN) | EV_RISE(S2_ENC_LEFT_PIN) | \
                    EV_FALL(S2_ENC_RIGHT_PIN) | EV_RISE(S2_ENC_RIGHT_PIN))

#define PAD_PUE (1 << 3) /* pad pull-up enable */
#define PAD_PDE (1 << 2) /* pad pull-down enable */

LOCAL volatile S2_ENC enc[2];
LOCAL UW reg_inte; /* PROCx_INTE2 of the core that takes the interrupt */
LOCAL UW reg_ints; /* PROCx_INTS2 of the same core */

LOCAL void enc_reset(volatile S2_ENC *e)
{
    e->rise = 0;
    e->fall = 0;
    e->t_rise_us = 0;
    e->t_last_us = 0;
    e->period_us = 0;
    e->min_dt_us = 0xFFFFFFFFU;
    e->glitches = 0;
}

LOCAL void enc_edge(volatile S2_ENC *e, UW now, BOOL rising)
{
    if ((e->rise | e->fall) != 0)
    {
        UW dt = now - e->t_last_us;
        if (dt < e->min_dt_us)
        {
            e->min_dt_us = dt;
        }
        if (dt < S2_ENC_GLITCH_US)
        {
            e->glitches++;
        }
    }
    e->t_last_us = now;

    if (rising)
    {
        if (e->rise != 0)
        {
            e->period_us = now - e->t_rise_us;
        }
        e->t_rise_us = now;
        e->rise++;
    }
    else
    {
        e->fall++;
    }
}

/* Interrupt context: no tm_printf, no waiting, keep it short. */
LOCAL void enc_inthdr(UINT intno)
{
    UW ev, now;

    (void)intno;

    /* The SDK handler does the same: clear the source, never the NVIC bit. */
    while ((ev = in_w(reg_ints) & ENC_EVENTS) != 0)
    {
        out_w(IO_INTR2, ev);        /* write 1 to clear a latched edge */
        now = in_w(TIMER_TIMERAWL); /* raw read: does not latch the high word */

        if (ev & EV_RISE(S2_ENC_LEFT_PIN))
            enc_edge(&enc[S2_ENC_LEFT], now, TRUE);
        if (ev & EV_FALL(S2_ENC_LEFT_PIN))
            enc_edge(&enc[S2_ENC_LEFT], now, FALSE);
        if (ev & EV_RISE(S2_ENC_RIGHT_PIN))
            enc_edge(&enc[S2_ENC_RIGHT], now, TRUE);
        if (ev & EV_FALL(S2_ENC_RIGHT_PIN))
            enc_edge(&enc[S2_ENC_RIGHT], now, FALSE);
    }
}

EXPORT ER s2_enc_init(void)
{
    const T_DINT dint = {
        .intatr = TA_HLNG,
        .inthdr = (FP)enc_inthdr,
    };
    const UINT pins[2] = {S2_ENC_LEFT_PIN, S2_ENC_RIGHT_PIN};
    INT core = 0;
    INT i;
    ER er;

#if TK_SUPPORT_SMP
    core = tk_get_prc() - 1; /* tk_get_prc() counts from 1 */
#endif
    reg_inte = core ? IO_PROC1_INTE2 : IO_PROC0_INTE2;
    reg_ints = core ? IO_PROC1_INTS2 : IO_PROC0_INTS2;

    for (i = 0; i < 2; i++)
    {
        gpio_set_pin(pins[i], GPIO_MODE_IN);
        clr_w(GPIO(pins[i]), PAD_PDE); /* the pad resets with a pull-down on */
        set_w(GPIO(pins[i]), PAD_PUE); /* needed if the encoder output is open-collector */
        enc_reset(&enc[i]);
    }
    clr_w(reg_inte, ENC_EVENTS);
    out_w(IO_INTR2, ENC_EVENTS);

    er = tk_def_int(INTNO_IO_BANK0, &dint);
    if (er < E_OK)
    {
        return er;
    }
    ClearInt(INTNO_IO_BANK0);
    set_w(reg_inte, ENC_EVENTS);
    EnableInt(INTNO_IO_BANK0, ENC_INT_LEVEL); /* enables it on THIS core's NVIC */

    tm_printf((UB *)"encoder: GP%d/GP%d edge interrupt on core %d\n",
              S2_ENC_LEFT_PIN, S2_ENC_RIGHT_PIN, core);
    return E_OK;
}

EXPORT void s2_enc_clear(void)
{
    clr_w(reg_inte, ENC_EVENTS); /* no edges while the counters are zeroed */
    enc_reset(&enc[S2_ENC_LEFT]);
    enc_reset(&enc[S2_ENC_RIGHT]);
    out_w(IO_INTR2, ENC_EVENTS);
    set_w(reg_inte, ENC_EVENTS);
}

EXPORT void s2_enc_get(INT wheel, S2_ENC *out)
{
    const volatile S2_ENC *e = &enc[wheel];

    /* 32-bit reads are atomic on the M0+; the fields may differ by one tick while turning. */
    out->rise = e->rise;
    out->fall = e->fall;
    out->t_rise_us = e->t_rise_us;
    out->t_last_us = e->t_last_us;
    out->period_us = e->period_us;
    out->min_dt_us = e->min_dt_us;
    out->glitches = e->glitches;
}
