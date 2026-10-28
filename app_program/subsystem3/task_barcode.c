/** @file task_barcode.c
 *
 * @brief Subsystem 3 -- task_barcode: hardware set-up, then barcode
 *        decode, fusion and publishing.
 *
 * @par Set-up (once, on core 1)
 * Line sensors and barcode pins, S3's handler on S2's shared GPIO
 * interrupt, the 2 kHz sampler, then task_line (and the drive test if
 * S3_DRIVE_TEST is 1). RP2040 interrupt enables are per core, which is
 * why this task, pinned to core 1, does it.
 *
 * @par Fusion
 * Each finished pass is decoded twice, from the digital edge gaps and
 * from the analogue window (s3_analog_widths()), with the same Code 39
 * decoder. Both decode the same letter -> ACCEPT (both agree); only one
 * decodes -> ACCEPT, flagged single-source; both decode, letters differ
 * -> REJECT; neither decodes -> REJECT.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#include "drv_adc.h"
#include "drv_barcode.h"
#include "drv_ir_centre.h"
#include "drv_ir_left.h"
#include "s3_analog.h"
#include "s3_api.h"
#include "s3_code39.h"
#include "s3_config.h"
#include "s3_nav.h"
#include "s3_private.h"
#include "task_barcode.h"
#include "task_drive_test.h"
#include "task_line.h"

#define US_PER_MS       (1000u)
#define NAV_SEQ_MASK    (0xFu)
#define NAV_SRC_MASK    (0x3u)
#define ODOM_16_MASK    (0xFFFFu)

/** Which path a result came from (s3_nav_msg_t.source). */
#define SRC_BOTH        (1u)
#define SRC_ANALOG      (2u)
#define SRC_DIGITAL     (3u)

#if (1 == S3_BC_VERBOSE)
#define BC_LOG(...)     tm_printf(__VA_ARGS__)
#else
#define BC_LOG(...)     ((void) 0)
#endif

static void         barcode_task(INT stacd, void * p_exinf);
static void         hardware_start(void);
static bool         pass_decode(char * p_ch, uint8_t * p_rev, uint8_t * p_src);
static void         nav_publish(char ch, uint8_t rev, uint8_t src);
#if (1 == S3_BC_VERBOSE)
static void         result_print(s3_c39_result_t res, char ch);
static char const * result_txt(s3_c39_result_t res);
static char const * opener_txt(uint32_t src);
#endif

/*!
 * @brief Create task_barcode on core 1.
 */
ER
task_barcode_start (void)
{
    ID tid = app_start_task((FP) barcode_task, S3_PRI_BARCODE, TP_PRC2,
                            "s3_barcode_task");

    return (tid < E_OK) ? tid : E_OK;
}

static void
barcode_task (INT stacd, void * p_exinf)
{
    char    ch;
    uint8_t rev;
    uint8_t src;

    (void) stacd;
    (void) p_exinf;
    hardware_start();

    for (;;)
    {
        tk_dly_tsk(S3_BC_POLL_MS);
        if (!drv_barcode_pass_done())
        {
            continue;
        }
        ch  = '?';
        rev = 0u;
        src = 0u;
#if (1 == S3_TIMING)
        {
            uint32_t t0     = in_w(TIMER_TIMERAWL);
            bool     b_ok   = pass_decode(&ch, &rev, &src);
            uint32_t run_us = in_w(TIMER_TIMERAWL) - t0;

            tm_printf((UB *) "S3 TIME barcode decode %d us (both paths)\n",
                      (INT) run_us);
            if (b_ok)
            {
                nav_publish(ch, rev, src);
            }
        }
#else
        if (pass_decode(&ch, &rev, &src))
        {
            nav_publish(ch, rev, src);
        }
#endif
    }
}

/*!
 * @brief Pins, interrupts and the other S3 tasks. Errors are printed;
 *        the task carries on so the rest of the robot keeps running.
 */
static void
hardware_start (void)
{
    drv_ir_left_init();
    drv_ir_centre_init();
    CHECK(drv_barcode_init(), "drv_barcode_init");
    CHECK(drv_adc_init(), "drv_adc_init");

    tm_printf((UB *) "S3 RUN: barcode DOUT=GP%d AOUT=ADC%d, outer IR=ADC%d "
                     "(GP%d), Centre=ADC%d (GP%d), Centre rides the %s edge\n",
              (INT) DRV_BARCODE_DOUT_PIN, (INT) DRV_BARCODE_ADC_CH,
              (INT) DRV_IR_LEFT_ADC_CH, (INT) DRV_IR_LEFT_PIN,
              (INT) DRV_IR_CENTRE_ADC_CH, (INT) DRV_IR_CENTRE_PIN,
              S3_EDGE_TXT);

    CHECK(task_line_start(), "task_line_start");
    CHECK(task_drive_test_start(), "task_drive_test_start");
}

/*!
 * @brief Decode one finished pass both ways and fuse the results.
 *
 * @return true if accepted; then *p_ch, *p_rev and *p_src are set.
 */
static bool
pass_decode (char * p_ch, uint8_t * p_rev, uint8_t * p_src)
{
    static uint32_t    an_dur[S3_AN_MAX_EXTREMA];  /* Static: stack.     */
    drv_barcode_pass_t pass;
    s3_an_stats_t      stats = {0};
    s3_c39_result_t    res_d = S3_C39_BAD_COUNT;
    s3_c39_result_t    res_a = S3_C39_BAD_COUNT;
    char               ch_d  = '?';
    char               ch_a  = '?';
    uint8_t            rev_d = 0u;
    uint8_t            rev_a = 0u;
    uint32_t           n_an  = 0u;
    bool               b_ok  = false;

    drv_barcode_take_pass(&pass);

    /* Digital path: not trusted if edges were dropped. */
    if ((pass.n_edges >= 2u) && (0u == pass.lost))
    {
        res_d = s3_code39_decode(pass.p_dur, pass.n_edges - 1u, &ch_d, &rev_d);
    }

    /* Analogue path. */
    if (pass.n_win > 0u)
    {
        n_an  = s3_analog_widths(pass.p_win, pass.n_win, DRV_ADC_SAMPLE_US,
                                 an_dur, S3_AN_MAX_EXTREMA, &stats);
        res_a = s3_code39_decode(an_dur, n_an, &ch_a, &rev_a);
    }

#if (1 == S3_BC_VERBOSE)
    tm_printf((UB *) "S3 PASS digital: edges=%d chatter=%d dropped=%d -> ",
              (INT) pass.n_edges, (INT) pass.glitches, (INT) pass.lost);
    result_print(res_d, ch_d);
    tm_printf((UB *) "S3 PASS analog : window=%dms (opened by %s)%s "
                     "stripes=%d white=%d black=%d P=%d -> ",
              (INT) ((pass.n_win * DRV_ADC_SAMPLE_US) / US_PER_MS),
              opener_txt(pass.src),
              pass.b_clipped ? " TOO SLOW, start lost" : "",
              (INT) n_an, (INT) stats.min << 4, (INT) stats.max << 4,
              (INT) stats.prominence << 4);
    result_print(res_a, ch_a);
#endif

    if ((S3_C39_OK == res_d) && (S3_C39_OK == res_a))
    {
        if (ch_d == ch_a)
        {
            BC_LOG((UB *) "S3 RESULT ACCEPT '%c' (both agree)\n", ch_d);
            *p_ch  = ch_d;
            *p_rev = rev_a;
            *p_src = SRC_BOTH;
            b_ok   = true;
        }
        else
        {
            BC_LOG((UB *) "S3 RESULT REJECT (digital '%c' vs analog '%c' "
                          "disagree)\n", ch_d, ch_a);
        }
    }
    else if (S3_C39_OK == res_a)
    {
        BC_LOG((UB *) "S3 RESULT ACCEPT '%c' (analog only)\n", ch_a);
        *p_ch  = ch_a;
        *p_rev = rev_a;
        *p_src = SRC_ANALOG;
        b_ok   = true;
    }
    else if (S3_C39_OK == res_d)
    {
        BC_LOG((UB *) "S3 RESULT ACCEPT '%c' (digital only)\n", ch_d);
        *p_ch  = ch_d;
        *p_rev = rev_d;
        *p_src = SRC_DIGITAL;
        b_ok   = true;
    }
    else
    {
        BC_LOG((UB *) "S3 RESULT REJECT (neither decoded)\n");
    }
    return b_ok;
}

/*!
 * @brief Accepted barcode -> navigation command (IF-01), status, event
 *        flag. A second barcode within S3_BC_COOLDOWN_MS is ignored.
 */
static void
nav_publish (char ch, uint8_t rev, uint8_t src)
{
    static uint32_t seq;
    static uint32_t last_ms;
    static bool     b_have_last;
    s3_nav_msg_t    msg    = {0};
    uint32_t        now_ms = s3_now_ms();
    ER              er;

    if (b_have_last && ((now_ms - last_ms) < S3_BC_COOLDOWN_MS))
    {
        tm_printf((UB *) "S3 NAV ignored '%c' (cooldown)\n", ch);
        return;
    }
    b_have_last  = true;
    last_ms      = now_ms;
    msg.ch_idx   = s3_code39_index(ch);
    msg.reversed = (0u != rev) ? 1u : 0u;
    msg.source   = src & NAV_SRC_MASK;
    msg.odom_mm  = s3_odom_mm() & ODOM_16_MASK;
    msg.cmd      = (uint32_t) s3_nav_lookup(ch);   /* s3_nav_cmd_t values */
                                                   /* == s3_cmd_t values. */
    s3_status_set_barcode((uint8_t) msg.ch_idx, (uint8_t) msg.cmd,
                          (uint8_t) msg.reversed);

    /* FR5: a letter that is not in today's map is decoded and shown in
     * the status, but never becomes a command. It still starts the
     * cooldown, so the same stripes are not read twice. seq counts sent
     * messages only, so a gap at the receiver always means a drop. */
    if (S3_NAV_NONE == (s3_nav_cmd_t) msg.cmd)
    {
        tm_printf((UB *) "S3 NAV '%c'%s not in today's map: ignored\n", ch,
                  (0u != rev) ? " (read backwards)" : "");
        return;
    }
    seq     = (seq + 1u) & NAV_SEQ_MASK;
    msg.seq = seq;

    er = tk_snd_mbf(mbf_nav, &msg, (INT) sizeof(msg), TMO_POL);  /* Never */
                                                                 /* block. */
    (void) tk_set_flg(flg_s3, FLG_S3_BARCODE);

    tm_printf((UB *) "S3 NAV #%d '%c' -> %s%s%s\n", (INT) msg.seq, ch,
              s3_nav_name((s3_nav_cmd_t) msg.cmd),
              (0u != rev) ? " (read backwards)" : "",
              (E_OK == er) ? "" : "  [mbf_nav full: nobody is reading it]");
}

#if (1 == S3_BC_VERBOSE)
static void
result_print (s3_c39_result_t res, char ch)
{
    if (S3_C39_OK == res)
    {
        tm_printf((UB *) "'%c'\n", ch);
    }
    else
    {
        tm_printf((UB *) "failed (%s)\n", result_txt(res));
    }
}

static char const *
result_txt (s3_c39_result_t res)
{
    char const * p_txt;

    switch (res)
    {
        case S3_C39_OK:
            p_txt = "ok";
            break;
        case S3_C39_BAD_COUNT:
            p_txt = "wrong stripe count";
            break;
        case S3_C39_BAD_RATIO:
            p_txt = "wide/narrow unclear";
            break;
        case S3_C39_BAD_SYMBOL:
            p_txt = "not a Code 39 letter";
            break;
        default:
            p_txt = "no * start/stop";
            break;
    }
    return p_txt;
}

static char const *
opener_txt (uint32_t src)
{
    char const * p_txt = "-";

    if ((DRV_BARCODE_SRC_AOUT | DRV_BARCODE_SRC_DOUT) == src)
    {
        p_txt = "both";
    }
    else if (DRV_BARCODE_SRC_AOUT == src)
    {
        p_txt = "AOUT";
    }
    else if (DRV_BARCODE_SRC_DOUT == src)
    {
        p_txt = "DOUT";
    }
    else
    {
        ;   /* Nothing opened it. */
    }
    return p_txt;
}
#endif

/*** end of file ***/
