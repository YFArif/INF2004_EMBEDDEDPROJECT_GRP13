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
#include "s3_analog.h"
#include "s3_api.h"
#include "s3_code39.h"
#include "s3_config.h"
#include "s3_private.h"

#define REJECT_MIN_STRIPES  (9u)  /* A rejected pass counts only with at  */
                                  /* least one symbol's stripes; shorter  */
                                  /* passes are line-edge noise.          */
#define US_PER_MS       (1000u)
#define NAV_SEQ_MASK    (0xFu)
#define NAV_SRC_MASK    (0x3u)
#define ODOM_16_MASK    (0xFFFFu)

/** Which path a result came from (s3_nav_msg_t.source). */
/* Piezo buzzer on GP22 (PWM slice 3A; 3B is GP7, a plain GPIO input, so
 * it is unaffected). 125 MHz / 46 296 = 2.7 kHz, 50 % duty.
 * shortcut: relies on S2's motor init having taken PWM out of reset; if
 * not, the beep is silent (never harmful). */
#define BUZZER_PIN      (22u)
#define BUZZER_TOP      (46296u)

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
static void         nav_publish(char letter, uint8_t rev, uint8_t src);
static void         beep(void);
#if (1 == S3_BC_VERBOSE)
static void         result_print(s3_c39_result_t res, char letter);
static void         stripes_print(char const * p_tag,
                                  uint32_t const * p_dur, uint32_t count);
static char const * result_txt(s3_c39_result_t res);
static char const * opener_txt(uint32_t src);
#endif

/*!
 * @brief Create task_barcode on core 1.
 */
ER
task_barcode_start (void)
{
    /* The cast is safe: FP is the kernel's generic entry address type,
     * and the kernel calls the task with the arguments it declares. */
    ID h_tid = app_start_task((FP) barcode_task, S3_PRI_BARCODE, TP_PRC2,
                              "s3_barcode_task");

    return (h_tid < E_OK) ? h_tid : E_OK;
}

/*!
 * @brief The barcode task: decode each finished pass and publish its
 *        letter.
 *
 * @param[in] stacd   Start code; not used.
 * @param[in] p_exinf Extended information; not used.
 */
static void
barcode_task (INT stacd, void * p_exinf)
{
    char    letter;
    uint8_t rev;
    uint8_t src;

    /* The casts mark the arguments as unused. */
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
        letter = '?';
        rev    = 0u;
        src    = 0u;
#if (1 == S3_TIMING)
        {
            uint32_t start_us = in_w(TIMER_TIMERAWL);
            bool     b_ok     = pass_decode(&letter, &rev, &src);
            uint32_t run_us   = in_w(TIMER_TIMERAWL) - start_us;

            /* The casts are for tm_printf only: the format is an ASCII literal,
             * and each value is printed with %d, so a value outside INT's range
             * would only print wrong.
             */
            tm_printf((UB *) "S3 TIME barcode decode %d us (both paths)\n",
                      (INT) run_us);
            if (b_ok)
            {
                nav_publish(letter, rev, src);
            }
        }
#else
        if (pass_decode(&letter, &rev, &src))
        {
            nav_publish(letter, rev, src);
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
    drv_adc_ir_init();
    CHECK(drv_barcode_init(), "drv_barcode_init");
    CHECK(drv_adc_init(), "drv_adc_init");

    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "S3 RUN: barcode DOUT=GP%d AOUT=ADC%d, Left IR=ADC%d "
                     "(GP%d), Centre=ADC%d (GP%d), line sensors: %s\n",
              (INT) DRV_BARCODE_DOUT_PIN, (INT) DRV_BARCODE_ADC_CH,
              (INT) DRV_IR_LEFT_ADC_CH, (INT) DRV_IR_LEFT_PIN,
              (INT) DRV_IR_CENTRE_ADC_CH, (INT) DRV_IR_CENTRE_PIN,
              S3_LAYOUT_TXT);

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
    static uint32_t    an_dur[S3_AN_MAX_EXTREMA];   /* Static: stack.     */
    static uint32_t    dig_dur[S3_AN_MAX_EXTREMA];
    static uint32_t    dig_alt[S3_AN_MAX_EXTREMA];  /* Shifted retry.    */
    uint32_t           n_raw = 0u;
    uint32_t           n_dig = 0u;
    uint32_t           idx;
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
        n_raw = pass.n_edges - 1u;
        n_raw = (n_raw > S3_AN_MAX_EXTREMA) ? S3_AN_MAX_EXTREMA : n_raw;
        for (idx = 0u; idx < n_raw; idx++)
        {
            dig_dur[idx] = pass.p_dur[idx];
            dig_alt[idx] = pass.p_dur[idx];
        }
        n_dig = s3_merge_gaps(dig_dur, n_raw);  /* DOUT flicker in a bar. */
        res_d = s3_code39_decode(dig_dur, n_dig, &ch_d, &rev_d);

        /* Pass began on black: stripe 0 is white and the odd stripes are
         * bars, so the merge above can join away a narrow bar. Retry with
         * the whites counted from stripe 1. */
        if ((S3_C39_OK != res_d) && (n_raw > 1u) &&
            (S3_C39_OK == s3_code39_decode(dig_alt,
                              s3_merge_gaps(&dig_alt[1], n_raw - 1u) + 1u,
                              &ch_d, &rev_d)))
        {
            res_d = S3_C39_OK;
        }
    }

    /* Analogue path. */
    if (pass.n_win > 0u)
    {
        n_an  = s3_analog_widths(pass.p_win, pass.n_win, DRV_ADC_SAMPLE_US,
                                 an_dur, S3_AN_MAX_EXTREMA, &stats);
        res_a = s3_code39_decode(an_dur, n_an, &ch_a, &rev_a);
    }

#if (1 == S3_BC_VERBOSE)
    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "S3 PASS digital: edges=%d chatter=%d dropped=%d -> ",
              (INT) pass.n_edges, (INT) pass.glitches, (INT) pass.lost);
    result_print(res_d, ch_d);
    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "S3 PASS analog : window=%dms (opened by %s)%s "
                     "stripes=%d white=%d black=%d P=%d -> ",
              (INT) ((pass.n_win * DRV_ADC_SAMPLE_US) / US_PER_MS),
              opener_txt(pass.src),
              (pass.b_clipped) ? " TOO SLOW, start lost" : "",
              (INT) n_an, (INT) stats.min << 4, (INT) stats.max << 4,
              (INT) stats.prominence << 4);
    result_print(res_a, ch_a);
    /* Sensitivity: where DOUT switches between this pass's white and
     * black (low % = trips near white, chatters; high % = misses narrow
     * bars). Contrast = black - white seen by AOUT. */
    tm_printf((UB *) "S3 SENS contrast=%d (white %d black %d), DOUT switches "
                     "at %d = %d%% of the way to black (%d switches)\n",
              (INT) ((stats.max - stats.min) << 4), (INT) stats.min << 4,
              (INT) stats.max << 4, (INT) pass.dout_level << 4,
              (INT) (((stats.max > stats.min) && (pass.dout_n > 0u)) ?
                     ((((INT) pass.dout_level - (INT) stats.min) * 100) /
                      ((INT) stats.max - (INT) stats.min)) : -1),
              (INT) pass.dout_n);
    stripes_print("digital", dig_dur, n_dig);
    stripes_print("analog ", an_dur, n_an);
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
    if ((!b_ok) && ((n_dig >= REJECT_MIN_STRIPES) ||
                    (n_an >= REJECT_MIN_STRIPES)))
    {
        s3_status_barcode_reject();     /* Looked like a code: count it. */
    }
    return b_ok;
}

/*!
 * @brief Accepted barcode -> navigation command (IF-01), status, event
 *        flag. A second barcode within S3_BC_COOLDOWN_MS is ignored.
 */
static void
nav_publish (char letter, uint8_t rev, uint8_t src)
{
    static uint32_t seq;
    static uint32_t last_ms;
    static bool     b_have_last;
    s3_nav_msg_t    msg    = {0};
    uint32_t        now_ms = s3_now_ms();
    ER              err;

    if (b_have_last && ((now_ms - last_ms) < S3_BC_COOLDOWN_MS))
    {
        /* The casts are for tm_printf only: the format is an ASCII literal, and
         * each value is printed with %d, so a value outside INT's range would
         * only print wrong.
         */
        tm_printf((UB *) "S3 NAV ignored '%c' (cooldown)\n", letter);
        return;
    }
    b_have_last  = true;
    last_ms      = now_ms;
    msg.ch_idx   = s3_code39_index(letter);
    msg.reversed = (0u != rev) ? 1u : 0u;
    msg.source   = src & NAV_SRC_MASK;
    msg.odom_mm  = s3_odom_mm() & ODOM_16_MASK;
    msg.cmd      = (uint32_t) s3_nav_lookup(letter);   /* s3_nav_cmd_t values */
                                                   /* == s3_cmd_t values. */
    s3_status_set_barcode((uint8_t) msg.ch_idx, (uint8_t) msg.cmd,
                          (uint8_t) msg.reversed);

    /* FR5: a letter that is not in today's map is decoded and shown in
     * the status, but never becomes a command. It still starts the
     * cooldown, so the same stripes are not read twice. seq counts sent
     * messages only, so a gap at the receiver always means a drop. */
    if (S3_NAV_NONE == (s3_nav_cmd_t) msg.cmd)
    {
        tm_printf((UB *) "S3 NAV '%c'%s not in today's map: ignored\n", letter,
                  (0u != rev) ? " (read backwards)" : "");
        beep();                     /* Decoded, so beep even if unmapped. */
        return;
    }
    seq     = (seq + 1u) & NAV_SEQ_MASK;
    msg.seq = seq;

    err = tk_snd_mbf(gh_mbf_nav, &msg, (INT) sizeof(msg), TMO_POL);  /* Never */
                                                                 /* block. */
    (void) tk_set_flg(gh_flg_s3, FLG_S3_BARCODE);

    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "S3 NAV #%d '%c' -> %s%s%s\n", (INT) msg.seq, letter,
              s3_nav_name((s3_nav_cmd_t) msg.cmd),
              (0u != rev) ? " (read backwards)" : "",
              (E_OK == err) ? "" : "  [mbf_nav full: nobody is reading it]");
    beep();                         /* After the send: S5 is not delayed. */
}

/*!
 * @brief One S3_BC_BEEP_MS beep on GP22 (0 = off). Blocks task_barcode for
 *        that long only, which is fine: it is event-driven and the next
 *        barcode cannot come within S3_BC_COOLDOWN_MS.
 */
static void
beep (void)
{
    static bool b_ready;

    if (0u == S3_BC_BEEP_MS)
    {
        return;
    }
    if (!b_ready)
    {
        /* The casts discard the results: the buzzer pin and its PWM values are
         * valid constants.
         */
        (void) pwm_set_wrap(BUZZER_PIN, BUZZER_TOP);
        (void) pwm_set_cc(BUZZER_PIN, 0u);      /* Silent until a beep. */
        (void) pwm_set_pin(BUZZER_PIN);
        (void) pwm_set_enabled(BUZZER_PIN, TRUE);
        b_ready = true;
    }
    (void) pwm_set_cc(BUZZER_PIN, BUZZER_TOP / 2u);
    tk_dly_tsk(S3_BC_BEEP_MS);
    (void) pwm_set_cc(BUZZER_PIN, 0u);          /* Output held low.     */
}

#if (1 == S3_BC_VERBOSE)
/*!
 * @brief Print one decode result (S3_BC_VERBOSE builds only).
 */
static void
result_print (s3_c39_result_t res, char letter)
{
    if (S3_C39_OK == res)
    {
        /* The casts are for tm_printf only: the format is an ASCII literal, and
         * each value is printed with %d, so a value outside INT's range would
         * only print wrong.
         */
        tm_printf((UB *) "'%c'\n", letter);
    }
    else
    {
        /* The casts are for tm_printf only: the format is an ASCII literal, and
         * each value is printed with %d, so a value outside INT's range would
         * only print wrong.
         */
        tm_printf((UB *) "failed (%s)\n", result_txt(res));
    }
}

/*!
 * @brief Every stripe of one path, e.g. "S3 STRIPES analog : 29 = nb 75,
 *        tw 190, ...": n/t = narrow/thick, b/w = black/white, ms. Stripe 1
 *        is taken as black. Narrow/thick is a rough guide for comparing
 *        with the printed code (>= 1.6 x the shortest stripe of the same
 *        colour = thick), not the decoder's decision.
 */
static void
stripes_print (char const * p_tag, uint32_t const * p_dur, uint32_t count)
{
    uint32_t min_dur[2] = {UINT32_MAX, UINT32_MAX};
    uint32_t idx;

    for (idx = 0u; idx < count; idx++)
    {
        if (p_dur[idx] < min_dur[idx & 1u])
        {
            min_dur[idx & 1u] = p_dur[idx];
        }
    }
    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "S3 STRIPES %s: %d =", p_tag, (INT) count);
    for (idx = 0u; idx < count; idx++)
    {
        /* The casts are for tm_printf only: the format is an ASCII literal, and
         * each value is printed with %d, so a value outside INT's range would
         * only print wrong.
         */
        tm_printf((UB *) " %c%c %d%s",
                  ((p_dur[idx] * 10u) >= (min_dur[idx & 1u] * 16u)) ? 't' : 'n',
                  (0u == (idx & 1u)) ? 'b' : 'w',
                  (INT) (p_dur[idx] / US_PER_MS),
                  ((idx + 1u) < count) ? "," : "");
    }
    /* The casts are for tm_printf only: the format is an ASCII literal, and
     * each value is printed with %d, so a value outside INT's range would only
     * print wrong.
     */
    tm_printf((UB *) "\n");
}

/*!
 * @brief Short text for a decode result.
 */
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

/*!
 * @brief Which path opened a pass: AOUT, DOUT or both.
 */
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

