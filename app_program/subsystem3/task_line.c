/** @file task_line.c
 *
 * @brief Subsystem 3 -- task_line: calibration, PD steering, junction and
 *        line lost, every S3_LINE_PERIOD_MS.
 *
 * @par Timing
 * A cyclic handler wakes the task every 10 ms, so the period is fixed
 * (tk_dly_tsk(10) would give 10 ms + run time + one tick, about 20 ms).
 * If the task overruns, the extra wake-up is cancelled so it never runs
 * twice in a row.
 *
 * @par Calibration relax
 * Calibration can always WIDEN (every step), but it only SHRINKS while
 * line following is off (turn, bypass, search). While following, the
 * Centre rides the edge and never sees pure white, so relaxing then
 * dragged "white" up towards the edge value (bench log 26 Oct: 581 ->
 * 1199 counts in about 4 s).
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"

#include "drv_adc.h"
#include "drv_ir_centre.h"
#include "drv_ir_left.h"
#include "s3_api.h"
#include "s3_config.h"
#include "s3_line.h"
#include "s3_private.h"
#include "task_line.h"

#define US_PER_MS       (1000u)
#define UM_PER_MM       (1000u)
#define PERMILLE        (1000)
#define SEQ_MASK        (0xFFu)
#define C_WHITE_AT_L    (0xFFFFu)   /* s3_line_t.c_black_at_l: Centre was */
                                    /* white when L turned black.         */

static ID g_tid_line;
static ID g_cyc_line;

static void         line_task(INT stacd, void * p_exinf);
static void         line_cyclic(void * p_exinf);
static uint32_t     moved_um_get(uint32_t dt_us);
static void         requests_apply(s3_line_t * p_ln);
static void         raw_read(uint16_t * p_left, uint16_t * p_centre);
static s3_ln_mode_t mode_from_flags(UINT pattern);
static void         events_publish(s3_line_out_t const * p_out);
static void         junction_debug(s3_line_t const * p_ln,
                                   s3_line_out_t const * p_out);
static void         status_print(s3_line_t const * p_ln,
                                 s3_line_out_t const * p_out,
                                 int32_t dv, uint32_t state);
static void         timing_report(uint32_t t_start_us, uint32_t dt_us);
#if (1 == S3_LINE_VERBOSE)
static char const * zone_txt(int32_t n_permille);
#endif

/*!
 * @brief Create task_line and its 10 ms cyclic handler.
 *
 * NOTE: Call from a task on core 1 after drv_adc_init(): raw_read()
 * disables interrupts on its own core to read both sensors together.
 */
ER
task_line_start (void)
{
    T_CCYC ccyc = {0};

    g_tid_line = app_start_task((FP) line_task, S3_PRI_LINE, TP_PRC2,
                                "s3_line_task");
    if (g_tid_line < E_OK)
    {
        return g_tid_line;
    }
    ccyc.cycatr = TA_HLNG | TA_STA;
    ccyc.cychdr = (FP) line_cyclic;
    ccyc.cyctim = S3_LINE_PERIOD_MS;
    ccyc.cycphs = S3_LINE_PERIOD_MS;
    g_cyc_line  = tk_cre_cyc(&ccyc);
    return (g_cyc_line < E_OK) ? g_cyc_line : E_OK;
}

static void
line_cyclic (void * p_exinf)
{
    (void) p_exinf;
    (void) tk_wup_tsk(g_tid_line);
}

static void
line_task (INT stacd, void * p_exinf)
{
    static s3_line_t ln;            /* Static: keeps the stack small.   */
    s3_line_out_t    out;
    s3_line_report_t rep = {0};
    T_RFLG           rflg;
    uint32_t         t_prev   = in_w(TIMER_TIMERAWL);
    uint32_t         relax_ms = 0u;
    uint32_t         seq      = 0u;
    uint32_t         t_now;
    uint32_t         dt_us;
    uint32_t         state;
    uint16_t         raw_l;
    uint16_t         raw_c;
    int32_t          dv;

    (void) stacd;
    (void) p_exinf;
    s3_line_init(&ln, S3_EDGE_SIDE);

    for (;;)
    {
        (void) tk_slp_tsk(TMO_FEVR);        /* Woken by line_cyclic.     */
        (void) tk_can_wup(TSK_SELF);        /* Overran: do not run twice. */
        t_now  = in_w(TIMER_TIMERAWL);
        dt_us  = t_now - t_prev;
        t_prev = t_now;

        /* IF-07: what the Vehicle Controller wants. Acts only when the
         * REQUESTED mode changes, so a found line is not re-searched
         * while LF_SEARCH_x is still set. */
        if (E_OK == tk_ref_flg(flg_line, &rflg))
        {
            s3_line_request_mode(&ln, mode_from_flags(rflg.flgptn));
        }
        requests_apply(&ln);

        raw_read(&raw_l, &raw_c);
        s3_line_step(&ln, raw_l, raw_c, moved_um_get(dt_us), dt_us, &out);
        timing_report(t_now, dt_us);

        relax_ms += dt_us / US_PER_MS;
        if (relax_ms >= S3_CAL_RELAX_MS)
        {
            relax_ms = 0u;
            if ((S3_LN_DISABLED == out.state) || (S3_LN_SEARCH == out.state))
            {
                s3_ir_cal_relax(&ln.cal_l);
                s3_ir_cal_relax(&ln.cal_c);
            }
        }

        /* IF-02 steer_sp: permille -> mm/s, 10-bit signed, one store. */
        state = (0u == out.calibrated) ? S3_LINE_UNCAL : (uint32_t) out.state;
        dv    = s3_line_dv(&out, s3_steer_max_get());   /* 10-bit clamp. */
        seq = (seq + 1u) & SEQ_MASK;
        s3_steer_publish(dv, seq, (S3_LINE_FOLLOW == state) ||
                                  (S3_LINE_JUNCTION == state) ||
                                  (S3_LINE_LOST == state));

        rep.steer_dv       = dv;
        rep.pos            = out.pos;
        rep.state          = state;
        rep.seq            = seq;
        rep.n_left         = out.n_l;
        rep.n_centre       = out.n_c;
        rep.cal.white_o    = (uint16_t) ln.cal_l.white;
        rep.cal.black_o    = (uint16_t) ln.cal_l.black;
        rep.cal.white_c    = (uint16_t) ln.cal_c.white;
        rep.cal.black_c    = (uint16_t) ln.cal_c.black;
        rep.overshoot      = out.overshoot;
        rep.junction_event = out.junction_event;
        rep.lost_event     = out.lost_event;
        s3_status_set_line(&rep);

        events_publish(&out);
        junction_debug(&ln, &out);
        status_print(&ln, &out, dv, state);
    }
}

/*!
 * @brief Distance moved this step: S2's odometry once it is handed over,
 *        else the assumed speed.
 */
static uint32_t
moved_um_get (uint32_t dt_us)
{
    static uint32_t odom_prev;
    static bool     b_odom_started;
    uint32_t        odom_now;
    uint32_t        moved_um = 0u;
    int32_t         d_mm;

    if (!s3_odom_read(&odom_now))
    {
        return (S3_ASSUMED_SPEED_MMS * dt_us) / US_PER_MS;
    }
    d_mm = (int32_t) (odom_now - odom_prev);  /* Reversing / reset: <= 0. */
    if (b_odom_started && (d_mm > 0) && (d_mm < S3_ODOM_MAX_STEP_MM))
    {
        moved_um = (uint32_t) d_mm * UM_PER_MM;
    }
    odom_prev      = odom_now;
    b_odom_started = true;
    return moved_um;
}

/*!
 * @brief Apply gain and recalibration requests from other tasks
 *        (IF-S3e), so s3_line_t is only touched by this task.
 */
static void
requests_apply (s3_line_t * p_ln)
{
    int32_t kp    = 0;
    int32_t kd    = 0;
    int32_t boost = 0;

    if (s3_gains_request_take(&kp, &kd))
    {
        s3_line_set_gains(p_ln, kp, kd);
        tm_printf((UB *) "S3 TUNE kp=%d kd=%d (x100)\n",
                  (INT) p_ln->kp_x100, (INT) p_ln->kd_x100);
    }
    if (s3_shape_request_take(&boost))
    {
        s3_line_set_shape(p_ln, boost);
        tm_printf((UB *) "S3 TUNE curve boost=%d (x100)\n",
                  (INT) p_ln->boost_x100);
    }
    if (s3_recal_request_take())
    {
        s3_line_recalibrate(p_ln);
        tm_printf((UB *) "S3 RECALIBRATE: sweep both sensors over white "
                         "and black\n");
    }
}

/*!
 * @brief Both sensors from the SAME 10 ms sample: the sampler interrupt
 *        runs on this core, so it cannot run between the two reads.
 */
static void
raw_read (uint16_t * p_left, uint16_t * p_centre)
{
    UINT imask;

    DI(imask);
    *p_left   = drv_ir_left_raw();
    *p_centre = drv_ir_centre_raw();
    EI(imask);
}

static s3_ln_mode_t
mode_from_flags (UINT pattern)
{
    if (0u != (pattern & LF_SEARCH_L))
    {
        return S3_LN_MODE_SEARCH_FROM_LEFT;
    }
    if (0u != (pattern & LF_SEARCH_R))
    {
        return S3_LN_MODE_SEARCH_FROM_RIGHT;
    }
    return (0u != (pattern & LF_ENABLE)) ? S3_LN_MODE_FOLLOW :
                                           S3_LN_MODE_DISABLED;
}

/*!
 * @brief IF-S3c events to the Vehicle Controller (and the console).
 */
static void
events_publish (s3_line_out_t const * p_out)
{
    if (0u != p_out->junction_event)
    {
        (void) tk_set_flg(flg_s3, FLG_S3_JUNCTION);
    }
    if (0u != p_out->lost_event)
    {
        (void) tk_set_flg(flg_s3, FLG_S3_LOST);
    }
    if (0u != p_out->line_found_event)
    {
        (void) tk_set_flg(flg_s3, FLG_S3_LINE_FOUND);
    }
#if (1 == S3_LINE_VERBOSE)
    if (0u != p_out->junction_event)
    {
        tm_printf((UB *) "S3 EVENT JUNCTION\n");
    }
    if (0u != p_out->lost_event)
    {
        tm_printf((UB *) "S3 EVENT LINE LOST\n");
    }
    if (0u != p_out->line_found_event)
    {
        tm_printf((UB *) "S3 EVENT LINE FOUND\n");
    }
#endif
}

/*!
 * @brief S3_JUNC_DEBUG: one line per pass of the Left IR over black,
 *        showing how close it came to the junction rules in s3_line.c
 *        (Centre black at most S3_LN_JUNC_LEAD_MM before, both black for
 *        at least S3_LN_JUNC_MM). Only while really tracking.
 */
static void
junction_debug (s3_line_t const * p_ln, s3_line_out_t const * p_out)
{
#if (1 == S3_JUNC_DEBUG)
    static bool     b_outer_black;
    static uint32_t both_max_um;
    static uint32_t c_before_um;

    if ((0u == p_out->calibrated) ||
        ((S3_LN_FOLLOW != p_out->state) && (S3_LN_JUNCTION != p_out->state)))
    {
        b_outer_black = false;
    }
    else if (p_out->n_l > S3_LN_BLACK)
    {
        if (!b_outer_black)
        {
            b_outer_black = true;
            both_max_um   = 0u;
            c_before_um   = p_ln->c_black_at_l;
        }
        if (p_ln->both_black_um > both_max_um)
        {
            both_max_um = p_ln->both_black_um;
        }
    }
    else if (b_outer_black)
    {
        b_outer_black = false;
        if (C_WHITE_AT_L == c_before_um)
        {
            tm_printf((UB *) "S3 JDBG outer black: Centre was WHITE -> "
                             "outer on the line (drift), not a junction\n");
        }
        else
        {
            tm_printf((UB *) "S3 JDBG outer black: Centre black %d mm before "
                             "(max %d), both black %d mm (need %d)\n",
                      (INT) (c_before_um / UM_PER_MM),
                      (INT) S3_LN_JUNC_LEAD_MM,
                      (INT) (both_max_um / UM_PER_MM), (INT) S3_LN_JUNC_MM);
        }
    }
    else
    {
        ;   /* Outer sensor not on black. */
    }
#else
    (void) p_ln;
    (void) p_out;
#endif
}

/*!
 * @brief S3_LINE_VERBOSE: one status line every S3_STATUS_PRINT_MS, e.g.
 *   S3 LINE  C=EDGE   512  outer=WHITE   12  -> turn RIGHT    40 mm/s
 *   FOLLOW   cal C 150-2500  O 130-3300
 */
static void
status_print (s3_line_t const * p_ln, s3_line_out_t const * p_out,
              int32_t dv, uint32_t state)
{
#if (1 == S3_LINE_VERBOSE)
    static uint32_t last_ms;
    uint32_t        now_ms = s3_now_ms();
    char const *    p_turn = "STRAIGHT";

    if ((now_ms - last_ms) < S3_STATUS_PRINT_MS)
    {
        return;
    }
    last_ms = now_ms;
    if (dv > 0)
    {
        p_turn = "RIGHT   ";
    }
    else if (dv < 0)
    {
        p_turn = "LEFT    ";
    }
    else
    {
        ;   /* Straight. */
    }
    tm_printf((UB *) "S3 LINE  C=%s %4d  outer=%s %4d  -> turn %s %4d mm/s"
                     "  %s   cal C %d-%d  O %d-%d\n",
              zone_txt(p_out->n_c), (INT) p_out->n_c,
              zone_txt(p_out->n_l), (INT) p_out->n_l,
              p_turn, (INT) ((dv < 0) ? -dv : dv),
              ((0u != p_out->off_line) && (S3_LINE_FOLLOW == state)) ?
              ((p_out->steer > 0) ? "OFF LINE -> searching RIGHT" :
                                    "OFF LINE -> searching LEFT") :
              s3_state_txt(state, p_out->overshoot),
              (INT) p_ln->cal_c.white, (INT) p_ln->cal_c.black,
              (INT) p_ln->cal_l.white, (INT) p_ln->cal_l.black);
#else
    (void) p_ln;
    (void) p_out;
    (void) dv;
    (void) state;
#endif
}

/*!
 * @brief S3_TIMING: every S3_TIMING_PRINT_MS, print the longest line step
 *        (wake-up to after s3_line_step), the longest and shortest period
 *        seen, and the longest sampler interrupt. Evidence for the
 *        Week 6 timing claims (line_task about 1 % of core 1). The 1 us
 *        timer reads mean a value of N is N to N + 1 us.
 *
 * @param[in] t_start_us  Timer value taken at wake-up.
 * @param[in] dt_us       Period since the previous wake-up.
 */
static void
timing_report (uint32_t t_start_us, uint32_t dt_us)
{
#if (1 == S3_TIMING)
    static uint32_t step_max_us;
    static uint32_t dt_max_us;
    static uint32_t dt_min_us = UINT32_MAX;
    static uint32_t last_ms;
    static uint32_t steps;
    uint32_t        run_us = in_w(TIMER_TIMERAWL) - t_start_us;
    uint32_t        now_ms;

    steps++;
    step_max_us = (run_us > step_max_us) ? run_us : step_max_us;
    if (steps > 1u)                     /* The first dt is from start-up. */
    {
        dt_max_us = (dt_us > dt_max_us) ? dt_us : dt_max_us;
        dt_min_us = (dt_us < dt_min_us) ? dt_us : dt_min_us;
    }
    now_ms = s3_now_ms();
    if ((now_ms - last_ms) < S3_TIMING_PRINT_MS)
    {
        return;
    }
    last_ms = now_ms;
    tm_printf((UB *) "S3 TIME line step max %d us, period %d-%d us, "
                     "sampler isr max %d us\n",
              (INT) step_max_us, (INT) dt_min_us, (INT) dt_max_us,
              (INT) drv_adc_isr_max_us_take());
    step_max_us = 0u;
    dt_max_us   = 0u;
    dt_min_us   = UINT32_MAX;
#else
    (void) t_start_us;
    (void) dt_us;
#endif
}

#if (1 == S3_LINE_VERBOSE)
/*!
 * @brief What one line sensor sees, with the same bands as s3_line.c.
 */
static char const *
zone_txt (int32_t n_permille)
{
    if (n_permille < S3_LN_WHITE)
    {
        return "WHITE";
    }
    if (n_permille > S3_LN_BLACK)
    {
        return "BLACK";
    }
    return "EDGE ";
}
#endif

/*** end of file ***/
