/** @file task_probe.c
 * @brief Opt-in GP27 following experiment. No overall timer/distance limit.
 *
 * Runs on core 1, the owner of the ADC snapshot. S5 tasks are excluded
 * from these images so there is exactly one motion/button controller.
 */
#include "s3_experiment.h"

#if ((S3_EXPERIMENT_MODE == 1) || (S3_EXPERIMENT_MODE == 2))
#include "../subsystem2/subsystem2.h"
#include "../subsystem5/drv_button.h"
#include "drv_adc.h"
#include "s3_private.h"
#include "s3_config.h"
#include "s3_probe.h"

#define PROBE_POLL_MS (20u)
#define PROBE_LOG_MS (100u)
#define PROBE_LATE_MS (120u)
#define PROBE_STILL_MS (2000u)

static void probe_task(INT stacd, void * p_exinf);
static void calibrate(s3_ir_cal_t * p_left, s3_ir_cal_t * p_right);
static void sample(s3_ir_cal_t * p_left, s3_ir_cal_t * p_right,
                    s3_probe_input_t * p_in);
static bool apply(s3_probe_output_t const * p_out);
static char const * state_name(s3_probe_state_t state);
static void log_run(s3_probe_t const * p, s3_probe_input_t const * p_in);

ER
task_probe_start (void)
{
    ID tid = app_start_task((FP) probe_task, S3_PRI_LINE, TP_PRC2,
                            "s3_probe");

    return (tid < E_OK) ? tid : E_OK;
}

static void
probe_task (INT stacd, void * p_exinf)
{
    static s3_ir_cal_t left;
    static s3_ir_cal_t right;
    s3_probe_t probe = {0};
    s3_probe_input_t in = {0};
    s3_probe_output_t out;
    uint32_t last_ms;
    uint32_t log_ms;
    uint32_t moved_ms;
    uint32_t sample_idx;
    int32_t last_mm;
    s3_probe_state_t previous;
    bool b_countdown_ok;

    (void) stacd;
    (void) p_exinf;
    drv_button_init();
    CHECK(subsystem2_stop(), "probe boot STOP");
    CHECK(tk_dly_tsk(3000), "probe USB settle");
    tm_printf((UB *) "\nEXPERIMENT %d: GP27 LEFT follows; GP26 RIGHT junction.\n"
                     "S5 mission/buttons/ultrasonic DISABLED. Clear floor only.\n"
                     "GP20 start/STOP. GP21 arm ONE LEFT turn (image 2).\n",
              (INT) S3_EXPERIMENT_MODE);
    calibrate(&left, &right);
    for (;;)
    {
        tm_printf((UB *) "\nREADY: put GP27 on black, GP26 on white.\n"
                         "Tap/release GP20 to start; hold GP21 to recalibrate.\n");
        for (;;)
        {
            CHECK(tk_dly_tsk(PROBE_POLL_MS), "probe wait");
            sample(&left, &right, &in);
            if (DRV_BUTTON_HELD ==
                drv_button_update(DRV_BUTTON_CAL, PROBE_POLL_MS))
            {
                calibrate(&left, &right);
            }
            if (DRV_BUTTON_PRESSED ==
                drv_button_update(DRV_BUTTON_START, PROBE_POLL_MS))
            {
                break;
            }
        }
        if (in.b_estop || (in.left < S3_PROBE_BLACK) ||
            (in.right >= S3_PROBE_LOSS))
        {
            tm_printf((UB *) "START REFUSED: L=%d R=%d estop=%d. "
                             "Reposition; power-cycle for a latched E-stop.\n",
                      (INT) in.left, (INT) in.right, (INT) in.b_estop);
            continue;
        }
        b_countdown_ok = true;
        for (uint32_t count = 3u; count > 0u; count--)
        {
            tm_printf((UB *) "STARTING IN %d; GP20 cancels.\n", (INT) count);
            for (uint32_t tick = 0u; tick < 50u; tick++)
            {
                CHECK(tk_dly_tsk(PROBE_POLL_MS), "probe countdown");
                if (drv_button_is_down(DRV_BUTTON_START))
                {
                    b_countdown_ok = false;
                }
            }
        }
        sample(&left, &right, &in);
        if ((!b_countdown_ok) || in.b_estop ||
            (in.left < S3_PROBE_BLACK) || (in.right >= S3_PROBE_LOSS))
        {
            tm_printf((UB *) "START CANCELLED: button, sensors or E-stop.\n");
            continue;
        }
        drv_button_init();
        s3_probe_start(&probe, &in, 2 == S3_EXPERIMENT_MODE);
        out = (s3_probe_output_t) {S3_PROBE_DRIVE, 0};
        if (!apply(&out))
        {
            continue;
        }
        tm_printf((UB *) "RUN: 40 mm/s; no overall timer/distance stop.\n"
                         "LOG: state armed L R dist heading done "
                         "targetL targetR speedL speedR\n");
        last_ms = in.now_ms;
        log_ms = last_ms;
        moved_ms = last_ms;
        last_mm = in.dist_mm;
        sample_idx = drv_adc_sample_index();
        while ((S3_PROBE_STOPPED != probe.state) &&
               (S3_PROBE_FAULT != probe.state))
        {
            CHECK(tk_dly_tsk(PROBE_POLL_MS), "probe run");
            sample(&left, &right, &in);
            in.b_stop = drv_button_is_down(DRV_BUTTON_START);
            in.b_arm = DRV_BUTTON_PRESSED ==
                       drv_button_update(DRV_BUTTON_CAL, PROBE_POLL_MS);
            previous = probe.state;
            if (in.dist_mm != last_mm)
            {
                moved_ms = in.now_ms;
                last_mm = in.dist_mm;
            }
            if (((in.now_ms - last_ms) > PROBE_LATE_MS) ||
                (sample_idx == drv_adc_sample_index()) ||
                (((S3_PROBE_FOLLOW == probe.state) ||
                  (S3_PROBE_ALIGN == probe.state)) &&
                 ((in.now_ms - moved_ms) > PROBE_STILL_MS)))
            {
                tm_printf((UB *) "FAULT: task late, ADC stopped, or no travel.\n");
                subsystem2_estop_set();
                in.b_estop = true;
            }
            sample_idx = drv_adc_sample_index();
            s3_probe_step(&probe, &in, &out);
            subsystem2_steer_set(0);
            if (!apply(&out))
            {
                probe.state = S3_PROBE_FAULT;
                probe.reason = S3_PROBE_ESTOP;
            }
            if ((previous != probe.state) ||
                ((in.now_ms - log_ms) >= PROBE_LOG_MS) || in.b_arm)
            {
                log_run(&probe, &in);
                log_ms = in.now_ms;
            }
            if ((S3_PROBE_FOLLOW != previous) &&
                (S3_PROBE_FOLLOW == probe.state))
            {
                moved_ms = in.now_ms;
            }
            last_ms = in.now_ms;
        }
        CHECK(subsystem2_stop(), "probe final STOP");
        tm_printf((UB *) "STOP reason=%d: 0 button, 1 E-stop, 2 timeout, "
                         "3 no line, 4 line lost aligning, 5 motion bound.\n",
                  (INT) probe.reason);
        while (drv_button_is_down(DRV_BUTTON_START))
        {
            CHECK(tk_dly_tsk(PROBE_POLL_MS), "probe release");
        }
        drv_button_init();
    }
}

static void
calibrate (s3_ir_cal_t * p_left, s3_ir_cal_t * p_right)
{
    uint16_t raw_right;
    uint16_t raw_left;
    uint32_t polls = 0u;
    bool b_ready;

    CHECK(subsystem2_stop(), "probe calibrate STOP");
    for (uint32_t count = 10u; count > 0u; count--)
    {
        tm_printf((UB *) "CAL GET READY %d; wait for SWEEP NOW.\n", (INT) count);
        CHECK(tk_dly_tsk(1000), "probe cal countdown");
    }
    s3_ir_cal_init(p_left);
    s3_ir_cal_init(p_right);
    drv_button_init();
    tm_printf((UB *) "\nSWEEP NOW: move BOTH sensors over white and black "
                     "at running height. GP20 finishes calibration.\n");
    for (;;)
    {
        CHECK(tk_dly_tsk(PROBE_POLL_MS), "probe calibrate");
        drv_adc_ir_read(&raw_right, &raw_left);
        s3_ir_cal_learn(p_left, s3_ir_cal_filter(p_left, raw_left));
        s3_ir_cal_learn(p_right, s3_ir_cal_filter(p_right, raw_right));
        b_ready = (0u != p_left->valid) && (0u != p_right->valid) &&
                  (p_left->black >= S3_DRIVE_CAL_BLACK_MIN) &&
                  (p_right->black >= S3_DRIVE_CAL_BLACK_MIN);
        if ((0u == (polls++ % 100u)) ||
            drv_button_is_down(DRV_BUTTON_START))
        {
            tm_printf((UB *) "CAL L %d..%d R %d..%d ready=%d. "
                             "Tap GP20 when ready.\n",
                      (INT) p_left->white, (INT) p_left->black,
                      (INT) p_right->white, (INT) p_right->black, (INT) b_ready);
        }
        if (DRV_BUTTON_PRESSED ==
            drv_button_update(DRV_BUTTON_START, PROBE_POLL_MS))
        {
            if (b_ready)
            {
                tm_printf((UB *) "CAL LOCKED: levels frozen until recalibration.\n");
                break;
            }
            tm_printf((UB *) "CAL REFUSED: sweep both sensors fully.\n");
        }
    }
    drv_button_init();
}

static void
sample (s3_ir_cal_t * p_left, s3_ir_cal_t * p_right,
         s3_probe_input_t * p_in)
{
    uint16_t raw_right;
    uint16_t raw_left;
    subsystem2_odom_t odom;
    subsystem2_status_t status;

    drv_adc_ir_read(&raw_right, &raw_left);
    subsystem2_odom_get(&odom);
    subsystem2_status_get(&status);
    p_in->now_ms = s3_now_ms();
    p_in->left = s3_ir_cal_norm(p_left, raw_left);
    p_in->right = s3_ir_cal_norm(p_right, raw_right);
    p_in->dist_mm = odom.dist_mm;
    p_in->heading_cdeg = odom.heading_cdeg;
    p_in->done = status.done_count;
    p_in->b_idle = S2_STATE_IDLE == status.state;
    p_in->b_estop = subsystem2_estop_is_set();
    p_in->b_stop = false;
    p_in->b_arm = false;
}

static bool
apply (s3_probe_output_t const * p_out)
{
    ER result = E_OK;

    switch (p_out->action)
    {
        case S3_PROBE_STOP:
            result = subsystem2_stop();
        break;
        case S3_PROBE_DRIVE:
            subsystem2_steer_set(0);
            result = subsystem2_follow(S3_PROBE_SPEED_MMS);
        break;
        case S3_PROBE_ROTATE:
            result = subsystem2_turn(p_out->angle_deg, S3_PROBE_TURN_MMS);
        break;
        default:
        break;
    }
    if (E_OK != result)
    {
        subsystem2_estop_set();
        tm_printf((UB *) "FAULT: motion command %d failed (%d); E-stop set.\n",
                  (INT) p_out->action, (INT) result);
    }
    return E_OK == result;
}

static char const *
state_name (s3_probe_state_t state)
{
    switch (state)
    {
        case S3_PROBE_STOPPED: return "STOPPED";
        case S3_PROBE_FOLLOW: return "FOLLOW";
        case S3_PROBE_BRAKE: return "BRAKE";
        case S3_PROBE_SEARCH: return "SEARCH";
        case S3_PROBE_ALIGN: return "ALIGN";
        case S3_PROBE_TURN: return "LEFT90";
        case S3_PROBE_TURN_BRAKE: return "TURN_BRAKE";
        default: return "FAULT";
    }
}

static void
log_run (s3_probe_t const * p, s3_probe_input_t const * p_in)
{
    subsystem2_status_t status;
    subsystem2_odom_t odom;

    subsystem2_status_get(&status);
    subsystem2_odom_get(&odom);
    tm_printf((UB *) "PROBE %s arm=%d L=%d R=%d mm=%d hdg=%d done=%d "
                     "tgt=%d,%d spd=%d,%d\n",
              state_name(p->state), (INT) p->armed, (INT) p_in->left,
              (INT) p_in->right, (INT) p_in->dist_mm,
              (INT) p_in->heading_cdeg, (INT) p_in->done,
              (INT) status.target_l_mms, (INT) status.target_r_mms,
              (INT) odom.speed_l_mms, (INT) odom.speed_r_mms);
}
#endif
