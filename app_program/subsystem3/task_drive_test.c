/** @file task_drive_test.c
 *
 * @brief Subsystem 3 -- TEST ONLY (S3_DRIVE_TEST 1): line following on
 *        Buddy 2's motors before the Vehicle Controller exists.
 *
 * @par How to use
 * Wait for the calibration countdown and SWEEP NOW banner. Sweep both
 * sensors across white and black, then wait for CALIBRATION READY.
 * Put both sensors on black, tap and release GP20, then wait for the
 * 3 s drive countdown. Any GP20 press while driving stops the car.
 * GP21: short press changes preset; hold 2 s restarts calibration.
 * Waiting prompts print only when readiness changes; wheel and sensor
 * diagnostics print during driving. Both
 * buttons are only read, with the pad pull-up, as in subsystem2_step.c.
 *
 * @par Stops by itself
 * Button 1, LINE LOST for S3_DRIVE_LOST_MM or S3_DRIVE_LOST_MS, no
 * odometry change for S3_DRIVE_STILL_MS (LOST is measured in distance, so
 * a car that stopped moving stayed in FOLLOW for ever: log 28 Oct), an S2
 * emergency stop (for example a wheel stall: duty >= 40 % and no encoder
 * tick for 300 ms latches MF_ESTOP and S2 then ignores every command), or
 * the distance limit. There is no elapsed-time run limit.
 *
 * @par S2 emergency stop: reported, NOT cleared
 * Buddy 2's reference (MOTOR_S3_REFERENCE.md section 10): "S3 must not
 * clear an E-stop just to make line-following tests run." If MF_ESTOP is
 * latched the run is refused with the S2 counters; power-cycle the car.
 * Steering is now limited to the base speed (preset steer_max), so the
 * inner wheel keeps rolling forward and should not stall.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem2/subsystem2.h"   /* follow / stop / E-stop.       */

#include "s3_api.h"
#include "s3_config.h"
#include "s3_private.h"

#if (1 == S3_DRIVE_TEST)

#define POLL_MS             (20u)
#define PRESS_MS            (60u)   /* Held this long = a real press.   */
#define CAL_PRINT_MS        (1000u) /* Check the levels this often.     */
#define SETTLE_MS           (100)
#define COUNTDOWN_STEP_MS   (1000)
#define AFTER_STOP_MS       (500)
#define CLEAR_WAIT_MS       (100)   /* Let motion_task take the CLEAR.  */
#define U8_WHITE_BELOW      (64u)   /* s3_line_status_t n_xxx: 0..255.  */
#define U8_BLACK_ABOVE      (191u)
#define PERMILLE            (1000u)
#define U8_FULL             (255u)

/** Preset: Kp x100, Kd x100, speed mm/s, steer_sp at full lock mm/s. */
typedef struct
{
    int16_t  kp_x100;
    int16_t  kd_x100;
    uint16_t speed_mms;
    uint16_t steer_max_mms;
} drive_preset_t;

/* v25 (28 Oct): gains for the CENTRED pair (both sensors on the line,
 * 12 mm apart). e = n_right - n_left runs -1000 ... +1000, so Kp 100
 * gives full lock when one sensor is white and the other black. Host
 * sweep: every set below finishes every track in the model (UNVERIFIED
 * on the car). Speed 40 mm/s and steer_max = speed throughout, so presets
 * differ ONLY in gains and can be compared directly. */
static drive_preset_t const g_presets[] =
{
    {100,   0,  40,  40},   /* 0: default.                              */
    { 75,   0,  40,  40},   /* 1: softer (if it weaves on straights).   */
    {150,   0,  40,  40},   /* 2: stronger (if it is late on corners).  */
    {100,  25,  40,  40},   /* 3: a little D (if it overshoots).        */
    {200,   0,  40,  40}    /* 4: much stronger (if it still runs wide).*/
};

#define N_PRESETS   (sizeof(g_presets) / sizeof(g_presets[0]))

/* End-of-run diagnosis limits (run_summary). */
#define DIAG_BOTH_BLACK_MIN_PCT (30u)   /* Both sensors on the line.    */
#define DIAG_BOTH_WHITE_MAX_PCT (10u)   /* Both off the line.           */
#define DIAG_FLIPS_PER_M_MAX    (20u)   /* Steering side changes per m. */
#define DIAG_WHEEL_ERR_MAX_MMS  (15u)   /* Mean |target - measured|.    */

/** Counted every poll of one run. */
typedef struct
{
    uint32_t polls;
    uint32_t both_black;
    uint32_t one_white;
    uint32_t both_white;
    uint32_t white_from;        /* moved_mm + 1 when both went white.   */
    uint32_t white_max_mm;      /* Longest both-white stretch.          */
    uint32_t flips;             /* dv changed sign.                     */
    int32_t  dv_prev;
    uint32_t wheel_err_sum;     /* Sum of |target - measured|, L + R.   */
} drive_stats_t;

static uint32_t g_preset = 0u;

static void drive_task(INT stacd, void * p_exinf);
static bool button1_down(void);
static bool button2_down(void);
static void button_input(uint32_t pin);
static void preset_next(void);
static void preset_apply(void);
static void wait_start(void);
static void cal_countdown(void);
static bool start_allowed(void);
static bool cal_ok(void);
static void cal_print(s3_cal_levels_t const * p_cal);
static bool cal_ready(s3_cal_levels_t const * p_cal);
static void cal_watch(bool * p_ready, bool * p_reported);
static bool estop_clear_ok(void);
static void drive_run(void);
static int32_t sensor_permille(s3_line_status_t const * p_st, bool b_left);
static char    zone_char(int32_t n_permille);
static void    run_log(uint32_t ms, uint32_t moved_mm,
                       s3_line_status_t const * p_st);
static void    stats_add(drive_stats_t * p_s, s3_line_status_t const * p_st,
                         uint32_t moved_mm);
static void    run_summary(drive_stats_t const * p_s, uint32_t moved_mm);

/*!
 * @brief Create the drive test task on core 0.
 */
ER
task_drive_test_start (void)
{
    ID tid = app_start_task((FP) drive_task, S3_PRI_DRIVE_TEST, TP_PRC1,
                            "s3_drive_task");

    return (tid < E_OK) ? tid : E_OK;
}

static void
drive_task (INT stacd, void * p_exinf)
{
    uint32_t i;

    (void) stacd;
    (void) p_exinf;
    button_input(S3_DRIVE_BTN1_PIN);
    button_input(S3_DRIVE_BTN2_PIN);
    tk_dly_tsk(SETTLE_MS);
    preset_apply();
    cal_countdown();

    for (;;)
    {
        wait_start();
        if (!cal_ok())
        {
            cal_countdown();        /* START before READY: calibrate.    */
            continue;
        }
        for (i = S3_DRIVE_GO_COUNT_S; i > 0u; i--)
        {
            tm_printf((UB *) ">>> STARTING IN %d ... release GP20 now\n",
                      (INT) i);
            tk_dly_tsk(COUNTDOWN_STEP_MS);
        }
        if (start_allowed() && estop_clear_ok())
        {
            drive_run();
        }
    }
}

/*!
 * @brief Countdown, then forget both calibration ranges: the sweep that
 *        follows sets them.
 */
static void
cal_countdown (void)
{
    uint32_t i;

    tm_printf((UB *) "\n================================================\n"
                     "  STEP 1: GET READY TO CALIBRATE - NOT DRIVING\n"
                     "================================================\n"
                     "Keep wheels on the floor. Wait for SWEEP NOW.\n"
                     "Calibration starts in %d s.\n",
              (INT) S3_DRIVE_CAL_COUNT_S);
    for (i = S3_DRIVE_CAL_COUNT_S; i > 0u; i--)
    {
        tm_printf((UB *) "S3 CAL %d\n", (INT) i);
        tk_dly_tsk(COUNTDOWN_STEP_MS);
    }
    s3_request_recalibrate();
    tk_dly_tsk(SETTLE_MS);
    tm_printf((UB *) "\n================================================\n"
                     "  STEP 2: SWEEP NOW - MOTORS NOT COMMANDED\n"
                     "================================================\n"
                     "Slide sideways across WHITE and BLACK repeatedly.\n"
                     "Keep wheels on the floor. Do not press START yet.\n"
                     "Wait for CALIBRATION READY below.\n");
}
/*!
 * @brief Only start from a calibrated sensor that can tell which side the
 *        line is on. UNCALIBRATED gets no steering (drives blind); LOST
 *        keeps its last correction (spins). "Both white" is refused: the
 *        line could be on either side.
 */
static bool
start_allowed (void)
{
    s3_line_status_t st = {0};

    if ((E_OK != s3_get_line_status(&st)) || (S3_LINE_FOLLOW != st.state))
    {
        tm_printf((UB *) "S3 DRIVE NOT STARTED: line state is %s. Sweep the "
                         "car sideways across the line until it is "
                         "calibrated, then put both sensors on the line.\n",
                  s3_state_txt((uint32_t) st.state, 0u));
        return false;
    }
    if ((st.n_centre < U8_WHITE_BELOW) && (st.n_outer < U8_WHITE_BELOW))
    {
        tm_printf((UB *) "S3 DRIVE NOT STARTED: neither sensor is on the line "
                         "(GP27=%d, GP26=%d of 1000). Put the car so both "
                         "sensors are over the black line.\n",
                  (INT) ((st.n_centre * PERMILLE) / U8_FULL),
                  (INT) ((st.n_outer * PERMILLE) / U8_FULL));
        return false;
    }
    return true;
}

/*!
 * @brief Both sensors swept far enough: black levels at least
 *        S3_DRIVE_CAL_BLACK_MIN. Prints the levels either way.
 */
static bool
cal_ok (void)
{
    s3_cal_levels_t cal = {0};

    s3_cal_levels_get(&cal);
    cal_print(&cal);
    if (cal_ready(&cal))
    {
        return true;
    }
    tm_printf((UB *) "S3 DRIVE NOT STARTED: need black >= %d and contrast "
                     ">= %d on BOTH sensors. Sweep white and black, or "
                     "hold GP21 for 2 s to recalibrate.\n",
              (INT) S3_DRIVE_CAL_BLACK_MIN, (INT) S3_IR_CAL_MIN_RANGE);
    return false;
}

static void
cal_print (s3_cal_levels_t const * p_cal)
{
    bool b_ready = cal_ready(p_cal);

    tm_printf((UB *) "S3 DRIVE CAL: GP26 white %d black %d, GP27 white %d "
                     "black %d (raw) -> %s\n",
              (INT) p_cal->white_o, (INT) p_cal->black_o,
              (INT) p_cal->white_c, (INT) p_cal->black_c,
              b_ready ? "READY" : "keep sweeping");
}

/*!
 * @brief Use the same contrast requirement as the line controller.
 */
static bool
cal_ready (s3_cal_levels_t const * p_cal)
{
    return (p_cal->black_o >= S3_DRIVE_CAL_BLACK_MIN) &&
           (p_cal->black_c >= S3_DRIVE_CAL_BLACK_MIN) &&
           (((int32_t) p_cal->black_o - p_cal->white_o) >=
            (int32_t) S3_IR_CAL_MIN_RANGE) &&
           (((int32_t) p_cal->black_c - p_cal->white_c) >=
            (int32_t) S3_IR_CAL_MIN_RANGE);
}

static void
cal_watch (bool * p_ready, bool * p_reported)
{
    s3_cal_levels_t cal = {0};
    bool            b_ready;

    s3_cal_levels_get(&cal);
    b_ready = cal_ready(&cal);
    if ((!*p_reported) || (b_ready != *p_ready))
    {
        cal_print(&cal);
        if (b_ready)
        {
            tm_printf((UB *) "\n================================================\n"
                             "  STEP 3: CALIBRATION READY - WAITING FOR YOU\n"
                             "================================================\n"
                             "Stop sweeping. Put BOTH sensors on BLACK.\n"
                             "Tap and RELEASE GP20 to start (3 s countdown).\n"
                             "GP21: tap = preset; hold 2 s = recalibrate.\n"
                             "No driving until you press GP20.\n");
        }
        else
        {
            tm_printf((UB *) "CALIBRATION NOT READY: keep sweeping BOTH "
                             "sensors across white and black.\n");
        }
        *p_ready    = b_ready;
        *p_reported = true;
    }
}

/*!
 * @brief Refuse to start while S2's MF_ESTOP is latched. S3 does not clear
 *        it (MOTOR_S3_REFERENCE.md section 10): power-cycle the car.
 */
static bool
estop_clear_ok (void)
{
    subsystem2_status_t s2 = {0};

    if (!subsystem2_estop_is_set())
    {
        return true;
    }
    subsystem2_status_get(&s2);
    tm_printf((UB *) "S3 DRIVE NOT STARTED: S2 E-STOP is latched (stalls=%d, "
                     "e-stops=%d). S3 may not clear it: power-cycle the car "
                     "(Buddy 2 rule).\n", (INT) s2.stall_count,
              (INT) s2.estop_count);
    return false;
}

/*!
 * @brief One run: FOLLOW until a stop condition, then stop the motors.
 *        Logs every S3_DRIVE_LOG_MS (run_log) and ends with a summary
 *        that points at the likely cause (run_summary).
 */
static void
drive_run (void)
{
    drive_preset_t const * p_set   = &g_presets[g_preset];
    s3_line_status_t       st      = {0};
    subsystem2_status_t    s2      = {0};
    drive_stats_t          stats   = {0};
    uint32_t               start_mm  = s3_odom_mm();
    uint32_t               moved_mm  = 0u;
    uint32_t               ms        = 0u;
    uint32_t               log_ms    = 0u;
    uint32_t               lost_from = 0u;  /* moved_mm + 1 at LOST.     */
    uint32_t               lost_ms   = 0u;  /* Time spent LOST.          */
    uint32_t               still_ms  = 0u;  /* Time without odometry.    */
    uint32_t               last_mm   = 0u;
    char const *           p_why     = "?";
    ER                     er;

    tm_printf((UB *) "\n================================================\n"
                     "  STEP 4: RUNNING - PRESS GP20 TO STOP\n"
                     "================================================\n");
    er = subsystem2_follow(p_set->speed_mms);
    tm_printf((UB *) "S3 DRIVE GO preset %d: %d mm/s, kp=%d kd=%d, steer max "
                     "%d mm/s, %s (er=%d). Press BUTTON 1 to stop.\n",
              (INT) g_preset, (INT) p_set->speed_mms, (INT) p_set->kp_x100,
              (INT) p_set->kd_x100, (INT) p_set->steer_max_mms, S3_LAYOUT_TXT,
              (INT) er);
    tm_printf((UB *) "S3 LOG legend: t ms | mm | sensors L R (0 white .. "
                     "1000 black) zone (B black, G grey, W white) | dv "
                     "(+ = right) | S2 wheel target, measured, duty L/R | "
                     "encoder ticks L/R | heading 0.01 deg\n");
    for (;;)
    {
        tk_dly_tsk(POLL_MS);
        ms      += POLL_MS;
        log_ms  += POLL_MS;
        moved_mm = s3_odom_mm() - start_mm;    /* Reversing wraps huge.  */
        (void) s3_get_line_status(&st);
        stats_add(&stats, &st, moved_mm);
        if (log_ms >= S3_DRIVE_LOG_MS)
        {
            log_ms = 0u;
            run_log(ms, moved_mm, &st);
        }
        if (button1_down())
        {
            p_why = "button";
            break;
        }
        still_ms = (moved_mm == last_mm) ? (still_ms + POLL_MS) : 0u;
        last_mm  = moved_mm;
        if (still_ms >= S3_DRIVE_STILL_MS)
        {
            p_why = "car not moving (no odometry change)";
            break;
        }
        if (subsystem2_estop_is_set())
        {
            p_why = "S2 E-STOP (wheel stall?), see S2 counters below";
            break;
        }
        /* LOST keeps the last (full) correction, so the car usually turns
         * back onto the line by itself; stop only if it stays lost, by
         * distance or, if it is not moving, by time. */
        if (S3_LINE_LOST == st.state)
        {
            lost_ms += POLL_MS;
            if (0u == lost_from)
            {
                lost_from = moved_mm + 1u;
            }
            if (((moved_mm + 1u - lost_from) >= S3_DRIVE_LOST_MM) ||
                (lost_ms >= S3_DRIVE_LOST_MS))
            {
                p_why = "LINE LOST, not found again";
                break;
            }
        }
        else
        {
            lost_from = 0u;
            lost_ms   = 0u;
        }
        if (moved_mm >= S3_DRIVE_MAX_MM)
        {
            p_why = "distance limit";
            break;
        }
    }
    (void) subsystem2_stop();
    subsystem2_status_get(&s2);
    tm_printf((UB *) "\n================ RUN STOPPED ================\n"
                     "S3 DRIVE STOP (%s) after %d mm, %d ms. S2: stalls=%d "
                     "e-stops=%d discarded=%d\n",
              p_why, (INT) moved_mm, (INT) ms, (INT) s2.stall_count,
              (INT) s2.estop_count, (INT) s2.discarded_count);
    run_summary(&stats, moved_mm);
    while (button1_down())          /* The stop press must not restart.  */
    {
        tk_dly_tsk(POLL_MS);
    }
    tk_dly_tsk(AFTER_STOP_MS);
}

/*!
 * @brief Line sensor reading, permille, of the robot's LEFT (b_left) or
 *        RIGHT sensor (S3_IR_L_SIDE says which channel that is).
 */
static int32_t
sensor_permille (s3_line_status_t const * p_st, bool b_left)
{
    bool     b_gp26 = (S3_LN_L_ON_LEFT == S3_IR_L_SIDE) ? b_left : (!b_left);
    uint32_t n_u8   = b_gp26 ? p_st->n_outer : p_st->n_centre;

    return (int32_t) ((n_u8 * PERMILLE) / U8_FULL);
}

static char
zone_char (int32_t n_permille)
{
    if (n_permille > S3_LN_BLACK)
    {
        return 'B';
    }
    return (n_permille < S3_LN_WHITE) ? 'W' : 'G';
}

/*!
 * @brief One "S3 LOG" line: what the sensors saw, what S3 asked for, and
 *        what S2's wheels and encoders did.
 */
static void
run_log (uint32_t ms, uint32_t moved_mm, s3_line_status_t const * p_st)
{
    subsystem2_status_t s2   = {0};
    subsystem2_odom_t   odom = {0};
    int32_t             n_l  = sensor_permille(p_st, true);
    int32_t             n_r  = sensor_permille(p_st, false);

    subsystem2_status_get(&s2);
    subsystem2_odom_get(&odom);
    tm_printf((UB *) "S3 LOG %6d | %4d mm | L %4d R %4d %c%c %s | dv %+4d | "
                     "tgt %+4d %+4d spd %+4d %+4d duty %+4d %+4d | "
                     "enc %6d %6d | hdg %+6d\n",
              (INT) ms, (INT) moved_mm, (INT) n_l, (INT) n_r,
              zone_char(n_l), zone_char(n_r),
              s3_state_txt((uint32_t) p_st->state, 0u),
              (INT) p_st->steer_dv,
              (INT) s2.target_l_mms, (INT) s2.target_r_mms,
              (INT) odom.speed_l_mms, (INT) odom.speed_r_mms,
              (INT) s2.duty_l, (INT) s2.duty_r,
              (INT) odom.count_l, (INT) odom.count_r,
              (INT) odom.heading_cdeg);
}

/*!
 * @brief Count, every poll, where the line was and how S2 kept up.
 */
static void
stats_add (drive_stats_t * p_s, s3_line_status_t const * p_st,
           uint32_t moved_mm)
{
    subsystem2_status_t s2   = {0};
    subsystem2_odom_t   odom = {0};
    int32_t             n_l  = sensor_permille(p_st, true);
    int32_t             n_r  = sensor_permille(p_st, false);
    bool                b_lw = (n_l < S3_LN_WHITE);
    bool                b_rw = (n_r < S3_LN_WHITE);
    int32_t             dv   = p_st->steer_dv;
    int32_t             err_l;
    int32_t             err_r;

    p_s->polls++;
    if ((n_l > S3_LN_BLACK) && (n_r > S3_LN_BLACK))
    {
        p_s->both_black++;
    }
    if (b_lw && b_rw)
    {
        p_s->both_white++;
        if (0u == p_s->white_from)
        {
            p_s->white_from = moved_mm + 1u;
        }
        if ((moved_mm + 1u - p_s->white_from) > p_s->white_max_mm)
        {
            p_s->white_max_mm = moved_mm + 1u - p_s->white_from;
        }
    }
    else
    {
        p_s->white_from = 0u;
        if (b_lw != b_rw)
        {
            p_s->one_white++;
        }
    }
    if (((dv > 0) && (p_s->dv_prev < 0)) || ((dv < 0) && (p_s->dv_prev > 0)))
    {
        p_s->flips++;
    }
    if (0 != dv)
    {
        p_s->dv_prev = dv;
    }

    subsystem2_status_get(&s2);
    subsystem2_odom_get(&odom);
    err_l = s2.target_l_mms - odom.speed_l_mms;
    err_r = s2.target_r_mms - odom.speed_r_mms;
    p_s->wheel_err_sum += (uint32_t) ((err_l < 0) ? -err_l : err_l) +
                          (uint32_t) ((err_r < 0) ? -err_r : err_r);
}

/*!
 * @brief End-of-run summary and the most likely cause.
 */
static void
run_summary (drive_stats_t const * p_s, uint32_t moved_mm)
{
    uint32_t n         = (0u == p_s->polls) ? 1u : p_s->polls;
    uint32_t pct_bb    = (100u * p_s->both_black) / n;
    uint32_t pct_one   = (100u * p_s->one_white) / n;
    uint32_t pct_ww    = (100u * p_s->both_white) / n;
    uint32_t flips_m   = (moved_mm < 100u) ? 0u :
                         (p_s->flips * 1000u) / moved_mm;
    uint32_t wheel_err = p_s->wheel_err_sum / (2u * n);

    tm_printf((UB *) "S3 SUMMARY: both sensors black %d %%, one sensor white "
                     "%d %%, both white %d %% (longest %d mm). Steering "
                     "flips %d (%d per m). Wheel speed error mean %d mm/s.\n",
              (INT) pct_bb, (INT) pct_one, (INT) pct_ww,
              (INT) p_s->white_max_mm, (INT) p_s->flips, (INT) flips_m,
              (INT) wheel_err);
    if (pct_bb < DIAG_BOTH_BLACK_MIN_PCT)
    {
        tm_printf((UB *) "S3 DIAG: SENSORS: both were black together only "
                         "%d %% of the time. They are too far apart or not "
                         "centred on the line: park on the line, both must "
                         "read BLACK in \"S3 LINE\".\n", (INT) pct_bb);
    }
    if (pct_ww > DIAG_BOTH_WHITE_MAX_PCT)
    {
        tm_printf((UB *) "S3 DIAG: LINE BETWEEN SENSORS: both white %d %% of "
                         "the time while driving: the line passes between "
                         "the sensors (gap too wide) or the car leaves it.\n",
                  (INT) pct_ww);
    }
    if (flips_m > DIAG_FLIPS_PER_M_MAX)
    {
        tm_printf((UB *) "S3 DIAG: OVERSTEER / WEAVE: steering changed side "
                         "%d times per metre. Try preset 1 (softer), or move "
                         "the sensors closer.\n", (INT) flips_m);
    }
    if (wheel_err > DIAG_WHEEL_ERR_MAX_MMS)
    {
        tm_printf((UB *) "S3 DIAG: MOTORS: wheels were %d mm/s off their "
                         "set-points on average: check the motor power, the "
                         "encoders (enc L/R in S3 LOG must both rise) and "
                         "S2 tuning.\n", (INT) wheel_err);
    }
    if ((pct_bb >= DIAG_BOTH_BLACK_MIN_PCT) &&
        (pct_ww <= DIAG_BOTH_WHITE_MAX_PCT) &&
        (flips_m <= DIAG_FLIPS_PER_M_MAX) &&
        (wheel_err <= DIAG_WHEEL_ERR_MAX_MMS))
    {
        tm_printf((UB *) "S3 DIAG: no problem found in this run.\n");
    }
}

/*!
 * @brief Wait for a Button 1 (GP20) press. Release before driving.
 *        Button 2 (GP21): press = next preset, hold
 *        S3_DRIVE_RECAL_MS = recalibrate (after a countdown).
 */
static void
wait_start (void)
{
    uint32_t        held1   = 0u;
    uint32_t        held2   = 0u;
    uint32_t        cal_ms  = 0u;
    bool            b_recal = false;    /* This Button 2 hold recalibrated. */
    bool            b_ready = false;
    bool            b_reported = false;

    cal_watch(&b_ready, &b_reported);
    while (button1_down())          /* Let go of the last press first.  */
    {
        tk_dly_tsk(POLL_MS);
    }
    for (;;)
    {
        tk_dly_tsk(POLL_MS);
        cal_ms += POLL_MS;
        if (cal_ms >= CAL_PRINT_MS)
        {
            cal_ms = 0u;
            cal_watch(&b_ready, &b_reported);
        }

        held1 = button1_down() ? (held1 + POLL_MS) : 0u;
        if (held1 >= PRESS_MS)
        {
            return;                 /* Press: start.                    */
        }

        if (button2_down())
        {
            held2 += POLL_MS;
            if ((held2 >= S3_DRIVE_RECAL_MS) && (!b_recal))
            {
                b_recal = true;
                cal_countdown();
                b_reported = false;
            }
        }
        else
        {
            if ((!b_recal) && (held2 >= PRESS_MS))
            {
                preset_next();
            }
            held2   = 0u;
            b_recal = false;
        }
    }
}
static void
preset_next (void)
{
    g_preset = (g_preset + 1u) % N_PRESETS;
    preset_apply();
}

static void
preset_apply (void)
{
    drive_preset_t const * p_set = &g_presets[g_preset];

    s3_set_line_gains(p_set->kp_x100, p_set->kd_x100);
    s3_set_line_shape(0);
    s3_set_steer_max(p_set->steer_max_mms);
    tm_printf((UB *) "S3 DRIVE PRESET %d of %d: %d mm/s, kp=%d kd=%d, steer "
                     "max %d mm/s\n",
              (INT) g_preset, (INT) (N_PRESETS - 1u), (INT) p_set->speed_mms,
              (INT) p_set->kp_x100, (INT) p_set->kd_x100,
              (INT) p_set->steer_max_mms);
}

static void
button_input (uint32_t pin)
{
    (void) gpio_set_pin(pin, GPIO_MODE_IN);
    clr_w(GPIO(pin), GPIO_PDE);
    set_w(GPIO(pin), GPIO_PUE);
}

static bool
button1_down (void)
{
    return (0u == gpio_get_val(S3_DRIVE_BTN1_PIN));
}

static bool
button2_down (void)
{
    return (0u == gpio_get_val(S3_DRIVE_BTN2_PIN));
}

#else /* S3_DRIVE_TEST */

/*!
 * @brief Drive test not built: nothing to start.
 */
ER
task_drive_test_start (void)
{
    return E_OK;
}

#endif /* S3_DRIVE_TEST */

/*** end of file ***/
