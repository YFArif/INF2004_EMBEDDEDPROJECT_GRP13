/** @file subsystem2_test.c
 *
 * @brief On-target tests for subsystem 2. Built only with: make TEST=1.
 *        Record every run in testing/subsystem2/TEST_LOG.md.
 *        TEST=1 MOTOR_DIAG=1 selects the lifted-wheel diagnostic;
 *        MOTOR_DIAG=2 selects five-second timed floor runs.
 *        Every powered stage requires GP20.
 *
 * @par Automatic (motors stay braked; safe in every test build)
 * - S2-U08: the 1 MHz timer against the kernel clock.
 * - S2-U05: motion_task period, and MF_ESTOP to brake latency, 20 trials
 *   (NFR6).
 * - S2-U07: E-stop latch and CLEAR protocol (Week 6 report 6.3).
 * - S2-U10: longest motion cycle and stack use (NFR11), run last.
 *
 * @par Bench (set the switch to 1, lift the wheels off the ground first)
 * - S2-U01: encoder ticks per wheel revolution, turned by hand (~90 s).
 * - S2-U06: open-loop drive of each motor (~35 s).
 * - S2-U02: closed-loop step 0 -> 250 mm/s on FOLLOW (~3 s).
 *
 * @par Floor (S2_FLOOR_TEST in subsystem2_floor.c)
 * - S2-U11 distance, S2-U03 turns, S2-U12 braking / slow speed / stall.
 *
 * @par
 * S2-U01 and S2-U06 drive the hardware directly. That is allowed only
 * because motion_task leaves the PWM alone while IDLE.
 */

#include <stdbool.h>
#include <stdint.h>

#include "../test_report.h"
#include "drv_encoder.h"
#include "drv_motor.h"
#include "subsystem2.h"
#if S2_MOTOR_DIAG
#include "../subsystem5/drv_button.h"
#endif

#if APP_TEST

#define S2_U01_BENCH        (0)     /* Manual, blocks ~90 s at power-up. */
#define S2_U06_BENCH        (0)     /* Spins each motor, ~35 s.          */
#define S2_U02_BENCH        (0)     /* Spins both motors, ~3 s.          */

#define POLL_MS             (10u)   /* One kernel tick.                  */
#define ALIVE_WINDOW_MS     (1000u)
#define ALIVE_MIN           (48u)   /* 50 cycles of 20 ms, +/- jitter.   */
#define ALIVE_MAX           (52u)
#define STATE_WAIT_MS       (200u)
#define SETTLE_MS           (60u)   /* Three motion cycles.              */

#define U05_TRIALS          (20u)
#define U05_LIMIT_US        (20000u)    /* One motion cycle (NFR6).      */
#define U07_STEPS           (4u)

/* S2-U08: 1 MHz timer against the kernel clock. tk_dly_tsk() runs whole
 * 10 ms ticks, so 1000 ms lasts 1000 ... 1010 ms; allow 3 % either way. */
#define U08_WINDOW_MS       (1000u)
#define U08_MIN_US          (970000u)
#define U08_MAX_US          (1040000u)

/* S2-U10 (NFR11): 20 % of the 20 ms period, 75 % of the stack. */
#define U10_CYCLE_LIMIT_US  (4000u)
#define U10_STACK_PCT       (75u)

/* S2-U13: the steering value goes stale after 100 ms. */
#define STEER_STALE_WAIT_MS (150u)

#define U01_REVS            (10u)
#define U01_WINDOW_S        (40u)

#define U06_DUTY            (400)
#define U06_SPIN_UP_MS      (2000u)
#define U06_COUNT_MS        (1000u)

#define U02_SPEED_MMS       (250)
#define U02_RUN_MS          (1500u)
#define U02_SETTLE_MS       (500u)      /* NFR4: within 0.5 s ...        */
#define U02_BAND_MMS        (25)        /* ... and within +/- 10 %.      */
#define U02_SAMPLE_MS       (20u)

#if TM_CONSOLE_USB_CDC
/* The USB console sends nothing a closed port can receive, so the tests
 * wait for the PC to open it. tm_usb_state() is the kernel console's own
 * link state (lib/libtm/.../usb_console.c); it has no header, hence the
 * declaration here (BARR-C 4.2c deviation, test builds only). */
#define HOST_WAIT_MS        (30000u)
#define HOST_POLL_MS        (100u)
#define HOST_SETTLE_MS      (1000u)     /* Let the terminal settle.      */
#define USB_HOST_OPEN       (3u)        /* Enumerated and DTR asserted.  */

extern UW tm_usb_state(void);
#endif

static void     host_wait(void);
static bool     motion_alive(void);
static bool     state_wait(subsystem2_state_t state, uint32_t timeout_ms);
static void     cmd_send(subsystem2_cmd_type_t type, int16_t value,
                         uint16_t speed_mms);
static void     delay_ms(uint32_t ms);
static bool     s2_u05_estop_latency(void);
static bool     s2_u07_estop_latch(void);
static bool     s2_u08_timer_rate(void);
static bool     s2_u10_resources(void);
static bool     s2_u13_steering(void);
#if S2_MOTOR_DIAG
static void     motor_diag(void);
#endif

#if S2_U01_BENCH
static bool     s2_u01_wheel(drv_encoder_t wheel, char const * p_name);
#endif
#if S2_U06_BENCH
static bool     s2_u06_motor(drv_motor_t motor, char const * p_name);
static uint32_t s2_u06_run(drv_motor_t motor, int32_t duty);
#endif
#if S2_U02_BENCH
static bool     s2_u02_step(void);
#endif

/*!
 * @brief Run subsystem 2's tests (called by usermain() in test builds).
 */
void
subsystem2_test (void)
{
    TEST_INIT();

    host_wait();
#if S2_MOTOR_DIAG
    if (!motion_alive())
    {
        subsystem2_estop_set();
        tm_printf((UB *) "DIAG refused: motion task timing check failed.\n");
        for (;;)
        {
            tk_dly_tsk(1000u);
        }
    }
    motor_diag();
#endif

    // NOTE: The cast is safe: the core is 0, 1 or UINT32_MAX, printed as
    // 0, 1 or -1.
    tm_printf((UB *) "[TEST] S2 INFO encoder interrupt on core %d"
              " (A4 wants core 1)\n", (int) drv_encoder_core());

    // Every microsecond figure below depends on this one.
    TEST_CHECK("S2-U08", s2_u08_timer_rate(),
               "1 MHz timer runs at 1 MHz against the kernel clock");

    TEST_CHECK("S2-U05", motion_alive(),
               "motion_task runs every 20 ms (48-52 cycles per second)");
    TEST_CHECK("S2-U05", s2_u05_estop_latency(),
               "MF_ESTOP brakes within one cycle, 20 of 20 trials");

    TEST_CHECK("S2-U13", s2_u13_steering(),
               "steering: a set value is used, then ignored after 100 ms");

    TEST_CHECK("S2-U07", s2_u07_estop_latch(),
               "E-stop latch and CLEAR protocol");

#if S2_U01_BENCH
    TEST_CHECK("S2-U01", s2_u01_wheel(DRV_ENCODER_LEFT, "LEFT"),
               "left wheel counted on GP16, GP17 silent");
    TEST_CHECK("S2-U01", s2_u01_wheel(DRV_ENCODER_RIGHT, "RIGHT"),
               "right wheel counted on GP17, GP16 silent");
#endif

#if S2_U06_BENCH
    TEST_CHECK("S2-U06", s2_u06_motor(DRV_MOTOR_LEFT, "LEFT"),
               "left motor turned the left encoder");
    TEST_CHECK("S2-U06", s2_u06_motor(DRV_MOTOR_RIGHT, "RIGHT"),
               "right motor turned the right encoder");
#endif

#if S2_U02_BENCH
    TEST_CHECK("S2-U02", s2_u02_step(),
               "both wheels within +/-10 % of 250 mm/s after 0.5 s");
#endif

    // Floor tests: S2_FLOOR_TEST in subsystem2_floor.c, 0 = none.
    subsystem2_floor_test();

    // Last, so it covers everything that ran above (driving included).
    TEST_CHECK("S2-U10", s2_u10_resources(),
               "NFR11: longest motion cycle <= 4000 us, stack <= 75 %");

    TEST_SUMMARY("subsystem2");
}

#if S2_MOTOR_DIAG
#define DIAG_PERIOD_MS (20u)
#if (S2_MOTOR_DIAG == 3)
#define DIAG_SAMPLES (500u)
#define DIAG_DURATION_US (40000000u)
#elif (S2_MOTOR_DIAG == 2)
#define DIAG_SAMPLES (250u)
#define DIAG_DURATION_US (5000000u)
#else
#define DIAG_SAMPLES (100u)
#define DIAG_DURATION_US (2000000u)
#endif
#define DIAG_MAX_GAP_US (120000u)

typedef struct
{
    uint32_t us;
    subsystem2_odom_t odom;
    subsystem2_status_t status;
} diag_sample_t;

static diag_sample_t g_diag_samples[DIAG_SAMPLES];

static bool
diag_wait_start (void)
{
    uint32_t held_ms = 0u;

    while (drv_button_is_down(DRV_BUTTON_START))
    {
        tk_dly_tsk(POLL_MS);
    }
#if (S2_MOTOR_DIAG >= 2)
    tm_printf((UB *) "DIAG READY: FLOOR, clear 1 m ahead; mark axle START. "
                     "Press/release GP20 to run. "
                     "GP20 during motion ABORTS ALL tests. No ultrasonic guard.\n");
#else
    tm_printf((UB *) "DIAG READY: wheels LIFTED. Press/release GP20 to run. "
                     "GP20 during motion ABORTS ALL tests. Power off if unsafe.\n");
#endif
    while (held_ms < 60u)
    {
        tk_dly_tsk(POLL_MS);
        if (subsystem2_estop_is_set())
        {
            tm_printf((UB *) "DIAG E-stop latched: reboot required.\n");
            return false;
        }
        held_ms = drv_button_is_down(DRV_BUTTON_START) ?
                  held_ms + POLL_MS : 0u;
    }
    while (drv_button_is_down(DRV_BUTTON_START))
    {
        tk_dly_tsk(POLL_MS);
    }
    return !subsystem2_estop_is_set();
}

#if (S2_MOTOR_DIAG != 3)
static bool
diag_run (uint32_t stage, bool b_closed, drv_motor_t motor, uint16_t value)
{
    drv_encoder_count_t left = {0};
    drv_encoder_count_t right = {0};
    subsystem2_odom_t start_odom = {0};
    subsystem2_odom_t stop_odom = {0};
    subsystem2_odom_t final_odom = {0};
    uint32_t start_us;
    uint32_t stop_request_us;
    uint32_t stopped_us;
    uint32_t first_l_us = 0u;
    uint32_t first_r_us = 0u;
    uint32_t count_l;
    uint32_t count_r;
    uint32_t samples = 0u;
    uint32_t elapsed_us = 0u;
    uint32_t previous_us = 0u;
    uint32_t idx;
    bool b_ok = true;
    char const * p_reason = "complete";
    ER er = E_OK;

    tm_printf((UB *) "\nDIAG STAGE %d mode=%s motor=%s value=%d %s duration_ms=%d\n",
              (INT) stage, b_closed ? "PI" : "FIXED_PWM",
              b_closed ? "BOTH" : ((DRV_MOTOR_LEFT == motor) ? "LEFT" : "RIGHT"),
              (INT) value, b_closed ? "mm/s" : "permille",
              (INT) (DIAG_DURATION_US / 1000u));
#if (S2_MOTOR_DIAG == 2)
    if (!b_closed)
    {
        tm_printf((UB *) "TIMED refused: floor mode requires closed-loop API.\n");
        return false;
    }
#endif
    if (!diag_wait_start())
    {
        return false;
    }
    if (!state_wait(S2_STATE_IDLE, STATE_WAIT_MS))
    {
        tm_printf((UB *) "DIAG refused: motion task not IDLE.\n");
        return false;
    }
    drv_encoder_get(DRV_ENCODER_LEFT, &left);
    drv_encoder_get(DRV_ENCODER_RIGHT, &right);
    count_l = left.ticks;
    count_r = right.ticks;
    subsystem2_odom_get(&start_odom);
    start_us = drv_encoder_now_us();
    if (b_closed)
    {
        subsystem2_steer_set(0);
        er = subsystem2_follow(value);
    }
    else
    {
        drv_motor_set(motor, value);
    }
    if (E_OK != er)
    {
        tm_printf((UB *) "DIAG FOLLOW failed er=%d\n", (INT) er);
        p_reason = "FOLLOW command failed";
        b_ok = false;
    }
    for (idx = 0u; (idx < DIAG_SAMPLES) && b_ok &&
                   (elapsed_us < DIAG_DURATION_US); idx++)
    {
        diag_sample_t * p_sample = &g_diag_samples[idx];

        er = tk_dly_tsk(DIAG_PERIOD_MS);
        if (E_OK != er)
        {
            p_reason = "sample sleep failed";
            b_ok = false;
            break;
        }
        if (drv_button_is_down(DRV_BUTTON_START))
        {
            p_reason = "GP20 pressed during motion";
            b_ok = false;
            break;
        }
        if (subsystem2_estop_is_set())
        {
            p_reason = "E-stop latched";
            b_ok = false;
            break;
        }
        drv_encoder_get(DRV_ENCODER_LEFT, &left);
        drv_encoder_get(DRV_ENCODER_RIGHT, &right);
        if ((0u == first_l_us) && (left.ticks != count_l))
        {
            first_l_us = left.tick_time_us - start_us;
        }
        if ((0u == first_r_us) && (right.ticks != count_r))
        {
            first_r_us = right.tick_time_us - start_us;
        }
        p_sample->us = drv_encoder_now_us() - start_us;
        subsystem2_odom_get(&p_sample->odom);
        subsystem2_status_get(&p_sample->status);
        elapsed_us = drv_encoder_now_us() - start_us;
        if ((elapsed_us - previous_us) > DIAG_MAX_GAP_US)
        {
            p_reason = "sample gap exceeded 120 ms";
            b_ok = false;
            break;
        }
        previous_us = elapsed_us;
        if (b_closed)
        {
            subsystem2_steer_set(0);
        }
        samples++;
    }
    if (b_ok && (elapsed_us < DIAG_DURATION_US))
    {
        p_reason = "sample buffer exhausted before duration";
        b_ok = false;
    }
    stop_request_us = drv_encoder_now_us() - start_us;
    subsystem2_odom_get(&stop_odom);
#if (S2_MOTOR_DIAG != 2)
    drv_motor_brake();
#endif
    er = subsystem2_stop();
    if (E_OK != er)
    {
        p_reason = "STOP command failed";
    }
    if ((E_OK != er) || (!b_ok))
    {
        subsystem2_estop_set();
        drv_motor_brake();
        tm_printf((UB *) "DIAG ABORT: %s, elapsed_us=%d, stop er=%d. "
                         "Reboot required.\n", p_reason,
                  (INT) elapsed_us, (INT) er);
        b_ok = false;
    }
    if (b_ok && !state_wait(S2_STATE_IDLE, STATE_WAIT_MS))
    {
        subsystem2_estop_set();
        drv_motor_brake();
        tm_printf((UB *) "DIAG STOP timeout; E-stop latched.\n");
        b_ok = false;
    }
    stopped_us = drv_encoder_now_us() - start_us;
    tk_dly_tsk(500u);
    subsystem2_odom_get(&final_odom);
#if (S2_MOTOR_DIAG == 2)
    tm_printf((UB *) "TIMED RESULT stage=%d completed=%d speed_mms=%d "
                     "nominal_ms=5000 nominal_mm=%d stop_request_ms=%d "
                     "idle_ms=%d encoder_at_stop_mm=%d encoder_final_mm=%d "
                     "left_mm=%d right_mm=%d\n",
              (INT) stage, (INT) b_ok, (INT) value, (INT) (value * 5u),
              (INT) (stop_request_us / 1000u), (INT) (stopped_us / 1000u),
              (INT) (stop_odom.dist_mm - start_odom.dist_mm),
              (INT) (final_odom.dist_mm - start_odom.dist_mm),
              (INT) (final_odom.dist_l_mm - start_odom.dist_l_mm),
              (INT) (final_odom.dist_r_mm - start_odom.dist_r_mm));
    tm_printf((UB *) "TIMED MEASURE: mark same axle reference at REST; "
                     "record tape distance and sideways drift BEFORE next GP20. "
                     "Encoder values are not an independent tape measurement.\n");
#else
    (void) stop_request_us;
    (void) stopped_us;
#endif
    tm_printf((UB *) "DIAG RESULT stage=%d completed=%d first_tick_us_L/R=%d/%d "
                     "(0=none; includes encoder phase and command latency)\n",
              (INT) stage, (INT) b_ok, (INT) first_l_us, (INT) first_r_us);
    tm_printf((UB *) "DIAG CSV stage,us,targetL,targetR,speedL,speedR,"
                     "dutyL,dutyR,ticksL,ticksR\n");
    for (idx = 0u; idx < samples; idx++)
    {
        diag_sample_t const * p = &g_diag_samples[idx];

        tm_printf((UB *) "D,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  (INT) stage, (INT) p->us,
                  (INT) p->status.target_l_mms, (INT) p->status.target_r_mms,
                  (INT) p->odom.speed_l_mms, (INT) p->odom.speed_r_mms,
                  (INT) (b_closed ? p->status.duty_l :
                         ((DRV_MOTOR_LEFT == motor) ? value : 0)),
                  (INT) (b_closed ? p->status.duty_r :
                         ((DRV_MOTOR_RIGHT == motor) ? value : 0)),
                  (INT) (p->odom.count_l - count_l),
                  (INT) (p->odom.count_r - count_r));
    }
    return b_ok;
}
#else
static bool
diag_move (uint32_t stage, int16_t distance, uint16_t speed)
{
    subsystem2_status_t before = {0};
    subsystem2_odom_t start = {0};
    subsystem2_odom_t end = {0};
    uint32_t start_us;
    uint32_t previous_us = 0u;
    uint32_t elapsed_us = 0u;
    uint32_t samples = 0u;
    uint32_t idx;
    bool b_done = false;
    char const * p_reason = "MOVE did not complete within 40 s";
    ER er;

    tm_printf((UB *) "\nANNA MOVE stage=%d distance_mm=%d speed_mms=%d. "
                     "Clear at least 600 mm in travel direction; "
                     "measure before next GP20.\n",
              (INT) stage, (INT) distance, (INT) speed);
    if (!diag_wait_start())
    {
        return false;
    }
    if (!state_wait(S2_STATE_IDLE, STATE_WAIT_MS))
    {
        subsystem2_estop_set();
        tm_printf((UB *) "MOVE refused: S2 not IDLE; E-stop latched.\n");
        return false;
    }
    subsystem2_status_get(&before);
    subsystem2_odom_get(&start);
    start_us = drv_encoder_now_us();
    er = subsystem2_move(distance, speed);
    if (E_OK != er)
    {
        p_reason = "MOVE enqueue failed";
    }
    while ((E_OK == er) && (elapsed_us < DIAG_DURATION_US) &&
           (samples < DIAG_SAMPLES))
    {
        diag_sample_t * p = &g_diag_samples[samples];

        er = tk_dly_tsk(100u);
        elapsed_us = drv_encoder_now_us() - start_us;
        if (E_OK != er)
        {
            p_reason = "sample sleep failed";
            break;
        }
        if (drv_button_is_down(DRV_BUTTON_START))
        {
            p_reason = "GP20 abort";
            break;
        }
        if (subsystem2_estop_is_set())
        {
            p_reason = "S2 E-stop/stall";
            break;
        }
        if ((elapsed_us - previous_us) > 250000u)
        {
            p_reason = "sample gap exceeded 250 ms";
            break;
        }
        previous_us = elapsed_us;
        p->us = elapsed_us;
        subsystem2_odom_get(&p->odom);
        subsystem2_status_get(&p->status);
        samples++;
        if (p->status.done_count != before.done_count)
        {
            b_done = (S2_STATE_IDLE == p->status.state);
            p_reason = "completion reported outside IDLE";
            break;
        }
    }
    if (!b_done)
    {
        subsystem2_estop_set();
        drv_motor_brake();
        tm_printf((UB *) "MOVE ABORT stage=%d: %s er=%d elapsed_ms=%d; "
                         "reboot required.\n", (INT) stage, p_reason,
                  (INT) er, (INT) (elapsed_us / 1000u));
    }
    tk_dly_tsk(500u);
    subsystem2_odom_get(&end);
    tm_printf((UB *) "MOVE RESULT stage=%d completed=%d requested_mm=%d "
                     "speed_mms=%d elapsed_ms=%d encoder_mm=%d "
                     "left_mm=%d right_mm=%d heading_change_cdeg=%d\n",
              (INT) stage, (INT) b_done, (INT) distance, (INT) speed,
              (INT) (elapsed_us / 1000u), (INT) (end.dist_mm - start.dist_mm),
              (INT) (end.dist_l_mm - start.dist_l_mm),
              (INT) (end.dist_r_mm - start.dist_r_mm),
              (INT) (((end.heading_cdeg - start.heading_cdeg + 54000) %
                       36000) - 18000));
    tm_printf((UB *) "MOVE CSV stage,us,targetL,targetR,speedL,speedR,"
                     "dutyL,dutyR,relative_left_mm,relative_right_mm\n");
    for (idx = 0u; idx < samples; idx++)
    {
        diag_sample_t const * p = &g_diag_samples[idx];

        tm_printf((UB *) "M,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                  (INT) stage, (INT) p->us,
                  (INT) p->status.target_l_mms, (INT) p->status.target_r_mms,
                  (INT) p->odom.speed_l_mms, (INT) p->odom.speed_r_mms,
                  (INT) p->status.duty_l, (INT) p->status.duty_r,
                  (INT) (p->odom.dist_l_mm - start.dist_l_mm),
                  (INT) (p->odom.dist_r_mm - start.dist_r_mm));
    }
    tm_printf((UB *) "MOVE MEASURE: record signed tape displacement, "
                     "sideways drift and final angle using SAME axle reference. "
                     "Do not assume reverse returns exactly to the start.\n");
    return b_done;
}
#endif

static void
motor_diag (void)
{
#if (S2_MOTOR_DIAG == 3)
    uint16_t const speeds[] = {40u, 60u, 100u};
    uint32_t idx;
    uint32_t stage = 0u;
    bool b_ok = true;

    drv_button_init();
    drv_motor_brake();
    tm_printf((UB *) "\nANNA MOVE TEST v37: +400 mm at 40/60/100 mm/s, "
                     "then -400 mm at 40/60/100 mm/s. "
                     "Total forward travel 1200 mm; clear space accordingly. "
                     "Each move requires GP20.\n"
                     "Same subsystem2_move API as Anna; waits for done_count "
                     "(S2 also signals EV_MOTION_DONE). No FOLLOW or IR steering.\n"
                     "S3/S5 off; NO ultrasonic protection. Clear floor/cable "
                     "both ways. GP20 while moving aborts ALL. "
                     "No gains, geometry or turn calibration changed.\n");
    for (idx = 0u; (idx < 3u) && b_ok; idx++)
    {
        b_ok = diag_move(++stage, 400, speeds[idx]);
    }
    for (idx = 0u; (idx < 3u) && b_ok; idx++)
    {
        b_ok = diag_move(++stage, -400, speeds[idx]);
    }
#elif (S2_MOTOR_DIAG == 2)
    uint16_t const speeds[] = {40u, 60u, 100u};
    uint32_t idx;
    bool b_ok = true;

    drv_button_init();
    drv_motor_brake();
    tm_printf((UB *) "\nTIMED FLOOR TEST v36: 40/60/100 mm/s, 5 s EACH; "
                     "nominal distances 200/300/500 mm.\n"
                     "Uses subsystem2_follow(speed), zero steering, then "
                     "subsystem2_stop(): same public API as teammates.\n"
                     "No gains/geometry changes. S3/S5 off, NO obstacle guard. "
                     "Startup/braking cause differences from speed x time.\n"
                     "Allow clear floor, stop access and slack USB cable. "
                     "Each stage waits for GP20. GP20 while moving aborts ALL.\n");
    for (idx = 0u; (idx < 3u) && b_ok; idx++)
    {
        b_ok = diag_run(idx + 1u, true, DRV_MOTOR_LEFT, speeds[idx]);
    }
#else
    uint16_t const duties[] = {100u, 150u, 200u, 300u, 400u};
    uint16_t const speeds[] = {10u, 40u, 250u};
    uint32_t idx;
    uint32_t side;
    uint32_t trial;
    uint32_t stage = 0u;
    bool b_ok = true;

    drv_button_init();
    drv_motor_brake();
    tm_printf((UB *) "\nMOTOR DIAGNOSTIC v35: WHEELS LIFTED ONLY. "
                     "S3/S5 off; no ultrasonic guard. No gains changed.\n"
                     "10 fixed-PWM stages, then 6 PI stages. "
                     "Each requires GP20. Watch forward direction by eye.\n"
                     "Fixed-PWM targets read zero: PI is intentionally idle.\n");
    for (idx = 0u; (idx < 5u) && b_ok; idx++)
    {
        for (side = 0u; (side < 2u) && b_ok; side++)
        {
            b_ok = diag_run(++stage, false,
                            (0u == side) ? DRV_MOTOR_LEFT : DRV_MOTOR_RIGHT,
                            duties[idx]);
        }
    }
    for (idx = 0u; (idx < 3u) && b_ok; idx++)
    {
        for (trial = 0u; (trial < 2u) && b_ok; trial++)
        {
            b_ok = diag_run(++stage, true, DRV_MOTOR_LEFT, speeds[idx]);
        }
    }
#endif
    subsystem2_estop_set();
    drv_motor_brake();
    tm_printf((UB *) "DIAG END completed=%d; motors locked until reboot. "
                     "Save full serial log and note direction/noise/binding.\n",
              (INT) b_ok);
    for (;;)
    {
        tk_dly_tsk(1000u);
    }
}
#endif

/*!
 * @brief S2-U13: a fresh subsystem2_steer_set() value is what motion_task
 *        reads, and it is ignored once it is older than 100 ms (a stopped
 *        line_task must not leave the robot steering on an old value).
 *
 * @return true if both held.
 */
static bool
s2_u13_steering (void)
{
    bool b_fresh = false;
    bool b_stale = false;

    subsystem2_steer_set(40);
    b_fresh = (40 == subsystem2_steer_get());

    delay_ms(STEER_STALE_WAIT_MS);
    b_stale = (40 != subsystem2_steer_get());

    subsystem2_steer_set(0);

    // NOTE: The casts are safe: the format string is ASCII and the
    // values are 0 or 1.
    tm_printf((UB *) "[TEST] S2-U13 INFO fresh value used %d, old value"
              " ignored %d\n", (int) b_fresh, (int) b_stale);

    return (b_fresh && b_stale);
}

/*!
 * @brief S2-U10: motion_task's longest cycle and deepest stack use so far.
 *
 * @return true if both are inside NFR11 (20 % of 20 ms; 75 % of stack).
 */
static bool
s2_u10_resources (void)
{
    subsystem2_status_t status = {0};
    uint32_t            used   = subsystem2_stack_used();

    subsystem2_status_get(&status);

    // NOTE: The casts are safe: the format string is ASCII and all values
    // are below 20000.
    tm_printf((UB *) "[TEST] S2-U10 INFO longest cycle %d us (limit %d),"
              " stack %d of %d bytes = %d %% (limit 75 %%), %d cycles\n",
              (int) status.cycle_max_us, (int) U10_CYCLE_LIMIT_US,
              (int) used, (int) S2_MOTION_STACK_BYTES,
              (int) ((used * 100u) / S2_MOTION_STACK_BYTES),
              (int) status.cycle_count);

    return ((status.cycle_max_us <= U10_CYCLE_LIMIT_US) &&
            ((used * 100u) <= (S2_MOTION_STACK_BYTES * U10_STACK_PCT)));
}

/*!
 * @brief Wait up to 30 s for the PC to open the serial port, so the S2
 *        results are not sent to a closed port and lost.
 */
static void
host_wait (void)
{
#if TM_CONSOLE_USB_CDC
    uint32_t waited = 0u;

    while ((USB_HOST_OPEN != tm_usb_state()) && (waited < HOST_WAIT_MS))
    {
        delay_ms(HOST_POLL_MS);
        waited += HOST_POLL_MS;
    }

    delay_ms(HOST_SETTLE_MS);
#endif

    // NOTE: The cast is safe: the format string is ASCII.
    tm_printf((UB *) "\n[TEST] subsystem2 tests start\n");
}

/*!
 * @brief S2-U08: count the 1 MHz timer over a 1000 ms kernel delay.
 *
 * @return true if it counted 0.97 ... 1.04 million.
 */
static bool
s2_u08_timer_rate (void)
{
    uint32_t start_us = drv_encoder_now_us();
    uint32_t count    = 0u;

    delay_ms(U08_WINDOW_MS);
    count = drv_encoder_now_us() - start_us;

    // NOTE: The casts are safe: the format string is ASCII, the count is
    // printed as unsigned (%u) and the register is printed in hex.
    tm_printf((UB *) "[TEST] S2-U08 INFO timer counted %u in %d ms;"
              " WATCHDOG TICK at boot 0x%x (12 cycles = 0x..0c)\n",
              (unsigned int) count, (int) U08_WINDOW_MS,
              (unsigned int) drv_encoder_tick_was());

    return ((count >= U08_MIN_US) && (count <= U08_MAX_US));
}

/*!
 * @brief Check that motion_task runs once every 20 ms: about 50 cycles in
 *        1000 ms (not fewer: it is stuck; not more: it is not sleeping).
 *        Also reports the longest cycle so far (NFR11: <= 4000 us).
 *
 * @return true if it ran ALIVE_MIN ... ALIVE_MAX cycles.
 */
static bool
motion_alive (void)
{
    subsystem2_status_t status = {0};
    uint32_t            first  = 0u;
    uint32_t            cycles = 0u;

    subsystem2_status_get(&status);
    first = status.cycle_count;
    delay_ms(ALIVE_WINDOW_MS);
    subsystem2_status_get(&status);
    cycles = status.cycle_count - first;

    // NOTE: The casts are safe: the format string is ASCII; the cycle
    // count is below 1000 and the time below 20000 us.
    tm_printf((UB *) "[TEST] S2-U05 INFO %d motion cycles in %d ms,"
              " longest cycle %d us\n", (int) cycles,
              (int) ALIVE_WINDOW_MS, (int) status.cycle_max_us);

    return ((cycles >= ALIVE_MIN) && (cycles <= ALIVE_MAX));
}

/*!
 * @brief S2-U05: time from setting MF_ESTOP to the brake being applied.
 *
 * @return true if every trial braked within U05_LIMIT_US.
 *
 * NOTE: This is the software part of S2-U05. The hardware part (GPIO
 * toggle and a scope on GP8 - GP11) is still to be done.
 */
static bool
s2_u05_estop_latency (void)
{
    subsystem2_status_t status  = {0};
    uint32_t            trial   = 0u;
    uint32_t            set_us  = 0u;
    uint32_t            delay   = 0u;
    uint32_t            worst   = 0u;
    uint32_t            total   = 0u;
    uint32_t            before  = 0u;
    bool                b_ok    = state_wait(S2_STATE_IDLE, STATE_WAIT_MS);

    for (trial = 0u; (trial < U05_TRIALS) && b_ok; trial++)
    {
        subsystem2_status_get(&status);
        before = status.estop_count;

        set_us = drv_encoder_now_us();
        subsystem2_estop_set();
        b_ok = state_wait(S2_STATE_ESTOP, STATE_WAIT_MS);

        subsystem2_status_get(&status);
        b_ok  = b_ok && ((before + 1u) == status.estop_count);
        delay = status.estop_brake_us - set_us;
        total += delay;
        if (delay > worst)
        {
            worst = delay;
        }

        subsystem2_estop_clear();
        cmd_send(S2_CMD_CLEAR, 0, 0u);
        b_ok = b_ok && state_wait(S2_STATE_IDLE, STATE_WAIT_MS);
    }

    // NOTE: The casts are safe: the format string is ASCII, and every
    // value is below 200000 us, which an int always holds.
    tm_printf((UB *) "[TEST] S2-U05 INFO %d trials: worst %d us, mean %d us"
              " (limit %d us)\n", (int) trial, (int) worst,
              (int) (total / U05_TRIALS), (int) U05_LIMIT_US);

    return (b_ok && (U05_TRIALS == trial) && (worst <= U05_LIMIT_US));
}

/*!
 * @brief S2-U07: commands before CLEAR are discarded, CLEAR is ignored
 *        while MF_ESTOP is still set, and a command sent after an
 *        accepted CLEAR runs. MOVE 0 is used, so no motor turns.
 *
 * @return true if all U07_STEPS steps passed.
 */
static bool
s2_u07_estop_latch (void)
{
    subsystem2_status_t status    = {0};
    uint32_t            discarded = 0u;
    uint32_t            done      = 0u;
    uint32_t            passed    = 0u;

    subsystem2_estop_set();
    if (state_wait(S2_STATE_ESTOP, STATE_WAIT_MS))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO did not enter ESTOP\n");
    }

    subsystem2_status_get(&status);
    discarded = status.discarded_count;
    done      = status.done_count;

    // A command sent while latched must be dropped.
    cmd_send(S2_CMD_MOVE, 100, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_ESTOP == status.state) &&
        ((discarded + 1u) == status.discarded_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO MOVE was not discarded\n");
    }

    // CLEAR while MF_ESTOP is still set must be ignored.
    cmd_send(S2_CMD_CLEAR, 0, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_ESTOP == status.state) &&
        ((discarded + 2u) == status.discarded_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO CLEAR was not ignored\n");
    }

    // Clear the latch, then CLEAR followed by MOVE 0: both must run.
    subsystem2_estop_clear();
    cmd_send(S2_CMD_CLEAR, 0, 0u);
    cmd_send(S2_CMD_MOVE, 0, 0u);
    delay_ms(SETTLE_MS);
    subsystem2_status_get(&status);
    if ((S2_STATE_IDLE == status.state) &&
        ((discarded + 2u) == status.discarded_count) &&
        ((done + 1u) == status.done_count))
    {
        passed++;
    }
    else
    {
        tm_printf((UB *) "[TEST] S2-U07 INFO CLEAR + MOVE 0 did not run\n");
    }

    // NOTE: The casts are safe: the format string is ASCII and the
    // counts are 0 ... 4.
    tm_printf((UB *) "[TEST] S2-U07 INFO %d of %d steps passed\n",
              (int) passed, (int) U07_STEPS);

    return (U07_STEPS == passed);
}

#if S2_U01_BENCH
/*!
 * @brief S2-U01: turn one wheel by hand U01_REVS times.
 *
 * @param[in] wheel  The wheel to turn.
 * @param[in] p_name "LEFT" or "RIGHT".
 *
 * @return true if it counted and the other wheel stayed under 1 %.
 */
static bool
s2_u01_wheel (drv_encoder_t wheel, char const * p_name)
{
    drv_encoder_t       other  = DRV_ENCODER_LEFT;
    drv_encoder_count_t start  = {0};
    drv_encoder_count_t now    = {0};
    drv_encoder_count_t o_from = {0};
    drv_encoder_count_t o_now  = {0};
    uint32_t            sec    = 0u;
    uint32_t            mine   = 0u;
    uint32_t            theirs = 0u;

    if (DRV_ENCODER_LEFT == wheel)
    {
        other = DRV_ENCODER_RIGHT;
    }

    subsystem2_step_ready((DRV_ENCODER_LEFT == wheel) ?
                          "S2-U01 turn the LEFT wheel 10 times by hand,"
                          " right wheel still (40 s)" :
                          "S2-U01 turn the RIGHT wheel 10 times by hand,"
                          " left wheel still (40 s)");

    drv_encoder_get(wheel, &start);
    drv_encoder_get(other, &o_from);
    tm_printf((UB *) "[S2-U01] GO: turn the %s wheel %d full revolutions\n",
              p_name, (int) U01_REVS);

    for (sec = 1u; sec <= U01_WINDOW_S; sec++)
    {
        delay_ms(1000u);
        drv_encoder_get(wheel, &now);
        tm_printf((UB *) "[S2-U01] %d s  ticks=%d  raw rise=%d fall=%d\n",
                  (int) sec, (int) (now.ticks - start.ticks),
                  (int) (now.raw_rise - start.raw_rise),
                  (int) (now.raw_fall - start.raw_fall));
    }

    drv_encoder_get(other, &o_now);
    mine   = now.ticks - start.ticks;
    theirs = o_now.ticks - o_from.ticks;

    // NOTE: The casts are safe: counts over 40 s stay below 100000.
    tm_printf((UB *) "[TEST] S2-U01 INFO %s: ticks=%d -> %d.%d per rev,"
              " rejected=%d, shortest gap=%d us\n", p_name, (int) mine,
              (int) (mine / U01_REVS), (int) ((mine * 10u / U01_REVS) % 10u),
              (int) (now.rejected - start.rejected), (int) now.min_gap_us);

    return ((mine > 0u) && ((theirs * 100u) <= mine));
}
#endif /* S2_U01_BENCH */

#if S2_U06_BENCH
/*!
 * @brief S2-U06: forward, reverse, then forward at double duty.
 *        Direction is judged by eye.
 *
 * @param[in] motor  The motor.
 * @param[in] p_name "LEFT" or "RIGHT".
 *
 * @return true if every run counted and double duty was faster.
 */
static bool
s2_u06_motor (drv_motor_t motor, char const * p_name)
{
    uint32_t slow = 0u;
    uint32_t back = 0u;
    uint32_t fast = 0u;
    bool     b_l  = (DRV_MOTOR_LEFT == motor);

    (void) p_name;

    // Each phase waits for Button 1, so the tester answers the PC's
    // questions while nothing moves, and watches the wheel when it does.
    subsystem2_step_ready(b_l ? "S2-U06 LEFT FORWARD (3 s): WATCH the left"
                                " wheel; does it drive the robot forward?"
                              : "S2-U06 RIGHT FORWARD (3 s): WATCH the right"
                                " wheel; does it drive the robot forward?");
    slow = s2_u06_run(motor, U06_DUTY);

    subsystem2_step_ready(b_l ? "S2-U06 LEFT REVERSE (3 s): WATCH the left"
                                " wheel; the opposite way?"
                              : "S2-U06 RIGHT REVERSE (3 s): WATCH the right"
                                " wheel; the opposite way?");
    back = s2_u06_run(motor, -U06_DUTY);

    subsystem2_step_ready(b_l ? "S2-U06 LEFT FORWARD FAST (3 s): WATCH the"
                                " left wheel; forward and faster?"
                              : "S2-U06 RIGHT FORWARD FAST (3 s): WATCH the"
                                " right wheel; forward and faster?");
    fast = s2_u06_run(motor, 2 * U06_DUTY);

    drv_motor_brake();

    return ((slow > 0u) && (back > 0u) && (fast > slow));
}

/*!
 * @brief Drive one motor, let it settle, then count its ticks for 1 s.
 *
 * @param[in] motor The motor (its encoder has the same index).
 * @param[in] duty  Permille, signed.
 *
 * @return Ticks per second; 0 if the other wheel also moved.
 */
static uint32_t
s2_u06_run (drv_motor_t motor, int32_t duty)
{
    drv_encoder_t       mine   = DRV_ENCODER_LEFT;
    drv_encoder_t       other  = DRV_ENCODER_RIGHT;
    drv_encoder_count_t m_from = {0};
    drv_encoder_count_t m_to   = {0};
    drv_encoder_count_t o_from = {0};
    drv_encoder_count_t o_to   = {0};
    uint32_t            ticks  = 0u;
    uint32_t            tpr    = (uint32_t) S2_TICKS_PER_REV_L;
    uint32_t            circ   = (uint32_t) S2_WHEEL_CIRC_UM_L;

    if (DRV_MOTOR_RIGHT == motor)
    {
        mine  = DRV_ENCODER_RIGHT;
        other = DRV_ENCODER_LEFT;
        tpr   = (uint32_t) S2_TICKS_PER_REV_R;
        circ  = (uint32_t) S2_WHEEL_CIRC_UM_R;
    }

    drv_motor_set(motor, duty);
    delay_ms(U06_SPIN_UP_MS);
    drv_encoder_get(mine, &m_from);
    drv_encoder_get(other, &o_from);
    delay_ms(U06_COUNT_MS);
    drv_encoder_get(mine, &m_to);
    drv_encoder_get(other, &o_to);
    drv_motor_set(motor, 0);
    delay_ms(1000u);

    ticks = m_to.ticks - m_from.ticks;

    // NOTE: The casts are safe: the format string is ASCII; ticks per
    // second stay below 5000 (x 209310 fits 32 bits) and the speed below
    // 2000 mm/s.
    tm_printf((UB *) "[S2-U06]   duty %d: %d ticks/s = %d mm/s,"
              " other wheel %d ticks\n", (int) duty, (int) ticks,
              (int) (((ticks * circ) / tpr) / 1000u),
              (int) (o_to.ticks - o_from.ticks));

    if (((o_to.ticks - o_from.ticks) * 100u) > ticks)
    {
        ticks = 0u;
    }

    return (ticks);
}
#endif /* S2_U06_BENCH */

#if S2_U02_BENCH
/*!
 * @brief S2-U02: FOLLOW at 250 mm/s with no steering; log both wheel
 *        speeds every 20 ms and check them after 0.5 s.
 *
 * @return true if both stayed within +/- U02_BAND_MMS after settling.
 */
static bool
s2_u02_step (void)
{
    subsystem2_odom_t   odom    = {0};
    subsystem2_status_t status  = {0};
    uint32_t            elapsed = 0u;
    bool                b_ok    = true;

    subsystem2_step_ready("S2-U02 both wheels run 1.5 s at 250 mm/s: WATCH"
                          " that BOTH turn the way that drives the robot"
                          " forward");
    tm_printf((UB *) "[S2-U02] t_ms,speed_l,speed_r,duty_l,duty_r\n");

    subsystem2_steer_set(0);
    cmd_send(S2_CMD_FOLLOW, 0, (uint16_t) U02_SPEED_MMS);

    for (elapsed = U02_SAMPLE_MS; elapsed <= U02_RUN_MS;
         elapsed += U02_SAMPLE_MS)
    {
        delay_ms(U02_SAMPLE_MS);
        subsystem2_odom_get(&odom);
        subsystem2_status_get(&status);

        // NOTE: The casts are safe: the format string is ASCII and all
        // values are below 2000.
        tm_printf((UB *) "[S2-U02] %d,%d,%d,%d,%d\n", (int) elapsed,
                  (int) odom.speed_l_mms, (int) odom.speed_r_mms,
                  (int) status.duty_l, (int) status.duty_r);

        if ((elapsed >= U02_SETTLE_MS) &&
            (((odom.speed_l_mms - U02_SPEED_MMS) > U02_BAND_MMS) ||
             ((U02_SPEED_MMS - odom.speed_l_mms) > U02_BAND_MMS) ||
             ((odom.speed_r_mms - U02_SPEED_MMS) > U02_BAND_MMS) ||
             ((U02_SPEED_MMS - odom.speed_r_mms) > U02_BAND_MMS)))
        {
            b_ok = false;
        }
    }

    cmd_send(S2_CMD_STOP, 0, 0u);

    return (b_ok && (S2_STATE_FOLLOWING == status.state));
}
#endif /* S2_U02_BENCH */

/*!
 * @brief Wait until motion_task reports a state.
 *
 * @param[in] state      The state.
 * @param[in] timeout_ms Longest wait.
 *
 * @return true if the state was reached.
 */
static bool
state_wait (subsystem2_state_t state, uint32_t timeout_ms)
{
    subsystem2_status_t status = {0};
    uint32_t            waited = 0u;

    subsystem2_status_get(&status);

    while ((state != status.state) && (waited < timeout_ms))
    {
        delay_ms(POLL_MS);
        waited += POLL_MS;
        subsystem2_status_get(&status);
    }

    return (state == status.state);
}

/*!
 * @brief Send one command to motion_task.
 *
 * @param[in] type      Command type.
 * @param[in] value     mm or degrees.
 * @param[in] speed_mms 0 = default.
 */
static void
cmd_send (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms)
{
    // NOTE: The cast is safe: command types are 0 ... 4.
    subsystem2_cmd_t cmd =
    {
        .type      = (uint8_t) type,
        .value     = value,
        .speed_mms = speed_mms
    };

    CHECK(subsystem2_motion_send(&cmd), "subsystem2_motion_send");
}

/*!
 * @brief Sleep, letting the other tasks run.
 *
 * @param[in] ms Milliseconds.
 */
static void
delay_ms (uint32_t ms)
{
    // NOTE: The cast is safe: test delays are at most a few seconds.
    CHECK(tk_dly_tsk((RELTIM) ms), "tk_dly_tsk");
}

#endif /* APP_TEST */

/*** end of file ***/
