/** @file obstacle_task.c
 *
 * @brief obstacle_task: forward monitoring, emergency stop, obstacle
 *        scan and side watch (Week 6 report 6.4, FR9, FR10, FR16).
 *
 * @par States (subsystem5_obst_state_t)
 * - MONITORING: servo ahead, one ping per 60 ms. Three pings under 35 cm
 *   raise EV_OBSTACLE_NEAR and enter SLOW.
 * - SLOW: still pinging. Three pings at 40 cm or more raise
 *   EV_OBSTACLE_CLEAR and return to MONITORING (the 5 cm gap stops
 *   flicker). From MONITORING or SLOW, two raw pings in a row under
 *   15 cm set MF_ESTOP directly in S2's flg_motion, raise EV_OBSTACLE
 *   and enter HOLD, so braking never waits for vehicle_ctrl_task (NFR6).
 * - SCANNING: coarse and fine scan (s5_scan.c), then the profile is
 *   published with EV_PROFILE_READY and the task enters HOLD.
 * - WATCH: servo pointed at the obstacle's side while the robot drives
 *   past it. Once the obstacle is seen and then lost for two pings,
 *   EV_OBST_PASSED is raised and the distance driven meanwhile is stored
 *   as the obstacle's depth. This is the "keep scanning to find the
 *   depth" step of the brief; the Week 6 report instead drove a fixed
 *   distance based on the width, which is a different dimension.
 * - HOLD: idle, servo ahead, no automatic stops. vehicle_ctrl_task must
 *   never be stopped by its own obstacle while it drives the bypass.
 * - FAULT: the sensor failed. MF_ESTOP is set, because a robot that
 *   cannot see must not move. Only OB_RESUME (after a RESET) leaves it.
 *
 * @par
 * Every transition is reported with s5_event_post() (FR16).
 */

#include <stdbool.h>
#include <stdint.h>

#include "../subsystem2/subsystem2.h"   /* flg_motion, MF_ESTOP, odom.   */
#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "obstacle_task.h"
#include "s5_event.h"
#include "s5_math.h"
#include "s5_profile.h"
#include "s5_ranger.h"
#include "s5_scan.h"
#include "subsystem5.h"

/* Forward monitoring (Week 6 report 6.4; preliminary until S5-U05). */
#define SLOW_BELOW_MM       (350u)
#define SLOW_EXIT_MM        (400u)
#define STOP_BELOW_MM       (150u)
#define NEAR_PINGS          (3u)
#define CLEAR_PINGS         (3u)
#define STOP_PINGS          (2u)

/* Side watch while the robot drives past the obstacle. */
#define WATCH_SEEN_MM       (300u)      /* Obstacle beside the robot.    */
#define WATCH_PASSED_MM     (400u)      /* Nothing beside: passed.       */
#define WATCH_PASSED_PINGS  (2u)

/* Calibration sweep (S5-U03): hold each angle long enough to measure. */
#define CAL_FIRST_DEG       (30)
#define CAL_LAST_DEG        (150)
#define CAL_STEP_DEG        (30)
#define CAL_HOLD_MS         (2000u)

#define NO_RANGE            (-1)        /* Trace value for "no echo".    */
#define DEPTH_MAX_MM        (0xFFFE)

/* Per-ping console log: servo angle and range. While monitoring, only
 * every LOG_MONITOR_EVERY-th ping is printed (about 3 lines a second);
 * during scans, side watch and the calibration sweep every ping is.
 * WARNING: Set LOG_PINGS to 0 for timing tests (NFR11) and the demo:
 * printing adds to obstacle_task's execution time.
 */
#define LOG_PINGS           (1)
#define LOG_MONITOR_EVERY   (5u)

/* TODO: Set to 0 once vehicle_ctrl_task sends OB_RESUME on EV_START.
 * Until then the task monitors from power-up, so the sensor can be
 * tested without the controller.
 */
#define BOOT_MONITORING     (1)

static subsystem5_obst_state_t  g_state          = S5_OBST_HOLD;
static subsystem5_obst_status_t g_status         = {0};
static uint32_t                 g_near_run       = 0u;
static uint32_t                 g_clear_run      = 0u;
static uint32_t                 g_stop_run       = 0u;
static s5_profile_t             g_profile        = {0};
static s5_scan_result_t         g_scan           = {0};
static bool                     g_b_watch_seen   = false;
static int32_t                  g_watch_start_mm = 0;
static uint32_t                 g_watch_far_run  = 0u;
static int32_t                  g_watch_deg      = DRV_SERVO_AHEAD_DEG;

static void           handle_requests(uint32_t requests);
static void           step_state(void);
static void           monitor_step(void);
static void           update_monitor(s5_ranger_reading_t const * p_reading);
static void           stop_robot(s5_ranger_reading_t const * p_reading);
static void           run_scan(void);
static void           publish_scan_profile(void);
static void           publish_clear_profile(void);
static s5_scan_step_t scan_measure(int16_t angle_deg, uint16_t * p_dist_mm);
static void           start_watch(int32_t side_deg);
static void           watch_step(void);
static void           finish_watch(void);
static void           run_cal_sweep(void);
static void           enter_monitoring(void);
static void           enter_hold(void);
static void           enter_fault(void);
static void           set_state(subsystem5_obst_state_t state);
static void           take_reading(s5_ranger_reading_t * p_reading);
static void           log_ping(s5_ranger_reading_t const * p_reading);
#if LOG_PINGS
static char const *   ping_reason(s5_ranger_status_t status);
#endif
static void           point_servo(int32_t angle_deg);
static void           request_estop(void);
static int32_t        odom_dist_mm(void);
static uint32_t       count_run(uint32_t run, bool b_condition);

/*!
 * @brief obstacle_task entry point (created by subsystem5_start()).
 *
 * @param[in] stacd   Start code from tk_sta_tsk(); unused.
 * @param[in] p_exinf Extended information; unused.
 */
void
obstacle_task (INT stacd, void * p_exinf)
{
    // The casts mark the kernel's start-up arguments as unused.
    (void) stacd;
    (void) p_exinf;

    // Give the serial monitor time to connect before the first line.
    subsystem5_delay_ms(S5_BOOT_WAIT_MS);

    // This task owns the sensor and the servo, so it sets them up itself.
    // It runs after every subsystemN_init(), so S2's PWM reset is over.
    drv_ultrasonic_init();

    uint32_t settle_ms = drv_servo_init();

    subsystem5_delay_ms(settle_ms);
    s5_ranger_init();

    // NOTE: The casts are safe: both values are a few thousand ms.
    s5_event_trace("BOOT obstacle_task up: boot_wait_ms servo_settle_ms",
                   (int32_t) S5_BOOT_WAIT_MS, (int32_t) settle_ms);

#if BOOT_MONITORING
    enter_monitoring();
#else
    enter_hold();
#endif

    for (;;)
    {
        // Block while idle; poll between pings while monitoring.
        bool b_idle = (S5_OBST_HOLD == g_state)
                      || (S5_OBST_FAULT == g_state);

        handle_requests(subsystem5_obst_take(b_idle));
        step_state();
    }
}

/*!
 * @brief Carry out the highest-priority pending request.
 *
 * @param[in] requests OB_xxx bits taken from flg_obst.
 */
static void
handle_requests (uint32_t requests)
{
    if (0u == requests)
    {
        // Nothing to do.
    }
    else if (0u != (requests & OB_HOLD))
    {
        enter_hold();
    }
    else if (S5_OBST_FAULT == g_state)
    {
        // In FAULT only OB_RESUME (sent after a RESET) is accepted.
        if (0u != (requests & OB_RESUME))
        {
            s5_ranger_reset_fault();
            enter_monitoring();
        }
    }
    else if (0u != (requests & OB_RESUME))
    {
        enter_monitoring();
    }
    else if (0u != (requests & OB_SCAN_REQ))
    {
        run_scan();
    }
    else if (0u != (requests & OB_WATCH_LEFT))
    {
        start_watch(DRV_SERVO_LEFT_DEG);
    }
    else if (0u != (requests & OB_WATCH_RIGHT))
    {
        start_watch(DRV_SERVO_RIGHT_DEG);
    }
    else if (0u != (requests & OB_CAL_SWEEP))
    {
        run_cal_sweep();
    }
    else
    {
        // Unknown bits are ignored.
    }
}

/*!
 * @brief Do one step of the current state's periodic work.
 */
static void
step_state (void)
{
    switch (g_state)
    {
        case S5_OBST_MONITORING:
        // SLOW pings in the same way as MONITORING.
        case S5_OBST_SLOW:
            monitor_step();
        break;

        case S5_OBST_WATCH:
            watch_step();
        break;

        default:
            // HOLD and FAULT wait for a request; SCANNING has finished.
        break;
    }
}

/*!
 * @brief One forward ping in MONITORING or SLOW.
 */
static void
monitor_step (void)
{
    s5_ranger_reading_t reading;

    take_reading(&reading);

    if (S5_RANGER_FAULT == reading.status)
    {
        enter_fault();
    }
    else if (s5_ranger_is_conclusive(&reading))
    {
        update_monitor(&reading);
    }
    else
    {
        // Busy or one unanswered ping: the counters wait for a real one.
    }
}

/*!
 * @brief Update the near, clear and stop counters with one reading and
 *        act on them.
 *
 * @param[in] p_reading A conclusive (OK or CLEAR) reading.
 */
static void
update_monitor (s5_ranger_reading_t const * p_reading)
{
    g_stop_run  = count_run(g_stop_run,
                            s5_ranger_is_within(p_reading, STOP_BELOW_MM));
    g_near_run  = count_run(g_near_run,
                            s5_ranger_is_within(p_reading, SLOW_BELOW_MM));
    g_clear_run = count_run(g_clear_run,
                            !s5_ranger_is_within(p_reading, SLOW_EXIT_MM));

    if (g_stop_run >= STOP_PINGS)
    {
        stop_robot(p_reading);
    }
    else if ((S5_OBST_MONITORING == g_state) && (g_near_run >= NEAR_PINGS))
    {
        set_state(S5_OBST_SLOW);
        subsystem5_mission_signal(EV_OBSTACLE_NEAR);
        s5_event_post(S5_EVENT_OBST_NEAR, p_reading->dist_mm,
                      DRV_SERVO_AHEAD_DEG);
    }
    else if ((S5_OBST_SLOW == g_state) && (g_clear_run >= CLEAR_PINGS))
    {
        set_state(S5_OBST_MONITORING);
        subsystem5_mission_signal(EV_OBSTACLE_CLEAR);
        s5_event_post(S5_EVENT_OBST_CLEAR, p_reading->dist_mm,
                      DRV_SERVO_AHEAD_DEG);
    }
    else
    {
        // No change.
    }
}

/*!
 * @brief Brake now, tell the controller, and wait for a scan request.
 *
 * @param[in] p_reading The reading that triggered the stop.
 */
static void
stop_robot (s5_ranger_reading_t const * p_reading)
{
    request_estop();
    subsystem5_mission_signal(EV_OBSTACLE);
    s5_event_post(S5_EVENT_OBST_STOP, p_reading->dist_mm,
                  DRV_SERVO_AHEAD_DEG);
    enter_hold();
}

/*!
 * @brief Scan and profile the obstacle (OB_SCAN_REQ).
 */
static void
run_scan (void)
{
    set_state(S5_OBST_SCANNING);
    s5_event_post(S5_EVENT_SCAN_START, 0u, drv_servo_angle());

    s5_scan_outcome_t outcome = s5_scan_run(scan_measure, &g_scan);

    switch (outcome)
    {
        case S5_SCAN_DONE:
            publish_scan_profile();
        break;

        case S5_SCAN_CLEAR:
            publish_clear_profile();
        break;

        case S5_SCAN_ABORTED:
            s5_event_post(S5_EVENT_SCAN_ABORT, 0u, drv_servo_angle());
            enter_hold();
        break;

        // A sensor fault and an unknown outcome are handled alike.
        case S5_SCAN_FAULT:
        default:
            enter_fault();
        break;
    }
}

/*!
 * @brief Build, publish and report the profile of a completed scan.
 */
static void
publish_scan_profile (void)
{
    uint32_t seq = g_profile.seq + 1u;

    s5_profile_build(g_scan.points, g_scan.count, &g_scan.left,
                     &g_scan.right, &g_profile);
    g_profile.seq = seq;
    subsystem5_profile_put(&g_profile);

    s5_event_post(S5_EVENT_PROFILE, g_profile.width_mm,
                  g_profile.closest_deg);
    s5_event_trace("PROFILE closest_mm width_mm", g_profile.closest_mm,
                   g_profile.width_mm);
    s5_event_trace("PROFILE clear_left_mm clear_right_mm",
                   g_profile.clear_left_mm, g_profile.clear_right_mm);
    s5_event_trace("PROFILE shift_left_mm shift_right_mm",
                   g_profile.shift_left_mm, g_profile.shift_right_mm);

    enter_hold();
    subsystem5_mission_signal(EV_PROFILE_READY);
}

/*!
 * @brief Publish and report a scan that found nothing in range.
 */
static void
publish_clear_profile (void)
{
    uint32_t seq = g_profile.seq + 1u;

    s5_profile_set_clear(&g_profile);
    g_profile.seq = seq;
    subsystem5_profile_put(&g_profile);
    s5_event_post(S5_EVENT_SCAN_CLEAR, 0u, DRV_SERVO_AHEAD_DEG);

    enter_hold();
    subsystem5_mission_signal(EV_OBSTACLE_CLEAR | EV_PROFILE_READY);
}

/*!
 * @brief Point the sensor and measure, for s5_scan_run().
 *
 * @param[in]  angle_deg Servo angle.
 * @param[out] p_dist_mm Range, or S5_PROFILE_NO_ECHO.
 *
 * @return OK, ABORT if OB_HOLD is pending, or FAULT.
 */
static s5_scan_step_t
scan_measure (int16_t angle_deg, uint16_t * p_dist_mm)
{
    s5_scan_step_t step = S5_SCAN_STEP_ABORT;

    if (!subsystem5_obst_pending(OB_HOLD))
    {
        s5_ranger_reading_t reading;

        point_servo(angle_deg);
        take_reading(&reading);

        // Retry once if the sensor was busy or missed this ping.
        if ((!s5_ranger_is_conclusive(&reading))
            && (S5_RANGER_FAULT != reading.status))
        {
            take_reading(&reading);
        }

        step = (S5_RANGER_FAULT == reading.status) ? S5_SCAN_STEP_FAULT
                                                   : S5_SCAN_STEP_OK;
        *p_dist_mm = (S5_RANGER_OK == reading.status) ? reading.dist_mm
                                                      : S5_PROFILE_NO_ECHO;
    }

    return (step);
}

/*!
 * @brief Point the sensor sideways and start watching (OB_WATCH_xxx).
 *
 * @param[in] side_deg DRV_SERVO_LEFT_DEG or DRV_SERVO_RIGHT_DEG.
 */
static void
start_watch (int32_t side_deg)
{
    g_b_watch_seen   = false;
    g_watch_start_mm = 0;
    g_watch_far_run  = 0u;
    g_watch_deg      = side_deg;

    point_servo(side_deg);
    set_state(S5_OBST_WATCH);
}

/*!
 * @brief One sideways ping in WATCH.
 */
static void
watch_step (void)
{
    s5_ranger_reading_t reading;

    take_reading(&reading);

    if (S5_RANGER_FAULT == reading.status)
    {
        enter_fault();
    }
    else if (!s5_ranger_is_conclusive(&reading))
    {
        // Busy or one unanswered ping: wait for a real reading.
    }
    else if (!g_b_watch_seen)
    {
        // Stage 1: wait until the obstacle is beside the robot.
        if (s5_ranger_is_within(&reading, WATCH_SEEN_MM))
        {
            g_b_watch_seen   = true;
            g_watch_start_mm = odom_dist_mm();
        }
    }
    else
    {
        // Stage 2: wait until it has been left behind.
        g_watch_far_run = count_run(g_watch_far_run,
                                    !s5_ranger_is_within(&reading,
                                                         WATCH_PASSED_MM));

        if (g_watch_far_run >= WATCH_PASSED_PINGS)
        {
            finish_watch();
        }
    }
}

/*!
 * @brief Record the obstacle's depth and report that it was passed.
 *
 * NOTE: The depth includes the beam's spread and relies on S2's odometry;
 * it reads 0 until motion_task publishes odom.
 */
static void
finish_watch (void)
{
    int32_t depth_mm = s5_math_abs(odom_dist_mm() - g_watch_start_mm);

    if (depth_mm > DEPTH_MAX_MM)
    {
        depth_mm = DEPTH_MAX_MM;
    }

    // NOTE: The cast is safe: depth_mm is 0 ... DEPTH_MAX_MM here.
    g_profile.depth_mm = (uint16_t) depth_mm;
    subsystem5_profile_put(&g_profile);

    // NOTE: The cast is safe: depth_mm is never negative here.
    s5_event_post(S5_EVENT_OBST_PASSED, (uint32_t) depth_mm, g_watch_deg);

    enter_hold();
    subsystem5_mission_signal(EV_OBST_PASSED);
}

/*!
 * @brief Slow sweep for calibrating the servo angles (OB_CAL_SWEEP,
 *        S5-U03): holds each coarse angle for CAL_HOLD_MS so it can be
 *        checked with a protractor, and prints the range there.
 */
static void
run_cal_sweep (void)
{
    set_state(S5_OBST_SCANNING);

    for (int32_t angle = CAL_FIRST_DEG;
         (angle <= CAL_LAST_DEG) && (!subsystem5_obst_pending(OB_HOLD));
         angle += CAL_STEP_DEG)
    {
        s5_ranger_reading_t reading;

        point_servo(angle);
        subsystem5_delay_ms(CAL_HOLD_MS);
        take_reading(&reading);

        // NOTE: The cast widens uint16_t, so no value is lost.
        s5_event_trace("CAL angle_deg range_mm", angle,
                       (S5_RANGER_OK == reading.status)
                       ? (int32_t) reading.dist_mm : NO_RANGE);
    }

    enter_hold();
}

/*!
 * @brief Point ahead, clear the counters and start monitoring.
 */
static void
enter_monitoring (void)
{
    point_servo(DRV_SERVO_AHEAD_DEG);
    g_near_run  = 0u;
    g_clear_run = 0u;
    g_stop_run  = 0u;
    set_state(S5_OBST_MONITORING);
}

/*!
 * @brief Point ahead and wait for a request.
 */
static void
enter_hold (void)
{
    point_servo(DRV_SERVO_AHEAD_DEG);
    set_state(S5_OBST_HOLD);
}

/*!
 * @brief The sensor failed: stop the robot and report the fault.
 */
static void
enter_fault (void)
{
    set_state(S5_OBST_FAULT);
    request_estop();
    subsystem5_fault_set(S5_FAULT_NO_ECHO);
    subsystem5_mission_signal(EV_FAULT);
    s5_event_post(S5_EVENT_SENSOR_FAULT, 0u, drv_servo_angle());
    point_servo(DRV_SERVO_AHEAD_DEG);
}

/*!
 * @brief Change state and publish it.
 *
 * @param[in] state The new state.
 */
static void
set_state (subsystem5_obst_state_t state)
{
    if (state != g_state)
    {
        // NOTE: The casts are safe: both states are small enum values.
        s5_event_trace("STATE from to", (int32_t) g_state, (int32_t) state);
    }

    g_state        = state;
    g_status.state = state;
    subsystem5_obst_status_put(&g_status);
}

/*!
 * @brief Measure once and publish the live status.
 *
 * @param[out] p_reading The measurement.
 */
static void
take_reading (s5_ranger_reading_t * p_reading)
{
    s5_ranger_measure(p_reading);

    // NOTE: The cast is safe: the servo angle is 0 ... 180.
    g_status.state         = g_state;
    g_status.b_range_valid = (S5_RANGER_OK == p_reading->status);
    g_status.range_mm      = p_reading->dist_mm;
    g_status.servo_deg     = (int16_t) drv_servo_angle();
    g_status.ping_count++;
    subsystem5_obst_status_put(&g_status);
    log_ping(p_reading);
}

/*!
 * @brief Print a ping on the console if LOG_PINGS is on (rate-limited
 *        while monitoring).
 *
 * @param[in] p_reading The measurement just taken.
 */
static void
log_ping (s5_ranger_reading_t const * p_reading)
{
#if LOG_PINGS
    bool b_monitoring = (S5_OBST_MONITORING == g_state)
                        || (S5_OBST_SLOW == g_state);

    if ((!b_monitoring)
        || (0u == (g_status.ping_count % LOG_MONITOR_EVERY)))
    {
        // NOTE: The cast widens uint16_t, so no value is lost.
        s5_event_log_ping(drv_servo_angle(),
                          (S5_RANGER_OK == p_reading->status)
                          ? (int32_t) p_reading->dist_mm : NO_RANGE,
                          p_reading->echo_us,
                          ping_reason(p_reading->status));
    }
#else
    // The cast marks the reading as unused while logging is off.
    (void) p_reading;
#endif
}

#if LOG_PINGS
/*!
 * @brief Plain-language reason a ping gave no range, for the console.
 *
 * @param[in] status The ranger's status for the ping.
 *
 * @return A short explanation.
 */
static char const *
ping_reason (s5_ranger_status_t status)
{
    char const * p_why = "unknown";

    switch (status)
    {
        case S5_RANGER_OK:
            p_why = "";
        break;

        case S5_RANGER_CLEAR:
            p_why = "clear: no echo heard back within 4 m";
        break;

        case S5_RANGER_BUSY:
            p_why = "busy: ECHO still high from the last ping";
        break;

        case S5_RANGER_INVALID:
            p_why = "no answer: ECHO never went high";
        break;

        case S5_RANGER_FAULT:
            p_why = "sensor fault";
        break;

        default:
            // Keep "unknown".
        break;
    }

    return (p_why);
}
#endif /* LOG_PINGS */

/*!
 * @brief Turn the servo and wait until it has settled.
 *
 * @param[in] angle_deg Target angle, 0 ... 180.
 */
static void
point_servo (int32_t angle_deg)
{
    subsystem5_delay_ms(drv_servo_move(angle_deg));
}

/*!
 * @brief Set S2's emergency-stop latch (IF-05). motion_task brakes on its
 *        next 20 ms cycle; only vehicle_ctrl_task clears the latch.
 */
static void
request_estop (void)
{
    CHECK(tk_set_flg(flg_motion, MF_ESTOP), "flg_motion");
}

/*!
 * @brief Distance driven since power-up, from S2's odometry (IF-06).
 *
 * @return Signed distance in mm.
 */
static int32_t
odom_dist_mm (void)
{
    ODOM odom = {0};

    s2_odom_get(&odom);

    return (odom.dist_mm);
}

/*!
 * @brief Count consecutive readings that meet a condition.
 *
 * @param[in] run         The count so far.
 * @param[in] b_condition Whether this reading meets the condition.
 *
 * @return run + 1 if it does, otherwise 0.
 */
static uint32_t
count_run (uint32_t run, bool b_condition)
{
    return (b_condition ? (run + 1u) : 0u);
}

/*** end of file ***/
