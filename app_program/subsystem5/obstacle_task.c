/** @file obstacle_task.c
 *
 * @brief obstacle_task: forward monitoring, emergency stop, obstacle
 *        scan and side watch (Week 6 report 6.4, FR9, FR10, FR16).
 *
 * @par States (subsystem5_obst_state_t)
 * - MONITORING: servo ahead, one ping per 60 ms. Three pings under 35 cm
 *   raise EV_OBSTACLE_NEAR and enter SLOW. All distances here are from
 *   the robot's front, not from the sensor (S5_SENSOR_SETBACK_MM).
 * - SLOW: still pinging. Three pings at 40 cm or more raise
 *   EV_OBSTACLE_CLEAR and return to MONITORING (the 5 cm gap stops
 *   flicker). From MONITORING or SLOW, two raw pings in a row under
 *   15 cm set S2's MF_ESTOP latch directly (subsystem2_estop_set), raise
 *   EV_OBSTACLE and enter HOLD, so braking never waits for
 *   vehicle_ctrl_task (NFR6).
 * - Lost target: in SLOW, "no echo" right after an echo under 40 cm
 *   means the beam has lost the obstacle (the robot drifted, or the
 *   obstacle is below the beam), not that the path is clear. Such pings
 *   never count towards clearing. The robot keeps driving slowly, and
 *   once S2's odometry says it has covered the last range down to 15 cm,
 *   it stops exactly as for a near echo; the scan then decides. Turns on
 *   the spot do not move the robot's centre, so they never trigger it.
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
 *   cannot see must not move. OB_HOLD or OB_RESUME leaves it; OB_RESUME
 *   also re-arms the sensor.
 *
 * @par Power-up
 * The task waits in HOLD (BOOT_MONITORING 0) until vehicle_ctrl_task
 * sends OB_RESUME at the start of a run.
 *
 * @par
 * Every transition is reported with s5_event_post() (FR16).
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_servo.h"
#include "drv_ultrasonic.h"
#include "obstacle_task.h"
#include "s5_event.h"
#include "s5_links.h"       /* S2: E-stop latch and odometry.           */
#include "s5_math.h"
#include "s5_profile.h"
#include "s5_ranger.h"
#include "s5_scan.h"
#include "subsystem5.h"

/* Forward monitoring (Week 6 report 6.4; preliminary until S5-U05). The
 * distances are from the robot's front; the sensor sits
 * S5_SENSOR_SETBACK_MM behind it, so that is added to each.
 */
#define SLOW_BELOW_MM       (350u + S5_SENSOR_SETBACK_MM)
#define SLOW_EXIT_MM        (400u + S5_SENSOR_SETBACK_MM)
#define STOP_BELOW_MM       (150u + S5_SENSOR_SETBACK_MM)
#define NEAR_PINGS          (3u)
#define CLEAR_PINGS         (3u)
#define STOP_PINGS          (2u)

/* Side watch while the robot drives past the obstacle. The sensor cannot
 * turn fully sideways (drv_servo.h "Reach"), so it holds at the end of
 * its reach on the obstacle's side and looks diagonally forward, a
 * degrees off straight ahead. An echo at range r comes from a point
 * r sin(a) to the side and r cos(a) ahead of the sensor. A "near" echo
 * is one under WATCH_SEEN_MM to the side: the obstacle. Any other echo
 * is "far": something beyond it. Three rules end the watch; each sets
 * how far the obstacle's end is still ahead of the sensor, which
 * s5_avoid.c adds to the tail before turning in (pass_ahead_mm):
 * 1. Near echoes, then none for WATCH_GONE_MM of driving. A shorter gap
 *    is an unstable ping: the obstacle is still there. End ahead: the
 *    last near echo's r cos(a), less what was driven since.
 * 2. No near echo at all, then WATCH_FAR_PINGS far echoes in a row. A
 *    flat side this slanted sends the sound away, so it is silent while
 *    beside the robot, and far echoes start when the beam clears its
 *    back corner (log 2026-11-06_1707: 6 cm of silence, then 80 ... 118
 *    cm). The beam crosses the obstacle's side line WATCH_SIDE_GAP_MM to
 *    the side, so the end is that gap x cot(a) ahead.
 * 3. Nothing at all for WATCH_LIMIT_MM: no information, so passed there,
 *    with the same end ahead as rule 2. Rare indoors: past the obstacle
 *    the room echoes.
 * The obstacle's length is never guessed from its width any more.
 * After the pass the watch goes on, the servo still at the side, until
 * s5_avoid.c sends OB_HOLD before turning in. A near echo then means a
 * rule fired too early (a stray far echo through a gap or a handle): if
 * it puts the obstacle's end more than WATCH_AGAIN_MM further on, the new
 * end is published and EV_OBST_AGAIN makes the tail longer.
 */
#define WATCH_SEEN_MM       (300)       /* Near: obstacle beside, to the */
                                        /* side.                         */
#define WATCH_GONE_MM       (40)        /* Rule 1: no near echo this far.*/
#define WATCH_FAR_PINGS     (2u)        /* Rule 2: far echoes in a row.  */
#define WATCH_LIMIT_MM      (400)       /* Rule 3: silent this far.      */
#define WATCH_AGAIN_MM      (10)        /* End moved on this far: tell.  */
#define WATCH_SIDE_GAP_MM   ((S5_ROBOT_WIDTH_MM / 2) + S5_PATH_MARGIN_MM)
                                        /* Sensor to the obstacle's side */
                                        /* as planned. It had 30 mm more */
                                        /* while the shift used untrimmed*/
                                        /* edges and the robot ended     */
                                        /* further out (s5_profile.c,    */
                                        /* method step 5).               */
#define RULE_ECHO_GONE      (1)         /* Watch rules, for the log.     */
#define RULE_CORNER         (2)
#define RULE_LIMIT          (3)

/* Calibration sweep (S5-U03): hold each angle long enough to measure. */
#define CAL_FIRST_DEG       (DRV_SERVO_RIGHT_LIMIT_DEG) /* The reach.  */
#define CAL_LAST_DEG        (DRV_SERVO_LEFT_LIMIT_DEG)
#define CAL_STEP_DEG        (20)
#define CAL_HOLD_MS         (2000u)

#define NO_RANGE            (-1)        /* Trace value for "no echo".    */

/* Per-ping console log: servo angle and range. While monitoring, only
 * every LOG_MONITOR_EVERY-th ping is printed (about 3 lines a second);
 * during scans, side watch and the calibration sweep every ping is.
 * WARNING: Set LOG_PINGS to 0 for timing tests (NFR11) and the demo:
 * printing adds to obstacle_task's execution time.
 */
#define LOG_PINGS           (1)
#define LOG_MONITOR_EVERY   (5u)

/* 0: wait in HOLD until vehicle_ctrl_task sends OB_RESUME on START, so
 * nothing near the robot at power-up sets MF_ESTOP before a run. Set to
 * 1 only to watch the sensor without starting a run.
 */
#define BOOT_MONITORING     (0)

/** obstacle_task's state and flags, packed into one word (minimum-bit
 *  requirement).
 */
typedef struct
{
    unsigned int state          : 3;    /* subsystem5_obst_state_t.      */
    unsigned int b_track        : 1;    /* Following a near echo.        */
    unsigned int b_watch_seen   : 1;    /* Side watch heard the obstacle.*/
    unsigned int b_watch_passed : 1;    /* Passed; watching the tail.    */
} obst_flags_t;

/* The state enum must fit its field; a misfit stops the build here. */
typedef char obst_state_check[(S5_OBST_FAULT <= S5_OBST_STATE_MASK)
                              ? 1 : -1];

static obst_flags_t             g_obst           = {S5_OBST_HOLD, false,
                                                    false, false};
static subsystem5_obst_status_t g_status         = {0};
static uint32_t                 g_near_run       = 0u;
static uint32_t                 g_clear_run      = 0u;
static uint32_t                 g_stop_run       = 0u;
static int32_t                  g_track_mm       = 0;   /* Last near echo. */
static int32_t                  g_track_odom_mm  = 0;   /* Odom at it.     */
static s5_profile_t             g_profile        = {0};
static s5_scan_result_t         g_scan           = {0};
static int32_t                  g_watch_start_mm = 0;
static int32_t                  g_watch_deg      = DRV_SERVO_AHEAD_DEG;
static int32_t                  g_watch_beam_mm  = 0;   /* Rules 2, 3:   */
                                                        /* end ahead.    */
static uint32_t                 g_watch_far_run  = 0u;
static int32_t                  g_watch_end_mm   = 0;   /* Odometry of   */
                                                        /* the end, as   */
                                                        /* the sensor's. */
static int32_t                  g_watch_begin_mm = 0;   /* Odometry when */
                                                        /* it began.     */
static int32_t                  g_watch_ahead_mm = 0;   /* Last echo's   */
                                                        /* point, ahead. */
static int32_t                  g_watch_last_mm  = 0;   /* Odometry when */
                                                        /* last seen.    */

static void           handle_requests(uint32_t requests);
static void           step_state(void);
static void           monitor_step(void);
static void           update_monitor(s5_ranger_reading_t const * p_reading);
static void           update_track(s5_ranger_reading_t const * p_reading);
static void           check_lost_target(void);
static void           stop_robot(uint32_t dist_mm);
static void           run_scan(void);
static void           publish_scan_profile(void);
static void           publish_clear_profile(void);
static s5_scan_step_t scan_measure(int16_t angle_deg, uint16_t * p_dist_mm);
static void           start_watch(int32_t side_deg);
static void           watch_step(void);
static void           watch_passed(int32_t rule, int32_t end_ahead_mm,
                                   int32_t driven_mm);
static void           finish_watch(void);
static void           watch_again(int32_t ahead_mm);
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
        bool b_idle = (S5_OBST_HOLD == g_obst.state)
                      || (S5_OBST_FAULT == g_obst.state);

        handle_requests(subsystem5_obst_take(b_idle));
        step_state();
    }
}

/*!
 * @brief Carry out pending requests: OB_HOLD (or else OB_RESUME) first,
 *        then the first of the others. A scan sent together with a HOLD
 *        is therefore still carried out, not lost.
 *
 * @param[in] requests OB_xxx bits taken from flg_obst.
 */
static void
handle_requests (uint32_t requests)
{
    if (0u != (requests & OB_HOLD))
    {
        enter_hold();
    }
    else if (0u != (requests & OB_RESUME))
    {
        // A RESUME follows a START, so it also re-arms a failed sensor;
        // if the sensor is still broken it faults again within 0.3 s.
        s5_ranger_reset_fault();
        enter_monitoring();
    }
    else
    {
        // No change of mode requested.
    }

    if (0u == (requests & ~(OB_HOLD | OB_RESUME)))
    {
        // No job requested.
    }
    else if (S5_OBST_FAULT == g_obst.state)
    {
        // In FAULT only OB_HOLD and OB_RESUME are accepted.
    }
    else if (0u != (requests & OB_SCAN_REQ))
    {
        run_scan();
    }
    else if (0u != (requests & OB_WATCH_LEFT))
    {
        start_watch(DRV_SERVO_LEFT_LIMIT_DEG);
    }
    else if (0u != (requests & OB_WATCH_RIGHT))
    {
        start_watch(DRV_SERVO_RIGHT_LIMIT_DEG);
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
    switch ((subsystem5_obst_state_t) g_obst.state)
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
    else if ((S5_RANGER_CLEAR == reading.status) && g_obst.b_track)
    {
        // The obstacle was close a moment ago: the beam has lost it, so
        // this ping says nothing about the path being clear.
        check_lost_target();
    }
    else if (s5_ranger_is_conclusive(&reading))
    {
        update_monitor(&reading);
        update_track(&reading);
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
        stop_robot(p_reading->dist_mm);
    }
    else if ((S5_OBST_MONITORING == g_obst.state) && (g_near_run >= NEAR_PINGS))
    {
        set_state(S5_OBST_SLOW);
        subsystem5_mission_signal(EV_OBSTACLE_NEAR);
        s5_event_post(S5_EVENT_OBST_NEAR, p_reading->dist_mm,
                      DRV_SERVO_AHEAD_DEG);
    }
    else if ((S5_OBST_SLOW == g_obst.state) && (g_clear_run >= CLEAR_PINGS))
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
 * @brief In SLOW, remember the last echo under SLOW_EXIT_MM and where
 *        the robot was; anything else ends the tracking.
 *
 * @param[in] p_reading A conclusive reading, already counted.
 */
static void
update_track (s5_ranger_reading_t const * p_reading)
{
    g_obst.b_track = (S5_OBST_SLOW == g_obst.state)
                && s5_ranger_is_within(p_reading, SLOW_EXIT_MM);

    if (g_obst.b_track)
    {
        // NOTE: The cast widens uint16_t, so no value is lost.
        g_track_mm      = (int32_t) p_reading->dist_mm;
        g_track_odom_mm = s5_links_odom_dist_mm();
    }
}

/*!
 * @brief The tracked obstacle gave no echo: stop once the distance
 *        driven since its last echo brings the robot to STOP_BELOW_MM.
 */
static void
check_lost_target (void)
{
    int32_t driven_mm = s5_links_odom_dist_mm() - g_track_odom_mm;
    int32_t left_mm   = g_track_mm - driven_mm;

    // NOTE: The cast is safe: STOP_BELOW_MM is 150.
    if (left_mm <= (int32_t) STOP_BELOW_MM)
    {
        s5_event_trace("MONITOR lost close: last_mm driven_mm", g_track_mm,
                       driven_mm);

        // NOTE: The cast is safe: left_mm is 1 ... 150 when converted.
        stop_robot((left_mm > 0) ? (uint32_t) left_mm : 0u);
    }
}

/*!
 * @brief Brake now, tell the controller, and wait for a scan request.
 *
 * @param[in] dist_mm Range that triggered the stop (measured or
 *                    estimated), for the event.
 */
static void
stop_robot (uint32_t dist_mm)
{
    s5_links_motion_estop();
    subsystem5_mission_signal(EV_OBSTACLE);
    s5_event_post(S5_EVENT_OBST_STOP, dist_mm, DRV_SERVO_AHEAD_DEG);
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
    uint32_t seq = s5_profile_next_seq(g_profile.seq);

    s5_profile_build(g_scan.points, g_scan.count, &g_scan.left,
                     &g_scan.right, &g_profile);
    g_profile.seq = seq & S5_PROFILE_SEQ_MASK;
    subsystem5_profile_put(&g_profile);

    s5_event_post(S5_EVENT_PROFILE, g_profile.width_mm,
                  s5_profile_angle(&g_profile));
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
    uint32_t seq = s5_profile_next_seq(g_profile.seq);

    s5_profile_set_clear(&g_profile);
    g_profile.seq = seq & S5_PROFILE_SEQ_MASK;
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
 * @brief Point the sensor as far towards the obstacle's side as it can
 *        turn and start watching (OB_WATCH_xxx).
 *
 * @param[in] side_deg DRV_SERVO_LEFT_LIMIT_DEG or
 *                     DRV_SERVO_RIGHT_LIMIT_DEG.
 */
static void
start_watch (int32_t side_deg)
{
    g_obst.b_watch_seen     = false;
    g_watch_start_mm        = s5_links_odom_dist_mm();
    g_watch_begin_mm        = g_watch_start_mm;
    g_watch_last_mm         = g_watch_start_mm;
    g_watch_ahead_mm        = 0;
    g_watch_deg             = side_deg;
    g_watch_far_run         = 0u;
    g_obst.b_watch_passed   = false;
    g_profile.pass_ahead_mm = 0u;

    // Where the beam crosses the obstacle's side line, ahead of the
    // sensor: the planned side gap x cot(a). The reach is well short of
    // straight ahead, so sin(a) is never zero.
    int32_t off_deg = s5_math_abs(side_deg - DRV_SERVO_AHEAD_DEG);

    g_watch_beam_mm = (WATCH_SIDE_GAP_MM * s5_math_cos_q15(off_deg))
                      / s5_math_sin_q15(off_deg);

    point_servo(side_deg);
    set_state(S5_OBST_WATCH);
}

/*!
 * @brief One ping in WATCH (see "Side watch" above).
 */
static void
watch_step (void)
{
    s5_ranger_reading_t reading;
    int32_t             off_deg = s5_math_abs(g_watch_deg
                                              - DRV_SERVO_AHEAD_DEG);
    int32_t             across  = 0;
    int32_t             ahead   = 0;

    take_reading(&reading);

    if (S5_RANGER_OK == reading.status)
    {
        // NOTE: The casts widen uint16_t, so no value is lost.
        across = s5_math_mul_q15((int32_t) reading.dist_mm,
                                 s5_math_sin_q15(off_deg));
        ahead  = s5_math_mul_q15((int32_t) reading.dist_mm,
                                 s5_math_cos_q15(off_deg));
    }

    if (S5_RANGER_FAULT == reading.status)
    {
        enter_fault();
    }
    else if (!s5_ranger_is_conclusive(&reading))
    {
        // Busy or one unanswered ping: wait for a real reading.
    }
    else
    {
        int32_t odom   = s5_links_odom_dist_mm();
        int32_t driven = s5_math_abs(odom - g_watch_begin_mm);
        int32_t unseen = s5_math_abs(odom - g_watch_last_mm);
        bool    b_ok   = (S5_RANGER_OK == reading.status);
        bool    b_near = b_ok && (across < WATCH_SEEN_MM);

        g_watch_far_run = count_run(g_watch_far_run, b_ok && (!b_near));

        if (g_obst.b_watch_passed)
        {
            // On the tail: only an echo of the obstacle matters.
            if (b_near)
            {
                watch_again(ahead);
            }
        }
        else if (b_near)
        {
            // Beside the robot. The depth is measured from the first
            // sight.
            if (!g_obst.b_watch_seen)
            {
                g_obst.b_watch_seen = true;
                g_watch_start_mm    = odom;
            }

            g_watch_last_mm  = odom;
            g_watch_ahead_mm = ahead;
        }
        else if (g_obst.b_watch_seen && (unseen >= WATCH_GONE_MM))
        {
            watch_passed(RULE_ECHO_GONE, g_watch_ahead_mm - unseen, driven);
        }
        else if ((!g_obst.b_watch_seen) && (g_watch_far_run >= WATCH_FAR_PINGS))
        {
            watch_passed(RULE_CORNER, g_watch_beam_mm, driven);
        }
        else if ((!g_obst.b_watch_seen) && (driven >= WATCH_LIMIT_MM))
        {
            watch_passed(RULE_LIMIT, g_watch_beam_mm, driven);
        }
        else
        {
            // Keep watching (a short gap in near echoes is an unstable
            // ping: the obstacle is still there).
        }
    }
}

/*!
 * @brief The side watch has ended: note how far the obstacle's end is
 *        still ahead of the sensor, log which rule ended it, report it.
 *
 * @param[in] rule         RULE_ECHO_GONE, RULE_CORNER or RULE_LIMIT.
 * @param[in] end_ahead_mm Obstacle's end ahead of the sensor, mm.
 * @param[in] driven_mm    Driven since the watch began, mm.
 */
static void
watch_passed (int32_t rule, int32_t end_ahead_mm, int32_t driven_mm)
{
    int32_t end_ahead = end_ahead_mm;

    if (end_ahead < 0)
    {
        end_ahead = 0;
    }

    g_profile.pass_ahead_mm = s5_profile_mm(end_ahead) & S5_PROFILE_MM_MASK;
    g_watch_end_mm          = s5_links_odom_dist_mm() + end_ahead;
    s5_event_trace("WATCH passed: rule (1 echo, 2 corner, 3 limit) "
                   "driven_mm", rule, driven_mm);
    s5_event_trace("WATCH end ahead of sensor: mm 0", end_ahead, 0);

    if (!g_obst.b_watch_seen)
    {
        g_watch_start_mm = s5_links_odom_dist_mm();
    }

    finish_watch();
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
    int32_t depth_mm = s5_math_abs(s5_links_odom_dist_mm()
                                   - g_watch_start_mm);

    g_profile.depth_mm = s5_profile_mm(depth_mm) & S5_PROFILE_MM_MASK;
    subsystem5_profile_put(&g_profile);

    // NOTE: The cast is safe: depth_mm is never negative here.
    s5_event_post(S5_EVENT_OBST_PASSED, (uint32_t) depth_mm, g_watch_deg);

    // Keep watching on the tail (see "Side watch" above).
    g_obst.b_watch_passed = true;
    subsystem5_mission_signal(EV_OBST_PASSED);
}

/*!
 * @brief On the tail the obstacle echoed again: if its end is now more
 *        than WATCH_AGAIN_MM further on, publish the new end and tell
 *        s5_avoid.c (EV_OBST_AGAIN).
 *
 * @param[in] ahead_mm The echo's point, ahead of the sensor.
 */
static void
watch_again (int32_t ahead_mm)
{
    int32_t end_mm = s5_links_odom_dist_mm() + ahead_mm;

    if (end_mm > (g_watch_end_mm + WATCH_AGAIN_MM))
    {
        g_watch_end_mm = end_mm;

        g_profile.pass_ahead_mm = s5_profile_mm(ahead_mm)
                                  & S5_PROFILE_MM_MASK;
        subsystem5_profile_put(&g_profile);
        s5_event_trace("WATCH obstacle again on the tail: ahead_mm 0",
                       ahead_mm, 0);
        subsystem5_mission_signal(EV_OBST_AGAIN);
    }
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
    g_near_run     = 0u;
    g_clear_run    = 0u;
    g_stop_run     = 0u;
    g_obst.b_track = false;
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
    s5_links_motion_estop();
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
    if (state != g_obst.state)
    {
        // NOTE: The casts are safe: both states are small enum values.
        s5_event_trace("STATE from to", (int32_t) g_obst.state,
                       (int32_t) state);
    }

    g_obst.state   = (uint32_t) state & S5_OBST_STATE_MASK;
    g_status.state = g_obst.state;
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

    // NOTE: The casts are safe: the servo angle is 0 ... 180 and the
    // range at most 0xFFFF; both are clamped or masked to their fields.
    g_status.state         = g_obst.state;
    g_status.b_range_valid = (S5_RANGER_OK == p_reading->status);
    g_status.range_mm      = s5_profile_mm((int32_t) p_reading->dist_mm)
                             & S5_PROFILE_MM_MASK;
    g_status.servo_deg     = (uint32_t) drv_servo_angle() & S5_OBST_DEG_MASK;
    g_status.ping_count    = (g_status.ping_count + 1u) & S5_OBST_COUNT_MASK;
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
    bool b_monitoring = (S5_OBST_MONITORING == g_obst.state)
                        || (S5_OBST_SLOW == g_obst.state);

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
