/** @file s5_mission.c
 *
 * @brief The mission state machine (Week 6 report section 5).
 *
 * @par Rules taken from the Week 6 report
 * - READY: motors braked, line following off, obstacle_task idle. A run
 *   starts only with START (Button 1 or the dashboard).
 * - While running, EV_IMPACT, a stall or a sensor fault enters FAULT
 *   (MF_ESTOP set, motors braked). Only RESET (Button 1 held 2 s, or the
 *   dashboard) leaves FAULT, back to READY with the motors still braked.
 * - EV_STOP ends the run in FINISHED, with a run summary.
 * - A barcode command waits until the junction: S5_JUNCTION_MM of
 *   travel (assumption A11) or EV_JUNCTION, whichever comes first. A
 *   newer barcode replaces an unexecuted one.
 * - An obstacle stop from any running state starts OBSTACLE_AVOIDANCE;
 *   a pending barcode command is then dropped (and logged).
 *
 * @par Without S3 (s5_links_line_is_real() false)
 * The robot drives straight in FOLLOW, and after a barcode turn it
 * carries on at once instead of searching for the line.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_avoid.h"
#include "s5_event.h"
#include "s5_links.h"
#include "s5_mission.h"
#include "subsystem5.h"

#define SPEED_CRUISE_MMS    (300u)      /* Week 6 Appendix D.            */
#define SPEED_SLOW_MMS      (150u)      /* Under 35 cm (FR9).            */
#define CAL_TIMEOUT_MS      (3000u)     /* Then go on without the IMU.   */
#define S5_JUNCTION_MM      (150)       /* TODO: measure (A11, INT-01).  */
#define REACQUIRE_MM        (300)       /* NFR10.                        */
#define TURN_MS_PER_DEG     (20u)
#define MOTION_SLACK_MS     (2000u)
#define RECOVERY_STEPS      (3u)        /* At most 3 retries (FR15).     */
#define TURN_LEFT_DEG       (90)        /* Barcode turns (FR6).          */
#define TURN_RIGHT_DEG      (-90)
#define TURN_UTURN_DEG      (180)
#define MS_PER_S            (1000u)
#define FAULTS_LATCHED      (S5_FAULT_STALL | S5_FAULT_ENCODER \
                             | S5_FAULT_IMU | S5_FAULT_NO_ECHO)

/* Line-loss sweep: left, then right past the start, then further left. */
static const int16_t g_recovery_deg[RECOVERY_STEPS] = {30, -60, 90};

/** Everything the mission remembers between steps. */
typedef struct
{
    subsystem5_ctrl_state_t state;
    uint32_t                run_start_ms;
    uint32_t                barcodes;
    uint32_t                speed_mms;      /* Current FOLLOW speed.     */
    bool                    b_nav_pending;
    uint8_t                 nav_cmd;        /* subsystem5_nav_cmd_t.     */
    int32_t                 mark_mm;        /* Odometry at a reference.  */
    uint32_t                recovery_step;
    uint32_t                wait_start_ms;  /* Current TURN, or the      */
    uint32_t                wait_limit_ms;  /* calibration, started.     */
} mission_t;

static mission_t g_mission = {S5_CTRL_INIT, 0u, 0u, SPEED_CRUISE_MMS, false,
                              0u, 0, 0u, 0u, 0u};

/* In test images the controller ignores events until a test arms it, so
 * the S2 tests that run first are never disturbed.
 */
#if APP_TEST
static volatile bool g_b_armed = false;
#else
static volatile bool g_b_armed = true;
#endif

static void set_state(subsystem5_ctrl_state_t state);
static bool is_running(void);
static void step_calibrating(uint32_t now_ms);
static void step_running(uint32_t events, uint32_t now_ms);
static void step_route(uint32_t events, uint32_t now_ms);
static bool obstacle_stopped(uint32_t events, uint32_t now_ms);
static void adjust_speed(uint32_t events);
static void take_barcodes(void);
static void execute_command(uint32_t now_ms);
static void step_turning(uint32_t events, uint32_t now_ms);
static void start_reacquire(void);
static void step_reacquire(uint32_t events, uint32_t now_ms);
static void start_recovery(uint32_t now_ms);
static void step_recovery(uint32_t events, uint32_t now_ms);
static void step_avoidance(uint32_t events, uint32_t now_ms);
static void start_turn(int32_t angle_deg, uint32_t now_ms);
static void start_run(uint32_t now_ms);
static void resume_following(void);
static void finish_run(uint32_t now_ms);
static void enter_ready(void);
static void enter_fault(subsystem5_ctrl_fault_t fault);
static subsystem5_ctrl_fault_t fault_from_flags(void);

/*!
 * @brief Start the mission: INIT, then CALIBRATING.
 *
 * @param[in] now_ms Current time.
 */
void
s5_mission_init (uint32_t now_ms)
{
    subsystem5_ctrl_state_put(S5_CTRL_INIT);
    g_mission.wait_start_ms = now_ms;
    set_state(S5_CTRL_CALIBRATING);
}

/*!
 * @brief Let the controller act on events and commands (test images
 *        start disarmed; see above).
 *
 * @param[in] b_armed true to act.
 */
void
s5_mission_arm (bool b_armed)
{
    g_b_armed = b_armed;
}

/*!
 * @brief Advance the mission with a batch of events. Call it at least
 *        every 50 ms, also with no events, so time limits are checked.
 *
 * @param[in] events EV_xxx bits taken from flg_mission (may be 0).
 * @param[in] now_ms Current time.
 */
void
s5_mission_step (uint32_t events, uint32_t now_ms)
{
    if (S5_CTRL_CALIBRATING == g_mission.state)
    {
        step_calibrating(now_ms);
    }
    else if (g_b_armed && is_running())
    {
        step_running(events, now_ms);
    }
    else
    {
        // READY, FAULT and FINISHED wait for a command.
    }
}

/*!
 * @brief Carry out a command from a button or the dashboard.
 *
 * @param[in] opcode S5_CMD_START, S5_CMD_STOP or S5_CMD_RESET.
 * @param[in] now_ms Current time.
 *
 * @return true if accepted (the ack says ACCEPTED).
 */
bool
s5_mission_command (uint8_t opcode, uint32_t now_ms)
{
    bool                    b_ok  = false;
    subsystem5_ctrl_state_t state = g_mission.state;

    if (!g_b_armed)
    {
        b_ok = false;
    }
    else if (S5_CMD_START == opcode)
    {
        if (S5_CTRL_READY == state)
        {
            start_run(now_ms);
            b_ok = true;
        }
        else if (S5_CTRL_FINISHED == state)
        {
            enter_ready();
            b_ok = true;
        }
        else
        {
            // Already running, calibrating, or in FAULT (needs RESET).
        }
    }
    else if (S5_CMD_STOP == opcode)
    {
        // Always accepted; only a running robot has anything to stop.
        if (is_running())
        {
            finish_run(now_ms);
        }

        b_ok = true;
    }
    else if (S5_CMD_RESET == opcode)
    {
        if ((S5_CTRL_FAULT == state) || (S5_CTRL_FINISHED == state))
        {
            subsystem5_fault_clear(FAULTS_LATCHED);
            s5_links_motion_release();
            enter_ready();
            b_ok = true;
        }
    }
    else
    {
        // Unknown opcode: refused.
    }

    return (b_ok);
}

/*!
 * @brief Button 2: calibrate again (only from READY).
 *
 * @param[in] now_ms Current time.
 */
void
s5_mission_calibrate (uint32_t now_ms)
{
    if (g_b_armed && (S5_CTRL_READY == g_mission.state))
    {
        g_mission.wait_start_ms = now_ms;
        set_state(S5_CTRL_CALIBRATING);
    }
}

/*!
 * @brief Tell whether a run is in progress (for Button 1: start or stop).
 *
 * @return true for LINE_FOLLOWING ... OBSTACLE_AVOIDANCE.
 */
bool
s5_mission_is_running (void)
{
    return (is_running());
}

/*!
 * @brief Change state, log it (FR16) and publish it for telemetry.
 *
 * @param[in] state The new state.
 */
static void
set_state (subsystem5_ctrl_state_t state)
{
    if (state != g_mission.state)
    {
        // NOTE: The casts are safe: both are small enum values.
        s5_event_post_args(S5_EVENT_CTRL_STATE, (uint32_t) state,
                           (uint32_t) g_mission.state);
    }

    g_mission.state = state;
    subsystem5_ctrl_state_put(state);
}

/*!
 * @brief Tell whether a run is in progress.
 *
 * @return true for LINE_FOLLOWING ... OBSTACLE_AVOIDANCE.
 */
static bool
is_running (void)
{
    return ((g_mission.state >= S5_CTRL_LINE_FOLLOWING)
            && (g_mission.state <= S5_CTRL_OBSTACLE_AVOIDANCE));
}

/*!
 * @brief CALIBRATING: wait for S4's IMU calibration (or its time limit),
 *        then READY.
 *
 * @param[in] now_ms Current time.
 *
 * TODO: Ask S3 for its IR sweep here once line_task is merged (FR2).
 */
static void
step_calibrating (uint32_t now_ms)
{
    bool b_timeout = ((now_ms - g_mission.wait_start_ms) >= CAL_TIMEOUT_MS);

    if (s5_links_imu_cal_done() || b_timeout)
    {
        if (b_timeout)
        {
            // NOTE: The cast is safe: the limit is 3000 ms.
            s5_event_trace("CAL IMU not calibrated after ms, carrying on",
                           (int32_t) CAL_TIMEOUT_MS, 0);
        }

        enter_ready();
    }
}

/*!
 * @brief Running: faults and STOP first, then the current state's work.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_running (uint32_t events, uint32_t now_ms)
{
    subsystem5_ctrl_fault_t fault = S5_CTRL_FAULT_NONE;

    if (0u != (events & EV_FAULT))
    {
        fault = fault_from_flags();
    }

    if (0u != (events & EV_IMPACT))
    {
        enter_fault(S5_CTRL_FAULT_IMPACT);
    }
    else if (S5_CTRL_FAULT_NONE != fault)
    {
        enter_fault(fault);
    }
    else if (0u != (events & EV_STOP))
    {
        finish_run(now_ms);
    }
    else if (S5_CTRL_OBSTACLE_AVOIDANCE == g_mission.state)
    {
        step_avoidance(events, now_ms);
    }
    else if (!obstacle_stopped(events, now_ms))
    {
        step_route(events, now_ms);
    }
    else
    {
        // Avoidance has just started.
    }
}

/*!
 * @brief The line, barcode and junction states.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_route (uint32_t events, uint32_t now_ms)
{
    int32_t travelled = s5_links_odom_dist_mm() - g_mission.mark_mm;

    switch (g_mission.state)
    {
        case S5_CTRL_LINE_FOLLOWING:
            adjust_speed(events);

            if (0u != (events & EV_BARCODE))
            {
                take_barcodes();
            }
            else if (0u != (events & EV_LINE_LOST))
            {
                start_recovery(now_ms);
            }
            else
            {
                // Keep following.
            }
        break;

        case S5_CTRL_APPROACH_JUNCTION:
            adjust_speed(events);

            if (0u != (events & EV_BARCODE))
            {
                take_barcodes();
            }
            else if ((0u != (events & EV_JUNCTION))
                     || (travelled >= S5_JUNCTION_MM))
            {
                execute_command(now_ms);
            }
            else if (0u != (events & EV_LINE_LOST))
            {
                start_recovery(now_ms);
            }
            else
            {
                // Keep approaching.
            }
        break;

        case S5_CTRL_EXECUTING_COMMAND:
            step_turning(events, now_ms);
        break;

        case S5_CTRL_REACQUIRE_LINE:
            step_reacquire(events, now_ms);
        break;

        case S5_CTRL_LINE_RECOVERY:
            step_recovery(events, now_ms);
        break;

        default:
            // Not a route state.
        break;
    }
}

/*!
 * @brief Start avoidance if obstacle_task has stopped the robot.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return true if avoidance started.
 */
static bool
obstacle_stopped (uint32_t events, uint32_t now_ms)
{
    bool b_stopped = (0u != (events & EV_OBSTACLE));

    if (b_stopped)
    {
        if (g_mission.b_nav_pending)
        {
            // NOTE: The cast is safe: the command is 0 ... 3.
            s5_event_trace("NAV dropped for obstacle: cmd 0",
                           (int32_t) g_mission.nav_cmd, 0);
            g_mission.b_nav_pending = false;
        }

        s5_links_line_enable(false);
        s5_avoid_start(now_ms);
        set_state(S5_CTRL_OBSTACLE_AVOIDANCE);
    }

    return (b_stopped);
}

/*!
 * @brief Slow down under 35 cm, speed up again at 40 cm (FR9).
 *
 * @param[in] events Mission events.
 */
static void
adjust_speed (uint32_t events)
{
    if (0u != (events & EV_OBSTACLE_NEAR))
    {
        g_mission.speed_mms = SPEED_SLOW_MMS;
        s5_links_motion_follow(SPEED_SLOW_MMS);
    }
    else if (0u != (events & EV_OBSTACLE_CLEAR))
    {
        g_mission.speed_mms = SPEED_CRUISE_MMS;
        s5_links_motion_follow(SPEED_CRUISE_MMS);
    }
    else
    {
        // No change.
    }
}

/*!
 * @brief Take every queued barcode command; the newest one counts. Start
 *        measuring the distance to its junction.
 */
static void
take_barcodes (void)
{
    subsystem5_nav_msg_t msg = {0u, '\0', false, 0u};

    while (subsystem5_nav_take(&msg))
    {
        g_mission.barcodes++;

        if (g_mission.b_nav_pending)
        {
            // NOTE: The casts are safe: commands are 0 ... 3.
            s5_event_trace("NAV replaced: old cmd new cmd",
                           (int32_t) g_mission.nav_cmd, (int32_t) msg.cmd);
        }

        g_mission.b_nav_pending = true;
        g_mission.nav_cmd       = msg.cmd;
        g_mission.mark_mm       = s5_links_odom_dist_mm();
        set_state(S5_CTRL_APPROACH_JUNCTION);
    }
}

/*!
 * @brief At the junction: stop and turn as the barcode said (FR6).
 *
 * @param[in] now_ms Current time.
 */
static void
execute_command (uint32_t now_ms)
{
    int32_t angle_deg = 0;

    switch (g_mission.nav_cmd)
    {
        case S5_NAV_LEFT:
            angle_deg = TURN_LEFT_DEG;
        break;

        case S5_NAV_RIGHT:
            angle_deg = TURN_RIGHT_DEG;
        break;

        case S5_NAV_UTURN:
            angle_deg = TURN_UTURN_DEG;
        break;

        default:
            // S5_NAV_STRAIGHT: no turn.
        break;
    }

    g_mission.b_nav_pending = false;
    s5_links_line_enable(false);
    s5_links_motion_stop();
    set_state(S5_CTRL_EXECUTING_COMMAND);

    if (0 == angle_deg)
    {
        start_reacquire();
    }
    else
    {
        start_turn(angle_deg, now_ms);
    }
}

/*!
 * @brief EXECUTING_COMMAND: wait for the turn, then find the line.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_turning (uint32_t events, uint32_t now_ms)
{
    if (0u != (events & EV_MOTION_DONE))
    {
        start_reacquire();
    }
    else if ((now_ms - g_mission.wait_start_ms) >= g_mission.wait_limit_ms)
    {
        enter_fault(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still turning.
    }
}

/*!
 * @brief After a barcode turn: look for the line while creeping forward
 *        (or, without S3, carry on at once).
 */
static void
start_reacquire (void)
{
    if (s5_links_line_is_real())
    {
        g_mission.mark_mm   = s5_links_odom_dist_mm();
        g_mission.speed_mms = SPEED_SLOW_MMS;
        s5_links_line_search(true);
        s5_links_line_enable(true);
        s5_links_motion_follow(SPEED_SLOW_MMS);
        set_state(S5_CTRL_REACQUIRE_LINE);
    }
    else
    {
        resume_following();
    }
}

/*!
 * @brief REACQUIRE_LINE: line found, or give up after REACQUIRE_MM.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_reacquire (uint32_t events, uint32_t now_ms)
{
    int32_t travelled = s5_links_odom_dist_mm() - g_mission.mark_mm;

    if (0u != (events & EV_LINE_FOUND))
    {
        resume_following();
    }
    else if (travelled >= REACQUIRE_MM)
    {
        start_recovery(now_ms);
    }
    else
    {
        // Still looking.
    }
}

/*!
 * @brief LINE_RECOVERY: stop and sweep left and right looking for the
 *        line (Week 6 report 6.1 SEARCHING).
 *
 * @param[in] now_ms Current time.
 */
static void
start_recovery (uint32_t now_ms)
{
    g_mission.recovery_step = 0u;
    s5_links_motion_stop();
    s5_links_line_search(true);
    set_state(S5_CTRL_LINE_RECOVERY);
    start_turn(g_recovery_deg[0], now_ms);
}

/*!
 * @brief LINE_RECOVERY: next sweep after each turn; fault after the last.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_recovery (uint32_t events, uint32_t now_ms)
{
    if (0u != (events & EV_LINE_FOUND))
    {
        s5_links_motion_stop();
        resume_following();
    }
    else if (0u != (events & EV_MOTION_DONE))
    {
        g_mission.recovery_step++;

        if (g_mission.recovery_step < RECOVERY_STEPS)
        {
            start_turn(g_recovery_deg[g_mission.recovery_step], now_ms);
        }
        else
        {
            enter_fault(S5_CTRL_FAULT_LINE_NOT_FOUND);
        }
    }
    else if ((now_ms - g_mission.wait_start_ms) >= g_mission.wait_limit_ms)
    {
        enter_fault(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still sweeping.
    }
}

/*!
 * @brief OBSTACLE_AVOIDANCE: let s5_avoid.c work, then act on its result.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_avoidance (uint32_t events, uint32_t now_ms)
{
    switch (s5_avoid_step(events, now_ms))
    {
        case S5_AVOID_RESUME:
            resume_following();
        break;

        case S5_AVOID_PLAN_ONLY:
            finish_run(now_ms);
        break;

        case S5_AVOID_FAILURE:
            enter_fault(s5_avoid_fault());
        break;

        default:
            // S5_AVOID_BUSY.
        break;
    }
}

/*!
 * @brief Send a TURN and start its time limit.
 *
 * @param[in] angle_deg Angle; positive = left.
 * @param[in] now_ms    Current time.
 */
static void
start_turn (int32_t angle_deg, uint32_t now_ms)
{
    // NOTE: The casts are safe: |angle_deg| is at most 180.
    uint32_t angle = (angle_deg < 0) ? (uint32_t) -angle_deg
                                     : (uint32_t) angle_deg;

    s5_links_motion_turn(angle_deg);
    g_mission.wait_start_ms = now_ms;
    g_mission.wait_limit_ms = (angle * TURN_MS_PER_DEG) + MOTION_SLACK_MS;
}

/*!
 * @brief START from READY: leave the E-stop and follow the line.
 *
 * @param[in] now_ms Current time.
 */
static void
start_run (uint32_t now_ms)
{
    g_mission.run_start_ms  = now_ms;
    g_mission.barcodes      = 0u;
    g_mission.b_nav_pending = false;
    s5_links_motion_release();
    resume_following();
}

/*!
 * @brief Back to LINE_FOLLOWING at cruise speed, with obstacle monitoring.
 */
static void
resume_following (void)
{
    g_mission.speed_mms = SPEED_CRUISE_MMS;
    s5_links_line_search(false);
    s5_links_line_enable(true);
    s5_links_motion_follow(SPEED_CRUISE_MMS);
    subsystem5_obst_request(OB_RESUME);
    set_state(S5_CTRL_LINE_FOLLOWING);
}

/*!
 * @brief End the run: brake, stop sensing and print the summary.
 *
 * @param[in] now_ms Current time.
 */
static void
finish_run (uint32_t now_ms)
{
    uint32_t run_s = (now_ms - g_mission.run_start_ms) / MS_PER_S;

    s5_links_motion_stop();
    s5_links_line_enable(false);
    s5_links_line_search(false);
    subsystem5_obst_request(OB_HOLD);

    // TODO: Add S4's peak hump height once its getter is merged (FR8).
    // NOTE: The casts are safe: both counts are far below 2^31.
    s5_event_post_args(S5_EVENT_RUN_DONE, g_mission.barcodes, run_s);
    s5_event_trace("RUN summary: barcodes run_s",
                   (int32_t) g_mission.barcodes, (int32_t) run_s);
    set_state(S5_CTRL_FINISHED);
}

/*!
 * @brief READY: braked, line following off, obstacle_task idle.
 */
static void
enter_ready (void)
{
    s5_links_line_enable(false);
    s5_links_line_search(false);
    subsystem5_obst_request(OB_HOLD);
    set_state(S5_CTRL_READY);
}

/*!
 * @brief FAULT: brake through the E-stop latch and report why (FR15).
 *
 * @param[in] fault The reason.
 *
 * TODO: Buzzer pattern on GP22 (Week 6 report section 5).
 */
static void
enter_fault (subsystem5_ctrl_fault_t fault)
{
    s5_links_motion_estop();
    s5_links_line_enable(false);
    s5_links_line_search(false);
    subsystem5_obst_request(OB_HOLD);

    // NOTE: The cast is safe: the reason is a small enum value.
    s5_event_post_args(S5_EVENT_CTRL_FAULT, (uint32_t) fault, 0u);
    set_state(S5_CTRL_FAULT);
}

/*!
 * @brief Work out which fault EV_FAULT reports, from flg_fault.
 *
 * @return The reason, or NONE if only a warning bit (IMU, WiFi) is set.
 */
static subsystem5_ctrl_fault_t
fault_from_flags (void)
{
    uint32_t                faults = subsystem5_fault_get();
    subsystem5_ctrl_fault_t fault  = S5_CTRL_FAULT_NONE;

    if (0u != (faults & S5_FAULT_STALL))
    {
        fault = S5_CTRL_FAULT_STALL;
    }
    else if (0u != (faults & S5_FAULT_NO_ECHO))
    {
        fault = S5_CTRL_FAULT_SENSOR;
    }
    else
    {
        // IMU and WiFi problems do not stop the run (Week 6 report 6.5).
    }

    return (fault);
}

/*** end of file ***/
