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
 * - A barcode command waits until EV_JUNCTION, or at most
 *   S5_JUNCTION_MAX_MM of travel (assumption A11). A newer barcode
 *   replaces an unexecuted one.
 * - An obstacle stop from any running state starts OBSTACLE_AVOIDANCE;
 *   a pending barcode command is then dropped (and logged).
 * - Line loss: stop and sweep, first towards the side S3 last steered
 *   to, while S3 searches; FAULT if the line is not found (FR15).
 *
 * @par Calibrating S3's line sensors (FR2)
 * Button 2 in READY, with the robot on the line: S3 forgets its levels
 * and the robot turns +30, -60, +30 degrees on the spot, so both
 * sensors pass over white and black. Nothing moves at power-up.
 *
 * @par Without S3 (s5_links_line_is_ready() false)
 * A START is refused while S3 is in use but not calibrated. Switched off
 * by the floor tests, S3 does not steer, so the robot drives straight in
 * FOLLOW. A barcode command then waits S5_JUNCTION_MM, and after a turn
 * or a bypass the line is taken as found where it is expected.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_avoid.h"
#include "s5_event.h"
#include "s5_links.h"
#include "s5_math.h"
#include "s5_mission.h"
#include "s5_profile.h"
#include "subsystem5.h"

#define SLOW_NEAR_MMS       (60u)       /* Under 35 cm (FR9).            */
#define CAL_TIMEOUT_MS      (3000u)     /* Then go on without the IMU.   */
#define S5_JUNCTION_MM      (150)       /* TODO: measure (A11, INT-01).  */
#define S5_JUNCTION_MAX_MM  (500)       /* With S3: then turn anyway.    */
#define REACQUIRE_MM        (300)       /* NFR10.                        */
#define TURN_MS_PER_DEG     (20u)
#define MOTION_SLACK_MS     (2000u)
#define SPOT_STEPS          (9u)        /* g_search_spot below (FR15).   */
#define SEARCH_SPOTS        (5u)        /* g_search_spot_mm below.       */
#define DIAG_MM             (150)       /* Drive at 45 degrees: about    */
                                        /* 106 mm sideways.              */
#define BLACK_STEADY_MM     (10)        /* Black this far = the line, not*/
                                        /* a barcode bar or thin cross.  */
#define BACK_MARGIN_MM      (20)        /* Added to the measured reverse.*/
#define BACK_MAX_MM         (250)       /* Never reverse further.        */
#define CAL_SWEEP_STEPS     (3u)
#define SLOW_MMS            (40u)       /* Calibration and recovery: S3  */
                                        /* car test 29 Oct, 150 abrupt.  */
#define SLOW_MS_PER_DEG     (80u)       /* About 3 x the time at 40 mm/s.*/
#define SLOW_MS_PER_MM      (75u)       /* About 3 x the time at 40 mm/s.*/
#define TURN_LEFT_DEG       (90)        /* Barcode turns (FR6).          */
#define TURN_RIGHT_DEG      (-90)
#define TURN_UTURN_DEG      (180)
#define MS_PER_S            (1000u)
#define FAULTS_LATCHED      (S5_FAULT_STALL | S5_FAULT_ENCODER \
                             | S5_FAULT_IMU | S5_FAULT_NO_ECHO)

/** Line-recovery phases. */
typedef enum
{
    RECOVERY_SEARCH,        /* Sweeping and searching.                   */
    RECOVERY_ONTO,          /* Found at an angle: wheels onto the line.  */
    RECOVERY_REALIGN        /* Turning back square to the line.          */
} recovery_phase_t;

/** One line-search step: a slow turn, or a slow move at the heading. */
typedef struct
{
    bool    b_move;         /* false: TURN value deg; true: MOVE value mm.*/
    int16_t value;
} recovery_step_t;

/* Line search (FR15), the same for a line lost while following and for
 * a line not found on the way back from a bypass; nothing depends on the
 * track's shape. Turns positive = towards the side the line most likely
 * is: the side S3 last steered to, or after a bypass the obstacle's side.
 * At each spot along the track:
 * 1. Sweep on the spot: face 90 that way, then 90 the other way. S3's IR
 *    sits only S5_AXLE_TO_LINE_IR_MM ahead of the turning centre, so this
 *    reaches about 80 mm to each side.
 * 2. Face 45 that way and drive DIAG_MM, then back: the IR also moves
 *    sideways, about 106 mm, so a line just out of the sweep's reach is
 *    crossed (Anna, 2026-11-05: the sweep stopped about 2 cm short, then
 *    faced straight and drove past parallel to the line).
 * 3. The same at 45 the other way, then face along the track again.
 * Each step ends as soon as S3 finds the line. Found while driving at
 * 45: the wheels go onto the line, then the robot turns back square to
 * it, as after a bypass (s5_avoid.c). After a line lost while following,
 * the robot first reverses as far as it drove since a sensor last saw
 * the line (log 2026-11-05_0018_track: a fixed 60 mm reverse did not
 * reach back).
 */
static const recovery_step_t g_search_spot[SPOT_STEPS] =
{
    {false, 90}, {false, -180}, {false, 135},
    {true, DIAG_MM}, {true, -DIAG_MM},
    {false, -90},
    {true, DIAG_MM}, {true, -DIAG_MM},
    {false, 45}
};

/* Spots along the track, from where the search began: here, then 150 mm
 * behind and ahead, then 300 mm (REACQUIRE_MM, NFR10). Never further, so
 * the robot cannot wander off the map; then LINE_NOT_FOUND.
 */
static const int16_t g_search_spot_mm[SEARCH_SPOTS] =
{
    0, -150, 150, -300, 300
};

/* Calibration sweep: left, right past the start, back to the start. 30
 * degrees left the centre sensor short of the line (log
 * 2026-11-04_1701_track: GP27 range 568, S3 needs 600).
 */
static const int16_t g_cal_sweep_deg[CAL_SWEEP_STEPS] = {45, -90, 45};

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
    recovery_phase_t        recovery_phase;
    uint32_t                recovery_spot;  /* g_search_spot_mm index.   */
    uint32_t                recovery_step;  /* Next g_search_spot step.  */
    int32_t                 recovery_dir;   /* +1 left first, -1 right.  */
    int32_t                 recovery_back_mm;   /* Reverse still to do.  */
    int32_t                 recovery_deg;   /* Heading off the track.    */
    bool                    b_recovery_diag;    /* Moving at 45 degrees. */
    int32_t                 line_mark_mm;   /* Odometry: line last seen. */
    int32_t                 black_from_mm;  /* Odometry: black began.    */
    bool                    b_black;        /* A sensor sees black now.  */
    uint32_t                cal_step;       /* Sweep turns started; 0 =  */
                                            /* waiting for the IMU.      */
    uint32_t                wait_start_ms;  /* Current TURN, or the      */
    uint32_t                wait_limit_ms;  /* calibration, started.     */
} mission_t;

static mission_t g_mission = {S5_CTRL_INIT, 0u, 0u, 0u, false,
                              0u, 0, RECOVERY_SEARCH, 0u, 0u, 1, 0, 0,
                              false, 0, 0, false, 0u, 0u, 0u};

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
static void step_calibrating(uint32_t events, uint32_t now_ms);
static void step_cal_sweep(uint32_t events, uint32_t now_ms);
static int32_t junction_limit_mm(void);
static s5_links_side_t reacquire_side(void);
static void step_running(uint32_t events, uint32_t now_ms);
static void step_route(uint32_t events, uint32_t now_ms);
static bool obstacle_stopped(uint32_t events, uint32_t now_ms);
static void adjust_speed(uint32_t events);
static void take_barcodes(void);
static void execute_command(uint32_t now_ms);
static void step_turning(uint32_t events, uint32_t now_ms);
static void start_reacquire(void);
static void step_reacquire(uint32_t events, uint32_t now_ms);
static void start_recovery(uint32_t now_ms, bool b_after_bypass);
static void step_recovery(uint32_t events, uint32_t now_ms);
static void step_avoidance(uint32_t events, uint32_t now_ms);
static void start_turn(int32_t angle_deg, uint32_t now_ms);
static void start_slow_turn(int32_t angle_deg, uint32_t now_ms);
static void start_recovery_step(uint32_t now_ms);
static void start_slow_move(int32_t move_mm, uint32_t now_ms);
static void update_line_mark(void);
static void set_line_mark(void);
static uint16_t cruise_mms(void);
static uint16_t slow_mms(void);
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
        step_calibrating(events, now_ms);
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
        if ((S5_CTRL_READY == state) && s5_links_line_is_used()
            && (!s5_links_line_is_ready()))
        {
            // Without calibrated line sensors the robot would drive
            // straight with no line to follow or find (log
            // 2026-11-04_1701_track: it ended at a wall).
            s5_event_trace("RUN refused: line sensors not calibrated, "
                           "press Button 2 on the line:", 0, 0);
        }
        else if (S5_CTRL_READY == state)
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
 * @brief Button 2: calibrate S3's line sensors with a sweep on the spot
 *        (only from READY; the robot must stand on the line).
 *
 * @param[in] now_ms Current time.
 */
void
s5_mission_calibrate (uint32_t now_ms)
{
    if (g_b_armed && (S5_CTRL_READY == g_mission.state))
    {
        s5_links_line_recalibrate();
        s5_links_motion_release();
        set_state(S5_CTRL_CALIBRATING);
        start_slow_turn(g_cal_sweep_deg[0], now_ms);
        g_mission.cal_step = 1u;
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
 * @brief CALIBRATING: after power-up, wait for S4's IMU calibration (or
 *        its time limit); after Button 2, run the line-sensor sweep.
 *        Then READY.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_calibrating (uint32_t events, uint32_t now_ms)
{
    bool b_timeout = ((now_ms - g_mission.wait_start_ms) >= CAL_TIMEOUT_MS);

    if (0u != g_mission.cal_step)
    {
        step_cal_sweep(events, now_ms);
    }
    else if (s5_links_imu_cal_done() || b_timeout)
    {
        if (b_timeout)
        {
            // NOTE: The cast is safe: the limit is 3000 ms.
            s5_event_trace("CAL IMU not calibrated after ms, carrying on",
                           (int32_t) CAL_TIMEOUT_MS, 0);
        }

        enter_ready();
    }
    else
    {
        // Still waiting for the IMU.
    }
}

/*!
 * @brief The line-sensor sweep: next turn after each one; READY after
 *        the last, with S3's result logged.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_cal_sweep (uint32_t events, uint32_t now_ms)
{
    if (0u != (events & EV_MOTION_DONE))
    {
        if (g_mission.cal_step < CAL_SWEEP_STEPS)
        {
            start_slow_turn(g_cal_sweep_deg[g_mission.cal_step], now_ms);
            g_mission.cal_step++;
        }
        else
        {
            s5_links_motion_stop();
            s5_event_trace(s5_links_line_is_ready()
                           ? "CAL line sensors calibrated: YES, ready for"
                             " Button 1"
                           : "CAL line sensors calibrated: NO, press"
                             " Button 2 again",
                           0, 0);
            enter_ready();
        }
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
            update_line_mark();

            if (0u != (events & EV_BARCODE))
            {
                take_barcodes();
            }
            else if (0u != (events & EV_LINE_LOST))
            {
                start_recovery(now_ms, false);
            }
            else
            {
                // Keep following.
            }
        break;

        case S5_CTRL_APPROACH_JUNCTION:
            adjust_speed(events);
            update_line_mark();

            if (0u != (events & EV_BARCODE))
            {
                take_barcodes();
            }
            else if ((0u != (events & EV_JUNCTION))
                     || (travelled >= junction_limit_mm()))
            {
                execute_command(now_ms);
            }
            else if (0u != (events & EV_LINE_LOST))
            {
                start_recovery(now_ms, false);
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
        g_mission.speed_mms = slow_mms();
        s5_links_motion_follow(slow_mms());
    }
    else if (0u != (events & EV_OBSTACLE_CLEAR))
    {
        g_mission.speed_mms = cruise_mms();
        s5_links_motion_follow(cruise_mms());
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

    while (s5_links_nav_take(&msg))
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
    if (s5_links_line_is_ready())
    {
        g_mission.mark_mm   = s5_links_odom_dist_mm();
        g_mission.speed_mms = slow_mms();
        set_line_mark();
        s5_links_line_enable(true);
        s5_links_line_search(reacquire_side());
        s5_links_motion_follow(slow_mms());
        set_state(S5_CTRL_REACQUIRE_LINE);
    }
    else
    {
        resume_following();
    }
}

/*!
 * @brief Which side of the new line the robot is likely on after a
 *        barcode turn. S2's turns tend to stop a few degrees short
 *        (S2-U03), so after a left turn the sensors are still right of
 *        the new line, and the other way round.
 *
 * @return The search side.
 */
static s5_links_side_t
reacquire_side (void)
{
    s5_links_side_t side = S5_LINKS_SIDE_LEFT;

    switch (g_mission.nav_cmd)
    {
        case S5_NAV_LEFT:
            side = S5_LINKS_SIDE_RIGHT;
        break;

        case S5_NAV_RIGHT:
            side = S5_LINKS_SIDE_LEFT;
        break;

        default:
            // Straight on or U-turn: where S3 last saw the line.
            side = s5_links_line_side();
        break;
    }

    return (side);
}

/*!
 * @brief How far to drive after a barcode before turning anyway.
 *
 * @return S5_JUNCTION_MAX_MM while S3 can report the junction itself,
 *         else S5_JUNCTION_MM.
 */
static int32_t
junction_limit_mm (void)
{
    return (s5_links_line_is_ready() ? S5_JUNCTION_MAX_MM : S5_JUNCTION_MM);
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
        start_recovery(now_ms, false);
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
 * @param[in] now_ms         Current time.
 * @param[in] b_after_bypass true: the line was not found on the way back
 *                           from a bypass (search from here); false: lost
 *                           while following (reverse first).
 */
static void
start_recovery (uint32_t now_ms, bool b_after_bypass)
{
    // Which side of the line the robot is. After a bypass it is on the
    // side it went round, and the line on the obstacle's side; otherwise
    // read S3's last steering before anything changes it.
    s5_links_side_t side = s5_links_line_side();

    if (b_after_bypass)
    {
        side = s5_avoid_passed_left() ? S5_LINKS_SIDE_LEFT
                                      : S5_LINKS_SIDE_RIGHT;
    }

    // Back to where the line was last seen, with a margin, capped.
    int32_t back = (s5_links_odom_dist_mm() - g_mission.line_mark_mm)
                   + BACK_MARGIN_MM;

    if (back > BACK_MAX_MM)
    {
        back = BACK_MAX_MM;
    }
    else if (back < BACK_MARGIN_MM)
    {
        back = BACK_MARGIN_MM;      // E.g. just after a reverse.
    }
    else
    {
        // Already inside the range.
    }

    if (b_after_bypass)
    {
        // Already where the line should be: search from here.
        back = 0;
        s5_event_trace("RECOVERY after bypass: search from here 0 0", 0, 0);
    }
    else
    {
        s5_event_trace("RECOVERY reverse_mm (driven since line seen + 20)"
                       " 0", back, 0);
    }

    // A robot left of the line turns right (negative) first.
    g_mission.recovery_dir     = (S5_LINKS_SIDE_LEFT == side) ? -1 : 1;
    g_mission.recovery_phase   = RECOVERY_SEARCH;
    g_mission.recovery_spot    = 0u;
    g_mission.recovery_step    = 0u;
    g_mission.recovery_back_mm = back;
    g_mission.recovery_deg     = 0;
    g_mission.b_recovery_diag  = false;
    s5_links_motion_settle();
    s5_links_line_search(side);
    set_state(S5_CTRL_LINE_RECOVERY);
    start_recovery_step(now_ms);
}

/*!
 * @brief Start the next line-search step: the measured reverse, then
 *        g_search_spot at each spot of g_search_spot_mm; LINE_NOT_FOUND
 *        after the last spot.
 *
 * @param[in] now_ms Current time.
 */
static void
start_recovery_step (uint32_t now_ms)
{
    g_mission.b_recovery_diag = false;

    if (0 != g_mission.recovery_back_mm)
    {
        // Line lost while following: back to where it was last seen.
        start_slow_move(-g_mission.recovery_back_mm, now_ms);
        g_mission.recovery_back_mm = 0;
    }
    else if (g_mission.recovery_step < SPOT_STEPS)
    {
        recovery_step_t const * p_step =
            &g_search_spot[g_mission.recovery_step];

        g_mission.recovery_step++;

        if (p_step->b_move)
        {
            // Every move in g_search_spot is at 45 degrees.
            g_mission.b_recovery_diag = true;
            start_slow_move(p_step->value, now_ms);
        }
        else
        {
            int32_t turn_deg = g_mission.recovery_dir * p_step->value;

            g_mission.recovery_deg += turn_deg;
            start_slow_turn(turn_deg, now_ms);
        }
    }
    else if ((g_mission.recovery_spot + 1u) < SEARCH_SPOTS)
    {
        // Facing along the track again: on to the next spot.
        int32_t move_mm = g_search_spot_mm[g_mission.recovery_spot + 1u]
                          - g_search_spot_mm[g_mission.recovery_spot];

        g_mission.recovery_spot++;
        g_mission.recovery_step = 0u;
        s5_event_trace("RECOVERY next spot: along_mm 0",
                       g_search_spot_mm[g_mission.recovery_spot], 0);
        start_slow_move(move_mm, now_ms);
    }
    else
    {
        enter_fault(S5_CTRL_FAULT_LINE_NOT_FOUND);
    }
}

/*!
 * @brief LINE_RECOVERY: search step by step until S3 finds the line;
 *        found while driving at 45 degrees, square up to it first.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_recovery (uint32_t events, uint32_t now_ms)
{
    bool b_found = (0u != (events & EV_LINE_FOUND));
    bool b_done  = (0u != (events & EV_MOTION_DONE));

    if ((RECOVERY_SEARCH == g_mission.recovery_phase) && b_found
        && g_mission.b_recovery_diag && (0 != g_mission.recovery_deg))
    {
        // S3's IR is on the line, ahead of the turning centre: drive the
        // wheels onto it, then turn back square (as s5_avoid.c's return).
        s5_event_trace("RECOVERY line found at an angle: deg 0",
                       g_mission.recovery_deg, 0);
        s5_links_motion_stop();
        start_slow_move(S5_AXLE_TO_LINE_IR_MM, now_ms);
        g_mission.recovery_phase = RECOVERY_ONTO;
    }
    else if ((RECOVERY_SEARCH == g_mission.recovery_phase) && b_found)
    {
        // resume_following() lets the wheels settle first.
        resume_following();
    }
    else if (b_done && (RECOVERY_ONTO == g_mission.recovery_phase))
    {
        start_slow_turn(-g_mission.recovery_deg, now_ms);
        g_mission.recovery_phase = RECOVERY_REALIGN;
    }
    else if (b_done && (RECOVERY_REALIGN == g_mission.recovery_phase))
    {
        resume_following();
    }
    else if (b_done)
    {
        start_recovery_step(now_ms);
    }
    else if ((now_ms - g_mission.wait_start_ms) >= g_mission.wait_limit_ms)
    {
        enter_fault(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still searching.
    }
}

/*!
 * @brief Send one slow MOVE (line search) and start its time limit.
 *
 * @param[in] move_mm Distance; negative = backwards.
 * @param[in] now_ms  Current time.
 */
static void
start_slow_move (int32_t move_mm, uint32_t now_ms)
{
    // NOTE: The cast is safe: |move_mm| is at most a few hundred mm.
    uint32_t dist = (uint32_t) s5_math_abs(move_mm);

    s5_links_motion_move(move_mm, SLOW_MMS);
    g_mission.wait_start_ms = now_ms;
    g_mission.wait_limit_ms = (dist * SLOW_MS_PER_MM) + MOTION_SLACK_MS;
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

        case S5_AVOID_SEARCH:
            // Search from here, ahead and behind (g_search_spot).
            start_recovery(now_ms, true);
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

    s5_links_motion_turn(angle_deg, 0u);
    g_mission.wait_start_ms = now_ms;
    g_mission.wait_limit_ms = (angle * TURN_MS_PER_DEG) + MOTION_SLACK_MS;
}

/*!
 * @brief Send one slow TURN (calibration sweep, line recovery) and start
 *        its time limit. At S2's default speed the centre sensor crossed
 *        the line too fast for S3 to learn its black level (log
 *        2026-11-04_1712_track: GP27 reached 505 of the 600 counts
 *        needed), and S3's car test found 150 mm/s too abrupt.
 *
 * @param[in] angle_deg Angle; positive = left.
 * @param[in] now_ms    Current time.
 */
static void
start_slow_turn (int32_t angle_deg, uint32_t now_ms)
{
    // NOTE: The casts are safe: |angle_deg| is at most 180.
    uint32_t angle = (angle_deg < 0) ? (uint32_t) -angle_deg
                                     : (uint32_t) angle_deg;

    s5_links_motion_turn(angle_deg, SLOW_MMS);
    g_mission.wait_start_ms = now_ms;
    g_mission.wait_limit_ms = (angle * SLOW_MS_PER_DEG) + MOTION_SLACK_MS;
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

    if (!s5_links_line_is_ready())
    {
        s5_event_trace("RUN line following off (floor test), driving "
                       "straight:", 0, 0);
    }

    s5_links_motion_release();
    resume_following();
}

/*!
 * @brief Back to LINE_FOLLOWING at cruise speed, with obstacle monitoring.
 */
static void
resume_following (void)
{
    g_mission.speed_mms = cruise_mms();

    // After a turn a wheel may still roll backwards; FOLLOW would then
    // spin the robot (s5_links_motion_settle()).
    s5_links_motion_settle();

    // S3's order: LF_ENABLE first, then end any search.
    s5_links_line_enable(true);
    s5_links_line_search(S5_LINKS_SIDE_NONE);
    s5_links_motion_follow(cruise_mms());
    subsystem5_obst_request(OB_RESUME);
    set_line_mark();
    set_state(S5_CTRL_LINE_FOLLOWING);
}

/*!
 * @brief While following: move the "line last seen" bookmark to here
 *        when a sensor has seen black steadily for BLACK_STEADY_MM, so a
 *        barcode bar or a thin crossing line beside the route does not
 *        count. A bookmark set too late only makes the measured reverse
 *        shorter; the sweeps then work as before.
 */
static void
update_line_mark (void)
{
    int32_t odom = s5_links_odom_dist_mm();

    if (!s5_links_line_sees_black())
    {
        g_mission.b_black = false;
    }
    else if (!g_mission.b_black)
    {
        g_mission.b_black       = true;
        g_mission.black_from_mm = odom;
    }
    else if ((odom - g_mission.black_from_mm) >= BLACK_STEADY_MM)
    {
        g_mission.line_mark_mm = odom;
    }
    else
    {
        // Black, but not for long enough yet.
    }
}

/*!
 * @brief Put the bookmark here: the robot is on (or turning onto) the
 *        line now.
 */
static void
set_line_mark (void)
{
    g_mission.line_mark_mm = s5_links_odom_dist_mm();
    g_mission.b_black      = false;
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
    s5_links_line_search(S5_LINKS_SIDE_NONE);
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
    g_mission.cal_step = 0u;
    s5_links_line_enable(false);
    s5_links_line_search(S5_LINKS_SIDE_NONE);
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
    g_mission.cal_step = 0u;
    s5_links_motion_estop();
    s5_links_line_enable(false);
    s5_links_line_search(S5_LINKS_SIDE_NONE);
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

/*!
 * @brief The cruise speed: S3's proven line-following speed (s5_links.c).
 *        The Week 6 report's 300 mm/s ran off the curves (2026-11-04).
 *
 * @return Robot speed in mm/s.
 */
static uint16_t
cruise_mms (void)
{
    return (s5_links_follow_speed_mms());
}

/*!
 * @brief The speed under 35 cm (FR9). With line following at 60 mm/s
 *        it is no slower; it slows down if cruise is raised later.
 *
 * @return Robot speed in mm/s.
 */
static uint16_t
slow_mms (void)
{
    // The lower of the two, so slowing down never speeds the robot up.
    return ((cruise_mms() < SLOW_NEAR_MMS) ? cruise_mms()
                                           : (uint16_t) SLOW_NEAR_MMS);
}

/*** end of file ***/
