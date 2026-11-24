/** @file s5_mission.c
 *
 * @brief The mission state machine (Week 6 report section 5).
 *
 * @par Rules taken from the Week 6 report
 * - READY: motors braked, line following off, obstacle_task idle. A run
 *   starts only with START (Button 1 or the dashboard).
 * - While running, overturning, a stall or a sensor fault enters FAULT
 *   (MF_ESTOP set, motors braked). Only RESET (Button 1 held 2 s, or the
 *   dashboard) leaves FAULT, back to READY with the motors still braked.
 * - EV_STOP ends the run in FINISHED, with a run summary.
 * - Lifting the robot ends the run the same way (EV_LIFTED from S4): in
 *   the demo it is the signal for the end of the run (brief feedback,
 *   2026-11-11, which also drops collision detection by the IMU). The
 *   robot tipped over (EV_OVERTURNED) is a FAULT. Both are acted on while
 *   running and during the Button 2 sweep (lifted cancels it, back to
 *   READY); S4 brakes the motors itself (MF_ESTOP).
 * - A barcode command is held until the next junction S3 reports (its
 *   event, or its junction counter moving on, so a junction arriving in
 *   the same tick as the barcode still counts). It is kept through a
 *   line search and through obstacle avoidance: afterwards the robot
 *   approaches the next junction again (Anna, 2026-11-17). Junctions
 *   in the first CROSS_MM after a found line do not count: that is the
 *   robot crossing the line. At the junction the robot drives
 *   JUNC_FWD_MM on before a turn, so it turns about the crossing.
 *   The robot stops at every junction, as Sean's drive test does (Anna,
 *   2026-11-18). S3 publishes a letter only about 1.5 s after the
 *   barcode, so a junction soon after one comes first: at a junction
 *   with no command waiting while S3 is still reading a barcode, the
 *   robot waits up to BARCODE_WAIT_MS for it, then acts on it at this
 *   junction (Sean's handover 2.3); none: straight on after the stop.
 *   With S3 it never turns blind; after
 *   NAV_GIVE_UP_MM of following without a junction the command is
 *   cancelled. Without S3 (floor tests) it turns after S5_JUNCTION_MM
 *   (assumption A11). A newer barcode replaces an unexecuted one.
 * - An obstacle stop from any running state starts OBSTACLE_AVOIDANCE.
 * - Line loss: stop and sweep, first towards the side S3 last steered
 *   to, while S3 searches; FAULT if the line is not found (FR15).
 * - A line found by a search or after a bypass counts only once S3 has
 *   followed it for CONFIRM_MM (see "Line or barcode bar" below).
 * - Just after a bypass handed the line over, neither sensor on black
 *   for WHITE_LOST_MM of driving also counts as lost: S3 then still
 *   reported FOLLOW and circled slowly, and the distance driven confirmed
 *   the line (log 2026-11-13_1444). Not while plainly following: on
 *   curves S3 drives on white for a moment, and this stopped the robot
 *   (Anna, 2026-11-18); there S3's own LOST counts.
 * - Lost just after a bypass found it: the robot crossed it (log
 *   2026-11-13_1444, 56 mm), not a barcode bar. Back to where it was
 *   seen and sweep, as for any line loss. A junction within CROSS_MM of
 *   that hand-over counts the same: S3 holds its steering straight while
 *   both sensors see black, so it was driving over the line (logs
 *   2026-11-13_1538/1539).
 *
 * @par Line or barcode bar
 * A barcode lies 20 mm beside the 18 mm line and is 141 mm long (Barcode
 * & Line Specification); its bars are short stripes across the track.
 * Searching, S3 may lock onto a bar. The line runs on for metres, a bar
 * ends after its own length, and S3 reports LOST 40 mm after losing it.
 * So a line found by a search is confirmed only after S3 has followed it
 * for CONFIRM_MM (bar length plus those 40 mm, with room to spare). LOST
 * sooner: it was not the line; the search goes on from there, at most
 * REJECTS_MAX times, then LINE_NOT_FOUND.
 * TODO: Measure the longest bar (across the track) and keep CONFIRM_MM
 * at least that plus 60 mm.
 *
 * @par Calibrating S3's line sensors (FR2)
 * Button 2 in READY, with the robot on the line: S3 forgets its levels
 * and the robot turns on the spot to headings of +15, 0, -15 and 0
 * degrees from where it stood, so both sensors cross the 18 mm line both
 * ways (about 33 mm each way at 15 degrees). Each turn is steered to its
 * heading: after it, the wheels settle, S2's odometry heading is read,
 * and CAL_TOL_CDEG or more off, the robot turns by the difference, at
 * most CAL_FIX_TRIES times (3 tries with the turn itself, as Sean's).
 * It ends where it started, still on the line
 * for Button 1 (Sean's sweep from S3's drive test, b39ab6b; the fixed
 * 45, -90, 45 turns drifted about 1 cm).
 * As in Sean's drive test, it also calibrates by itself (Anna,
 * 2026-11-18): once at power-up, as soon as S4's IMU calibration is done
 * (the robot must stand centred on the line), and when Button 1 is
 * pressed while S3 is not calibrated (then press Button 1 again). Not in
 * test images (the controller is not armed there) or with S3 switched
 * off.
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
#include "subsystem5.h"

#define SLOW_NEAR_MMS       (60u)       /* Under 35 cm (FR9).            */
#define CAL_TIMEOUT_MS      (6000u)     /* S4 needs 2 s still after boot;*/
                                        /* then go on without the IMU.   */
#define S5_JUNCTION_MM      (150)       /* TODO: measure (A11, INT-01).  */
#define JUNC_FWD_MM         (125)       /* Junction seen by the line     */
                                        /* sensors: drive this far so the*/
                                        /* rear axle is on the crossing, */
                                        /* then turn (Sean's car value,  */
                                        /* 5 Nov: 60 too early, 150 25 mm*/
                                        /* past; measured sensor-to-axle */
                                        /* 133 less the roll on stop).   */
#define JUNC_FWD_MMS        (60u)       /* As in Sean's drive test.      */
#define BARCODE_WAIT_MS     (2000u)     /* Junction while a barcode is   */
                                        /* still being read: wait this   */
                                        /* long for it (Sean's value).   */
#define NAV_GIVE_UP_MM      (2000)      /* With S3: no junction this far */
                                        /* after a barcode: cancel it.   */
#define JUNC_COUNT_MASK     (0xFFu)     /* S3's 8-bit junction counter.  */
#define REACQUIRE_MM        (300)       /* NFR10.                        */
#define JUNC_TURN_MMS       (60u)       /* Junction turn: the speed Sean */
                                        /* tuned JUNC_FWD_MM with (S2's  */
                                        /* default is 150 mm/s).         */
#define TURN_MS_PER_DEG     (40u)       /* About 2 x the time at 60 mm/s.*/
#define MOTION_SLACK_MS     (2000u)
#define SEARCH_STEPS        (7u)        /* g_search below (FR15).        */
#define SEARCH_MOVE_MM      (100)       /* On between the two sweeps.    */
#define CONFIRM_MM          (120)       /* Followed this far: the line.  */
#define REJECTS_MAX         (2u)        /* Not-the-line finds per search.*/
#define LOST_CHECK_MM       (20)        /* Then S3's LOST state counts.  */
#define CROSS_MM            (50)        /* Junction this soon after a    */
                                        /* bypass hand-over: crossed it. */
#define WHITE_LOST_MM       (100)       /* No black this far: lost (S3's */
                                        /* own LOST comes after 40 mm).  */
#define BLACK_STEADY_MM     (10)        /* Black this far = the line, not*/
                                        /* a barcode bar or thin cross.  */
#define BACK_MARGIN_MM      (20)        /* Added to the measured reverse.*/
#define BACK_MAX_MM         (250)       /* Never reverse further.        */
#define CAL_TARGETS         (4u)        /* Headings in g_cal_target_deg. */
#define CAL_TOL_CDEG        (200)       /* 2 deg or more off: turn the   */
                                        /* rest (Sean's drive test).     */
#define CAL_FIX_TRIES       (2u)        /* Corrections per heading: 3    */
                                        /* tries with the turn itself.   */
#define CDEG_PER_DEG        (100)
#define CDEG_HALF_TURN      (18000)
#define SLOW_MMS            (40u)       /* Calibration sweep: S3 car test*/
                                        /* 29 Oct, 150 abrupt.           */
#define RECOVERY_MMS        (60u)       /* Line recovery moves and turns:*/
                                        /* 40 tripped S2's stall E-stop  */
                                        /* (Sean's drive test, 5 Nov).   */
#define SLOW_MS_PER_DEG     (80u)       /* About 3 x the time at 40 mm/s.*/
#define SLOW_MS_PER_MM      (75u)       /* About 3 x the time at 40 mm/s.*/
#define TURN_LEFT_DEG       (90)        /* Barcode turns (FR6).          */
#define TURN_RIGHT_DEG      (-90)
#define TURN_UTURN_DEG      (180)
#define MS_PER_S            (1000u)
#define FAULTS_LATCHED      (S5_FAULT_STALL | S5_FAULT_ENCODER \
                             | S5_FAULT_IMU | S5_FAULT_NO_ECHO)

/** One line-search step: a slow turn, or a slow move at the heading. */
typedef struct
{
    bool    b_move;         /* false: TURN value deg; true: MOVE value mm.*/
    int16_t value;
} recovery_step_t;

/* Line search (FR15), the same for a line lost while following and for
 * a line not found on the way back from a bypass. Turns positive =
 * towards the side the line most likely is: the side S3 last steered to,
 * or after a bypass the obstacle's side. Sweep on the spot (face 90 that
 * way, 90 the other way, then the start heading again: S3's IR sits
 * about 80 mm ahead of the turning centre, so this reaches about 80 mm
 * to each side), drive SEARCH_MOVE_MM on, sweep again, then
 * LINE_NOT_FOUND. After a bypass the heading is still the steep way back
 * towards the line, so "on" goes on towards it; after a line lost while
 * following, the robot first reverses as far as it drove since a sensor
 * last saw the line (log 2026-11-05_0018_track). Each step ends as soon
 * as S3 finds the line.
 */
static const recovery_step_t g_search[SEARCH_STEPS] =
{
    {false, 90}, {false, -180}, {false, 90},
    {true, SEARCH_MOVE_MM},
    {false, 90}, {false, -180}, {false, 90}
};

/* Calibration sweep: headings from the start, + = left (see
 * "Calibrating S3's line sensors" above).
 */
static const int16_t g_cal_target_deg[CAL_TARGETS] = {15, 0, -15, 0};

/** Everything the mission remembers between steps. The state and the
 *  flags share one word at the end (minimum-bit requirement); times and
 *  odometry stay 32-bit, as they are counters that wrap.
 */
typedef struct
{
    uint32_t                run_start_ms;
    uint32_t                barcodes;
    uint32_t                speed_mms;      /* Current FOLLOW speed.     */
    uint8_t                 nav_cmd;        /* subsystem5_nav_cmd_t.     */
    int32_t                 mark_mm;        /* Odometry at a reference.  */
    uint32_t                recovery_step;  /* Next g_search step.       */
    int32_t                 recovery_dir;   /* +1 left first, -1 right.  */
    int32_t                 recovery_back_mm;   /* Reverse still to do.  */
    s5_links_side_t         search_side;    /* Robot's side of the line. */
    int32_t                 confirm_mm;     /* Odometry where it was.    */
    uint32_t                rejects;        /* Not-the-line finds.       */
    int32_t                 follow_mm;      /* Odometry when FOLLOW began*/
    int32_t                 line_mark_mm;   /* Odometry: line last seen. */
    int32_t                 black_from_mm;  /* Odometry: black began.    */
    uint32_t                cal_step;       /* Sweep turns started; 0 =  */
                                            /* waiting for the IMU.      */
    uint32_t                wait_start_ms;  /* Current TURN, or the      */
    uint32_t                wait_limit_ms;  /* calibration, started.     */
    int32_t                 cal_start_cdeg; /* Heading at Button 2.      */
    uint32_t                cal_tries;      /* Corrections at this one.  */
    unsigned int            state         : 4;  /* ..._ctrl_state_t.     */
    unsigned int            b_nav_pending : 1;  /* Barcode not acted on. */
    unsigned int            b_confirming  : 1;  /* Found line not sure.  */
    unsigned int            b_bypass_find : 1;  /* Found by a bypass.    */
    unsigned int            b_junc_fwd    : 1;  /* Driving onto the      */
                                                /* junction before the   */
                                                /* turn.                 */
    unsigned int            b_black       : 1;  /* A sensor sees black.  */
    unsigned int            b_wait_code   : 1;  /* Stopped at a junction,*/
                                                /* waiting for a barcode.*/
    unsigned int            junc_from     : 8;  /* S3's junction count   */
                                                /* when approaching began*/
} mission_t;

/* The state enum must fit its field; a misfit stops the build here. */
typedef char mission_state_check[(S5_CTRL_FINISHED <= S5_CTRL_STATE_MASK)
                                 ? 1 : -1];

static mission_t g_mission = {0u, 0u, 0u, 0u, 0, 0u, 1, 0,
                              S5_LINKS_SIDE_NONE, 0, 0u, 0, 0, 0, 0u, 0u,
                              0u, 0, 0u, S5_CTRL_INIT, false, false,
                              false, false, false, false, 0u};

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
static int32_t cal_error_deg(int32_t target_deg);
static int32_t cal_error_cdeg(int32_t target_deg);
static int32_t command_turn_deg(void);
static void start_cal_sweep(uint32_t now_ms);
static bool cal_needed(void);
static s5_links_side_t reacquire_side(void);
static void step_running(uint32_t events, uint32_t now_ms);
static void step_route(uint32_t events, uint32_t now_ms);
static bool obstacle_stopped(uint32_t events, uint32_t now_ms);
static void adjust_speed(uint32_t events);
static void take_barcodes(void);
static void start_approach(void);
static bool junction_reached(uint32_t events);
static void resume_approach(void);
static void step_following(uint32_t events, uint32_t now_ms);
static bool junction_now(uint32_t events);
static void step_wait_code(uint32_t events, uint32_t now_ms);
static void execute_command(uint32_t now_ms);
static void step_turning(uint32_t events, uint32_t now_ms);
static void start_reacquire(void);
static void step_reacquire(uint32_t events, uint32_t now_ms);
static void start_recovery(uint32_t now_ms, bool b_reverse_first,
                           s5_links_side_t side);
static void line_lost(uint32_t now_ms);
static void begin_confirm(bool b_after_bypass);
static void check_confirm(void);
static bool line_is_lost(uint32_t events);
static void step_recovery(uint32_t events, uint32_t now_ms);
static void step_avoidance(uint32_t events, uint32_t now_ms);
static void start_turn(int32_t angle_deg, uint32_t now_ms);
static void start_slow_turn(int32_t angle_deg, uint16_t speed_mms,
                            uint32_t now_ms);
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
    // NOTE: The cast is safe: the field only ever holds a state value.
    subsystem5_ctrl_state_t state = (subsystem5_ctrl_state_t) g_mission.state;

    if (!g_b_armed)
    {
        b_ok = false;
    }
    else if (S5_CMD_START == opcode)
    {
        if ((S5_CTRL_READY == state) && cal_needed())
        {
            // Without calibrated line sensors the robot would drive
            // straight with no line to follow or find (log
            // 2026-11-04_1701_track: it ended at a wall). So calibrate
            // first, as Sean's drive test does; the run is not started.
            s5_event_trace("RUN not started: calibrating first, press "
                           "Button 1 again after CAL YES:", 0, 0);
            start_cal_sweep(now_ms);
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
        start_cal_sweep(now_ms);
    }
}

/*!
 * @brief Start the line-sensor sweep (see "Calibrating S3's line
 *        sensors"): S3 forgets its levels, then the first turn.
 *
 * @param[in] now_ms Current time.
 */
static void
start_cal_sweep (uint32_t now_ms)
{
    s5_links_line_recalibrate();
    s5_links_motion_release();
    set_state(S5_CTRL_CALIBRATING);
    g_mission.cal_start_cdeg = s5_links_odom_heading_cdeg();
    g_mission.cal_tries      = 0u;
    g_mission.cal_step       = 1u;
    start_slow_turn(cal_error_deg(g_cal_target_deg[0]), SLOW_MMS, now_ms);
}

/*!
 * @brief Tell whether S3's line sensors still need calibrating: S3 is in
 *        use (not a floor test) and not calibrated yet.
 *
 * @return true if a sweep is needed before a run.
 */
static bool
cal_needed (void)
{
    return (s5_links_line_is_used() && (!s5_links_line_is_ready()));
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

    g_mission.state = (uint32_t) state & S5_CTRL_STATE_MASK;
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

    if (0u != (events & EV_LIFTED))
    {
        // Picked up during the sweep: S4 has already braked. Without
        // this the turn never finishes and ends in MOTION_TIMEOUT.
        s5_links_motion_stop();
        s5_event_trace("CAL cancelled: robot lifted, press Button 2 again "
                       "0 0", 0, 0);
        enter_ready();
    }
    else if (0u != (events & EV_OVERTURNED))
    {
        enter_fault(S5_CTRL_FAULT_OVERTURNED);
    }
    else if (0u != g_mission.cal_step)
    {
        step_cal_sweep(events, now_ms);
    }
    else if (s5_links_imu_cal_done() || b_timeout)
    {
        if (b_timeout)
        {
            // NOTE: The cast is safe: the limit is 6000 ms.
            s5_event_trace("CAL IMU not calibrated after ms, carrying on",
                           (int32_t) CAL_TIMEOUT_MS, 0);
        }

        if (g_b_armed && cal_needed())
        {
            // As Sean's drive test: calibrate by itself at power-up.
            s5_event_trace("CAL auto: robot centred on the line, it turns "
                           "left and right by itself 0 0", 0, 0);
            start_cal_sweep(now_ms);
        }
        else
        {
            enter_ready();
        }
    }
    else
    {
        // Still waiting for the IMU.
    }
}

/*!
 * @brief The line-sensor sweep: after each turn, correct it if it ended
 *        CAL_TOL_CDEG or more off its heading, else turn to the next
 *        heading; READY after the last, with S3's result logged.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * NOTE: cal_step is 1 ... CAL_TARGETS: the heading being turned to.
 */
static void
step_cal_sweep (uint32_t events, uint32_t now_ms)
{
    if (0u != (events & EV_MOTION_DONE))
    {
        // NOTE: The cast is safe: cal_step is 1 ... CAL_TARGETS here.
        int32_t target = g_cal_target_deg[g_mission.cal_step - 1u];

        // Let the wheels stop before reading where the turn ended.
        s5_links_motion_settle();

        int32_t error = cal_error_deg(target);

        if ((s5_math_abs(cal_error_cdeg(target)) >= CAL_TOL_CDEG)
            && (g_mission.cal_tries < CAL_FIX_TRIES))
        {
            g_mission.cal_tries++;
            s5_event_trace("CAL correcting: target_deg error_deg", target,
                           error);
            start_slow_turn(error, SLOW_MMS, now_ms);
        }
        else if (g_mission.cal_step < CAL_TARGETS)
        {
            g_mission.cal_tries = 0u;
            g_mission.cal_step++;
            start_slow_turn(cal_error_deg(g_cal_target_deg[
                                g_mission.cal_step - 1u]), SLOW_MMS,
                            now_ms);
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

    if (0u != (events & EV_LIFTED))
    {
        // The demo's end-of-run signal, not a fault.
        s5_event_trace("RUN ended: robot lifted 0 0", 0, 0);
        finish_run(now_ms);
    }
    else if (0u != (events & EV_OVERTURNED))
    {
        enter_fault(S5_CTRL_FAULT_OVERTURNED);
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

    switch ((subsystem5_ctrl_state_t) g_mission.state)
    {
        case S5_CTRL_LINE_FOLLOWING:
            if (g_mission.b_wait_code)
            {
                // Stopped at a junction for a barcode: no speed changes.
                step_wait_code(events, now_ms);
            }
            else
            {
                step_following(events, now_ms);
            }
        break;

        case S5_CTRL_APPROACH_JUNCTION:
            adjust_speed(events);
            update_line_mark();
            check_confirm();

            // A newer barcode replaces the command; its junction may
            // come in the same tick, so this is not an else.
            if (0u != (events & EV_BARCODE))
            {
                take_barcodes();
            }

            if (junction_reached(events)
                || ((!s5_links_line_is_ready())
                    && (travelled >= S5_JUNCTION_MM)))
            {
                execute_command(now_ms);
            }
            else if (s5_links_line_is_ready()
                     && (travelled >= NAV_GIVE_UP_MM))
            {
                // NOTE: The cast is safe: the command is 0 ... 3.
                s5_event_trace("NAV cancelled, no junction: cmd travelled_mm",
                               (int32_t) g_mission.nav_cmd, travelled);
                g_mission.b_nav_pending = false;
                set_state(S5_CTRL_LINE_FOLLOWING);
            }
            else if (line_is_lost(events))
            {
                line_lost(now_ms);
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
            // Kept: after the bypass the robot approaches the next
            // junction (resume_approach()).
            // NOTE: The cast is safe: the command is 0 ... 3.
            s5_event_trace("NAV kept through the obstacle: cmd 0",
                           (int32_t) g_mission.nav_cmd, 0);
        }

        g_mission.b_wait_code = false;
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
    uint32_t speed = g_mission.speed_mms;

    if (0u != (events & EV_OBSTACLE_NEAR))
    {
        speed = slow_mms();
    }
    else if (0u != (events & EV_OBSTACLE_CLEAR))
    {
        speed = cruise_mms();
    }
    else
    {
        // No change.
    }

    // Only a real change is sent: a repeated FOLLOW disturbed the
    // steering on curves (slow and cruise are both 50 mm/s now).
    if (speed != g_mission.speed_mms)
    {
        g_mission.speed_mms = speed;

        // NOTE: The cast is safe: speeds are a few hundred mm/s at most.
        s5_links_motion_follow((uint16_t) speed);
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
        start_approach();
    }
}

/*!
 * @brief Approach the next junction for the pending command: from here,
 *        count S3's junctions and measure the distance.
 */
static void
start_approach (void)
{
    g_mission.junc_from = s5_links_junction_count() & JUNC_COUNT_MASK;
    g_mission.mark_mm   = s5_links_odom_dist_mm();
    set_state(S5_CTRL_APPROACH_JUNCTION);
}

/*!
 * @brief With a command pending: has the robot reached a junction since
 *        the approach began? S3's event, or its junction counter moved
 *        on (it catches a junction whose event came with the barcode).
 *
 * @param[in] events Mission events.
 *
 * @return true if a junction counts. Never in the first CROSS_MM after
 *         a line was found by a search or a bypass: that "junction" is
 *         the robot crossing the line at an angle (the line-loss rules
 *         handle it); the count starts again from there. A junction
 *         5 ... 12 cm after the find counts (it was ignored until the line
 *         was confirmed, Anna 2026-11-18).
 */
static bool
junction_reached (uint32_t events)
{
    uint32_t count     = s5_links_junction_count() & JUNC_COUNT_MASK;
    bool     b_reached = false;

    if (!g_mission.b_nav_pending)
    {
        // Nothing to do at a junction.
    }
    else if (g_mission.b_confirming
             && ((s5_links_odom_dist_mm() - g_mission.confirm_mm)
                 < CROSS_MM))
    {
        // Within CROSS_MM of the find a "junction" is the robot crossing
        // the line (the hand-over lines the robot up, so later ones are
        // real): start the count again from here.
        g_mission.junc_from = count & JUNC_COUNT_MASK;
    }
    else
    {
        b_reached = (0u != (events & EV_JUNCTION))
                    || (count != (uint32_t) g_mission.junc_from);
    }

    return (b_reached);
}

/*!
 * @brief LINE_FOLLOWING: S3 follows the line; take barcodes, wait at a
 *        junction for a barcode still being read, and watch for line
 *        loss.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_following (uint32_t events, uint32_t now_ms)
{
    adjust_speed(events);
    update_line_mark();
    check_confirm();

    if (0u != (events & EV_BARCODE))
    {
        take_barcodes();

        // The junction may have come in the same tick.
        if (junction_reached(events))
        {
            execute_command(now_ms);
        }
    }
    else if (junction_now(events))
    {
        // Stop and decide at every junction, as Sean's drive test does:
        // brake and let the wheels stop first.
        s5_links_motion_settle();

        if (s5_links_barcode_reading())
        {
            // Its letter is not out yet: wait for it here.
            // NOTE: The cast is safe: the wait is 2000 ms.
            s5_event_trace("NAV junction while a barcode is read: waiting "
                           "ms 0", (int32_t) BARCODE_WAIT_MS, 0);
            g_mission.b_wait_code   = true;
            g_mission.wait_start_ms = now_ms;
        }
        else
        {
            s5_event_trace("NAV junction, no command: straight on 0 0", 0,
                           0);
            resume_following();
        }
    }
    else if (line_is_lost(events))
    {
        line_lost(now_ms);
    }
    else
    {
        // Keep following.
    }
}

/*!
 * @brief A junction under the robot now (S3's event), not counting the
 *        first CROSS_MM after a found line (crossing it at an angle).
 *
 * @param[in] events Mission events.
 *
 * @return true for a real junction.
 */
static bool
junction_now (uint32_t events)
{
    bool b_crossing = g_mission.b_confirming
                      && ((s5_links_odom_dist_mm() - g_mission.confirm_mm)
                          < CROSS_MM);

    return ((0u != (events & EV_JUNCTION)) && (!b_crossing));
}

/*!
 * @brief Stopped at a junction while S3 reads a barcode (see the file
 *        comment): act on its command here when it comes, else go on
 *        straight after BARCODE_WAIT_MS.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_wait_code (uint32_t events, uint32_t now_ms)
{
    if (0u != (events & EV_BARCODE))
    {
        g_mission.b_wait_code = false;
        take_barcodes();

        if (g_mission.b_nav_pending)
        {
            // The robot is at the junction the barcode was for.
            execute_command(now_ms);
        }
        else
        {
            resume_following();
        }
    }
    else if ((now_ms - g_mission.wait_start_ms) >= BARCODE_WAIT_MS)
    {
        s5_event_trace("NAV no barcode came: straight on 0 0", 0, 0);
        g_mission.b_wait_code = false;
        resume_following();
    }
    else
    {
        // Still waiting.
    }
}

/*!
 * @brief Following again after a line search or a bypass: with a command
 *        still pending, approach the next junction instead (Anna,
 *        2026-11-17: after avoiding an obstacle it forgot the turn).
 */
static void
resume_approach (void)
{
    if (g_mission.b_nav_pending)
    {
        // NOTE: The cast is safe: the command is 0 ... 3.
        s5_event_trace("NAV kept: approaching the next junction cmd 0",
                       (int32_t) g_mission.nav_cmd, 0);
        start_approach();
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
    int32_t angle_deg = command_turn_deg();

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
        // The line sensors are on the crossing, the rear axle the robot
        // turns about is still behind it: drive onto it first, then turn
        // (step_turning()).
        // NOTE: The cast is safe: JUNC_FWD_MM is a few hundred mm.
        uint32_t dist = (uint32_t) JUNC_FWD_MM;

        g_mission.b_junc_fwd    = true;
        s5_links_motion_move(JUNC_FWD_MM, (uint16_t) JUNC_FWD_MMS);
        g_mission.wait_start_ms = now_ms;
        g_mission.wait_limit_ms = ((dist * MS_PER_S) / JUNC_FWD_MMS)
                                  + MOTION_SLACK_MS;
    }
}

/*!
 * @brief The turn for the command taken at the junction.
 *
 * @return Degrees, + = left; 0 for STRAIGHT.
 */
static int32_t
command_turn_deg (void)
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

    return (angle_deg);
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
    if ((0u != (events & EV_MOTION_DONE)) && g_mission.b_junc_fwd)
    {
        // The rear axle is on the crossing now: turn.
        g_mission.b_junc_fwd = false;
        start_turn(command_turn_deg(), now_ms);
    }
    else if (0u != (events & EV_MOTION_DONE))
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
        line_lost(now_ms);
    }
    else
    {
        // Still looking.
    }
}

/*!
 * @brief The line was lost while following. While a newly found line is
 *        still being confirmed, that was not the line (a barcode bar or
 *        a mark): search on from here. Otherwise reverse to where it was
 *        last seen, then search.
 *
 * @param[in] now_ms Current time.
 */
static void
line_lost (uint32_t now_ms)
{
    // NOTE: The cast is safe: the flag is 0 or 1.
    s5_event_trace("LINE lost: white_mm s3_lost",
                   s5_links_odom_dist_mm() - g_mission.line_mark_mm,
                   (int32_t) s5_links_line_is_lost());

    if (g_mission.b_confirming && g_mission.b_bypass_find)
    {
        // Crossed it after the bypass: not a bar. Search forward first
        // (S3's last side pointed back at the obstacle, log
        // 2026-11-13_1554); a robot right of the line turns left first.
        g_mission.b_confirming = false;
        start_recovery(now_ms, true,
                       s5_avoid_forward_is_left() ? S5_LINKS_SIDE_RIGHT
                                                  : S5_LINKS_SIDE_LEFT);
    }
    else if (g_mission.b_confirming)
    {
        g_mission.b_confirming = false;
        g_mission.rejects++;

        // NOTE: The cast is safe: the count is at most REJECTS_MAX + 1.
        s5_event_trace("LINE rejected, not the line: followed_mm rejects",
                       s5_links_odom_dist_mm() - g_mission.confirm_mm,
                       (int32_t) g_mission.rejects);

        if (g_mission.rejects > REJECTS_MAX)
        {
            enter_fault(S5_CTRL_FAULT_LINE_NOT_FOUND);
        }
        else
        {
            start_recovery(now_ms, false, g_mission.search_side);
        }
    }
    else
    {
        // Read S3's last steering before anything changes it.
        start_recovery(now_ms, true, s5_links_line_side());
    }
}

/*!
 * @brief While following: has the line been lost? Either S3's LOST event,
 *        or S3 still in LOST once the robot has driven LOST_CHECK_MM
 *        since FOLLOW began (S3 then holds its last steering, so the
 *        robot would circle for ever), or no black under either sensor
 *        for WHITE_LOST_MM of driving (with S3 in use).
 *
 * @param[in] events Mission events.
 *
 * @return true if the line is lost.
 */
static bool
line_is_lost (uint32_t events)
{
    int32_t odom   = s5_links_odom_dist_mm();
    int32_t driven = odom - g_mission.follow_mm;
    bool    b_over = g_mission.b_confirming && g_mission.b_bypass_find
                     && (0u != (events & EV_JUNCTION))
                     && (driven < CROSS_MM);

    if (b_over)
    {
        s5_event_trace("LINE crossed after the bypass (junction): driven_mm"
                       " 0", driven, 0);
    }

    return (b_over || (0u != (events & EV_LINE_LOST))
            || ((driven >= LOST_CHECK_MM) && s5_links_line_is_lost())
            || (s5_links_line_is_ready() && g_mission.b_confirming
                && g_mission.b_bypass_find
                && ((odom - g_mission.line_mark_mm) >= WHITE_LOST_MM)));
}

/*!
 * @brief A search or a bypass found a line: follow it, and confirm it
 *        over CONFIRM_MM (see "Line or barcode bar" above).
 *
 * @param[in] b_after_bypass true if a bypass found it.
 */
static void
begin_confirm (bool b_after_bypass)
{
    g_mission.b_bypass_find = b_after_bypass;
    g_mission.b_confirming  = true;
    g_mission.confirm_mm   = s5_links_odom_dist_mm();
}

/*!
 * @brief While following: the found line has been followed CONFIRM_MM,
 *        so it is the line.
 */
static void
check_confirm (void)
{
    int32_t followed = s5_links_odom_dist_mm() - g_mission.confirm_mm;

    if (g_mission.b_confirming && (followed >= CONFIRM_MM))
    {
        g_mission.b_confirming = false;
        g_mission.rejects      = 0u;
        s5_event_trace("LINE confirmed: followed_mm 0", followed, 0);
    }
}

/*!
 * @brief LINE_RECOVERY: stop and sweep left and right looking for the
 *        line (Week 6 report 6.1 SEARCHING).
 *
 * @param[in] now_ms          Current time.
 * @param[in] b_reverse_first true: lost while following, so first back
 *                            to where it was last seen; false: search
 *                            from here (after a bypass or a rejected
 *                            find).
 * @param[in] side            The robot's side of the line, if known.
 */
static void
start_recovery (uint32_t now_ms, bool b_reverse_first, s5_links_side_t side)
{
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

    if (b_reverse_first)
    {
        s5_event_trace("RECOVERY reverse_mm (driven since line seen + 20)"
                       " 0", back, 0);
    }
    else
    {
        back = 0;
        s5_event_trace("RECOVERY search from here 0 0", 0, 0);
    }

    // A robot left of the line turns right (negative) first.
    g_mission.recovery_dir     = (S5_LINKS_SIDE_LEFT == side) ? -1 : 1;
    g_mission.recovery_step    = 0u;
    g_mission.recovery_back_mm = back;
    g_mission.search_side      = side;
    s5_links_motion_settle();
    s5_links_line_search(side);
    set_state(S5_CTRL_LINE_RECOVERY);
    start_recovery_step(now_ms);
}

/*!
 * @brief Start the next line-search step: the measured reverse, then
 *        g_search; LINE_NOT_FOUND after the last step.
 *
 * @param[in] now_ms Current time.
 */
static void
start_recovery_step (uint32_t now_ms)
{
    if (0 != g_mission.recovery_back_mm)
    {
        // Line lost while following: back to where it was last seen.
        start_slow_move(-g_mission.recovery_back_mm, now_ms);
        g_mission.recovery_back_mm = 0;
    }
    else if (g_mission.recovery_step < SEARCH_STEPS)
    {
        recovery_step_t const * p_step = &g_search[g_mission.recovery_step];

        g_mission.recovery_step++;

        if (p_step->b_move)
        {
            start_slow_move(p_step->value, now_ms);
        }
        else
        {
            start_slow_turn(g_mission.recovery_dir * p_step->value,
                            RECOVERY_MMS, now_ms);
        }
    }
    else
    {
        enter_fault(S5_CTRL_FAULT_LINE_NOT_FOUND);
    }
}

/*!
 * @brief LINE_RECOVERY: search step by step until S3 finds the line,
 *        then follow it and confirm it.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 */
static void
step_recovery (uint32_t events, uint32_t now_ms)
{
    // Found: S3's event, or S3 already following (the event came before
    // the search began, or mid-swing).
    if ((0u != (events & EV_LINE_FOUND)) || s5_links_line_is_locked())
    {
        // Stop the sweep; resume_following() lets the wheels settle.
        s5_links_motion_stop();
        resume_following();
        begin_confirm(false);
        resume_approach();
    }
    else if (0u != (events & EV_MOTION_DONE))
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
 * @brief Send one slow MOVE (line search, at RECOVERY_MMS) and start its
 *        time limit.
 *
 * @param[in] move_mm Distance; negative = backwards.
 * @param[in] now_ms  Current time.
 */
static void
start_slow_move (int32_t move_mm, uint32_t now_ms)
{
    // NOTE: The cast is safe: |move_mm| is at most a few hundred mm.
    uint32_t dist = (uint32_t) s5_math_abs(move_mm);

    s5_links_motion_move(move_mm, RECOVERY_MMS);
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
            begin_confirm(true);
            resume_approach();
        break;

        case S5_AVOID_PLAN_ONLY:
            finish_run(now_ms);
        break;

        case S5_AVOID_FAILURE:
            enter_fault(s5_avoid_fault());
        break;

        case S5_AVOID_SEARCH:
            // Search from here, towards the obstacle's side (g_search).
            start_recovery(now_ms, false,
                           s5_avoid_passed_left() ? S5_LINKS_SIDE_LEFT
                                                  : S5_LINKS_SIDE_RIGHT);
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

    // NOTE: The cast is safe: JUNC_TURN_MMS fits uint16_t.
    s5_links_motion_turn(angle_deg, (uint16_t) JUNC_TURN_MMS);
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
 * @param[in] speed_mms SLOW_MMS (calibration) or RECOVERY_MMS.
 * @param[in] now_ms    Current time.
 */
static void
start_slow_turn (int32_t angle_deg, uint16_t speed_mms, uint32_t now_ms)
{
    // NOTE: The casts are safe: |angle_deg| is at most 180.
    uint32_t angle = (angle_deg < 0) ? (uint32_t) -angle_deg
                                     : (uint32_t) angle_deg;

    s5_links_motion_turn(angle_deg, speed_mms);
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
    g_mission.b_confirming  = false;
    g_mission.b_wait_code   = false;
    g_mission.rejects       = 0u;

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
    g_mission.follow_mm = s5_links_odom_dist_mm();
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

    // The run summary also reports S4's highest hump of the run (brief:
    // "in telemetry and in the run summary").
    uint32_t hump_mm = s5_links_hump_peak_mm();

    // NOTE: The casts are safe: both counts are far below 2^31, and the
    // hump is at most 255 mm.
    s5_event_post_args(S5_EVENT_RUN_DONE, g_mission.barcodes, run_s);
    s5_event_post_args(S5_EVENT_RUN_HUMP, hump_mm, 0u);
    s5_event_trace("RUN summary: barcodes run_s",
                   (int32_t) g_mission.barcodes, (int32_t) run_s);
    s5_event_trace("RUN summary: hump_peak_mm 0", (int32_t) hump_mm, 0);
    set_state(S5_CTRL_FINISHED);
}

/*!
 * @brief The calibration sweep: the turn from S2's odometry heading now
 *        to a heading relative to where the robot stood at Button 2.
 *
 * @param[in] target_deg Heading from the start, + = left.
 *
 * @return Whole degrees, + = left, the shorter way round.
 */
static int32_t
cal_error_deg (int32_t target_deg)
{
    int32_t diff = cal_error_cdeg(target_deg);

    // Round to whole degrees.
    return ((diff + ((diff >= 0) ? (CDEG_PER_DEG / 2)
                                 : -(CDEG_PER_DEG / 2)))
            / CDEG_PER_DEG);
}

/*!
 * @brief As cal_error_deg(), in 1/100 degree (for the tolerance).
 *
 * @param[in] target_deg Heading from the start, + = left.
 *
 * @return 1/100 degree, + = left, -18000 ... 18000.
 */
static int32_t
cal_error_cdeg (int32_t target_deg)
{
    int32_t diff = (g_mission.cal_start_cdeg + (target_deg * CDEG_PER_DEG))
                   - s5_links_odom_heading_cdeg();

    if (diff > CDEG_HALF_TURN)
    {
        diff -= 2 * CDEG_HALF_TURN;
    }
    else if (diff < -CDEG_HALF_TURN)
    {
        diff += 2 * CDEG_HALF_TURN;
    }
    else
    {
        // Already the shorter way.
    }

    return (diff);
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
 * TODO: Buzzer pattern for a fault, through S3's buzzer function once S3
 * offers one (S3 owns GP22, TEAM.md; Week 6 report section 5).
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
