/** @file s5_avoid.c
 *
 * @brief Obstacle avoidance state machine (see s5_avoid.h and the leg
 *        diagram in s5_plan.h).
 *
 * @par Waiting for motion
 * Every MOVE and TURN ends with EV_MOTION_DONE from motion_task. Each
 * wait has a time limit worked out from the distance and speed; if it
 * runs out, avoidance fails with S5_CTRL_FAULT_MOTION_TIMEOUT instead of
 * waiting for ever.
 *
 * @par Going round
 * The plan (s5_plan.c) works out the turn-out angle from the obstacle's
 * width, so that the robot's front corner clears the obstacle by
 * S5_PATH_MARGIN_MM, and the legs from the robot's measured size. While
 * passing, obstacle_task watches the obstacle's side and reports when it
 * has been passed; the tail then puts the obstacle behind the rear axle
 * before turning in. Obstacles are never placed back to back (Anna,
 * 2026-11-06), so nothing else is watched on the way round.
 *
 * @par Squaring up first
 * The robot often stops crooked (about 20 degrees in the logs of
 * 2026-11-05/06: a box on the line looked centred at 70 degrees). A flat
 * face then slants away from the sensor on one side, and beyond about 20
 * degrees off square it sends no echo, so the scan ends that edge too
 * early (log 2026-11-06_1521: the left edge read 50 mm short and the
 * robot touched the box while passing). A flat face is nearest along its
 * own normal, so after the first scan, if the closest point is more than
 * SQUARE_MIN_DEG off straight ahead, the robot turns that way (at most
 * SQUARE_MAX_DEG) and scans again, once per obstacle. Square on, both
 * edges echo, and the bypass is built square to the obstacle.
 *
 * @par Finding the line again
 * The way back mirrors the way out (same angle, same leg) and S3
 * searches from the side the robot went round on. As soon as S3 has the
 * line (its LINE_FOUND event, or already following it), also while still
 * turning in, the robot stops and S3's line following takes over (S3
 * locks onto the correct edge from either side, IF-07); the mission then
 * confirms it is the line and not a barcode bar (s5_mission.c). This
 * replaced "drive the turning centre onto the line, then turn along it",
 * which relied on a guessed sensor distance and ended past the line
 * (logs 2026-11-06_1436 and _1514). Not found within SEARCH_EXTRA_MM
 * past the plan: the mission searches on the spot (S5_AVOID_SEARCH).
 * While S3 is not ready (floor tests) the return leg is driven its
 * planned length and the robot turns back along the expected line.
 *
 * NOTE: When the side watch reports the obstacle passed, the long MOVE
 * alongside is cut short with STOP. If its EV_MOTION_DONE arrives in the
 * same instant, it is taken together with the pass, so the following
 * tail MOVE is not mistaken as finished.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_avoid.h"
#include "s5_event.h"
#include "s5_links.h"
#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"
#include "subsystem5.h"

#define SPEED_MMS           (70u)       /* Moves and turns going round:  */
                                        /* slow for accuracy and for S3  */
                                        /* to see the line.              */
#define REVERSE_MM          (100)       /* Week 6 report 6.4.            */
#define RETRIES_MAX         (2u)        /* Then NO_PATH.                 */
#define WATCH_MAX_MM        (800)       /* Longest obstacle expected.    */
#define SEARCH_EXTRA_MM     (60)        /* Past where the line should be.*/
#define AHEAD_DEG           (90)        /* Servo angle straight ahead.   */
#define SQUARE_MIN_DEG      (8)         /* Closer to square: no turn.    */
#define SQUARE_MAX_DEG      (30)        /* Largest squaring turn.        */
#define SCAN_TIMEOUT_MS     (8000u)     /* A scan takes about 2.5 s.     */
#define MOTION_SLACK_MS     (2000u)     /* Added to every motion limit.  */
#define TURN_MS_PER_DEG     (40u)       /* About 2 x the time at 70 mm/s.*/
#define MS_PER_S            (1000u)
#define MM_PER_CM           (10u)

/** Everything the avoidance remembers between steps. */
typedef struct
{
    s5_avoid_state_t        state;
    s5_plan_t               plan;
    uint32_t                retries;
    uint32_t                wait_start_ms;
    uint32_t                wait_limit_ms;
    uint32_t                profile_seq;
    subsystem5_ctrl_fault_t fault;
    bool                    b_squared;      /* Squared up already.       */
    bool                    b_passed_early; /* Passed while turning back.*/
} avoid_t;

static avoid_t       g_avoid       = {S5_AVOID_IDLE,
                                      {S5_PLAN_CONTINUE, 0, 0, 0u, 0u, 0u,
                                       0u, 0u},
                                      0u, 0u, 0u, 0u, S5_CTRL_FAULT_NONE,
                                      false, false};
static volatile bool g_b_bypass_on = true;

static s5_avoid_result_t step_scanning(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t act_on_plan(uint32_t now_ms);
static bool              square_up(s5_profile_t const * p_profile,
                                   uint32_t now_ms);
static s5_avoid_result_t step_bypass(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t next_leg(uint32_t now_ms);
static s5_avoid_result_t step_return(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t hand_over(void);
static void              request_scan(uint32_t now_ms);
static void              start_move(int32_t dist_mm, uint32_t now_ms);
static void              start_turn(int32_t angle_deg, uint32_t now_ms);
static bool              is_timed_out(uint32_t now_ms);
static s5_avoid_result_t fail(subsystem5_ctrl_fault_t fault);
static void              enter(s5_avoid_state_t state);

/*!
 * @brief Allow or forbid the bypass itself (S5 floor test S5-U08 plans
 *        without moving round the obstacle).
 *
 * @param[in] b_enabled false: stop after the plan.
 */
void
s5_avoid_set_bypass (bool b_enabled)
{
    g_b_bypass_on = b_enabled;
}

/*!
 * @brief Begin avoidance: the robot has just been stopped by
 *        obstacle_task, so ask it for a scan.
 *
 * @param[in] now_ms Current time.
 */
void
s5_avoid_start (uint32_t now_ms)
{
    g_avoid.retries   = 0u;
    g_avoid.b_squared = false;
    g_avoid.fault     = S5_CTRL_FAULT_NONE;
    request_scan(now_ms);
}

/*!
 * @brief Advance the avoidance with a batch of mission events.
 *
 * @param[in] events EV_xxx bits taken from flg_mission (may be 0).
 * @param[in] now_ms Current time.
 *
 * @return BUSY until finished; then RESUME, PLAN_ONLY, FAILURE or SEARCH.
 */
s5_avoid_result_t
s5_avoid_step (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;

    switch (g_avoid.state)
    {
        case S5_AVOID_SCANNING:
            result = step_scanning(events, now_ms);
        break;

        case S5_AVOID_RETURN:
            result = step_return(events, now_ms);
        break;

        case S5_AVOID_IDLE:
        case S5_AVOID_DONE:
            result = S5_AVOID_RESUME;
        break;

        case S5_AVOID_FAILED:
            result = S5_AVOID_FAILURE;
        break;

        // Every other state waits for one MOVE or TURN to finish.
        default:
            result = step_bypass(events, now_ms);
        break;
    }

    return (result);
}

/*!
 * @brief Which side the last bypass went round: the line is then on the
 *        other side of the robot (the obstacle's side).
 *
 * @return True if the robot went round the obstacle's left.
 */
bool
s5_avoid_passed_left (void)
{
    return (g_avoid.plan.turn_deg > 0);
}

/*!
 * @brief Why the avoidance failed.
 *
 * @return The fault, or S5_CTRL_FAULT_NONE.
 */
subsystem5_ctrl_fault_t
s5_avoid_fault (void)
{
    return (g_avoid.fault);
}

/*!
 * @brief SCANNING: wait for a new profile, then plan.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return The step's result.
 */
static s5_avoid_result_t
step_scanning (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t result  = S5_AVOID_BUSY;
    s5_profile_t      profile = {0};

    subsystem5_profile_get(&profile);

    if ((0u != (events & EV_PROFILE_READY))
        && (profile.seq != g_avoid.profile_seq))
    {
        if (!square_up(&profile, now_ms))
        {
            s5_plan_make(&profile, &g_avoid.plan);
            result = act_on_plan(now_ms);
        }
    }
    else if ((now_ms - g_avoid.wait_start_ms) >= SCAN_TIMEOUT_MS)
    {
        result = fail(S5_CTRL_FAULT_SCAN_TIMEOUT);
    }
    else
    {
        // Still scanning.
    }

    return (result);
}

/*!
 * @brief Turn to face the obstacle squarely, once, if the robot stopped
 *        crooked (see "Squaring up first" above).
 *
 * @param[in] p_profile The first scan's profile.
 * @param[in] now_ms    Current time.
 *
 * @return true if a turn was started (a new scan follows it).
 */
static bool
square_up (s5_profile_t const * p_profile, uint32_t now_ms)
{
    bool    b_turning = false;
    int32_t off_deg   = (int32_t) p_profile->closest_deg - AHEAD_DEG;

    if (off_deg > SQUARE_MAX_DEG)
    {
        off_deg = SQUARE_MAX_DEG;
    }
    else if (off_deg < -SQUARE_MAX_DEG)
    {
        off_deg = -SQUARE_MAX_DEG;
    }
    else
    {
        // Already inside the limits.
    }

    if ((!g_avoid.b_squared) && g_b_bypass_on && p_profile->b_on_path
        && (s5_math_abs(off_deg) > SQUARE_MIN_DEG))
    {
        // Servo angles above 90 are to the left, as are positive turns.
        s5_event_trace("AVOID squaring up: turn_deg closest_deg", off_deg,
                       (int32_t) p_profile->closest_deg);
        g_avoid.b_squared = true;
        s5_links_motion_release();
        start_turn(off_deg, now_ms);
        enter(S5_AVOID_SQUARING);
        b_turning = true;
    }

    return (b_turning);
}

/*!
 * @brief Carry out a fresh plan: continue, reverse or start the bypass.
 *
 * @param[in] now_ms Current time.
 *
 * @return The step's result.
 */
static s5_avoid_result_t
act_on_plan (uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    s5_plan_t const * p_plan = &g_avoid.plan;

    // FR16: log every decision with the clearance it was based on.
    // NOTE: The casts are safe: the action is a small enum and the
    // clearance is at most 1000 mm.
    s5_event_post_args(S5_EVENT_AVOID_PLAN, (uint32_t) p_plan->action,
                       (uint32_t) p_plan->clearance_mm / MM_PER_CM);
    s5_event_trace("PLAN action out_mm", (int32_t) p_plan->action,
                   (int32_t) p_plan->out_mm);
    s5_event_trace("PLAN turn_deg tail_mm", p_plan->turn_deg,
                   (int32_t) p_plan->tail_mm);
    s5_event_trace("PLAN back_mm reverse_first_mm",
                   (int32_t) p_plan->back_mm, (int32_t) p_plan->room_mm);

    if (!g_b_bypass_on)
    {
        enter(S5_AVOID_DONE);
        result = S5_AVOID_PLAN_ONLY;
    }
    else if (S5_PLAN_CONTINUE == p_plan->action)
    {
        s5_links_motion_release();
        enter(S5_AVOID_DONE);
        result = S5_AVOID_RESUME;
    }
    else if (S5_PLAN_REVERSE == p_plan->action)
    {
        g_avoid.retries++;

        if (g_avoid.retries > RETRIES_MAX)
        {
            result = fail(S5_CTRL_FAULT_NO_PATH);
        }
        else
        {
            s5_links_motion_release();
            start_move(-REVERSE_MM, now_ms);
            enter(S5_AVOID_REVERSING);
        }
    }
    else if (0u != p_plan->room_mm)
    {
        // Too close to turn out and miss it: reverse first.
        // NOTE: The cast widens uint16_t, so no value is lost.
        s5_links_motion_release();
        start_move(-(int32_t) p_plan->room_mm, now_ms);
        enter(S5_AVOID_MAKE_ROOM);
    }
    else
    {
        s5_links_motion_release();
        start_turn(p_plan->turn_deg, now_ms);
        enter(S5_AVOID_TURN_OUT);
    }

    return (result);
}

/*!
 * @brief One bypass leg: when the current MOVE or TURN is done, start
 *        the next one.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or FAILURE on a time-out.
 */
static s5_avoid_result_t
step_bypass (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    bool              b_done = (0u != (events & EV_MOTION_DONE));
    bool              b_pass = (0u != (events & EV_OBST_PASSED));

    // Passed while still turning back parallel: acted on when it ends.
    if ((S5_AVOID_TURN_PARALLEL == g_avoid.state) && b_pass)
    {
        g_avoid.b_passed_early = true;
    }

    if ((S5_AVOID_TURN_IN == g_avoid.state)
        && ((0u != (events & EV_LINE_FOUND)) || s5_links_line_is_locked()))
    {
        // S3 has the line while still turning in: it takes over.
        result = hand_over();
    }
    else if ((S5_AVOID_PASSING == g_avoid.state) && (b_pass || b_done))
    {
        // Passed (or drove the longest expected obstacle): the far end is
        // level with the sensor; the tail puts it behind the rear axle.
        s5_links_motion_stop();
        subsystem5_obst_request(OB_HOLD);
        start_move(g_avoid.plan.tail_mm, now_ms);
        enter(S5_AVOID_TAIL);
    }
    else if (b_done)
    {
        result = next_leg(now_ms);
    }
    else if (is_timed_out(now_ms))
    {
        result = fail(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still moving.
    }

    return (result);
}

/*!
 * @brief The current MOVE or TURN has finished: start the next leg.
 *
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or RESUME after the last leg.
 */
static s5_avoid_result_t
next_leg (uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    s5_plan_t const * p_plan = &g_avoid.plan;

    switch (g_avoid.state)
    {
        case S5_AVOID_REVERSING:
        // Squared up: scan again, now facing the obstacle.
        case S5_AVOID_SQUARING:
            request_scan(now_ms);
        break;

        case S5_AVOID_MAKE_ROOM:
            start_turn(p_plan->turn_deg, now_ms);
            enter(S5_AVOID_TURN_OUT);
        break;

        case S5_AVOID_TURN_OUT:
            start_move(p_plan->out_mm, now_ms);
            enter(S5_AVOID_MOVE_OUT);
        break;

        case S5_AVOID_MOVE_OUT:
            // Start the side watch now, so the servo is already pointing
            // sideways when passing begins (a small obstacle is passed
            // within its 0.4 s swing). Going round on the left leaves the
            // obstacle on the right.
            g_avoid.b_passed_early = false;
            subsystem5_obst_request((p_plan->turn_deg > 0) ? OB_WATCH_RIGHT
                                                           : OB_WATCH_LEFT);
            start_turn(-p_plan->turn_deg, now_ms);
            enter(S5_AVOID_TURN_PARALLEL);
        break;

        case S5_AVOID_TURN_PARALLEL:
            if (g_avoid.b_passed_early)
            {
                start_move(p_plan->tail_mm, now_ms);
                enter(S5_AVOID_TAIL);
            }
            else
            {
                start_move(WATCH_MAX_MM, now_ms);
                enter(S5_AVOID_PASSING);
            }
        break;

        case S5_AVOID_TAIL:
            start_turn(-p_plan->in_deg, now_ms);
            enter(S5_AVOID_TURN_IN);
        break;

        case S5_AVOID_TURN_IN:
            s5_links_line_search((p_plan->turn_deg > 0) ? S5_LINKS_SIDE_LEFT
                                                        : S5_LINKS_SIDE_RIGHT);
            start_move(p_plan->back_mm
                       + (s5_links_line_is_ready() ? SEARCH_EXTRA_MM : 0),
                       now_ms);
            enter(S5_AVOID_RETURN);
        break;

        case S5_AVOID_REALIGN:
            // Floor tests only (no S3): along the expected line.
            enter(S5_AVOID_DONE);
            result = S5_AVOID_RESUME;
        break;

        default:
            // Not a waiting state; nothing to do.
        break;
    }

    return (result);
}

/*!
 * @brief RETURN: drive back towards the line until S3 has it, then let
 *        S3's line following take over.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return BUSY, SEARCH if the line is not where it should be, or FAILURE.
 */
static s5_avoid_result_t
step_return (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t result  = S5_AVOID_BUSY;
    bool              b_found = (0u != (events & EV_LINE_FOUND));
    bool              b_done  = (0u != (events & EV_MOTION_DONE));

    if (b_found || s5_links_line_is_locked())
    {
        result = hand_over();
    }
    else if (b_done && (!s5_links_line_is_ready()))
    {
        // Without S3, where the line should be.
        start_turn(g_avoid.plan.in_deg, now_ms);
        enter(S5_AVOID_REALIGN);
    }
    else if (b_done)
    {
        // Not found where it should be (it may curve away): the mission
        // searches from here.
        s5_event_trace("AVOID line not found on return: searching here 0",
                       0, 0);
        enter(S5_AVOID_DONE);
        result = S5_AVOID_SEARCH;
    }
    else if (is_timed_out(now_ms))
    {
        result = fail(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still driving back.
    }

    return (result);
}

/*!
 * @brief S3 has the line: stop here and let its line following take
 *        over (see "Finding the line again" above).
 *
 * @return S5_AVOID_RESUME: the mission resumes following and confirms
 *         the line.
 */
static s5_avoid_result_t
hand_over (void)
{
    // NOTE: The cast is safe: the state is a small enum value.
    s5_event_trace("AVOID line found: S3 takes over, state",
                   (int32_t) g_avoid.state, 0);
    s5_links_motion_stop();
    enter(S5_AVOID_DONE);

    return (S5_AVOID_RESUME);
}

/*!
 * @brief Ask obstacle_task for a scan and wait for the new profile.
 *
 * @param[in] now_ms Current time.
 */
static void
request_scan (uint32_t now_ms)
{
    s5_profile_t profile = {0};

    // Remember the current profile so only a newer one counts.
    subsystem5_profile_get(&profile);
    g_avoid.profile_seq   = profile.seq;
    g_avoid.wait_start_ms = now_ms;

    subsystem5_obst_request(OB_SCAN_REQ);
    enter(S5_AVOID_SCANNING);
}

/*!
 * @brief Send a MOVE and start its time limit.
 *
 * @param[in] dist_mm Distance; negative = reverse.
 * @param[in] now_ms  Current time.
 */
static void
start_move (int32_t dist_mm, uint32_t now_ms)
{
    uint32_t dist = (dist_mm < 0) ? (uint32_t) -dist_mm : (uint32_t) dist_mm;

    // NOTE: The casts above are safe: |dist_mm| is at most a few metres.
    s5_links_motion_move(dist_mm, SPEED_MMS);
    g_avoid.wait_start_ms = now_ms;
    g_avoid.wait_limit_ms = ((dist * MS_PER_S) / SPEED_MMS) + MOTION_SLACK_MS;
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
    uint32_t angle = (angle_deg < 0) ? (uint32_t) -angle_deg
                                     : (uint32_t) angle_deg;

    // NOTE: The casts above are safe: |angle_deg| is at most 180.
    s5_links_motion_turn(angle_deg, SPEED_MMS);
    g_avoid.wait_start_ms = now_ms;
    g_avoid.wait_limit_ms = (angle * TURN_MS_PER_DEG) + MOTION_SLACK_MS;
}

/*!
 * @brief Tell whether the current MOVE or TURN has taken too long.
 *
 * @param[in] now_ms Current time.
 *
 * @return true once its time limit has passed.
 */
static bool
is_timed_out (uint32_t now_ms)
{
    return ((now_ms - g_avoid.wait_start_ms) >= g_avoid.wait_limit_ms);
}

/*!
 * @brief Give up: stop the robot and remember why.
 *
 * @param[in] fault The reason.
 *
 * @return S5_AVOID_FAILURE.
 */
static s5_avoid_result_t
fail (subsystem5_ctrl_fault_t fault)
{
    s5_links_motion_stop();
    s5_links_line_search(S5_LINKS_SIDE_NONE);
    subsystem5_obst_request(OB_HOLD);
    g_avoid.fault = fault;
    enter(S5_AVOID_FAILED);

    return (S5_AVOID_FAILURE);
}

/*!
 * @brief Change step and log it (FR16).
 *
 * @param[in] state The new step.
 */
static void
enter (s5_avoid_state_t state)
{
    g_avoid.state = state;

    // NOTE: The cast is safe: the state is a small enum value.
    s5_event_post_args(S5_EVENT_AVOID_STEP, (uint32_t) state, 0u);
}

/*** end of file ***/
