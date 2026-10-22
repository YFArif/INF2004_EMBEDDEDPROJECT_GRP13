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
 * @par Without S3 (s5_links_line_is_real() false)
 * The return leg is driven its planned length and the line is taken as
 * found there, so the whole manoeuvre can be tested before line_task is
 * merged.
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
#include "s5_plan.h"
#include "s5_profile.h"
#include "subsystem5.h"

#define SPEED_MMS           (150u)      /* All bypass legs, for accuracy.*/
#define REVERSE_MM          (100)       /* Week 6 report 6.4.            */
#define RETRIES_MAX         (2u)        /* Then NO_PATH.                 */
#define WATCH_MAX_MM        (800)       /* Longest obstacle expected.    */
#define SEARCH_EXTRA_MM     (300)       /* NFR10: line within 300 mm.    */
#define SCAN_TIMEOUT_MS     (8000u)     /* A scan takes about 2.5 s.     */
#define MOTION_SLACK_MS     (2000u)     /* Added to every motion limit.  */
#define TURN_MS_PER_DEG     (20u)       /* About 3 x the time at 150 mm/s*/
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
} avoid_t;

static avoid_t       g_avoid       = {S5_AVOID_IDLE, {S5_PLAN_CONTINUE, 0, 0u,
                                     0u, 0u, 0u}, 0u, 0u, 0u, 0u,
                                     S5_CTRL_FAULT_NONE};
static volatile bool g_b_bypass_on = true;

static s5_avoid_result_t step_scanning(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t act_on_plan(uint32_t now_ms);
static s5_avoid_result_t step_bypass(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t step_return(uint32_t events, uint32_t now_ms);
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
    g_avoid.retries = 0u;
    g_avoid.fault   = S5_CTRL_FAULT_NONE;
    request_scan(now_ms);
}

/*!
 * @brief Advance the avoidance with a batch of mission events.
 *
 * @param[in] events EV_xxx bits taken from flg_mission (may be 0).
 * @param[in] now_ms Current time.
 *
 * @return BUSY until finished; then RESUME, PLAN_ONLY or FAILURE.
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
        s5_plan_make(&profile, &g_avoid.plan);
        result = act_on_plan(now_ms);
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
    int32_t           turn   = g_avoid.plan.turn_deg;

    if ((S5_AVOID_PASSING == g_avoid.state) && (b_pass || b_done))
    {
        // Passed (or drove the longest expected obstacle): clear the rear.
        s5_links_motion_stop();
        start_move(g_avoid.plan.tail_mm, now_ms);
        enter(S5_AVOID_TAIL);
    }
    else if (!b_done)
    {
        result = is_timed_out(now_ms) ? fail(S5_CTRL_FAULT_MOTION_TIMEOUT)
                                      : S5_AVOID_BUSY;
    }
    else
    {
        switch (g_avoid.state)
        {
            case S5_AVOID_REVERSING:
                request_scan(now_ms);
            break;

            case S5_AVOID_TURN_OUT:
                start_move(g_avoid.plan.out_mm, now_ms);
                enter(S5_AVOID_MOVE_OUT);
            break;

            case S5_AVOID_MOVE_OUT:
                start_turn(-turn, now_ms);
                enter(S5_AVOID_TURN_PARALLEL);
            break;

            case S5_AVOID_TURN_PARALLEL:
                // Bypassing on the left leaves the obstacle on the right.
                subsystem5_obst_request((turn > 0) ? OB_WATCH_RIGHT
                                                   : OB_WATCH_LEFT);
                start_move(WATCH_MAX_MM, now_ms);
                enter(S5_AVOID_PASSING);
            break;

            case S5_AVOID_TAIL:
                start_turn(-turn, now_ms);
                enter(S5_AVOID_TURN_IN);
            break;

            case S5_AVOID_TURN_IN:
                s5_links_line_search(true);
                start_move(g_avoid.plan.back_mm
                           + (s5_links_line_is_real() ? SEARCH_EXTRA_MM : 0),
                           now_ms);
                enter(S5_AVOID_RETURN);
            break;

            case S5_AVOID_REALIGN:
                s5_links_line_search(false);
                enter(S5_AVOID_DONE);
                result = S5_AVOID_RESUME;
            break;

            default:
                // Not a waiting state; nothing to do.
            break;
        }
    }

    return (result);
}

/*!
 * @brief RETURN: drive back toward the line until it is found, then turn
 *        to run along it.
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or FAILURE if the line is not found.
 */
static s5_avoid_result_t
step_return (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t result  = S5_AVOID_BUSY;
    bool              b_found = (0u != (events & EV_LINE_FOUND));
    bool              b_done  = (0u != (events & EV_MOTION_DONE));

    if (b_found || (b_done && (!s5_links_line_is_real())))
    {
        // On the line (or, without S3, where the line should be).
        s5_links_motion_stop();
        start_turn(g_avoid.plan.turn_deg, now_ms);
        enter(S5_AVOID_REALIGN);
    }
    else if (b_done)
    {
        result = fail(S5_CTRL_FAULT_LINE_NOT_FOUND);
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
    s5_links_motion_turn(angle_deg);
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
    s5_links_line_search(false);
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
