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
 * @par Finding the line again
 * On the way back S3 searches from the side the robot went round on (a
 * bypass on the left leaves the robot left of the line), and the return
 * leg may run SEARCH_EXTRA_MM past the plan. Still not found: the
 * mission's line recovery sweeps on the spot (S5_AVOID_SEARCH) instead
 * of driving further out, so a line that curves away near the end of
 * the track does not draw the robot off the map. While S3
 * is not ready (s5_links_line_is_ready() false: not calibrated, or
 * switched off by the floor tests) the return leg is driven its planned
 * length and the line is taken as found there.
 *
 * @par Checking the gap at the rear wheels (step 3w)
 * The robot turns about the middle of its rear axle, and the diagonal
 * ends with that axle about level with the obstacle's front face. The
 * rear wheel on the obstacle's side swings towards it while turning
 * parallel, and clipped a wide box's front corner (2026-11-05, Anna:
 * short turns and the scan's trimmed edges eat the 50 mm margin). So the
 * turn back is done in two halves. Between them the robot stands still
 * and pings sideways: the sensor sits about 10 mm ahead of the rear
 * axle, and half turned back the beam meets the obstacle's side nearly
 * square (at the full 45 degrees it would glance off). The sideways gap
 * there is the gap at the rear wheel. Under WHEEL_NEED_MM: turn out
 * again and drive out the missing distance (corner watch on), then try
 * again; at most WHEEL_CHECKS_MAX times, then NO_PATH. No echo: the
 * side is clear.
 *
 * @par Checking the gap beside the obstacle (step 3a)
 * Turns are not exact, and the edge from the scan is an estimate, so
 * after turning parallel the robot stands still while the side watch
 * measures the real gap to the obstacle's side. Closer than
 * SIDE_NEED_MM from the sensor (log 2026-11-04_s5_avoid_4: the widest
 * part of the robot grazed with 86 mm beside the sensor), it eases out
 * by exactly the missing distance: a JOG_DEG turn out, the short leg,
 * the turn back, then a new measurement with fresh pings only. At most
 * JOGS_MAX times, then NO_PATH rather than graze. No echo means the side
 * is not beside the sensor yet: it carries on. The return leg grows by
 * every extra move, so the robot still comes back to the line.
 * While passing, a side ping under SIDE_NEED_MM stops the robot and runs
 * this check again (log 2026-11-05_0136_track: not quite parallel, the
 * side closed in from 169 to 143 mm and the robot rubbed a wide box).
 *
 * @par Meeting the line early
 * The line may bend towards the robot, or the robot drift back, while it
 * passes the obstacle. If S3's sensors see black for LINE_MET_TICKS steps
 * in a row on the tail, the line is already under the robot: the bypass
 * ends there and line following resumes (log 2026-11-05_0154_track: both
 * sensors black, then the turn in drove away from the line and the
 * search found nothing). Not while passing: the obstacle may still be
 * beside the robot, and following the line there steered into a box
 * (log 2026-11-05_0224_track).
 * TODO: Set SIDE_NEED_MM from the measured robot (sensor line to the
 * widest point, plus about 25 mm).
 *
 * NOTE: When the side watch reports the obstacle passed, the long MOVE
 * alongside is cut short with STOP. If its EV_MOTION_DONE arrives in the
 * same instant, it is taken together with the pass, so the following
 * tail MOVE is not mistaken as finished.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_servo.h"      /* Side-watch angles only.                  */
#include "s5_avoid.h"
#include "s5_event.h"
#include "s5_links.h"
#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"
#include "subsystem5.h"

#define SPEED_MMS           (70u)       /* Moves and turns going round:  */
                                        /* slow for accuracy and for S3  */
                                        /* to see the line (was 150).    */
#define STEER_DEG           (15)        /* Corner close: turn out more.  */
#define WHEEL_NEED_MM       ((S5_ROBOT_WIDTH_MM / 2) + 30)  /* Sensor to */
                                        /* the obstacle's side, at the   */
                                        /* rear wheels: half the robot   */
                                        /* plus the wheel's swing (15 mm)*/
                                        /* and 15 mm to spare.           */
#define WHEEL_CHECKS_MAX    (2u)
#define OUT_MAX_DEG         (90)        /* Never further than sideways.  */
#define REVERSE_MM          (100)       /* Week 6 report 6.4.            */
#define RETRIES_MAX         (2u)        /* Then NO_PATH.                 */
#define WATCH_MAX_MM        (800)       /* Longest obstacle expected.    */
#define SEARCH_EXTRA_MM     (100)       /* Past where the line should be.*/
                                        /* Then search on the spot: an   */
                                        /* end-of-track curve must not   */
                                        /* draw the robot off the map.   */
#define SCAN_TIMEOUT_MS     (8000u)     /* A scan takes about 2.5 s.     */
#define MOTION_SLACK_MS     (2000u)     /* Added to every motion limit.  */
#define TURN_MS_PER_DEG     (40u)       /* About 2 x the time at 70 mm/s.*/
#define MS_PER_S            (1000u)
#define MM_PER_CM           (10u)
#define SIDE_NEED_MM        (110)       /* Sensor to the obstacle's side.*/
#define SIDE_CHECK_MS       (900u)      /* Servo 400 ms, then ~8 pings.  */
#define SIDE_NONE_MM        (0xFFFFu)   /* No echo beside yet.           */
#define JOG_DEG             (20)        /* Gentle: little swing.         */
#define JOGS_MAX            (3u)
#define LINE_MET_TICKS      (3u)        /* Black this often in a row.    */
#define LEG_MAX_MM          (32767)     /* Back leg is uint16_t.         */

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
    uint32_t                jogs;           /* Extra moves out so far.   */
    int32_t                 jog_mm;         /* Current extra move.       */
    uint32_t                side_min_mm;    /* Closest beside, this check*/
    uint32_t                check_pings;    /* Ping count at its start.  */
    int32_t                 out_deg;        /* Turned out so far (> 0).  */
    int32_t                 shift_mm;       /* Sideways move planned.    */
    int32_t                 out_done_mm;    /* Sideways move done.       */
    int32_t                 move_mark_mm;   /* Odometry at MOVE start.   */
    int32_t                 half_deg;       /* First half of the turn    */
                                            /* back to parallel.         */
    uint32_t                wheel_checks;   /* Rear-wheel moves out.     */
    uint32_t                black_run;      /* Steps in a row on black.  */
} avoid_t;

static avoid_t       g_avoid       = {S5_AVOID_IDLE, {S5_PLAN_CONTINUE, 0, 0u,
                                     0u, 0u, 0u, 0u}, 0u, 0u, 0u, 0u,
                                     S5_CTRL_FAULT_NONE, 0u, 0,
                                     SIDE_NONE_MM, 0u, 0, 0, 0, 0, 0,
                                     0u, 0u};
static volatile bool g_b_bypass_on = true;

static s5_avoid_result_t step_scanning(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t act_on_plan(uint32_t now_ms);
static s5_avoid_result_t step_bypass(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t step_return(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t step_side_check(uint32_t now_ms);
static void              start_side_check(uint32_t now_ms);
static void              request_watch(void);
static bool              side_too_close(void);
static bool              line_met(void);
static void              start_out(uint32_t now_ms);
static void              move_out(uint32_t now_ms);
static s5_avoid_result_t corner_close(uint32_t now_ms);
static void              half_parallel(uint32_t now_ms);
static void              start_side_ping(s5_avoid_state_t state,
                                         uint32_t now_ms);
static void              collect_side_ping(void);
static s5_avoid_result_t step_wheel_check(uint32_t now_ms);
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
    g_avoid.jogs      = 0u;
    g_avoid.black_run = 0u;
    g_avoid.fault     = S5_CTRL_FAULT_NONE;
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

        case S5_AVOID_SIDE_CHECK:
            result = step_side_check(now_ms);
        break;

        case S5_AVOID_WHEEL_CHECK:
            result = step_wheel_check(now_ms);
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
    s5_event_trace("PLAN reverse_first_mm clearance_mm",
                   (int32_t) p_plan->room_mm,
                   (int32_t) p_plan->clearance_mm);

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
        // Too close to turn out without touching it: reverse first.
        // NOTE: The cast widens uint16_t, so no value is lost.
        s5_links_motion_release();
        start_move(-(int32_t) p_plan->room_mm, now_ms);
        enter(S5_AVOID_MAKE_ROOM);
    }
    else
    {
        s5_links_motion_release();
        start_out(now_ms);
    }

    return (result);
}

/*!
 * @brief Step 1: turn out, and note the sideways move the plan needs
 *        (its 45 degree leg covers it).
 *
 * @param[in] now_ms Current time.
 */
static void
start_out (uint32_t now_ms)
{
    int32_t turn = g_avoid.plan.turn_deg;

    // NOTE: The cast widens uint16_t, so no value is lost.
    g_avoid.out_deg     = s5_math_abs(turn);
    g_avoid.shift_mm    = s5_math_mul_q15((int32_t) g_avoid.plan.out_mm,
                                          s5_math_sin_q15(g_avoid.out_deg));
    g_avoid.out_done_mm  = 0;
    g_avoid.wheel_checks = 0u;
    start_turn(turn, now_ms);
    enter(S5_AVOID_TURN_OUT);
}

/*!
 * @brief Step 2: drive out the rest of the sideways move at the current
 *        angle, with the corner watch on (the obstacle's side).
 *
 * @param[in] now_ms Current time.
 */
static void
move_out (uint32_t now_ms)
{
    int32_t remaining = g_avoid.shift_mm - g_avoid.out_done_mm;

    if (remaining <= 0)
    {
        // Already far enough out (after a steer-out).
        half_parallel(now_ms);
    }
    else
    {
        int32_t leg = (remaining * S5_MATH_Q15_ONE)
                      / s5_math_sin_q15(g_avoid.out_deg);

        // Passing on the left leaves the obstacle on the right.
        subsystem5_obst_request((g_avoid.plan.turn_deg > 0)
                                ? OB_CORNER_RIGHT : OB_CORNER_LEFT);
        g_avoid.move_mark_mm = s5_links_odom_dist_mm();
        start_move(leg, now_ms);
        enter(S5_AVOID_MOVE_OUT);
    }
}

/*!
 * @brief Step 3, first half: corner watch off, turn back half of
 *        everything turned out; then the rear-wheel check.
 *
 * @param[in] now_ms Current time.
 */
static void
half_parallel (uint32_t now_ms)
{
    int32_t sign = (g_avoid.plan.turn_deg > 0) ? 1 : -1;

    subsystem5_obst_request(OB_HOLD);
    g_avoid.half_deg = g_avoid.out_deg / 2;
    start_turn(-sign * g_avoid.half_deg, now_ms);
    enter(S5_AVOID_HALF_PARALLEL);
}

/*!
 * @brief The corner watch saw the obstacle close to the front corner
 *        while driving out (a short turn, or a wider obstacle than
 *        scanned): stop, turn STEER_DEG further out, drive on.
 *
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or FAILURE if already moving sideways.
 */
static s5_avoid_result_t
corner_close (uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    int32_t           driven = s5_links_odom_dist_mm()
                               - g_avoid.move_mark_mm;
    int32_t           sign   = (g_avoid.plan.turn_deg > 0) ? 1 : -1;

    s5_links_motion_settle();
    g_avoid.out_done_mm += s5_math_mul_q15(driven,
                                           s5_math_sin_q15(g_avoid.out_deg));

    if ((g_avoid.out_deg + STEER_DEG) <= OUT_MAX_DEG)
    {
        s5_event_trace("AVOID corner close: turning out to deg, done_mm",
                       g_avoid.out_deg + STEER_DEG, g_avoid.out_done_mm);
        g_avoid.out_deg += STEER_DEG;
        start_turn(sign * STEER_DEG, now_ms);
        enter(S5_AVOID_STEER_OUT);
    }
    else
    {
        // Already moving sideways and still too close: no way past.
        result = fail(S5_CTRL_FAULT_NO_PATH);
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

    if ((S5_AVOID_MOVE_OUT == g_avoid.state)
        && (0u != (events & EV_CORNER_CLOSE)))
    {
        result = corner_close(now_ms);
    }
    else if ((S5_AVOID_PASSING == g_avoid.state) && (b_pass || b_done))
    {
        // Passed (or drove the longest expected obstacle): clear the rear.
        s5_links_motion_stop();
        start_move(g_avoid.plan.tail_mm, now_ms);
        enter(S5_AVOID_TAIL);
    }
    else if ((S5_AVOID_PASSING == g_avoid.state) && side_too_close())
    {
        // Drifting into the obstacle's side: stop and measure again; the
        // side check eases out, then passing starts over.
        s5_event_trace("PASS side too close: range_mm need_mm",
                       (int32_t) g_avoid.side_min_mm, SIDE_NEED_MM);
        s5_links_motion_stop();
        start_side_check(now_ms);
    }
    else if ((S5_AVOID_TAIL == g_avoid.state) && line_met())
    {
        // The line is already under the robot: no turn in. The search
        // side is the one the return leg would use.
        s5_event_trace("AVOID line met on the tail: resuming 0", 0, 0);
        s5_links_motion_stop();
        s5_links_line_search((turn > 0) ? S5_LINKS_SIDE_LEFT
                                        : S5_LINKS_SIDE_RIGHT);
        enter(S5_AVOID_DONE);
        result = S5_AVOID_RESUME;
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

            case S5_AVOID_MAKE_ROOM:
                start_out(now_ms);
            break;

            case S5_AVOID_TURN_OUT:
            // After a steer-out the rest of the move is driven too.
            case S5_AVOID_STEER_OUT:
                move_out(now_ms);
            break;

            case S5_AVOID_MOVE_OUT:
                // Count the sideways distance this leg covered.
                g_avoid.out_done_mm +=
                    s5_math_mul_q15(s5_links_odom_dist_mm()
                                    - g_avoid.move_mark_mm,
                                    s5_math_sin_q15(g_avoid.out_deg));
                half_parallel(now_ms);
            break;

            case S5_AVOID_HALF_PARALLEL:
                start_side_ping(S5_AVOID_WHEEL_CHECK, now_ms);
            break;

            case S5_AVOID_WHEEL_OUT:
                move_out(now_ms);
            break;

            case S5_AVOID_TURN_PARALLEL:
            // The jog ends parallel too: measure the gap again.
            case S5_AVOID_JOG_BACK:
                start_side_check(now_ms);
            break;

            case S5_AVOID_JOG_OUT:
                start_move(g_avoid.jog_mm, now_ms);
                enter(S5_AVOID_JOG_MOVE);
            break;

            case S5_AVOID_JOG_MOVE:
                start_turn((turn > 0) ? -JOG_DEG : JOG_DEG, now_ms);
                enter(S5_AVOID_JOG_BACK);
            break;

            case S5_AVOID_TAIL:
                start_turn(-turn, now_ms);
                enter(S5_AVOID_TURN_IN);
            break;

            case S5_AVOID_TURN_IN:
                s5_links_line_search((turn > 0) ? S5_LINKS_SIDE_LEFT
                                                : S5_LINKS_SIDE_RIGHT);
                start_move(g_avoid.plan.back_mm
                           + (s5_links_line_is_ready() ? SEARCH_EXTRA_MM
                                                       : 0),
                           now_ms);
                enter(S5_AVOID_RETURN);
            break;

            case S5_AVOID_ONTO_LINE:
                start_turn(g_avoid.plan.turn_deg, now_ms);
                enter(S5_AVOID_REALIGN);
            break;

            case S5_AVOID_REALIGN:
                // The search ends in resume_following(), after LF_ENABLE
                // is set, so S3 is never DISABLED in between.
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

    if (b_found)
    {
        // S3's sensors are on the line, ahead of the turning centre.
        // Turning now would swing them off it again (log
        // 2026-11-04_1719_track: "OFF LINE -> searching" after the turn,
        // and S3 steered hard to find it). Drive the turning centre onto
        // the line first; the turn then swings the sensors along it.
        s5_links_motion_stop();
        start_move(S5_AXLE_TO_LINE_IR_MM, now_ms);
        enter(S5_AVOID_ONTO_LINE);
    }
    else if (b_done && (!s5_links_line_is_ready()))
    {
        // Without S3, where the line should be.
        start_turn(g_avoid.plan.turn_deg, now_ms);
        enter(S5_AVOID_REALIGN);
    }
    else if (b_done)
    {
        // Not found where it should be (it may curve away): no further
        // out; the mission's line recovery sweeps on the spot.
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
 * @brief Point the sensor at the obstacle's side and start measuring
 *        the gap, with the robot standing still (step 3a).
 *
 * @param[in] now_ms Current time.
 */
static void
start_side_check (uint32_t now_ms)
{
    start_side_ping(S5_AVOID_SIDE_CHECK, now_ms);
}

/*!
 * @brief Stand still and ping sideways at the obstacle (side check and
 *        rear-wheel check).
 *
 * @param[in] state  The checking state to enter.
 * @param[in] now_ms Current time.
 */
static void
start_side_ping (s5_avoid_state_t state, uint32_t now_ms)
{
    subsystem5_obst_status_t status = {0};

    // Remember the ping count: the published status still holds the
    // last reading before this check (log 2026-11-04_s5_avoid_6: an old
    // 97 mm caused a second, needless move out).
    subsystem5_obst_status_get(&status);
    g_avoid.check_pings   = status.ping_count;

    request_watch();
    g_avoid.side_min_mm   = SIDE_NONE_MM;
    g_avoid.wait_start_ms = now_ms;
    enter(state);
}

/*!
 * @brief Keep the closest new sideways ping: only pings taken after the
 *        check began, with the servo pointing sideways, count.
 */
static void
collect_side_ping (void)
{
    subsystem5_obst_status_t status   = {0};
    int32_t                  side_deg = (g_avoid.plan.turn_deg > 0)
                                        ? DRV_SERVO_RIGHT_DEG
                                        : DRV_SERVO_LEFT_DEG;

    subsystem5_obst_status_get(&status);

    if (status.b_range_valid && (side_deg == status.servo_deg)
        && (status.ping_count != g_avoid.check_pings)
        && (status.range_mm < g_avoid.side_min_mm))
    {
        g_avoid.side_min_mm = status.range_mm;
    }
}

/*!
 * @brief WHEEL_CHECK: the gap at the rear wheels, half turned back.
 *        Enough (or no echo): finish the turn. Too little: turn out
 *        again and move out the missing distance.
 *
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or FAILURE after WHEEL_CHECKS_MAX moves out.
 */
static s5_avoid_result_t
step_wheel_check (uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    int32_t           sign   = (g_avoid.plan.turn_deg > 0) ? 1 : -1;
    int32_t           rest   = g_avoid.out_deg - g_avoid.half_deg;
    int32_t           gap    = SIDE_NONE_MM;

    collect_side_ping();

    // The beam is rest degrees off square to the obstacle's side.
    // NOTE: The cast is safe: side_min_mm is at most SIDE_NONE_MM.
    if (SIDE_NONE_MM != g_avoid.side_min_mm)
    {
        gap = s5_math_mul_q15((int32_t) g_avoid.side_min_mm,
                              s5_math_cos_q15(rest));
    }

    if ((now_ms - g_avoid.wait_start_ms) < SIDE_CHECK_MS)
    {
        // Still measuring.
    }
    else if (gap >= WHEEL_NEED_MM)
    {
        s5_event_trace("WHEEL gap_mm ok, need_mm", gap, WHEEL_NEED_MM);
        subsystem5_obst_request(OB_HOLD);
        start_turn(-sign * rest, now_ms);
        enter(S5_AVOID_TURN_PARALLEL);
    }
    else if (g_avoid.wheel_checks >= WHEEL_CHECKS_MAX)
    {
        result = fail(S5_CTRL_FAULT_NO_PATH);
    }
    else
    {
        // NOTE: The cast is safe: the back leg is limited to LEG_MAX_MM.
        int32_t missing = WHEEL_NEED_MM - gap;
        int32_t back    = (int32_t) g_avoid.plan.back_mm
                          + ((missing * S5_MATH_Q15_ONE)
                             / s5_math_sin_q15(S5_PLAN_TURN_DEG));

        s5_event_trace("WHEEL gap_mm too close, moving out mm", gap,
                       missing);
        g_avoid.wheel_checks++;
        g_avoid.shift_mm    += missing;
        g_avoid.plan.back_mm = (uint16_t) ((back > LEG_MAX_MM) ? LEG_MAX_MM
                                                               : back);
        subsystem5_obst_request(OB_HOLD);
        start_turn(sign * g_avoid.half_deg, now_ms);
        enter(S5_AVOID_WHEEL_OUT);
    }

    return (result);
}

/*!
 * @brief SIDE_CHECK: keep the closest sideways range for SIDE_CHECK_MS,
 *        then drive past, or move further out first.
 *
 * @param[in] now_ms Current time.
 *
 * @return BUSY, or FAILURE if still too close after JOGS_MAX moves.
 */
static s5_avoid_result_t
step_side_check (uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;
    int32_t           turn   = g_avoid.plan.turn_deg;

    collect_side_ping();

    if ((now_ms - g_avoid.wait_start_ms) < SIDE_CHECK_MS)
    {
        // Still measuring.
    }
    else if (g_avoid.side_min_mm >= (uint32_t) SIDE_NEED_MM)
    {
        // Wide enough, or nothing beside yet: drive past, watching.
        // NOTE: The cast is safe: SIDE_NEED_MM is a few hundred mm.
        s5_event_trace("SIDE gap_mm ok, need_mm",
                       (int32_t) g_avoid.side_min_mm, SIDE_NEED_MM);

        // Restart the watch, so a pass seen while standing still is
        // not lost and the depth is measured from here.
        request_watch();
        start_move(WATCH_MAX_MM, now_ms);
        enter(S5_AVOID_PASSING);
    }
    else if (g_avoid.jogs >= JOGS_MAX)
    {
        result = fail(S5_CTRL_FAULT_NO_PATH);
    }
    else
    {
        // NOTE: The casts are safe: the gap is below SIDE_NEED_MM, so
        // the missing distance is 1 ... SIDE_NEED_MM, and the back leg
        // is limited to 0 ... LEG_MAX_MM.
        int32_t missing = SIDE_NEED_MM - (int32_t) g_avoid.side_min_mm;
        int32_t jog_deg = (turn > 0) ? JOG_DEG : -JOG_DEG;
        int32_t back    = (int32_t) g_avoid.plan.back_mm
                          + ((missing * S5_MATH_Q15_ONE)
                             / s5_math_sin_q15(s5_math_abs(turn)));

        s5_event_trace("SIDE gap_mm too close, moving out mm",
                       (int32_t) g_avoid.side_min_mm, missing);
        g_avoid.jogs++;
        g_avoid.jog_mm       = (missing * S5_MATH_Q15_ONE)
                               / s5_math_sin_q15(JOG_DEG);
        g_avoid.plan.back_mm = (uint16_t) ((back > LEG_MAX_MM) ? LEG_MAX_MM
                                                               : back);
        subsystem5_obst_request(OB_HOLD);
        start_turn(jog_deg, now_ms);
        enter(S5_AVOID_JOG_OUT);
    }

    return (result);
}

/*!
 * @brief While passing: is the latest sideways ping closer than
 *        SIDE_NEED_MM? Keeps that range in side_min_mm for the trace.
 *
 * @return True if the obstacle's side is too close.
 */
static bool
side_too_close (void)
{
    subsystem5_obst_status_t status   = {0};
    bool                     b_close  = false;
    int32_t                  side_deg = (g_avoid.plan.turn_deg > 0)
                                        ? DRV_SERVO_RIGHT_DEG
                                        : DRV_SERVO_LEFT_DEG;

    subsystem5_obst_status_get(&status);

    // NOTE: The cast is safe: SIDE_NEED_MM is a few hundred mm.
    if (status.b_range_valid && (side_deg == status.servo_deg)
        && (status.range_mm < (uint16_t) SIDE_NEED_MM))
    {
        g_avoid.side_min_mm = status.range_mm;
        b_close             = true;
    }

    return (b_close);
}

/*!
 * @brief On the tail: have S3's sensors seen black for LINE_MET_TICKS
 *        steps in a row? One dark speck is not the line.
 *
 * @return True once the line is under the robot.
 */
static bool
line_met (void)
{
    if (s5_links_line_sees_black())
    {
        g_avoid.black_run++;
    }
    else
    {
        g_avoid.black_run = 0u;
    }

    return (g_avoid.black_run >= LINE_MET_TICKS);
}

/*!
 * @brief Start the side watch on the obstacle's side: a bypass on the
 *        left leaves the obstacle on the right.
 */
static void
request_watch (void)
{
    subsystem5_obst_request((g_avoid.plan.turn_deg > 0) ? OB_WATCH_RIGHT
                                                        : OB_WATCH_LEFT);
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
