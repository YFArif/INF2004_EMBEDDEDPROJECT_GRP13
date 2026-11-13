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
 * S5_PATH_MARGIN_MM, and the legs from the robot's measured size.
 * Obstacles are never placed back to back (Anna, 2026-11-06), so nothing
 * else is watched on the way round.
 *
 * @par Looking at the side while passing
 * The servo turns at most 40 degrees, and a flat side that slants more
 * than about 20 degrees from square sends no echo back: with the robot
 * parallel, the side watch never heard a box's side (2026-11-12 logs,
 * 2026-11-13_1417: 77 pings, none answered), lost nothing, and drove on
 * by distance. So the robot looks (Anna's choice, 2026-11-13): it turns
 * S5_LOOK_DEG towards the obstacle on the spot, with the servo at its
 * limit the beam meets the side 75 degrees off the track (15 from
 * square), takes LOOK_PINGS pings and turns back to the track's heading.
 * An echo across less than the side's expected place plus
 * LOOK_NEAR_SLACK_MM: the side is still there, drive LOOK_STEP_MM and
 * look again. Silence: its end is behind the beam, which meets the side
 * about level with the sensor. One silent look is not trusted (log
 * 2026-11-13_1607: silent at once, the box was still there and the
 * rear hit it later), so the robot steps on and looks again; after the
 * second silent look in a row it does not turn back but drives straight
 * on from there, still turned S5_LOOK_DEG, to the line (Anna,
 * 2026-11-13: no tail and no 60 degree turn in). Its side then crosses
 * the obstacle's side line about 11 cm ahead of where the axle was at
 * the look, and S3's barcode sensor's tip about 6 cm ahead; after two
 * silent looks 10 cm apart the obstacle's end is behind the axle. Pings
 * during the look's turn count too: one showing the side
 * nearer than the swing reaches (s5_profile_look_reach_mm() plus
 * LOOK_STOP_SPARE_MM) stops the turn at once, counts as an echo, and
 * the robot turns back (log 2026-11-13_1549: it heard the side at 9 cm
 * and kept turning until it stalled against it). The plan passes far
 * enough out for the look's swing
 * (s5_profile_pass_mm()), and drives look_mm first so the first look
 * already falls on the side, not in front of the face.
 *
 * @par Seeing past a corner
 * The servo turns 40 degrees either way. Stopped close, or with the
 * obstacle off to one side on a bend, the scan can still hear it at the
 * limit, so that edge is "not found" and the side unusable, however free
 * it is (log 2026-11-13_1457: the robot went right, where it had heard
 * nothing, and met a foot it could not see). So if exactly one edge is
 * missing, the robot turns PROBE_DEG that way on the spot and scans
 * again; obstacle_task works that profile out in the heading the robot
 * stopped at (subsystem5_obst_scan_turned()), and that side's edge,
 * clearance and shift replace the first scan's. If that scan hears
 * nothing in the way, the side is taken as free (Anna, 2026-11-13: the
 * turned scan came back silent twice, logs 1513 and 1518), with the
 * shift assumed from the first scan's last echo (S5_EDGE_UNSEEN_MM past
 * it). Either way, a side found usable like this is the one taken. Then
 * it turns back and plans. Once per obstacle; not closer than
 * PROBE_MIN_MM (the swing).
 *
 * @par Checking the way out
 * The scan sees about 52 degrees either side at most (servo limit plus
 * half the beam); a steep turn out points the robot where it never
 * looked. So after turning out, before the out leg, it pings straight
 * ahead (OB_PING_AHEAD). An echo closer than the out leg plus the
 * sensor's setback and CHECK_MARGIN_MM: the way is blocked. Often that
 * is the obstacle's own near corner, the obstacle sitting at an angle
 * (log 2026-11-13_1616_2: heard at 19 cm, 56 degrees out). So it first
 * turns OUT_STEP_DEG further out on the spot (that moves the front away
 * from anything ahead) and pings again, up to OUT_STEEP_MAX_DEG; the out
 * leg is worked out again for the same shift (shorter when steeper), and
 * the drive before the first look grows by the forward distance the out
 * leg no longer covers (Anna, 2026-11-13). Still blocked at the steepest:
 * it turns back, scans again, and that side stays out of the plan (until
 * a reverse). Something that sends no echo (low, soft) is still missed.
 *
 * @par No squaring up
 * Turning to face the obstacle squarely before the scan was removed
 * (Anna, 2026-11-13): it set every heading of the bypass 20 degrees off
 * the track (log 2026-11-13_1444) and made a first approach come out as
 * CONTINUE, so the robot stopped twice (log 2026-11-13_1417). The wider
 * pass for the look covers the edge read short from a crooked stop.
 *
 * @par Finding the line again
 * From the last look the robot drives straight on at S5_LOOK_DEG, S3
 * searching from the side it went round on; the line sensors meet the
 * line after (shift - S5_AXLE_TO_LINE_IR_MM sin(look)) / sin(look),
 * plus SEARCH_EXTRA_MM. S3 is handed the line only with the robot along
 * it and facing forward: it holds its steering straight while both its
 * sensors see black (a junction to it), so crossing at an angle it
 * drove over the line (logs 2026-11-13_1538/1539) and turned back
 * towards the obstacle, the way it last saw black (log 1554). So when S3
 * first sees the line more than ONTO_MIN_DEG across, the robot stops,
 * drives S5_AXLE_TO_LINE_IR_MM on (the rear axle, straight behind the
 * line sensors, is then on the line) and turns slowly towards the
 * track's heading (forward, away from the obstacle) with S3 searching.
 * With the axle on the line the sensors meet it again exactly when the
 * robot is along it, so the turn stops there and S3 takes over; on a
 * bend that is before the old heading (turning all the way to it
 * pointed the robot 50 degrees off the line, log 1534). Not seen by
 * ALIGN_EXTRA_DEG past the old heading, or not found on the way: the
 * mission searches (S5_AVOID_SEARCH). The mission also takes a junction
 * just after the hand-over as crossing the line, and then searches
 * forward first (s5_mission.c). While S3 is not ready (floor tests) the
 * way is driven its planned length and the robot turns back to the
 * track's heading.
 *
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
#define REVERSE_MMS         (40u)       /* Backwards: gentle, as S4 takes*/
                                        /* a jolt over 2.0 g as an impact*/
                                        /* (imu_task.c IMPACT_MG). At 70 */
                                        /* the reverse after the stop    */
                                        /* read 2.2 and 2.6 g (logs      */
                                        /* 2026-11-11_1727).             */
#define REVERSE_MM          (100)       /* Week 6 report 6.4.            */
#define RETRIES_MAX         (2u)        /* Then NO_PATH.                 */
#define LOOK_STEP_MM        (100)       /* Alongside between looks.      */
#define LOOKS_MAX           (8u)        /* 8 x 100: longest obstacle     */
                                        /* expected, then NO_PATH.       */
#define LOOK_PINGS          (5u)        /* Pings heard at each look.     */
#define LOOK_SKIP_PINGS     (1u)        /* The first may be from the turn*/
#define LOOK_MS             (1500u)     /* No pings this long: SENSOR.   */
#define LOOK_STOP_SPARE_MM  (10)        /* Side nearer than the look's   */
                                        /* reach plus this: stop turning.*/
#define LOOK_NEAR_SLACK_MM  (100)       /* Echo this far beyond where the*/
                                        /* side should be still counts.  */
#define SENSOR_AHEAD_MM     (S5_AXLE_TO_FRONT_MM - S5_SENSOR_SETBACK_MM)
#define BACK_MAX_FIX_DEG    (15)        /* Turn back by the odometry's   */
                                        /* heading unless it differs from*/
                                        /* the plan by more than this.   */
#define CDEG_PER_DEG        (100)
#define CDEG_HALF_TURN      (18000)
#define SEARCH_EXTRA_MM     (60)        /* Past where the line should be.*/
#define ONTO_MIN_DEG        (10)        /* Nearer along the line: hand   */
                                        /* over at once.                 */
#define ALIGN_EXTRA_DEG     (20)        /* Turn on past the old heading. */
#define ALIGN_MMS           (40u)       /* Slow: S3 must see the line    */
                                        /* pass under the sensors.       */
#define AHEAD_DEG           (90)        /* Servo angle straight ahead.   */
#define PROBE_DEG           (30)        /* Turn to see past a corner.    */
#define PROBE_MIN_MM        (120)       /* Closest echo: the face is then*/
                                        /* 205 mm from the axle, past the*/
                                        /* 175 mm swing plus 20.         */
#define PROBE_NONE          (0u)
#define PROBE_LEFT          (1u)
#define PROBE_RIGHT         (2u)
#define PROBE_DONE          (3u)
#define SIDE_LEFT           (1u)        /* Way out blocked, bit masks.   */
#define SIDE_RIGHT          (2u)
#define CHECK_MARGIN_MM     (50)        /* Kept ahead of the front.      */
#define OUT_STEP_DEG        (12)        /* Way out blocked: turn further */
                                        /* out by this (half the beam).  */
#define OUT_STEEP_MAX_DEG   (80)        /* ... up to this turn out.      */
#define SCAN_TIMEOUT_MS     (8000u)     /* A scan takes about 2.5 s.     */
#define MOTION_SLACK_MS     (2000u)     /* Added to every motion limit.  */
#define TURN_MS_PER_DEG     (40u)       /* About 2 x the time at 70 mm/s.*/
#define MS_PER_S            (1000u)
#define MM_PER_CM           (10u)

/** Everything the avoidance remembers between steps. The state, fault,
 *  retry count, profile sequence and flag share one word (minimum-bit
 *  requirement); times and odometry stay 32-bit, as they are counters
 *  that wrap.
 */
typedef struct
{
    s5_plan_t               plan;
    uint32_t                wait_start_ms;
    uint32_t                wait_limit_ms;
    int32_t                 start_cdeg;     /* Odometry heading when the */
                                            /* turn out began.           */
    s5_profile_t            profile;        /* What the plan is made of. */
    unsigned int            state       : 5;    /* s5_avoid_state_t.     */
    unsigned int            fault       : 4;    /* ..._ctrl_fault_t.     */
    unsigned int            profile_seq : 5;    /* Last profile planned. */
    unsigned int            retries     : 2;    /* 0 ... RETRIES_MAX + 1.*/
    unsigned int            looks       : 4;    /* 0 ... LOOKS_MAX.      */
    unsigned int            b_look_near : 1;    /* This look echoed.     */
    unsigned int            look_from   : 10;   /* ping_count at its     */
                                                /* start.                */
    unsigned int            probe       : 2;    /* PROBE_xxx.            */
    unsigned int            blocked     : 2;    /* SIDE_xxx bits.        */
    unsigned int            prefer      : 2;    /* SIDE_xxx: looked past */
                                                /* the corner, free.     */
    unsigned int            b_silent    : 1;    /* Last look was silent. */
} avoid_t;

/* The enums and counts must fit their fields; a misfit stops the build
 * here (an array of size -1).
 */
typedef char avoid_state_check[((S5_AVOID_FAILED <= S5_AVOID_STATE_MASK)
                                && (S5_CTRL_FAULT_OVERTURNED
                                    <= S5_CTRL_FAULT_MASK)
                                && ((RETRIES_MAX + 1u) <= 3u)
                                && (LOOKS_MAX <= 15u)) ? 1 : -1];

static avoid_t       g_avoid       = {{S5_PLAN_CONTINUE, 0, 0, 0u, 0u, 0u,
                                       0u, 0u, 0u},
                                      0u, 0u, 0, {0}, S5_AVOID_IDLE,
                                      S5_CTRL_FAULT_NONE, 0u, 0u, 0u, false,
                                      0u, PROBE_NONE, 0u, 0u, false};
static volatile bool g_b_bypass_on = true;

static s5_avoid_result_t step_scanning(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t on_profile(s5_profile_t const * p_profile,
                                    uint32_t now_ms);
static bool              needs_probe(s5_profile_t const * p_profile);
static void              merge_probe(s5_profile_t const * p_probe);
static s5_avoid_result_t plan_and_act(uint32_t now_ms);
static s5_avoid_result_t act_on_plan(uint32_t now_ms);
static s5_avoid_result_t step_check(uint32_t now_ms);
static void              listen(uint32_t now_ms);
static void              turn_out_more(int32_t step_deg, uint32_t now_ms);
static uint32_t          fresh_pings(subsystem5_obst_status_t * p_status);
static s5_avoid_result_t step_bypass(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t next_leg(uint32_t now_ms);
static s5_avoid_result_t step_return(uint32_t events, uint32_t now_ms);
static s5_avoid_result_t line_found(uint32_t now_ms);
static s5_avoid_result_t hand_over(void);
static void              start_way_back(uint32_t now_ms);
static void              start_look(uint32_t now_ms);
static s5_avoid_result_t step_look_turn(uint32_t events, uint32_t now_ms);
static bool              is_too_close(subsystem5_obst_status_t const *
                                      p_status);
static s5_avoid_result_t step_look(uint32_t now_ms);
static bool              is_side_echo(subsystem5_obst_status_t const *
                                      p_status);
static int32_t           look_deg(void);
static int32_t           back_turn_deg(int32_t planned_deg);
static int32_t           heading_off_deg(void);
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
    g_avoid.retries    = 0u;
    g_avoid.probe      = PROBE_NONE;
    g_avoid.blocked    = 0u;
    g_avoid.prefer     = 0u;
    g_avoid.b_silent   = false;
    g_avoid.fault      = S5_CTRL_FAULT_NONE;
    g_avoid.start_cdeg = s5_links_odom_heading_cdeg();
    subsystem5_obst_scan_turned(0);
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

    switch ((s5_avoid_state_t) g_avoid.state)
    {
        case S5_AVOID_SCANNING:
            result = step_scanning(events, now_ms);
        break;

        case S5_AVOID_RETURN:
            result = step_return(events, now_ms);
        break;

        case S5_AVOID_LOOK_TURN:
            result = step_look_turn(events, now_ms);
        break;

        case S5_AVOID_LOOK_PINGS:
            result = step_look(now_ms);
        break;

        case S5_AVOID_CHECK_OUT:
            result = step_check(now_ms);
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
 * @brief After a bypass: is forward along the track (the heading the
 *        robot had before turning out, away from the obstacle) to the
 *        robot's left now? For the mission's search after losing the
 *        line, so it turns forward first.
 *
 * @return true if turning left heads forward.
 */
bool
s5_avoid_forward_is_left (void)
{
    return (heading_off_deg() > 0);
}

/*!
 * @brief Why the avoidance failed.
 *
 * @return The fault, or S5_CTRL_FAULT_NONE.
 */
subsystem5_ctrl_fault_t
s5_avoid_fault (void)
{
    // NOTE: The cast is safe: the field only ever holds a fault value.
    return ((subsystem5_ctrl_fault_t) g_avoid.fault);
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
        result = on_profile(&profile, now_ms);
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
            // From further back the ways out may be free again.
            g_avoid.blocked = 0u;
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
        g_avoid.start_cdeg = s5_links_odom_heading_cdeg();
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

    bool              b_line = (0u != (events & EV_LINE_FOUND))
                               || s5_links_line_is_locked();

    if ((S5_AVOID_ALIGN == g_avoid.state) && b_line)
    {
        // Along the line now, facing forward: S3 takes over.
        result = hand_over();
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

    switch ((s5_avoid_state_t) g_avoid.state)
    {
        case S5_AVOID_REVERSING:
        // Way out blocked and turned back: scan again from here; that
        // side stays out of the plan.
        case S5_AVOID_CHECK_BACK:
            request_scan(now_ms);
        break;

        case S5_AVOID_PROBE_TURN:
            // Scan from here; the profile comes in the heading the robot
            // stopped at (turned left by minus the turn back).
            subsystem5_obst_scan_turned(-heading_off_deg());
            request_scan(now_ms);
        break;

        case S5_AVOID_PROBE_BACK:
            result = plan_and_act(now_ms);
        break;

        case S5_AVOID_MAKE_ROOM:
            g_avoid.start_cdeg = s5_links_odom_heading_cdeg();
            start_turn(p_plan->turn_deg, now_ms);
            enter(S5_AVOID_TURN_OUT);
        break;

        case S5_AVOID_TURN_OUT:
            // Ping the way out first (see "Checking the way out").
            subsystem5_obst_request(OB_PING_AHEAD);
            listen(now_ms);
            enter(S5_AVOID_CHECK_OUT);
        break;

        case S5_AVOID_MOVE_OUT:
            // Back to the heading it had before turning out, so any drift
            // on the way out is turned away too.
            start_turn(back_turn_deg(-p_plan->turn_deg), now_ms);
            enter(S5_AVOID_TURN_PARALLEL);
        break;

        case S5_AVOID_TURN_PARALLEL:
            // Point the sensor at its limit towards the obstacle and keep
            // it pinging (the side watch; only the pings are used now).
            // Going round on the left leaves the obstacle on the right.
            subsystem5_obst_request((p_plan->turn_deg > 0) ? OB_WATCH_RIGHT
                                                           : OB_WATCH_LEFT);
            g_avoid.looks    = 0u;
            g_avoid.b_silent = false;

            if (0u != p_plan->look_mm)
            {
                // NOTE: The cast widens uint16_t, so no value is lost.
                start_move((int32_t) p_plan->look_mm, now_ms);
                enter(S5_AVOID_LOOK_STEP);
            }
            else
            {
                start_look(now_ms);
            }
        break;

        case S5_AVOID_LOOK_STEP:
            start_look(now_ms);
        break;

        case S5_AVOID_LOOK_TURN:
            listen(now_ms);
            enter(S5_AVOID_LOOK_PINGS);
        break;

        case S5_AVOID_LOOK_BACK:
            // Echo: still beside it. First silence: look once more.
            if (g_avoid.b_look_near && (g_avoid.looks >= LOOKS_MAX))
            {
                // Longer than any obstacle expected: something else is
                // beside the track. Turning in could hit it.
                result = fail(S5_CTRL_FAULT_NO_PATH);
            }
            else
            {
                start_move(LOOK_STEP_MM, now_ms);
                enter(S5_AVOID_LOOK_STEP);
            }
        break;

        case S5_AVOID_ONTO_LINE:
        {
            // The axle is on the line: turn slowly towards the track's
            // heading, S3 searching again (a new request, so the first
            // find does not count), until it sees the line.
            int32_t turn = heading_off_deg();

            turn += (turn > 0) ? ALIGN_EXTRA_DEG : -ALIGN_EXTRA_DEG;
            s5_links_line_search((p_plan->turn_deg > 0) ? S5_LINKS_SIDE_LEFT
                                                        : S5_LINKS_SIDE_RIGHT);
            s5_links_motion_turn(turn, ALIGN_MMS);
            g_avoid.wait_start_ms = now_ms;

            // NOTE: The cast is safe: |turn| is at most 200 degrees.
            g_avoid.wait_limit_ms = ((uint32_t) s5_math_abs(turn)
                                     * TURN_MS_PER_DEG * 2u)
                                    + MOTION_SLACK_MS;
            enter(S5_AVOID_ALIGN);
        }
        break;

        case S5_AVOID_ALIGN:
            // Turned past the old heading without seeing the line: the
            // mission searches from here.
            s5_event_trace("AVOID not along the line after turning: search"
                           " 0 0", 0, 0);
            enter(S5_AVOID_DONE);
            result = S5_AVOID_SEARCH;
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
        result = line_found(now_ms);
    }
    else if (b_done && (!s5_links_line_is_ready()))
    {
        // Without S3, where the line should be: along the track.
        start_turn(heading_off_deg(), now_ms);
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
 * @brief A turn back parallel to the track: from the odometry heading
 *        now back to the one the robot had when it turned out, so any
 *        drift since is turned away too.
 *
 * @param[in] planned_deg The turn if there were no drift, + = left.
 *
 * @return Signed angle, + = left; planned_deg if the odometry's answer
 *         differs from it by more than BACK_MAX_FIX_DEG.
 */
static int32_t
back_turn_deg (int32_t planned_deg)
{
    int32_t planned  = planned_deg;
    int32_t odom_deg = heading_off_deg();
    int32_t turn     = planned;

    if (s5_math_abs(odom_deg - planned) <= BACK_MAX_FIX_DEG)
    {
        turn = odom_deg;
    }

    s5_event_trace("AVOID back to the track heading: planned_deg turn_deg",
                   planned, turn);

    return (turn);
}

/*!
 * @brief The turn from the odometry heading now back to the one the robot
 *        had when it turned out (the track's).
 *
 * @return Whole degrees, + = left, -180 ... 180.
 */
static int32_t
heading_off_deg (void)
{
    int32_t diff = g_avoid.start_cdeg - s5_links_odom_heading_cdeg();

    // The shorter way round.
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

    // Round to whole degrees.
    return ((diff + ((diff >= 0) ? (CDEG_PER_DEG / 2)
                                 : -(CDEG_PER_DEG / 2)))
            / CDEG_PER_DEG);
}

/*!
 * @brief The second silent look in a row: drive straight on to the line
 *        from here, still turned S5_LOOK_DEG towards it, S3 searching
 *        (see "Finding the line again").
 *
 * @param[in] now_ms Current time.
 *
 * NOTE: The axle is the plan's shift, out_mm sin(turn), to the side of
 * the line; turned by the look, the line sensors are
 * S5_AXLE_TO_LINE_IR_MM sin(look) nearer, and every mm driven brings
 * them sin(look) nearer.
 */
static void
start_way_back (uint32_t now_ms)
{
    int32_t sin_l    = s5_math_sin_q15(S5_LOOK_DEG);

    // NOTE: The cast widens uint16_t, so no value is lost.
    int32_t shift_mm = s5_math_mul_q15((int32_t) g_avoid.plan.out_mm,
                                       s5_math_sin_q15(
                                           s5_math_abs(g_avoid.plan.turn_deg)));
    int32_t lat_mm   = shift_mm
                       - s5_math_mul_q15(S5_AXLE_TO_LINE_IR_MM, sin_l);
    int32_t leg_mm   = 0;

    // NOTE: lat_mm is a few hundred mm, so x 32768 stays below 2^31.
    if (lat_mm > 0)
    {
        leg_mm = (lat_mm * S5_MATH_Q15_ONE) / sin_l;
    }

    if (s5_links_line_is_ready())
    {
        leg_mm += SEARCH_EXTRA_MM;
    }

    s5_event_trace("AVOID passed it: straight on to the line_mm shift_mm",
                   leg_mm, shift_mm);
    subsystem5_obst_request(OB_HOLD);
    s5_links_line_search((g_avoid.plan.turn_deg > 0) ? S5_LINKS_SIDE_LEFT
                                                     : S5_LINKS_SIDE_RIGHT);
    start_move(leg_mm, now_ms);
    enter(S5_AVOID_RETURN);
}

/*!
 * @brief Start a look: turn S5_LOOK_DEG towards the obstacle (see
 *        "Looking at the side while passing").
 *
 * @param[in] now_ms Current time.
 */
static void
start_look (uint32_t now_ms)
{
    subsystem5_obst_status_t status = {0};

    // Pings during the turn are watched from the next one on.
    subsystem5_obst_status_get(&status);
    g_avoid.look_from = status.ping_count & S5_OBST_COUNT_MASK;
    g_avoid.looks     = (g_avoid.looks + 1u) & 0xFu;
    start_turn(look_deg(), now_ms);
    enter(S5_AVOID_LOOK_TURN);
}

/*!
 * @brief LOOK_TURN: wait for the turn, but stop it as soon as a ping
 *        shows the side too close for the rest of the swing (see
 *        "Looking at the side while passing").
 *
 * @param[in] events Mission events.
 * @param[in] now_ms Current time.
 *
 * @return S5_AVOID_BUSY, or a failure.
 */
static s5_avoid_result_t
step_look_turn (uint32_t events, uint32_t now_ms)
{
    s5_avoid_result_t        result = S5_AVOID_BUSY;
    subsystem5_obst_status_t status = {0};
    uint32_t                 fresh  = fresh_pings(&status);

    if ((fresh > 0u) && is_too_close(&status))
    {
        // NOTE: The cast widens the 10-bit field, so no value is lost.
        s5_event_trace("AVOID look stopped, side too close: range_mm"
                       " turned_deg", (int32_t) status.range_mm,
                       s5_math_abs(heading_off_deg()));
        s5_links_motion_stop();
        g_avoid.b_look_near = true;
        g_avoid.b_silent    = false;
        start_turn(heading_off_deg(), now_ms);
        enter(S5_AVOID_LOOK_BACK);
    }
    else if (0u != (events & EV_MOTION_DONE))
    {
        result = next_leg(now_ms);
    }
    else if (is_timed_out(now_ms))
    {
        result = fail(S5_CTRL_FAULT_MOTION_TIMEOUT);
    }
    else
    {
        // Still turning.
    }

    return (result);
}

/*!
 * @brief Tell whether a ping during the look's turn shows the side
 *        nearer than the turn will swing the robot.
 *
 * @param[in] p_status The ping.
 *
 * @return true if the side is within s5_profile_look_reach_mm() plus
 *         LOOK_STOP_SPARE_MM of the centre line.
 *
 * NOTE: Turned t so far (odometry), the beam is t plus the servo's
 * angle off ahead from the track, and the sensor SENSOR_AHEAD_MM sin(t)
 * nearer the side; an echo r away is r sin(beam) further across.
 */
static bool
is_too_close (subsystem5_obst_status_t const * p_status)
{
    bool b_close = false;

    if (p_status->b_range_valid)
    {
        bool    b_box_left = (g_avoid.plan.turn_deg < 0);
        int32_t turned     = s5_math_abs(heading_off_deg());

        // NOTE: The casts widen the bit-fields, so no value is lost.
        int32_t servo_off  = s5_math_abs((int32_t) p_status->servo_deg
                                         - AHEAD_DEG);
        int32_t side_mm    = s5_math_mul_q15((int32_t) p_status->range_mm,
                                             s5_math_sin_q15(turned
                                                             + servo_off))
                             + s5_math_mul_q15(SENSOR_AHEAD_MM,
                                               s5_math_sin_q15(turned));

        b_close = (side_mm < (s5_profile_look_reach_mm(b_box_left)
                              + LOOK_STOP_SPARE_MM));
    }

    return (b_close);
}

/*!
 * @brief A new profile while SCANNING: probe past a corner, merge the
 *        probe, or plan (see "Seeing past a corner").
 *
 * @param[in] p_profile The new profile.
 * @param[in] now_ms    Current time.
 *
 * @return S5_AVOID_BUSY, or act_on_plan()'s result.
 */
static s5_avoid_result_t
on_profile (s5_profile_t const * p_profile, uint32_t now_ms)
{
    s5_avoid_result_t result = S5_AVOID_BUSY;

    if ((PROBE_LEFT == g_avoid.probe) || (PROBE_RIGHT == g_avoid.probe))
    {
        // The scan from the turned body: that side from it, then back.
        merge_probe(p_profile);
        g_avoid.probe = PROBE_DONE;
        subsystem5_obst_scan_turned(0);
        start_turn(heading_off_deg(), now_ms);
        enter(S5_AVOID_PROBE_BACK);
    }
    else if (needs_probe(p_profile))
    {
        int32_t turn_deg = p_profile->b_left_edge ? -PROBE_DEG : PROBE_DEG;

        g_avoid.profile = *p_profile;
        g_avoid.probe   = p_profile->b_left_edge ? PROBE_RIGHT : PROBE_LEFT;
        s5_event_trace("AVOID edge beyond the servo: turning to see turn_deg"
                       " 0", turn_deg, 0);
        s5_links_motion_release();
        start_turn(turn_deg, now_ms);
        enter(S5_AVOID_PROBE_TURN);
    }
    else
    {
        g_avoid.profile = *p_profile;
        result          = plan_and_act(now_ms);
    }

    return (result);
}

/*!
 * @brief Tell whether to turn and scan again to see past a corner.
 *
 * @param[in] p_profile The first scan's profile.
 *
 * @return true if exactly one edge is missing, that side has not been
 *         found blocked, there was no probe yet, and there is room to
 *         turn.
 */
static bool
needs_probe (s5_profile_t const * p_profile)
{
    uint32_t side     = p_profile->b_left_edge ? SIDE_RIGHT : SIDE_LEFT;
    bool     b_one_of = (p_profile->b_left_edge != p_profile->b_right_edge);

    // NOTE: The cast widens the 10-bit field, so no value is lost.
    return (g_b_bypass_on && p_profile->b_on_path && b_one_of
            && (PROBE_NONE == g_avoid.probe)
            && (0u == (g_avoid.blocked & side))
            && ((int32_t) p_profile->closest_mm >= PROBE_MIN_MM));
}

/*!
 * @brief Take the probed side's edge, clearance and shift from the probe
 *        scan into the first scan's profile.
 *
 * @param[in] p_probe The profile from the turned body (already in the
 *                    stopping heading).
 */
static void
merge_probe (s5_profile_t const * p_probe)
{
    bool b_left = (PROBE_LEFT == g_avoid.probe);
    bool b_edge = b_left ? p_probe->b_left_edge : p_probe->b_right_edge;

    // Nothing heard in the way: free, with the first scan's assumed
    // shift (see "Seeing past a corner").
    bool b_free = (!b_edge) && (!p_probe->b_on_path);

    if (b_left && b_edge)
    {
        g_avoid.profile.b_left_edge   = 1u;
        g_avoid.profile.clear_left_mm = p_probe->clear_left_mm;
        g_avoid.profile.shift_left_mm = p_probe->shift_left_mm;
    }
    else if (b_left && b_free)
    {
        g_avoid.profile.b_left_edge   = 1u;
        g_avoid.profile.clear_left_mm = S5_CLEAR_OPEN_MM;
    }
    else if (b_edge)
    {
        g_avoid.profile.b_right_edge   = 1u;
        g_avoid.profile.clear_right_mm = p_probe->clear_right_mm;
        g_avoid.profile.shift_right_mm = p_probe->shift_right_mm;
    }
    else if (b_free)
    {
        g_avoid.profile.b_right_edge   = 1u;
        g_avoid.profile.clear_right_mm = S5_CLEAR_OPEN_MM;
    }
    else
    {
        // The obstacle still reaches past the servo: that side stays out.
    }

    if (b_edge || b_free)
    {
        g_avoid.prefer = b_left ? SIDE_LEFT : SIDE_RIGHT;
    }

    // NOTE: The casts are safe: the flags are 0 or 1 and the shifts at
    // most 1023 mm.
    s5_event_trace("AVOID past the corner (1 left 2 right): side edge_found",
                   (int32_t) g_avoid.probe, b_edge ? 1 : 0);
    s5_event_trace("AVOID past the corner: free shift_mm", b_free ? 1 : 0,
                   (int32_t) (b_left ? g_avoid.profile.shift_left_mm
                                     : g_avoid.profile.shift_right_mm));
}

/*!
 * @brief Plan from g_avoid.profile, leaving out any side whose way out
 *        was found blocked, and act on it.
 *
 * @param[in] now_ms Current time.
 *
 * @return act_on_plan()'s result.
 */
static s5_avoid_result_t
plan_and_act (uint32_t now_ms)
{
    if (0u != (g_avoid.blocked & SIDE_LEFT))
    {
        g_avoid.profile.b_left_edge   = 0u;
        g_avoid.profile.clear_left_mm = 0u;
    }

    if (0u != (g_avoid.blocked & SIDE_RIGHT))
    {
        g_avoid.profile.b_right_edge   = 0u;
        g_avoid.profile.clear_right_mm = 0u;
    }

    int32_t prefer = S5_PLAN_PREFER_NONE;

    if (SIDE_LEFT == g_avoid.prefer)
    {
        prefer = S5_PLAN_PREFER_LEFT;
    }
    else if (SIDE_RIGHT == g_avoid.prefer)
    {
        prefer = S5_PLAN_PREFER_RIGHT;
    }
    else
    {
        // No side looked at: the planner's own rule.
    }

    s5_plan_make(&g_avoid.profile, prefer, &g_avoid.plan);

    return (act_on_plan(now_ms));
}

/*!
 * @brief CHECK_OUT: listen until LOOK_PINGS pings ahead have been made,
 *        then drive out, or turn back if the way is blocked (see
 *        "Checking the way out").
 *
 * @param[in] now_ms Current time.
 *
 * @return S5_AVOID_BUSY, or S5_AVOID_FAILURE if the sensor stopped
 *         pinging.
 */
static s5_avoid_result_t
step_check (uint32_t now_ms)
{
    s5_avoid_result_t        result = S5_AVOID_BUSY;
    subsystem5_obst_status_t status = {0};
    uint32_t                 fresh  = fresh_pings(&status);

    // NOTE: The casts widen uint16_t and the 10-bit field, so no value is
    // lost.
    int32_t limit_mm = (int32_t) g_avoid.plan.out_mm + S5_SENSOR_SETBACK_MM
                       + CHECK_MARGIN_MM;

    if ((fresh > LOOK_SKIP_PINGS) && status.b_range_valid
        && ((int32_t) status.range_mm < limit_mm))
    {
        g_avoid.b_look_near = true;
    }

    if (fresh >= LOOK_PINGS)
    {
        subsystem5_obst_request(OB_HOLD);
        s5_event_trace("AVOID way out: blocked limit_mm",
                       g_avoid.b_look_near ? 1 : 0, limit_mm);

        if (!g_avoid.b_look_near)
        {
            // NOTE: The cast widens uint16_t, so no value is lost.
            start_move((int32_t) g_avoid.plan.out_mm, now_ms);
            enter(S5_AVOID_MOVE_OUT);
        }
        else if ((s5_math_abs(g_avoid.plan.turn_deg) + OUT_STEP_DEG)
                 <= OUT_STEEP_MAX_DEG)
        {
            // Likely the obstacle's own corner: try further out.
            turn_out_more((g_avoid.plan.turn_deg > 0) ? OUT_STEP_DEG
                                                      : -OUT_STEP_DEG,
                          now_ms);
        }
        else
        {
            g_avoid.blocked = (g_avoid.blocked
                               | ((g_avoid.plan.turn_deg > 0) ? SIDE_LEFT
                                                              : SIDE_RIGHT))
                              & (SIDE_LEFT | SIDE_RIGHT);
            start_turn(back_turn_deg(-g_avoid.plan.turn_deg), now_ms);
            enter(S5_AVOID_CHECK_BACK);
        }
    }
    else if (is_timed_out(now_ms))
    {
        result = fail(S5_CTRL_FAULT_SENSOR);
    }
    else
    {
        // Still listening.
    }

    return (result);
}

/*!
 * @brief The way out is blocked: turn step_deg further out and check
 *        again (see "Checking the way out"), with the out leg worked out
 *        again for the same shift.
 *
 * @param[in] step_deg Further turn, same sign as the turn out.
 * @param[in] now_ms   Current time.
 *
 * NOTE: shift = out sin(a) stays; the new out leg is shift / sin(a'),
 * and it reaches out cos(a) - out' cos(a') less far forward, which the
 * drive before the first look makes up.
 */
static void
turn_out_more (int32_t step_deg, uint32_t now_ms)
{
    s5_plan_t * p_plan   = &g_avoid.plan;
    int32_t     old_deg  = s5_math_abs(p_plan->turn_deg);
    int32_t     new_deg  = old_deg + s5_math_abs(step_deg);

    // NOTE: The casts widen uint16_t, so no value is lost.
    int32_t     old_out  = (int32_t) p_plan->out_mm;
    int32_t     shift_mm = s5_math_mul_q15(old_out,
                                           s5_math_sin_q15(old_deg));

    // NOTE: shift_mm is a few hundred mm, so x 32768 stays below 2^31,
    // and sin is above zero for 1 ... 80 degrees.
    int32_t     new_out  = (shift_mm * S5_MATH_Q15_ONE)
                           / s5_math_sin_q15(new_deg);
    int32_t     less_fwd = s5_math_mul_q15(old_out,
                                           s5_math_cos_q15(old_deg))
                           - s5_math_mul_q15(new_out,
                                             s5_math_cos_q15(new_deg));
    int32_t     look_mm  = (int32_t) p_plan->look_mm
                           + ((less_fwd > 0) ? less_fwd : 0);

    // NOTE: The casts are safe: the angle is at most OUT_STEEP_MAX_DEG
    // and the legs a few hundred mm.
    p_plan->turn_deg = (int16_t) (p_plan->turn_deg + step_deg);
    p_plan->in_deg   = p_plan->turn_deg;
    p_plan->out_mm   = (uint16_t) new_out;
    p_plan->look_mm  = (uint16_t) look_mm;
    s5_event_trace("AVOID way out blocked: turning out more to_deg out_mm",
                   p_plan->turn_deg, new_out);
    start_turn(step_deg, now_ms);
    enter(S5_AVOID_TURN_OUT);
}

/*!
 * @brief Start listening to obstacle_task's pings: from the next one on,
 *        for at most LOOK_MS.
 *
 * @param[in] now_ms Current time.
 */
static void
listen (uint32_t now_ms)
{
    subsystem5_obst_status_t status = {0};

    subsystem5_obst_status_get(&status);
    g_avoid.look_from     = status.ping_count & S5_OBST_COUNT_MASK;
    g_avoid.b_look_near   = false;
    g_avoid.wait_start_ms = now_ms;
    g_avoid.wait_limit_ms = LOOK_MS;
}

/*!
 * @brief The latest ping, and how many pings since listen().
 *
 * @param[out] p_status The latest ping's status.
 *
 * @return Pings made since listen(), 0 ... 1023.
 */
static uint32_t
fresh_pings (subsystem5_obst_status_t * p_status)
{
    subsystem5_obst_status_get(p_status);

    // NOTE: The casts widen the 10-bit fields, so no value is lost.
    return (((uint32_t) p_status->ping_count - (uint32_t) g_avoid.look_from)
            & S5_OBST_COUNT_MASK);
}

/*!
 * @brief LOOK_PINGS: listen until LOOK_PINGS pings have been made, then
 *        turn back along the track.
 *
 * @param[in] now_ms Current time.
 *
 * @return S5_AVOID_BUSY, or S5_AVOID_FAILURE if the sensor stopped
 *         pinging.
 *
 * NOTE: The status holds the last ping only and this runs every tick
 * (50 ms) or sooner, so a ping now and then is not seen; a flat side
 * square on answers every ping.
 */
static s5_avoid_result_t
step_look (uint32_t now_ms)
{
    s5_avoid_result_t        result = S5_AVOID_BUSY;
    subsystem5_obst_status_t status = {0};

    uint32_t                 fresh  = fresh_pings(&status);

    if ((fresh > LOOK_SKIP_PINGS) && is_side_echo(&status))
    {
        g_avoid.b_look_near = true;
    }

    if (fresh >= LOOK_PINGS)
    {
        // NOTE: The casts are safe: looks is at most LOOKS_MAX.
        s5_event_trace("AVOID look: number echo", (int32_t) g_avoid.looks,
                       g_avoid.b_look_near ? 1 : 0);

        if ((!g_avoid.b_look_near) && g_avoid.b_silent)
        {
            // Silent twice in a row: past it. Straight on from here.
            start_way_back(now_ms);
        }
        else
        {
            g_avoid.b_silent = !g_avoid.b_look_near;
            start_turn(back_turn_deg(-look_deg()), now_ms);
            enter(S5_AVOID_LOOK_BACK);
        }
    }
    else if (is_timed_out(now_ms))
    {
        result = fail(S5_CTRL_FAULT_SENSOR);
    }
    else
    {
        // Still listening.
    }

    return (result);
}

/*!
 * @brief Tell whether a look's ping came back from the obstacle's side.
 *
 * @param[in] p_status The ping.
 *
 * @return true for an echo no further across than the side's expected
 *         place plus LOOK_NEAR_SLACK_MM.
 *
 * NOTE: Turned S5_LOOK_DEG, the sensor is SENSOR_AHEAD_MM x sin(look)
 * nearer the side than the centre line, which passes at
 * s5_profile_pass_mm(). The beam is the look plus the servo's angle off
 * ahead from the track; an echo r away is r x sin(beam) across.
 */
static bool
is_side_echo (subsystem5_obst_status_t const * p_status)
{
    bool b_echo = false;

    if (p_status->b_range_valid)
    {
        // NOTE: The casts widen the bit-fields, so no value is lost.
        int32_t servo_off = s5_math_abs((int32_t) p_status->servo_deg
                                        - AHEAD_DEG);
        int32_t across    = s5_math_mul_q15((int32_t) p_status->range_mm,
                                            s5_math_sin_q15(S5_LOOK_DEG
                                                            + servo_off));
        int32_t side_mm   = s5_profile_pass_mm(g_avoid.plan.turn_deg < 0)
                            - s5_math_mul_q15(SENSOR_AHEAD_MM,
                                              s5_math_sin_q15(S5_LOOK_DEG));

        b_echo = (across < (side_mm + LOOK_NEAR_SLACK_MM));
    }

    return (b_echo);
}

/*!
 * @brief The look's turn: towards the obstacle.
 *
 * @return S5_LOOK_DEG, + = left: going round on the left leaves the
 *         obstacle on the right.
 */
static int32_t
look_deg (void)
{
    return ((g_avoid.plan.turn_deg > 0) ? -S5_LOOK_DEG : S5_LOOK_DEG);
}

/*!
 * @brief S3 sees the line on the way back: hand over if the robot is
 *        nearly along it, else first put the rear axle on it and turn
 *        forward until S3 sees it again (see "Finding the line again").
 *
 * @param[in] now_ms Current time.
 *
 * @return S5_AVOID_RESUME if handed over, else S5_AVOID_BUSY.
 */
static s5_avoid_result_t
line_found (uint32_t now_ms)
{
    s5_avoid_result_t result   = S5_AVOID_BUSY;
    int32_t           back_deg = heading_off_deg();

    if (s5_math_abs(back_deg) < ONTO_MIN_DEG)
    {
        result = hand_over();
    }
    else
    {
        s5_event_trace("AVOID line crossed: cross_deg, axle onto it_mm",
                       back_deg, S5_AXLE_TO_LINE_IR_MM);

        // S3 only watches meanwhile; its search is asked for again before
        // the turn.
        s5_links_motion_stop();
        s5_links_line_search(S5_LINKS_SIDE_NONE);
        start_move(S5_AXLE_TO_LINE_IR_MM, now_ms);
        enter(S5_AVOID_ONTO_LINE);
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
 * @brief Send a MOVE and start its time limit. Backwards is slower
 *        (REVERSE_MMS).
 *
 * @param[in] dist_mm Distance; negative = reverse.
 * @param[in] now_ms  Current time.
 */
static void
start_move (int32_t dist_mm, uint32_t now_ms)
{
    uint32_t dist  = (dist_mm < 0) ? (uint32_t) -dist_mm : (uint32_t) dist_mm;
    uint32_t speed = (dist_mm < 0) ? REVERSE_MMS : SPEED_MMS;

    // NOTE: The casts above are safe: |dist_mm| is at most a few metres,
    // and both speeds fit the uint16_t speed argument.
    s5_links_motion_move(dist_mm, (uint16_t) speed);
    g_avoid.wait_start_ms = now_ms;
    g_avoid.wait_limit_ms = ((dist * MS_PER_S) / speed) + MOTION_SLACK_MS;
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
    g_avoid.fault = (uint32_t) fault & S5_CTRL_FAULT_MASK;
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
    // NOTE: The casts are safe: the state is a small enum value.
    g_avoid.state = (uint32_t) state & S5_AVOID_STATE_MASK;
    s5_event_post_args(S5_EVENT_AVOID_STEP, (uint32_t) state, 0u);
}

/*** end of file ***/
