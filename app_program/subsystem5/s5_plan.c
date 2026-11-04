/** @file s5_plan.c
 *
 * @brief Avoidance planning (FR11).
 *
 * @par Rules (Week 6 report 6.4 PLAN)
 * - Nothing in the robot's path: continue.
 * - A side may be used only if its edge was found and its clearance is
 *   at least the robot's width plus S5_PATH_MARGIN_MM.
 * - If both sides may be used, take the one with the larger clearance
 *   (Week 6 report: "bypass on the side with greater clearance"; the
 *   brief: "on the left-hand side we've got a bigger gap"). On a tie,
 *   the one needing the smaller sideways shift, so the robot stays
 *   closest to the line (Project Write-up: avoidance "should minimise
 *   deviation from the original route"); then the left.
 * - If neither may be used: reverse and rescan.
 *
 * @par Leg lengths
 * The diagonal leg covers the sideways shift: out = shift / sin(turn).
 * The way back uses the same leg, so the robot returns to the line.
 * The tail is short: turning in towards the line swings the robot's rear
 * AWAY from the obstacle, so only the side near its front must clear the
 * obstacle's back corner. With the gap s5_avoid.c makes sure of (sensor
 * at least 110 mm from the obstacle's side) that needs about 50 mm; the
 * tail is 100 mm. It was one robot length plus the margin (300 mm) until
 * log 2026-11-04_s5_avoid_7, where that drove far past the obstacle.
 *
 * @par Room to turn out (step 0)
 * The robot turns about the middle of its axle, then drives the
 * diagonal leg. Seen from above, with F = axle to front and W = width:
 * - Turning, its front corners swing forward by sqrt(F^2 + (W/2)^2) - F.
 * - On the 45 degree leg it moves forward as far as sideways. Its front
 *   corner on the obstacle's side reaches the obstacle's face first, and
 *   must by then be past the edge by the margin. That needs a gap of
 *   shift + W x (sin 45 - 1/2) - F; shift already holds the edge, half
 *   the width and the margin.
 * The gap is the scan's closest range less the sensor's setback. If it
 * is smaller than either need, the robot first reverses the difference.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"

#define NEED_CLEARANCE_MM   (S5_ROBOT_WIDTH_MM + S5_PATH_MARGIN_MM)
#define TAIL_MM             (100)       /* See "Leg lengths" above.      */
#define LEG_MAX_MM          (1000)      /* Longer would leave the track. */
#define HALF_WIDTH_MM       (S5_ROBOT_WIDTH_MM / 2)
#define TURN_MARGIN_MM      (20)        /* Kept while turning on the spot*/
#define ROOM_MAX_MM         (300)       /* Longest reverse before a turn.*/

static bool     side_is_usable(bool b_edge, uint16_t clearance_mm);
static uint16_t leg_for_shift(uint16_t shift_mm);
static uint16_t room_for_bypass(uint16_t shift_mm, uint16_t closest_mm);
static void     fill_bypass(s5_plan_t * p_plan, bool b_left,
                            s5_profile_t const * p_profile);

/*!
 * @brief Decide what to do about a profiled obstacle.
 *
 * @param[in]  p_profile The profile from obstacle_task.
 * @param[out] p_plan    The decision and, for a bypass, its legs.
 */
void
s5_plan_make (s5_profile_t const * p_profile, s5_plan_t * p_plan)
{
    bool b_left_ok  = side_is_usable(p_profile->b_left_edge,
                                     p_profile->clear_left_mm);
    bool b_right_ok = side_is_usable(p_profile->b_right_edge,
                                     p_profile->clear_right_mm);

    p_plan->action       = S5_PLAN_REVERSE;
    p_plan->turn_deg     = 0;
    p_plan->out_mm       = 0u;
    p_plan->tail_mm      = 0u;
    p_plan->back_mm      = 0u;
    p_plan->clearance_mm = 0u;
    p_plan->room_mm      = 0u;

    if (!p_profile->b_on_path)
    {
        p_plan->action = S5_PLAN_CONTINUE;
    }
    else if (b_left_ok && b_right_ok)
    {
        // Larger gap first; then the smaller shift; then the left.
        bool b_left = (p_profile->clear_left_mm > p_profile->clear_right_mm)
                      || ((p_profile->clear_left_mm
                           == p_profile->clear_right_mm)
                          && (p_profile->shift_left_mm
                              <= p_profile->shift_right_mm));

        fill_bypass(p_plan, b_left, p_profile);
    }
    else if (b_left_ok)
    {
        fill_bypass(p_plan, true, p_profile);
    }
    else if (b_right_ok)
    {
        fill_bypass(p_plan, false, p_profile);
    }
    else
    {
        // Neither side is wide enough: S5_PLAN_REVERSE, set above.
    }
}

/*!
 * @brief Tell whether the robot fits past the obstacle on one side.
 *
 * @param[in] b_edge       The edge on that side was found.
 * @param[in] clearance_mm Free gap beside that edge.
 *
 * @return true if the side may be used.
 */
static bool
side_is_usable (bool b_edge, uint16_t clearance_mm)
{
    return (b_edge && (clearance_mm >= NEED_CLEARANCE_MM));
}

/*!
 * @brief Length of the diagonal leg that moves the robot sideways.
 *
 * @param[in] shift_mm Sideways distance needed.
 *
 * @return Leg length in mm, at most LEG_MAX_MM.
 */
static uint16_t
leg_for_shift (uint16_t shift_mm)
{
    // out = shift / sin(turn), in Q15: shift * 32768 / sin_q15.
    // NOTE: The cast widens uint16_t; shift * 32768 < 2^31 and sin(45)
    // is positive, so the result is never negative.
    int32_t leg_mm = ((int32_t) shift_mm * S5_MATH_Q15_ONE)
                     / s5_math_sin_q15(S5_PLAN_TURN_DEG);

    if (leg_mm > LEG_MAX_MM)
    {
        leg_mm = LEG_MAX_MM;
    }

    // NOTE: The cast is safe: leg_mm is 0 ... LEG_MAX_MM here.
    return ((uint16_t) leg_mm);
}

/*!
 * @brief Fill in a bypass on one side.
 *
 * @param[out] p_plan    The plan.
 * @param[in]  b_left    true for the left, false for the right.
 * @param[in]  p_profile The obstacle profile.
 */
static void
fill_bypass (s5_plan_t * p_plan, bool b_left, s5_profile_t const * p_profile)
{
    uint16_t shift_mm = b_left ? p_profile->shift_left_mm
                               : p_profile->shift_right_mm;

    p_plan->action       = b_left ? S5_PLAN_LEFT : S5_PLAN_RIGHT;
    p_plan->turn_deg     = b_left ? S5_PLAN_TURN_DEG : -S5_PLAN_TURN_DEG;
    p_plan->out_mm       = leg_for_shift(shift_mm);
    p_plan->tail_mm      = TAIL_MM;
    p_plan->back_mm      = p_plan->out_mm;
    p_plan->clearance_mm = b_left ? p_profile->clear_left_mm
                                  : p_profile->clear_right_mm;
    p_plan->room_mm      = room_for_bypass(shift_mm, p_profile->closest_mm);
}

/*!
 * @brief How far to reverse before turning out (see "Room to turn out"
 *        above).
 *
 * @param[in] shift_mm   Sideways move of the chosen side.
 * @param[in] closest_mm The obstacle's closest range from the sensor.
 *
 * @return 0 ... ROOM_MAX_MM.
 */
static uint16_t
room_for_bypass (uint16_t shift_mm, uint16_t closest_mm)
{
    // NOTE: The casts are safe: all values are a few hundred mm, and
    // the square root of a positive sum is below 2^16.
    uint32_t corner_sq = (uint32_t) ((S5_AXLE_TO_FRONT_MM
                                      * S5_AXLE_TO_FRONT_MM)
                                     + (HALF_WIDTH_MM * HALF_WIDTH_MM));
    int32_t  swing_mm  = (int32_t) s5_math_isqrt(corner_sq)
                         - S5_AXLE_TO_FRONT_MM + TURN_MARGIN_MM;
    int32_t  leg_mm    = (int32_t) shift_mm
                         + s5_math_mul_q15(S5_ROBOT_WIDTH_MM,
                                           s5_math_sin_q15(S5_PLAN_TURN_DEG))
                         - HALF_WIDTH_MM - S5_AXLE_TO_FRONT_MM;
    int32_t  need_mm   = (leg_mm > swing_mm) ? leg_mm : swing_mm;
    int32_t  room_mm   = need_mm
                         - ((int32_t) closest_mm - S5_SENSOR_SETBACK_MM);

    if (room_mm < 0)
    {
        room_mm = 0;
    }
    else if (room_mm > ROOM_MAX_MM)
    {
        room_mm = ROOM_MAX_MM;
    }
    else
    {
        // Already inside the range.
    }

    // NOTE: The cast is safe: room_mm is 0 ... ROOM_MAX_MM here.
    return ((uint16_t) room_mm);
}

/*** end of file ***/
