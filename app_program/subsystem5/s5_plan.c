/** @file s5_plan.c
 *
 * @brief Avoidance planning (FR11).
 *
 * @par Rules (Week 6 report 6.4 PLAN)
 * - Nothing in the robot's path: continue.
 * - A side may be used only if its edge was found and its clearance is
 *   at least the robot's width plus S5_PATH_MARGIN_MM.
 * - If both sides may be used, take the one with the larger clearance
 *   (the brief: "on the left-hand side we've got a bigger gap"); on a
 *   tie, the one needing the smaller sideways shift; then the left.
 * - If neither may be used: reverse and rescan.
 *
 * @par Leg lengths
 * The diagonal leg covers the sideways shift: out = shift / sin(turn).
 * The way back uses the same leg, so the robot returns to the line.
 * The tail is one robot length plus the margin, so the rear clears the
 * obstacle after the sensor (at the front) has passed it.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"

#define NEED_CLEARANCE_MM   (S5_ROBOT_WIDTH_MM + S5_PATH_MARGIN_MM)
#define TAIL_MM             (S5_ROBOT_LENGTH_MM + S5_PATH_MARGIN_MM)
#define LEG_MAX_MM          (1000)      /* Longer would leave the track. */

static bool     side_is_usable(bool b_edge, uint16_t clearance_mm);
static uint16_t leg_for_shift(uint16_t shift_mm);
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
}

/*** end of file ***/
