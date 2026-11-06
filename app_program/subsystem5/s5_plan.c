/** @file s5_plan.c
 *
 * @brief Avoidance planning (FR11).
 *
 * @par Rules (Week 6 report 6.4 PLAN)
 * - Nothing in the robot's path: continue.
 * - A side may be used only if its edge was found and its clearance is
 *   at least the robot's width plus S5_PATH_MARGIN_MM.
 * - If both sides may be used, take the one with the larger clearance
 *   (Week 6 report: "bypass on the side with greater clearance"). On a
 *   tie, the one needing the smaller sideways shift, so the robot stays
 *   closest to the line (Project Write-up: avoidance "should minimise
 *   deviation from the original route"); then the left.
 * - If neither may be used: reverse and rescan.
 *
 * @par Turn-out angle (seen from above)
 * The robot turns about the middle of its rear axle. Its front corner on
 * the obstacle's side is S5_AXLE_TO_FRONT_MM ahead of the axle and half
 * the width to the side. The profile's shift is how far the axle must
 * move sideways so the robot's side clears the obstacle's edge by
 * S5_PATH_MARGIN_MM. Driving d at angle a moves the axle d sin(a)
 * sideways and d cos(a) forward. The angle chosen is the smallest one,
 * from S5_PLAN_OUT_MIN_DEG, at which that front corner is already
 * (shift - half width) to the side when it reaches the obstacle's front
 * face, so it never touches the face or its corner. A thin obstacle gets
 * a gentle angle, a wide one a steep angle (about 33 degrees for 6 cm,
 * 45 for 20 cm, 52 for 30 cm, with the face 150 mm from the front). If
 * even S5_PLAN_OUT_MAX_DEG is not enough, the robot reverses first; the
 * reverse is worked out at that angle.
 *
 * @par Leg lengths
 * - out = shift / sin(turn): the axle ends shift to the side, and turning
 *   back parallel only swings the front in, to exactly the margin.
 * - tail: when the side watch reports the obstacle passed, its far end is
 *   level with the sensor, which is (S5_AXLE_TO_FRONT_MM -
 *   S5_SENSOR_SETBACK_MM) ahead of the axle. One more margin puts it
 *   behind the axle: turning in then swings only the part ahead of the
 *   axle towards the obstacle, and that part is already past it.
 * - The way back mirrors the way out: the same angle and the same leg,
 *   so on a straight track the axle ends on the line. (A steep 60 degree
 *   way back overshot: every mm past the line moved the robot 0.87 mm
 *   off it; Anna, 2026-11-06.)
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_plan.h"
#include "s5_profile.h"

#define NEED_CLEARANCE_MM   (S5_ROBOT_WIDTH_MM + S5_PATH_MARGIN_MM)
#define HALF_WIDTH_MM       (S5_ROBOT_WIDTH_MM / 2)
#define SENSOR_AHEAD_MM     (S5_AXLE_TO_FRONT_MM - S5_SENSOR_SETBACK_MM)
#define TAIL_MM             (SENSOR_AHEAD_MM + S5_PATH_MARGIN_MM)
#define TURN_MARGIN_MM      (20)        /* Kept while turning on the spot*/
#define LEG_MAX_MM          (1000)      /* Longer would leave the track. */
#define ROOM_MAX_MM         (300)       /* Longest reverse before a turn.*/

static bool     side_is_usable(bool b_edge, uint16_t clearance_mm);
static void     fill_bypass(s5_plan_t * p_plan, bool b_left,
                            s5_profile_t const * p_profile);
static int32_t  corner_out_mm(int32_t angle_deg, int32_t face_mm);
static int32_t  choose_angle(int32_t need_mm, int32_t face_mm,
                             int32_t * p_room_mm);
static uint16_t leg_for_shift(int32_t shift_mm, int32_t angle_deg);

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
    p_plan->in_deg       = 0;
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
 * @brief Fill in a bypass on one side.
 *
 * @param[out] p_plan    The plan.
 * @param[in]  b_left    true for the left, false for the right.
 * @param[in]  p_profile The obstacle profile.
 */
static void
fill_bypass (s5_plan_t * p_plan, bool b_left, s5_profile_t const * p_profile)
{
    // NOTE: The casts widen uint16_t, so no value is lost.
    int32_t shift_mm = (int32_t) (b_left ? p_profile->shift_left_mm
                                         : p_profile->shift_right_mm);
    int32_t face_mm  = ((int32_t) p_profile->closest_mm
                        - S5_SENSOR_SETBACK_MM) + S5_AXLE_TO_FRONT_MM;
    int32_t room_mm  = 0;
    int32_t angle    = choose_angle(shift_mm - HALF_WIDTH_MM, face_mm,
                                    &room_mm);

    // NOTE: The casts are safe: the angles are at most 90 degrees, the
    // tail a few hundred mm and room_mm 0 ... ROOM_MAX_MM.
    p_plan->action       = b_left ? S5_PLAN_LEFT : S5_PLAN_RIGHT;
    p_plan->turn_deg     = (int16_t) (b_left ? angle : -angle);
    p_plan->in_deg       = p_plan->turn_deg;
    p_plan->out_mm       = leg_for_shift(shift_mm, angle);
    p_plan->tail_mm      = (uint16_t) TAIL_MM;
    p_plan->back_mm      = p_plan->out_mm;
    p_plan->clearance_mm = b_left ? p_profile->clear_left_mm
                                  : p_profile->clear_right_mm;
    p_plan->room_mm      = (uint16_t) room_mm;
}

/*!
 * @brief How far to the side the robot's front corner on the obstacle's
 *        side is when it reaches the obstacle's front face, after turning
 *        out by angle_deg and driving straight.
 *
 * @param[in] angle_deg Turn-out angle, S5_PLAN_OUT_MIN_DEG ... 90.
 * @param[in] face_mm   Obstacle's face ahead of the rear axle.
 *
 * @return Sideways distance from the robot's starting centre line, mm
 *         (negative: still inside it).
 */
static int32_t
corner_out_mm (int32_t angle_deg, int32_t face_mm)
{
    int32_t sin_q15 = s5_math_sin_q15(angle_deg);
    int32_t cos_q15 = s5_math_cos_q15(angle_deg);

    // The corner just after the turn: across and ahead of the axle.
    int32_t across  = s5_math_mul_q15(S5_AXLE_TO_FRONT_MM, sin_q15)
                      - s5_math_mul_q15(HALF_WIDTH_MM, cos_q15);
    int32_t ahead   = s5_math_mul_q15(S5_AXLE_TO_FRONT_MM, cos_q15)
                      + s5_math_mul_q15(HALF_WIDTH_MM, sin_q15);
    int32_t reach   = face_mm - ahead;

    // Driving reach / cos(a) moves it reach x tan(a) further across.
    // NOTE: cos(a) is above zero for the angles used, and reach x sin(a)
    // stays far below 2^31.
    return (across + ((reach * sin_q15) / cos_q15));
}

/*!
 * @brief The smallest turn-out angle at which the front corner clears
 *        the obstacle (see "Turn-out angle" above), and any reverse
 *        needed first.
 *
 * @param[in]  need_mm   How far the corner must be to the side at the
 *                       face: the shift less half the robot's width.
 * @param[in]  face_mm   Obstacle's face ahead of the rear axle.
 * @param[out] p_room_mm Reverse needed first, 0 ... ROOM_MAX_MM.
 *
 * @return Turn-out angle, S5_PLAN_OUT_MIN_DEG ... S5_PLAN_OUT_MAX_DEG.
 */
static int32_t
choose_angle (int32_t need_mm, int32_t face_mm, int32_t * p_room_mm)
{
    // NOTE: The casts are safe: the corner's distance from the axle is a
    // few hundred mm, so its square root is far below 2^16.
    int32_t  angle    = S5_PLAN_OUT_MIN_DEG;
    uint32_t swing_sq = (uint32_t) ((S5_AXLE_TO_FRONT_MM
                                     * S5_AXLE_TO_FRONT_MM)
                                    + (HALF_WIDTH_MM * HALF_WIDTH_MM));
    int32_t  swing_mm = ((int32_t) s5_math_isqrt(swing_sq)
                         + TURN_MARGIN_MM);
    int32_t  room_mm  = 0;

    while ((angle < S5_PLAN_OUT_MAX_DEG)
           && (corner_out_mm(angle, face_mm) < need_mm))
    {
        angle++;
    }

    if (corner_out_mm(angle, face_mm) < need_mm)
    {
        // Too close even at the steepest angle: back off far enough. At
        // a fixed angle the corner gains tan(a) across per mm further.
        int32_t short_mm = need_mm - corner_out_mm(angle, face_mm);

        room_mm = (short_mm * s5_math_cos_q15(angle))
                  / s5_math_sin_q15(angle);
    }

    // Turning on the spot swings the front corners forward too.
    if ((face_mm + room_mm) < swing_mm)
    {
        room_mm = swing_mm - face_mm;
    }

    if (room_mm > ROOM_MAX_MM)
    {
        room_mm = ROOM_MAX_MM;
    }
    else if (room_mm < 0)
    {
        room_mm = 0;
    }
    else
    {
        // Already inside the range.
    }

    *p_room_mm = room_mm;

    return (angle);
}

/*!
 * @brief Length of a straight leg that moves the axle sideways.
 *
 * @param[in] shift_mm  Sideways distance needed.
 * @param[in] angle_deg Angle of the leg to the line, 1 ... 90.
 *
 * @return Leg length in mm, 0 ... LEG_MAX_MM.
 */
static uint16_t
leg_for_shift (int32_t shift_mm, int32_t angle_deg)
{
    // leg = shift / sin(angle), in Q15: shift * 32768 / sin_q15.
    // NOTE: shift is a few hundred mm, so shift * 32768 < 2^31, and sin
    // is positive for the angles used.
    int32_t leg_mm = (shift_mm * S5_MATH_Q15_ONE)
                     / s5_math_sin_q15(angle_deg);

    if (leg_mm > LEG_MAX_MM)
    {
        leg_mm = LEG_MAX_MM;
    }
    else if (leg_mm < 0)
    {
        leg_mm = 0;
    }
    else
    {
        // Already inside the range.
    }

    // NOTE: The cast is safe: leg_mm is 0 ... LEG_MAX_MM here.
    return ((uint16_t) leg_mm);
}

/*** end of file ***/
