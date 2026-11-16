/** @file s5_plan.c
 *
 * @brief Avoidance planning (FR11).
 *
 * @par Rules (Week 6 report 6.4 PLAN)
 * - Nothing in the robot's path: continue.
 * - A side may be used only if its edge was found and its clearance
 *   takes the robot passing at s5_profile_pass_mm() from the edge, out
 *   to its far side (S3's barcode sensor included on the left).
 * - If both sides may be used and the caller prefers one, take it: the
 *   side s5_avoid.c looked past the corner at and found free (Anna,
 *   2026-11-13: on a left bend the box sits left of the nose, and the
 *   smaller shift kept sending the robot right, logs 1513 and 1518).
 * - Otherwise, if both sides may be used, take the one with the larger
 *   clearance
 *   (Week 6 report: "bypass on the side with greater clearance"). On a
 *   tie, the one needing the smaller sideways shift, so the robot stays
 *   closest to the line (Project Write-up: avoidance "should minimise
 *   deviation from the original route"); then the left.
 * - If neither may be used: reverse and rescan.
 *
 * @par Turn-out angle (seen from above)
 * The robot turns about the middle of its rear axle. Its front corner on
 * the obstacle's side is S5_AXLE_TO_FRONT_MM ahead of the axle and half
 * the width to the side; going round on the right, S3's barcode sensor's
 * tip (S5_SIDE_EXTRA_xxx) faces the obstacle too. The profile's shift is
 * how far the axle must move sideways to pass at s5_profile_pass_mm()
 * from the obstacle's edge (the side margin, or the room to turn and
 * look while passing). Driving d at angle a moves the axle d sin(a)
 * sideways and d cos(a) forward. The angle chosen is the smallest one,
 * from S5_PLAN_OUT_MIN_DEG, at which the front corner and the tip are
 * already where they will pass when they reach the obstacle's front
 * face, so they never touch the face or its corner. A thin obstacle gets
 * a gentle angle, a wide one a steep angle (with the face 150 mm from the
 * front, about 34 degrees for 6 cm, 42 for 18 cm, 48 for 30 cm going
 * left; 41, 49 and 55 going right). If even S5_PLAN_OUT_MAX_DEG is not
 * enough, the robot reverses first; the reverse is worked out at that
 * angle.
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

#define HALF_WIDTH_MM       (S5_ROBOT_WIDTH_MM / 2)
#define SENSOR_AHEAD_MM     (S5_AXLE_TO_FRONT_MM - S5_SENSOR_SETBACK_MM)
#define TAIL_MM             (SENSOR_AHEAD_MM + S5_PATH_MARGIN_MM)
#define LOOK_SPARE_MM       (20)        /* Past the face before looking. */
#define TURN_MARGIN_MM      (20)        /* Kept while turning on the spot*/
#define LEG_MAX_MM          (1000)      /* Longer would leave the track. */
#define ROOM_MAX_MM         (300)       /* Longest reverse before a turn.*/

static bool     side_is_usable(bool b_edge, uint16_t clearance_mm,
                               bool b_left);
static void     fill_bypass(s5_plan_t * p_plan, bool b_left,
                            s5_profile_t const * p_profile);
static int32_t  point_out_mm(int32_t angle_deg, int32_t face_mm,
                             int32_t ahead_mm, int32_t side_mm);
static int32_t  shortfall_mm(int32_t angle_deg, int32_t shift_mm,
                             int32_t extra_mm, int32_t face_mm);
static int32_t  choose_angle(int32_t shift_mm, int32_t extra_mm,
                             int32_t face_mm, int32_t * p_room_mm);
static uint16_t look_leg_mm(int32_t face_mm, int32_t angle_deg,
                            uint16_t out_mm);
static uint16_t leg_for_shift(int32_t shift_mm, int32_t angle_deg);

/*!
 * @brief Decide what to do about a profiled obstacle.
 *
 * @param[in]  p_profile The profile from obstacle_task.
 * @param[in]  prefer    S5_PLAN_PREFER_xxx: the side to take if both may
 *                       be used.
 * @param[out] p_plan    The decision and, for a bypass, its legs.
 */
void
s5_plan_make (s5_profile_t const * p_profile, int32_t prefer,
              s5_plan_t * p_plan)
{
    bool b_left_ok  = side_is_usable(p_profile->b_left_edge,
                                     p_profile->clear_left_mm, true);
    bool b_right_ok = side_is_usable(p_profile->b_right_edge,
                                     p_profile->clear_right_mm, false);

    p_plan->action       = S5_PLAN_REVERSE;
    p_plan->turn_deg     = 0;
    p_plan->in_deg       = 0;
    p_plan->out_mm       = 0u;
    p_plan->look_mm      = 0u;
    p_plan->tail_mm      = 0u;
    p_plan->back_mm      = 0u;
    p_plan->clearance_mm = 0u;
    p_plan->room_mm      = 0u;

    if (!p_profile->b_on_path)
    {
        p_plan->action = S5_PLAN_CONTINUE;
    }
    else if (b_left_ok && b_right_ok && (S5_PLAN_PREFER_NONE != prefer))
    {
        fill_bypass(p_plan, (S5_PLAN_PREFER_LEFT == prefer), p_profile);
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
 * @param[in] b_left       true for the left side.
 *
 * @return true if the side may be used.
 *
 * NOTE: Going round on the left leaves the obstacle on the robot's
 * right, and the other way round.
 */
static bool
side_is_usable (bool b_edge, uint16_t clearance_mm, bool b_left)
{
    int32_t need_mm = s5_profile_pass_mm(!b_left)
                      + s5_profile_half_mm(b_left);

    // NOTE: The cast widens uint16_t, so no value is lost.
    return (b_edge && ((int32_t) clearance_mm >= need_mm));
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
    int32_t extra_mm = b_left ? S5_SIDE_EXTRA_RIGHT_MM
                              : S5_SIDE_EXTRA_LEFT_MM;
    int32_t room_mm  = 0;
    int32_t angle    = choose_angle(shift_mm, extra_mm, face_mm, &room_mm);

    // NOTE: The casts are safe: the angles are at most 90 degrees, the
    // tail a few hundred mm and room_mm 0 ... ROOM_MAX_MM.
    p_plan->action       = b_left ? S5_PLAN_LEFT : S5_PLAN_RIGHT;
    p_plan->turn_deg     = (int16_t) (b_left ? angle : -angle);
    p_plan->in_deg       = p_plan->turn_deg;
    p_plan->out_mm       = leg_for_shift(shift_mm, angle);
    p_plan->look_mm      = look_leg_mm(face_mm + room_mm, angle,
                                       p_plan->out_mm);
    p_plan->tail_mm      = (uint16_t) TAIL_MM;
    p_plan->back_mm      = p_plan->out_mm;
    p_plan->clearance_mm = b_left ? p_profile->clear_left_mm
                                  : p_profile->clear_right_mm;
    p_plan->room_mm      = (uint16_t) room_mm;
}

/*!
 * @brief How far to the side a point of the robot on the obstacle's side
 *        is when it reaches the obstacle's front face, after turning out
 *        by angle_deg and driving straight.
 *
 * @param[in] angle_deg Turn-out angle, S5_PLAN_OUT_MIN_DEG ... 90.
 * @param[in] face_mm   Obstacle's face ahead of the rear axle.
 * @param[in] ahead_mm  The point, ahead of the rear axle.
 * @param[in] side_mm   The point, to the side of the centre line.
 *
 * @return Sideways distance from the robot's starting centre line, mm
 *         (negative: still inside it).
 */
static int32_t
point_out_mm (int32_t angle_deg, int32_t face_mm, int32_t ahead_mm,
              int32_t side_mm)
{
    int32_t sin_q15 = s5_math_sin_q15(angle_deg);
    int32_t cos_q15 = s5_math_cos_q15(angle_deg);

    // The point just after the turn: across and ahead of the axle.
    int32_t across  = s5_math_mul_q15(ahead_mm, sin_q15)
                      - s5_math_mul_q15(side_mm, cos_q15);
    int32_t ahead   = s5_math_mul_q15(ahead_mm, cos_q15)
                      + s5_math_mul_q15(side_mm, sin_q15);
    int32_t reach   = face_mm - ahead;

    // Driving reach / cos(a) moves it reach x tan(a) further across.
    // NOTE: cos(a) is above zero for the angles used, and reach x sin(a)
    // stays far below 2^31.
    return (across + ((reach * sin_q15) / cos_q15));
}

/*!
 * @brief How much further to the side the robot would need to be, at
 *        the obstacle's face, after turning out by angle_deg: the worse
 *        of the front corner and, if one sticks out on that side, S3's
 *        barcode sensor's tip.
 *
 * @param[in] angle_deg Turn-out angle.
 * @param[in] shift_mm  Sideways shift of the axle (the profile's).
 * @param[in] extra_mm  What sticks out beyond the body on the obstacle's
 *                      side.
 * @param[in] face_mm   Obstacle's face ahead of the rear axle.
 *
 * @return mm; 0 or less: both clear.
 */
static int32_t
shortfall_mm (int32_t angle_deg, int32_t shift_mm, int32_t extra_mm,
              int32_t face_mm)
{
    int32_t short_mm = (shift_mm - HALF_WIDTH_MM)
                       - point_out_mm(angle_deg, face_mm,
                                      S5_AXLE_TO_FRONT_MM, HALF_WIDTH_MM);

    if (extra_mm > 0)
    {
        int32_t tip_mm = (shift_mm - (HALF_WIDTH_MM + extra_mm))
                         - point_out_mm(angle_deg, face_mm,
                                        S5_SIDE_EXTRA_AHEAD_MM,
                                        HALF_WIDTH_MM + extra_mm);

        if (tip_mm > short_mm)
        {
            short_mm = tip_mm;
        }
    }

    return (short_mm);
}

/*!
 * @brief The smallest turn-out angle at which the front corner and the
 *        barcode sensor clear the obstacle (see "Turn-out angle" above),
 *        and any reverse needed first.
 *
 * @param[in]  shift_mm  Sideways shift of the axle (the profile's).
 * @param[in]  extra_mm  What sticks out beyond the body on the
 *                       obstacle's side.
 * @param[in]  face_mm   Obstacle's face ahead of the rear axle.
 * @param[out] p_room_mm Reverse needed first, 0 ... ROOM_MAX_MM.
 *
 * @return Turn-out angle, S5_PLAN_OUT_MIN_DEG ... S5_PLAN_OUT_MAX_DEG.
 */
static int32_t
choose_angle (int32_t shift_mm, int32_t extra_mm, int32_t face_mm,
              int32_t * p_room_mm)
{
    // NOTE: The casts are safe: the distances from the axle are a few
    // hundred mm, so the squares are far below 2^31 and the roots below
    // 2^16.
    int32_t  angle    = S5_PLAN_OUT_MIN_DEG;
    int32_t  tip_side = HALF_WIDTH_MM + extra_mm;
    uint32_t swing_sq = (uint32_t) ((S5_AXLE_TO_FRONT_MM
                                     * S5_AXLE_TO_FRONT_MM)
                                    + (HALF_WIDTH_MM * HALF_WIDTH_MM));
    uint32_t tip_sq   = (uint32_t) ((S5_SIDE_EXTRA_AHEAD_MM
                                     * S5_SIDE_EXTRA_AHEAD_MM)
                                    + (tip_side * tip_side));
    int32_t  room_mm  = 0;
    int32_t  short_mm = 0;

    // Turning on the spot swings the side facing the obstacle forward:
    // the front corner, or further, the barcode sensor's tip.
    if (tip_sq > swing_sq)
    {
        swing_sq = tip_sq;
    }

    int32_t swing_mm = (int32_t) s5_math_isqrt(swing_sq) + TURN_MARGIN_MM;

    while ((angle < S5_PLAN_OUT_MAX_DEG)
           && (shortfall_mm(angle, shift_mm, extra_mm, face_mm) > 0))
    {
        angle++;
    }

    short_mm = shortfall_mm(angle, shift_mm, extra_mm, face_mm);

    if (short_mm > 0)
    {
        // Too close even at the steepest angle: back off far enough. At
        // a fixed angle a point gains tan(a) across per mm further.
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
 * @brief The drive after turning back parallel, before the first look:
 *        it puts the obstacle's face level with the sensor when the
 *        robot has turned S5_LOOK_DEG to look, so the first look already
 *        falls on the obstacle's side (silence in front of the face would
 *        read as "passed").
 *
 * @param[in] face_mm   Obstacle's face ahead of the rear axle when
 *                      turning out.
 * @param[in] angle_deg Turn-out angle.
 * @param[in] out_mm    Out leg.
 *
 * @return mm, 0 ... LEG_MAX_MM.
 */
static uint16_t
look_leg_mm (int32_t face_mm, int32_t angle_deg, uint16_t out_mm)
{
    // NOTE: The cast widens uint16_t, so no value is lost.
    int32_t gone_mm   = s5_math_mul_q15((int32_t) out_mm,
                                        s5_math_cos_q15(angle_deg));
    int32_t sensor_mm = s5_math_mul_q15(SENSOR_AHEAD_MM,
                                        s5_math_cos_q15(S5_LOOK_DEG));
    int32_t leg_mm    = ((face_mm - gone_mm) - sensor_mm) + LOOK_SPARE_MM;

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

    // NOTE: The cast is safe: 0 ... LEG_MAX_MM.
    return ((uint16_t) leg_mm);
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
