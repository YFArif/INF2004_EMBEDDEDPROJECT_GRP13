/** @file s5_profile.c
 *
 * @brief Obstacle profile from ultrasonic scan points (FR10).
 *
 * @par Method (Week 6 report 6.4)
 * 1. Each fine sweep stops at the first point that is no longer on the
 *    obstacle; the point before it is the edge's last hit.
 * 2. The 15 degree HC-SR04 beam sees an edge early, so each edge angle is
 *    moved inward by S5_EDGE_CORR_DEG (assumption A16). After the
 *    bisection reading in s5_scan.c the last hit is on average about 1
 *    degree inside the beam's outer limit, so the correction is half the
 *    beam (7.5) less that, rounded to 6 degrees. In a simulated scan of
 *    80 to 300 mm boxes this kept the width error within -5 to +8 %; a
 *    full half-beam correction without bisection was 7 to 22 % low.
 *    S5-U04 fits the real value on the robot.
 * 3. Width is the straight-line distance between the two corrected edge
 *    points (the same result as the law of cosines in the report).
 * 4. A side's clearance is the free sideways gap between that edge and
 *    the nearest other echo beside it, looking S5_CLEAR_DEPTH_MM past
 *    the edge. With nothing seen it is S5_CLEAR_OPEN_MM.
 * 5. A side's shift (how far the rear axle moves sideways to pass) is
 *    the corrected edge (step 2) plus s5_profile_pass_mm(): the robot's
 *    side (with S3's barcode sensor on the left) and S5_PATH_MARGIN_MM,
 *    or more where the look while passing needs room to turn. The
 *    untrimmed edge made every detour about 3 cm wider than needed
 *    (logs 2026-11-11_1744/1746). It was untrimmed
 *    after logs 2026-11-05_0124 and _0224, where trimmed edges let the
 *    robot's side hit; then the servo's recorded angles were wrong
 *    (drv_servo.h "Reach") and the robot did not square up first.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_profile.h"

#define AHEAD_DEG       (90)
#define PIVOT_AHEAD_MM  (S5_AXLE_TO_FRONT_MM - S5_SENSOR_SETBACK_MM)

/* The packed profile must be exactly three words (no padding), and the
 * field widths must hold their largest values. A wrong size stops the
 * build here (an array of size -1).
 */
typedef char s5_profile_size_check[(S5_PROFILE_BYTES
                                    == sizeof(s5_profile_t)) ? 1 : -1];
typedef char s5_profile_mm_check[(S5_PROFILE_MM_MASK
                                  == ((1u << S5_PROFILE_MM_BITS) - 1u))
                                 && (S5_CLEAR_OPEN_MM <= S5_PROFILE_MM_MAX)
                                 ? 1 : -1];
typedef char s5_profile_step_check[((uint32_t) (S5_PROFILE_ANGLE_MAX_DEG
                                                - S5_PROFILE_ANGLE_MIN_DEG)
                                    <= S5_PROFILE_STEP_MASK) ? 1 : -1];

static bool     point_has_echo(s5_profile_point_t const * p_point);
static uint32_t clamp_mm(int32_t value_mm, int32_t max_mm);
static void     correct_edges(s5_profile_edge_t const * p_left,
                              s5_profile_edge_t const * p_right,
                              s5_profile_point_t * p_left_pt,
                              s5_profile_point_t * p_right_pt);
static void     find_closest(s5_profile_point_t const * p_points,
                             uint32_t count, int32_t right_deg,
                             int32_t left_deg, s5_profile_t * p_out);
static bool     is_path_blocked(s5_profile_point_t const * p_points,
                                uint32_t count, int32_t body_deg);
static uint32_t side_clearance(s5_profile_point_t const * p_points,
                               uint32_t count,
                               s5_profile_point_t const * p_edge,
                               int32_t edge_raw_deg, bool b_left,
                               int32_t body_deg);
static void     point_xy(s5_profile_point_t const * p_point,
                         int32_t body_deg, int32_t * p_x_mm,
                         int32_t * p_y_mm);

/*!
 * @brief Tell whether a point is an obstacle hit (closer than
 *        S5_HIT_RANGE_MM).
 *
 * @param[in] p_point The reading to test.
 *
 * @return true for a hit.
 */
bool
s5_profile_is_hit (s5_profile_point_t const * p_point)
{
    return ((point_has_echo(p_point))
            && (p_point->dist_mm < S5_HIT_RANGE_MM));
}

/*!
 * @brief Tell whether the next sweep point still lies on the same object
 *        as the previous one.
 *
 * @param[in] prev_mm Range of the previous point on the object.
 * @param[in] p_next  The next reading in the sweep.
 *
 * @return true if the range changed by no more than S5_EDGE_JUMP_MM.
 */
bool
s5_profile_same_object (uint16_t prev_mm, s5_profile_point_t const * p_next)
{
    bool b_same = false;

    if (point_has_echo(p_next))
    {
        // NOTE: Both casts widen uint16_t to int32_t, so no value is lost.
        int32_t step_mm = (int32_t) p_next->dist_mm - (int32_t) prev_mm;

        b_same = (s5_math_abs(step_mm) <= S5_EDGE_JUMP_MM);
    }

    return (b_same);
}

/*!
 * @brief Convert a reading to robot-frame coordinates.
 *
 * @param[in]  p_point The reading; it must have an echo.
 * @param[out] p_x_mm  Sideways offset, positive to the right.
 * @param[out] p_y_mm  Distance ahead.
 */
void
s5_profile_to_xy (s5_profile_point_t const * p_point,
                  int32_t * p_x_mm, int32_t * p_y_mm)
{
    int32_t dist_mm = (int32_t) p_point->dist_mm;   // Widening, no loss.

    *p_x_mm = s5_math_mul_q15(dist_mm, s5_math_cos_q15(p_point->angle_deg));
    *p_y_mm = s5_math_mul_q15(dist_mm, s5_math_sin_q15(p_point->angle_deg));
}

/*!
 * @brief Build an obstacle profile from one scan.
 *
 * @param[in]  p_points All coarse and fine readings of the scan.
 * @param[in]  count    Number of readings in p_points.
 * @param[in]  p_left   Result of the left (higher angle) fine sweep.
 * @param[in]  p_right  Result of the right (lower angle) fine sweep.
 * @param[in]  body_deg The robot's body was turned this far (+ = left)
 *                      from the heading the profile is wanted in; 0 for
 *                      a normal scan (s5_avoid.c, "Seeing past a
 *                      corner"). Every point is turned back about the
 *                      rear axle first, so the edges, the shifts, the
 *                      clearances and the path check are in that heading
 *                      (the closest point's angle stays the servo's).
 * @param[out] p_out    The profile. seq is left as it was and depth_mm is
 *                      reset to 0, because neither comes from the scan.
 */
void
s5_profile_build (s5_profile_point_t const * p_points, uint32_t count,
                  s5_profile_edge_t const * p_left,
                  s5_profile_edge_t const * p_right, int32_t body_deg,
                  s5_profile_t * p_out)
{
    s5_profile_point_t left_pt  = p_left->last_hit;
    s5_profile_point_t right_pt = p_right->last_hit;
    int32_t            left_x   = 0;
    int32_t            left_y   = 0;
    int32_t            right_x  = 0;
    int32_t            right_y  = 0;

    // Step 1: Move each found edge inward by half the beam width.
    correct_edges(p_left, p_right, &left_pt, &right_pt);
    point_xy(&left_pt, body_deg, &left_x, &left_y);
    point_xy(&right_pt, body_deg, &right_x, &right_y);

    // Step 2: Width is the distance between the two edge points.
    int32_t  dx_mm   = left_x - right_x;
    int32_t  dy_mm   = left_y - right_y;

    // NOTE: Both casts are safe: with ranges below 5 m the sum of squares
    // is never negative and stays below 2^31, so its root is below 2^16.
    uint32_t sum_sq  = (uint32_t) ((dx_mm * dx_mm) + (dy_mm * dy_mm));

    p_out->width_mm = s5_profile_mm((int32_t) s5_math_isqrt(sum_sq))
                      & S5_PROFILE_MM_MASK;

    // Step 3: Closest point and whether anything blocks the path.
    find_closest(p_points, count, p_right->last_hit.angle_deg,
                 p_left->last_hit.angle_deg, p_out);
    p_out->b_on_path = is_path_blocked(p_points, count, body_deg);

    // Step 4: Free gap beside each edge; a side with no edge is unusable.
    p_out->b_left_edge    = p_left->b_found;
    p_out->b_right_edge   = p_right->b_found;
    p_out->clear_left_mm  = 0u;
    p_out->clear_right_mm = 0u;
    p_out->shift_left_mm  = 0u;
    p_out->shift_right_mm = 0u;

    if (p_left->b_found)
    {
        p_out->clear_left_mm = side_clearance(p_points, count, &left_pt,
                                              p_left->last_hit.angle_deg,
                                              true, body_deg)
                               & S5_PROFILE_MM_MASK;

        // The robot ends up left of the left edge, far enough out to
        // look at the obstacle on its right (s5_profile_pass_mm()).
        p_out->shift_left_mm = s5_profile_mm(s5_profile_pass_mm(false)
                                             - left_x)
                               & S5_PROFILE_MM_MASK;
    }
    else
    {
        // Still echoing at the servo's limit: assume the obstacle ends
        // S5_EDGE_UNSEEN_MM past that echo. The side stays unusable
        // (no edge) unless s5_avoid.c's look past the corner finds that
        // way free; then this shift is used.
        p_out->shift_left_mm = s5_profile_mm((s5_profile_pass_mm(false)
                                              - left_x)
                                             + S5_EDGE_UNSEEN_MM)
                               & S5_PROFILE_MM_MASK;
    }

    if (p_right->b_found)
    {
        p_out->clear_right_mm = side_clearance(p_points, count, &right_pt,
                                               p_right->last_hit.angle_deg,
                                               false, body_deg)
                                & S5_PROFILE_MM_MASK;

        // The robot ends up right of the right edge, far enough out to
        // look at the obstacle on its left.
        p_out->shift_right_mm = s5_profile_mm(s5_profile_pass_mm(true)
                                              + right_x)
                                & S5_PROFILE_MM_MASK;
    }
    else
    {
        // As on the left.
        p_out->shift_right_mm = s5_profile_mm((s5_profile_pass_mm(true)
                                               + right_x)
                                              + S5_EDGE_UNSEEN_MM)
                                & S5_PROFILE_MM_MASK;
    }

    p_out->depth_mm = 0u;
    p_out->b_valid  = true;
}

/*!
 * @brief Fill in the profile of a scan that found nothing in range.
 *
 * @param[out] p_out The profile: valid, nothing on the path, both sides
 *                   open. seq is left as it was.
 */
void
s5_profile_set_clear (s5_profile_t * p_out)
{
    p_out->b_valid        = true;
    p_out->b_on_path      = false;
    p_out->b_left_edge    = false;
    p_out->b_right_edge   = false;
    p_out->closest_mm     = S5_PROFILE_MM_MASK;     // 1023: nothing near.
    p_out->width_mm       = 0u;
    p_out->clear_left_mm  = S5_CLEAR_OPEN_MM;
    p_out->clear_right_mm = S5_CLEAR_OPEN_MM;
    p_out->shift_left_mm  = 0u;
    p_out->shift_right_mm = 0u;
    p_out->depth_mm       = 0u;
    s5_profile_set_angle(p_out, AHEAD_DEG);
}

/*!
 * @brief Limit a millimetre value to what a profile distance field holds.
 *
 * @param[in] value_mm The value; negative values become 0.
 *
 * @return 0 ... S5_PROFILE_MM_MAX (1023 means 1023 mm or more).
 */
uint32_t
s5_profile_mm (int32_t value_mm)
{
    return (clamp_mm(value_mm, S5_PROFILE_MM_MAX));
}

/*!
 * @brief Store the angle of the nearest point.
 *
 * @param[out] p_profile The profile.
 * @param[in]  angle_deg Servo angle; limited to the servo's reach
 *                       (S5_PROFILE_ANGLE_MIN_DEG ... _MAX_DEG).
 */
void
s5_profile_set_angle (s5_profile_t * p_profile, int32_t angle_deg)
{
    int32_t limited = angle_deg;

    if (limited < S5_PROFILE_ANGLE_MIN_DEG)
    {
        limited = S5_PROFILE_ANGLE_MIN_DEG;
    }
    else if (limited > S5_PROFILE_ANGLE_MAX_DEG)
    {
        limited = S5_PROFILE_ANGLE_MAX_DEG;
    }
    else
    {
        // Inside the servo's reach.
    }

    // NOTE: The cast is safe: limited - ANGLE_MIN_DEG is 0 ... 80 here.
    p_profile->closest_step = (uint32_t) (limited - S5_PROFILE_ANGLE_MIN_DEG)
                              & S5_PROFILE_STEP_MASK;
}

/*!
 * @brief Read the angle of the nearest point.
 *
 * @param[in] p_profile The profile.
 *
 * @return Servo angle in degrees, S5_PROFILE_ANGLE_MIN_DEG ... _MAX_DEG.
 */
int32_t
s5_profile_angle (s5_profile_t const * p_profile)
{
    // NOTE: The cast is safe: the field is 7 bits, at most 127.
    return ((int32_t) p_profile->closest_step + S5_PROFILE_ANGLE_MIN_DEG);
}

/*!
 * @brief The sequence number after seq, wrapping in the 5-bit field.
 *
 * @param[in] seq The current sequence number.
 *
 * @return (seq + 1) modulo 32.
 */
uint32_t
s5_profile_next_seq (uint32_t seq)
{
    return ((seq + 1u) & S5_PROFILE_SEQ_MASK);
}

/*!
 * @brief From the robot's centre line out to its widest point on one
 *        side, S3's barcode sensor included.
 *
 * @param[in] b_left true for the left side.
 *
 * @return mm: 125 on the left, 70 on the right.
 */
int32_t
s5_profile_half_mm (bool b_left)
{
    return ((S5_ROBOT_WIDTH_MM / 2) + (b_left ? S5_SIDE_EXTRA_LEFT_MM
                                              : S5_SIDE_EXTRA_RIGHT_MM));
}

/*!
 * @brief How far the robot's centre line passes from the obstacle's
 *        side: the side margin, and room to turn S5_LOOK_DEG towards the
 *        obstacle on the spot (about the rear axle's middle) and look.
 *
 * @param[in] b_box_left true if the obstacle is on the robot's left.
 *
 * @return mm: 164 with the obstacle on the right (the front corner's
 *         swing decides), 195 on the left (the barcode sensor's 55 mm
 *         plus the margin decide).
 *
 * NOTE: A point x ahead of the axle and y to the side ends up
 * y cos(a) + x sin(a) to the side after turning a towards that side. The
 * front corner and the barcode sensor's tip reach furthest; the body's
 * side behind the corner reaches less.
 */
int32_t
s5_profile_pass_mm (bool b_box_left)
{
    int32_t look_mm = s5_profile_look_reach_mm(b_box_left)
                      + S5_LOOK_SPARE_MM;
    int32_t pass_mm = s5_profile_half_mm(b_box_left) + S5_PATH_MARGIN_MM;

    return ((look_mm > pass_mm) ? look_mm : pass_mm);
}

/*!
 * @brief How far to the side the robot reaches when it has turned
 *        S5_LOOK_DEG towards the obstacle on the spot: the front corner
 *        or S3's barcode sensor's tip, whichever is further.
 *
 * @param[in] b_box_left true if the obstacle is on the robot's left.
 *
 * @return mm from the centre line it had before turning: 149 either
 *         side with the measured robot.
 */
int32_t
s5_profile_look_reach_mm (bool b_box_left)
{
    int32_t sin_q15 = s5_math_sin_q15(S5_LOOK_DEG);
    int32_t cos_q15 = s5_math_cos_q15(S5_LOOK_DEG);
    int32_t corner  = s5_math_mul_q15(S5_ROBOT_WIDTH_MM / 2, cos_q15)
                      + s5_math_mul_q15(S5_AXLE_TO_FRONT_MM, sin_q15);
    int32_t tip     = s5_math_mul_q15(s5_profile_half_mm(b_box_left),
                                      cos_q15)
                      + s5_math_mul_q15(S5_SIDE_EXTRA_AHEAD_MM, sin_q15);

    return ((tip > corner) ? tip : corner);
}

/*!
 * @brief A reading in the heading the profile is wanted in: robot-frame
 *        coordinates (s5_profile_to_xy()), turned back by body_deg about
 *        the rear axle.
 *
 * @param[in]  p_point  The reading; it must have an echo.
 * @param[in]  body_deg How far the body was turned, + = left.
 * @param[out] p_x_mm   Sideways offset from the servo, + = right.
 * @param[out] p_y_mm   Distance ahead of the servo.
 *
 * NOTE: Turned left by b, the robot's ahead is (-sin b, cos b) and its
 * right (cos b, sin b) in the wanted frame; the servo is
 * PIVOT_AHEAD_MM ahead of the axle it turned about.
 */
static void
point_xy (s5_profile_point_t const * p_point, int32_t body_deg,
          int32_t * p_x_mm, int32_t * p_y_mm)
{
    int32_t x_mm = 0;
    int32_t y_mm = 0;

    s5_profile_to_xy(p_point, &x_mm, &y_mm);

    if (0 != body_deg)
    {
        int32_t sin_q15 = s5_math_sin_q15(body_deg);
        int32_t cos_q15 = s5_math_cos_q15(body_deg);
        int32_t side_mm = x_mm;
        int32_t fwd_mm  = y_mm + PIVOT_AHEAD_MM;

        x_mm = s5_math_mul_q15(side_mm, cos_q15)
               - s5_math_mul_q15(fwd_mm, sin_q15);
        y_mm = (s5_math_mul_q15(side_mm, sin_q15)
                + s5_math_mul_q15(fwd_mm, cos_q15)) - PIVOT_AHEAD_MM;
    }

    *p_x_mm = x_mm;
    *p_y_mm = y_mm;
}

/*!
 * @brief Tell whether a reading received an echo.
 *
 * @param[in] p_point The reading.
 *
 * @return true if the range is valid.
 */
static bool
point_has_echo (s5_profile_point_t const * p_point)
{
    return (S5_PROFILE_NO_ECHO != p_point->dist_mm);
}

/*!
 * @brief Limit a millimetre value to 0 ... max_mm.
 *
 * @param[in] value_mm The value.
 * @param[in] max_mm   Upper limit, never negative.
 *
 * @return The limited value.
 */
static uint32_t
clamp_mm (int32_t value_mm, int32_t max_mm)
{
    int32_t limited = value_mm;

    if (limited < 0)
    {
        limited = 0;
    }
    else if (limited > max_mm)
    {
        limited = max_mm;
    }
    else
    {
        // Already inside the range.
    }

    // NOTE: The cast is safe: limited is 0 ... max_mm here.
    return ((uint32_t) limited);
}

/*!
 * @brief Apply the beam-width correction to the found edges.
 *
 * @param[in]  p_left     Left sweep result.
 * @param[in]  p_right    Right sweep result.
 * @param[out] p_left_pt  Corrected left edge point.
 * @param[out] p_right_pt Corrected right edge point.
 *
 * NOTE: If the corrected edges would cross (an obstacle narrower than
 * the beam), both are put at the midpoint angle instead.
 */
static void
correct_edges (s5_profile_edge_t const * p_left,
               s5_profile_edge_t const * p_right,
               s5_profile_point_t * p_left_pt,
               s5_profile_point_t * p_right_pt)
{
    int32_t left_deg  = p_left->last_hit.angle_deg;
    int32_t right_deg = p_right->last_hit.angle_deg;

    if (p_left->b_found)
    {
        left_deg -= S5_EDGE_CORR_DEG;
    }

    if (p_right->b_found)
    {
        right_deg += S5_EDGE_CORR_DEG;
    }

    if (left_deg < right_deg)
    {
        int32_t mid_deg = (p_left->last_hit.angle_deg
                           + p_right->last_hit.angle_deg) / 2;

        left_deg  = mid_deg;
        right_deg = mid_deg;
    }

    // NOTE: Both angles stay within the 0 to 180 degree servo range, so
    // they fit in int16_t.
    p_left_pt->angle_deg  = (int16_t) left_deg;
    p_right_pt->angle_deg = (int16_t) right_deg;
}

/*!
 * @brief Find the nearest reading between the two raw edges.
 *
 * @param[in]  p_points  Scan readings.
 * @param[in]  count     Number of readings.
 * @param[in]  right_deg Right edge angle (lower bound).
 * @param[in]  left_deg  Left edge angle (upper bound).
 * @param[out] p_out     closest_mm and the angle are written; with no
 *                       echo between the edges, 1023 mm straight ahead.
 */
static void
find_closest (s5_profile_point_t const * p_points, uint32_t count,
              int32_t right_deg, int32_t left_deg, s5_profile_t * p_out)
{
    uint32_t best_mm  = S5_PROFILE_NO_ECHO;
    int32_t  best_deg = AHEAD_DEG;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        s5_profile_point_t const * p_pt = &p_points[idx];

        if ((point_has_echo(p_pt))
            && (p_pt->angle_deg >= right_deg)
            && (p_pt->angle_deg <= left_deg)
            && (p_pt->dist_mm < best_mm))
        {
            best_mm  = p_pt->dist_mm;
            best_deg = p_pt->angle_deg;
        }
    }

    // NOTE: The cast is safe: best_mm is at most 0xFFFF.
    p_out->closest_mm = s5_profile_mm((int32_t) best_mm) & S5_PROFILE_MM_MASK;
    s5_profile_set_angle(p_out, best_deg);
}

/*!
 * @brief Tell whether any hit lies in the robot's path.
 *
 * @param[in] p_points Scan readings.
 * @param[in] count    Number of readings.
 *
 * @return true if a hit is closer to the centre line than the robot's
 *         side on that side (s5_profile_half_mm()) plus the margin.
 */
static bool
is_path_blocked (s5_profile_point_t const * p_points, uint32_t count,
                 int32_t body_deg)
{
    bool b_blocked = false;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (s5_profile_is_hit(&p_points[idx]))
        {
            int32_t x_mm = 0;
            int32_t y_mm = 0;

            point_xy(&p_points[idx], body_deg, &x_mm, &y_mm);

            // x is + to the right. The barcode sensor widens the left.
            int32_t half_mm = s5_profile_half_mm(x_mm < 0)
                              + S5_PATH_MARGIN_MM;

            if (s5_math_abs(x_mm) < half_mm)
            {
                b_blocked = true;
            }
        }
    }

    return (b_blocked);
}

/*!
 * @brief Free sideways gap between an edge and the next echo beyond it.
 *
 * @param[in] p_points     Scan readings.
 * @param[in] count        Number of readings.
 * @param[in] p_edge       Corrected edge point.
 * @param[in] edge_raw_deg Raw (uncorrected) edge angle; only readings
 *                         beyond it are considered.
 * @param[in] b_left       true for the left side, false for the right.
 *
 * @return The gap in mm, at most S5_CLEAR_OPEN_MM.
 */
static uint32_t
side_clearance (s5_profile_point_t const * p_points, uint32_t count,
                s5_profile_point_t const * p_edge, int32_t edge_raw_deg,
                bool b_left, int32_t body_deg)
{
    int32_t edge_x  = 0;
    int32_t edge_y  = 0;
    int32_t gap_min = S5_CLEAR_OPEN_MM;

    point_xy(p_edge, body_deg, &edge_x, &edge_y);

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        s5_profile_point_t const * p_pt = &p_points[idx];

        // Only readings on the far side of the edge can narrow the gap.
        bool b_beyond = b_left ? (p_pt->angle_deg > edge_raw_deg)
                               : (p_pt->angle_deg < edge_raw_deg);

        if (b_beyond && (point_has_echo(p_pt)))
        {
            int32_t x_mm = 0;
            int32_t y_mm = 0;

            point_xy(p_pt, body_deg, &x_mm, &y_mm);

            int32_t gap_mm = b_left ? (edge_x - x_mm) : (x_mm - edge_x);

            if ((y_mm <= (edge_y + S5_CLEAR_DEPTH_MM))
                && (gap_mm >= 0) && (gap_mm < gap_min))
            {
                gap_min = gap_mm;
            }
        }
    }

    return (clamp_mm(gap_min, S5_CLEAR_OPEN_MM));
}

/*** end of file ***/
