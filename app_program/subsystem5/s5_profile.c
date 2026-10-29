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
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_profile.h"

#define HALF_PATH_MM    ((S5_ROBOT_WIDTH_MM / 2) + S5_PATH_MARGIN_MM)
#define MM_MAX          (0xFFFE)    /* Largest storable range in mm.      */
#define AHEAD_DEG       (90)

static bool     point_has_echo(s5_profile_point_t const * p_point);
static uint16_t clamp_mm(int32_t value_mm, int32_t max_mm);
static void     correct_edges(s5_profile_edge_t const * p_left,
                              s5_profile_edge_t const * p_right,
                              s5_profile_point_t * p_left_pt,
                              s5_profile_point_t * p_right_pt);
static void     find_closest(s5_profile_point_t const * p_points,
                             uint32_t count, int32_t right_deg,
                             int32_t left_deg, s5_profile_t * p_out);
static bool     is_path_blocked(s5_profile_point_t const * p_points,
                                uint32_t count);
static uint16_t side_clearance(s5_profile_point_t const * p_points,
                               uint32_t count,
                               s5_profile_point_t const * p_edge,
                               int32_t edge_raw_deg, bool b_left);

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
 * @param[out] p_out    The profile. seq is left as it was and depth_mm is
 *                      reset to 0, because neither comes from the scan.
 */
void
s5_profile_build (s5_profile_point_t const * p_points, uint32_t count,
                  s5_profile_edge_t const * p_left,
                  s5_profile_edge_t const * p_right,
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
    s5_profile_to_xy(&left_pt, &left_x, &left_y);
    s5_profile_to_xy(&right_pt, &right_x, &right_y);

    // Step 2: Width is the distance between the two edge points.
    int32_t  dx_mm   = left_x - right_x;
    int32_t  dy_mm   = left_y - right_y;

    // NOTE: Both casts are safe: with ranges below 5 m the sum of squares
    // is never negative and stays below 2^31, so its root is below 2^16.
    uint32_t sum_sq  = (uint32_t) ((dx_mm * dx_mm) + (dy_mm * dy_mm));

    p_out->width_mm = clamp_mm((int32_t) s5_math_isqrt(sum_sq), MM_MAX);

    // Step 3: Closest point and whether anything blocks the path.
    find_closest(p_points, count, p_right->last_hit.angle_deg,
                 p_left->last_hit.angle_deg, p_out);
    p_out->b_on_path = is_path_blocked(p_points, count);

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
                                              true);

        // The robot's right side must end up left of the left edge.
        p_out->shift_left_mm = clamp_mm(HALF_PATH_MM - left_x, MM_MAX);
    }

    if (p_right->b_found)
    {
        p_out->clear_right_mm = side_clearance(p_points, count, &right_pt,
                                               p_right->last_hit.angle_deg,
                                               false);

        // The robot's left side must end up right of the right edge.
        p_out->shift_right_mm = clamp_mm(HALF_PATH_MM + right_x, MM_MAX);
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
    p_out->closest_mm     = S5_PROFILE_NO_ECHO;
    p_out->closest_deg    = AHEAD_DEG;
    p_out->width_mm       = 0u;
    p_out->clear_left_mm  = S5_CLEAR_OPEN_MM;
    p_out->clear_right_mm = S5_CLEAR_OPEN_MM;
    p_out->shift_left_mm  = 0u;
    p_out->shift_right_mm = 0u;
    p_out->depth_mm       = 0u;
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
 * @param[in] max_mm   Upper limit, at most MM_MAX.
 *
 * @return The limited value.
 */
static uint16_t
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

    // NOTE: limited is 0 ... MM_MAX here, so it fits in uint16_t.
    return ((uint16_t) limited);
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
 * @param[out] p_out     closest_mm and closest_deg are written.
 */
static void
find_closest (s5_profile_point_t const * p_points, uint32_t count,
              int32_t right_deg, int32_t left_deg, s5_profile_t * p_out)
{
    p_out->closest_mm  = S5_PROFILE_NO_ECHO;
    p_out->closest_deg = 0;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        s5_profile_point_t const * p_pt = &p_points[idx];

        if ((point_has_echo(p_pt))
            && (p_pt->angle_deg >= right_deg)
            && (p_pt->angle_deg <= left_deg)
            && (p_pt->dist_mm < p_out->closest_mm))
        {
            p_out->closest_mm  = p_pt->dist_mm;
            p_out->closest_deg = p_pt->angle_deg;
        }
    }
}

/*!
 * @brief Tell whether any hit lies in the robot's path.
 *
 * @param[in] p_points Scan readings.
 * @param[in] count    Number of readings.
 *
 * @return true if a hit is closer to the centre line than half the robot
 *         width plus the margin.
 */
static bool
is_path_blocked (s5_profile_point_t const * p_points, uint32_t count)
{
    bool b_blocked = false;

    for (uint32_t idx = 0u; idx < count; idx++)
    {
        if (s5_profile_is_hit(&p_points[idx]))
        {
            int32_t x_mm = 0;
            int32_t y_mm = 0;

            s5_profile_to_xy(&p_points[idx], &x_mm, &y_mm);

            if (s5_math_abs(x_mm) < HALF_PATH_MM)
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
static uint16_t
side_clearance (s5_profile_point_t const * p_points, uint32_t count,
                s5_profile_point_t const * p_edge, int32_t edge_raw_deg,
                bool b_left)
{
    int32_t edge_x  = 0;
    int32_t edge_y  = 0;
    int32_t gap_min = S5_CLEAR_OPEN_MM;

    s5_profile_to_xy(p_edge, &edge_x, &edge_y);

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

            s5_profile_to_xy(p_pt, &x_mm, &y_mm);

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
