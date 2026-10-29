/** @file s5_scan.c
 *
 * @brief Coarse-then-fine servo scan of an obstacle.
 *
 * @par Method
 * 1. Coarse scan at 30, 60, 90, 120 and 150 degrees.
 * 2. The hit nearest straight ahead is the obstacle. Neighbouring coarse
 *    hits at a similar range belong to it as well.
 * 3. One fine sweep per side, in 5 degree steps, from the outermost hit
 *    toward the next coarse angle. A sweep stops at the first reading
 *    that is no longer on the obstacle, so it is often shorter than six
 *    steps.
 * 4. One more reading halfway between the last hit and that first miss
 *    halves the angle uncertainty of the edge.
 *
 * NOTE: This replaces the "two sweeps of +/-15 degrees around the hit
 * sector" in Week 6 report 6.4. That window cannot reach the edges of an
 * obstacle wider than about 80 mm at a 150 mm stop. The two edge sweeps
 * stay inside the report's fine-scan budget of 2 x 7 steps.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s5_math.h"
#include "s5_profile.h"
#include "s5_scan.h"

#define COARSE_STEP_DEG     (30)
#define FINE_STEP_DEG       (5)
#define HALF_STEP_DEG       (2)     /* Bisection step, rounded down.     */
#define AHEAD_DEG           (90)
#define SERVO_RIGHT_END_DEG (0)
#define SERVO_LEFT_END_DEG  (180)
#define NO_INDEX            (-1)
#define DIR_RIGHT           (-1)    /* Angles decrease toward the right. */
#define DIR_LEFT            (1)

static const int16_t g_coarse_deg[S5_SCAN_COARSE_COUNT] =
{
    30, 60, 90, 120, 150
};

static s5_scan_step_t coarse_scan(s5_scan_measure_fn_t p_measure,
                                  s5_scan_result_t * p_result);
static int32_t        find_primary(s5_scan_result_t const * p_result);
static int32_t        extend_sector(s5_scan_result_t const * p_result,
                                    int32_t start_idx, int32_t dir);
static s5_scan_step_t fine_sweep(s5_scan_measure_fn_t p_measure,
                                 s5_scan_result_t * p_result,
                                 s5_profile_point_t start, int32_t dir,
                                 s5_profile_edge_t * p_edge);
static s5_scan_step_t refine_edge(s5_scan_measure_fn_t p_measure,
                                  s5_scan_result_t * p_result, int32_t dir,
                                  s5_profile_point_t * p_last);
static void           add_point(s5_scan_result_t * p_result,
                                s5_profile_point_t point);

/*!
 * @brief Run a full obstacle scan.
 *
 * @param[in]  p_measure Function that points the sensor and measures.
 * @param[out] p_result  All readings and both edges.
 *
 * @return S5_SCAN_DONE when both fine sweeps finished.
 */
s5_scan_outcome_t
s5_scan_run (s5_scan_measure_fn_t p_measure, s5_scan_result_t * p_result)
{
    s5_scan_outcome_t outcome = S5_SCAN_DONE;
    int32_t           primary = NO_INDEX;

    p_result->count         = 0u;
    p_result->left.b_found  = false;
    p_result->right.b_found = false;

    // Stage 1: Coarse scan; its points are entries 0 to 4.
    s5_scan_step_t step = coarse_scan(p_measure, p_result);

    if (S5_SCAN_STEP_OK == step)
    {
        primary = find_primary(p_result);
    }

    // Stage 2: Fine sweep outward from each end of the obstacle.
    if (S5_SCAN_STEP_ABORT == step)
    {
        outcome = S5_SCAN_ABORTED;
    }
    else if (S5_SCAN_STEP_FAULT == step)
    {
        outcome = S5_SCAN_FAULT;
    }
    else if (NO_INDEX == primary)
    {
        outcome = S5_SCAN_CLEAR;
    }
    else
    {
        int32_t right_idx = extend_sector(p_result, primary, DIR_RIGHT);
        int32_t left_idx  = extend_sector(p_result, primary, DIR_LEFT);

        step = fine_sweep(p_measure, p_result, p_result->points[right_idx],
                          DIR_RIGHT, &p_result->right);

        if (S5_SCAN_STEP_OK == step)
        {
            step = fine_sweep(p_measure, p_result,
                              p_result->points[left_idx], DIR_LEFT,
                              &p_result->left);
        }

        if (S5_SCAN_STEP_ABORT == step)
        {
            outcome = S5_SCAN_ABORTED;
        }
        else if (S5_SCAN_STEP_FAULT == step)
        {
            outcome = S5_SCAN_FAULT;
        }
        else
        {
            outcome = S5_SCAN_DONE;
        }
    }

    return (outcome);
}

/*!
 * @brief Measure at every coarse angle, right to left.
 *
 * @param[in]  p_measure Measurement function.
 * @param[out] p_result  Coarse points are appended as entries 0 to 4.
 *
 * @return S5_SCAN_STEP_OK, or the first abort or fault.
 */
static s5_scan_step_t
coarse_scan (s5_scan_measure_fn_t p_measure, s5_scan_result_t * p_result)
{
    s5_scan_step_t step = S5_SCAN_STEP_OK;

    for (uint32_t idx = 0u;
         (idx < S5_SCAN_COARSE_COUNT) && (S5_SCAN_STEP_OK == step);
         idx++)
    {
        s5_profile_point_t point;

        point.angle_deg = g_coarse_deg[idx];
        point.dist_mm   = S5_PROFILE_NO_ECHO;
        step            = p_measure(point.angle_deg, &point.dist_mm);
        add_point(p_result, point);
    }

    return (step);
}

/*!
 * @brief Pick the coarse hit nearest straight ahead.
 *
 * @param[in] p_result Scan result holding the five coarse points.
 *
 * @return Index of that point, or NO_INDEX if nothing was hit.
 */
static int32_t
find_primary (s5_scan_result_t const * p_result)
{
    int32_t best_idx = NO_INDEX;
    int32_t best_off = AHEAD_DEG + 1;   // Larger than any real offset.

    // NOTE: Both casts below are safe: the count is 5, and widening an
    // int16_t angle to int32_t loses nothing.
    for (int32_t idx = 0; idx < (int32_t) S5_SCAN_COARSE_COUNT; idx++)
    {
        s5_profile_point_t const * p_pt = &p_result->points[idx];
        int32_t offset = s5_math_abs((int32_t) p_pt->angle_deg - AHEAD_DEG);

        if ((s5_profile_is_hit(p_pt)) && (offset < best_off))
        {
            best_idx = idx;
            best_off = offset;
        }
    }

    return (best_idx);
}

/*!
 * @brief Grow the obstacle over neighbouring coarse hits at a similar
 *        range.
 *
 * @param[in] p_result  Scan result holding the coarse points.
 * @param[in] start_idx Index of the primary hit.
 * @param[in] dir       DIR_LEFT or DIR_RIGHT.
 *
 * @return Index of the outermost coarse point on the obstacle.
 */
static int32_t
extend_sector (s5_scan_result_t const * p_result, int32_t start_idx,
               int32_t dir)
{
    int32_t idx    = start_idx;
    bool    b_grow = true;

    while (b_grow)
    {
        int32_t next = idx + dir;

        // NOTE: The cast is safe: the count is 5.
        if ((next < 0) || (next >= (int32_t) S5_SCAN_COARSE_COUNT))
        {
            b_grow = false;
        }
        else if ((s5_profile_is_hit(&p_result->points[next]))
                 && (s5_profile_same_object(p_result->points[idx].dist_mm,
                                            &p_result->points[next])))
        {
            idx = next;
        }
        else
        {
            b_grow = false;
        }
    }

    return (idx);
}

/*!
 * @brief Sweep outward in fine steps until the obstacle ends.
 *
 * @param[in]  p_measure Measurement function.
 * @param[out] p_result  Fine points are appended.
 * @param[in]  start     Outermost coarse point on the obstacle.
 * @param[in]  dir       DIR_LEFT or DIR_RIGHT.
 * @param[out] p_edge    The edge found.
 *
 * @return S5_SCAN_STEP_OK, or the first abort or fault.
 *
 * NOTE: The sweep stops at the next coarse angle. If that is the servo's
 * end (0 or 180 degrees) and the obstacle still has not ended, the edge
 * is out of reach and p_edge->b_found is false.
 */
static s5_scan_step_t
fine_sweep (s5_scan_measure_fn_t p_measure, s5_scan_result_t * p_result,
            s5_profile_point_t start, int32_t dir, s5_profile_edge_t * p_edge)
{
    s5_scan_step_t     step     = S5_SCAN_STEP_OK;
    s5_profile_point_t prev     = start;
    int32_t            limit    = (int32_t) start.angle_deg
                                  + (dir * COARSE_STEP_DEG);
    int32_t            angle    = (int32_t) start.angle_deg
                                  + (dir * FINE_STEP_DEG);
    bool               b_on_obj = true;

    // NOTE: The two casts above widen int16_t; coarse angles are 30 to
    // 150, so limit is always inside the servo's 0 to 180 degree range.
    while ((S5_SCAN_STEP_OK == step) && b_on_obj
           && ((dir * (limit - angle)) >= 0))
    {
        s5_profile_point_t point;

        // NOTE: angle stays inside 0 to 180 here, so it fits in int16_t.
        point.angle_deg = (int16_t) angle;
        point.dist_mm   = S5_PROFILE_NO_ECHO;
        step            = p_measure(point.angle_deg, &point.dist_mm);

        if (S5_SCAN_STEP_OK == step)
        {
            add_point(p_result, point);
            b_on_obj = s5_profile_same_object(prev.dist_mm, &point);

            if (b_on_obj)
            {
                prev = point;
            }
        }

        angle += dir * FINE_STEP_DEG;
    }

    // Halve the 5 degree uncertainty with one reading between the last
    // hit and the first miss.
    if ((S5_SCAN_STEP_OK == step) && (!b_on_obj))
    {
        step = refine_edge(p_measure, p_result, dir, &prev);
    }

    // The edge is out of reach only if the sweep ran to the servo's end.
    bool b_at_end = (SERVO_RIGHT_END_DEG == limit)
                    || (SERVO_LEFT_END_DEG == limit);

    p_edge->last_hit = prev;
    p_edge->b_found  = (!b_on_obj) || (!b_at_end);

    return (step);
}

/*!
 * @brief Take one reading halfway between the last hit and the first
 *        miss, and keep it as the last hit if it is still on the obstacle.
 *
 * @param[in]     p_measure Measurement function.
 * @param[out]    p_result  The reading is appended.
 * @param[in]     dir       DIR_LEFT or DIR_RIGHT.
 * @param[in,out] p_last    Last hit; replaced if the midpoint is a hit.
 *
 * @return S5_SCAN_STEP_OK, or an abort or fault.
 */
static s5_scan_step_t
refine_edge (s5_scan_measure_fn_t p_measure, s5_scan_result_t * p_result,
             int32_t dir, s5_profile_point_t * p_last)
{
    s5_profile_point_t point;

    // NOTE: The last hit is at most 175 degrees from the right end, so
    // the midpoint stays inside 0 to 180 and fits in int16_t.
    point.angle_deg = (int16_t) (p_last->angle_deg + (dir * HALF_STEP_DEG));
    point.dist_mm   = S5_PROFILE_NO_ECHO;

    s5_scan_step_t step = p_measure(point.angle_deg, &point.dist_mm);

    if (S5_SCAN_STEP_OK == step)
    {
        add_point(p_result, point);

        if (s5_profile_same_object(p_last->dist_mm, &point))
        {
            *p_last = point;
        }
    }

    return (step);
}

/*!
 * @brief Append a point if there is room.
 *
 * @param[out] p_result Scan result.
 * @param[in]  point    The reading.
 */
static void
add_point (s5_scan_result_t * p_result, s5_profile_point_t point)
{
    if (p_result->count < S5_SCAN_MAX_POINTS)
    {
        p_result->points[p_result->count] = point;
        p_result->count++;
    }
}

/*** end of file ***/
