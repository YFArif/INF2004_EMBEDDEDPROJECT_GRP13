/** @file s5_plan.h
 *
 * @brief Avoidance planning: from an obstacle profile, decide whether to
 *        continue, bypass left, bypass right or reverse and rescan, and
 *        work out the legs of the bypass (FR11, Week 6 report 6.4 PLAN).
 *
 * @par The bypass (seen from above, bypassing on the left)
 *   0. MOVE -room_mm, only if the obstacle is too close to turn out by
 *      at most S5_PLAN_OUT_MAX_DEG and still miss it.
 *   1. TURN +turn_deg (away from the obstacle; worked out from its width).
 *   2. MOVE out_mm: the whole robot is then S5_PATH_MARGIN_MM clear.
 *   3. TURN -turn_deg: parallel to the line again.
 *   4. MOVE alongside while obstacle_task watches the obstacle's side,
 *      until it has been passed.
 *   5. MOVE tail_mm: the obstacle is then behind the rear axle.
 *   6. TURN -in_deg (towards the line: the same angle as step 1).
 *   7. MOVE back_mm towards the line, S3 searching: the same distance
 *      as step 2, so on a straight track the robot ends on the line.
 *      As soon as S3 has the line (also during step 6), S3's line
 *      following takes over.
 *   8. Floor tests without S3 only: TURN +in_deg, along the line.
 * For the right side every angle has the opposite sign.
 *
 * @par
 * Pure computation: no kernel or hardware calls (tested on a PC and in
 * S5-U09).
 */

#ifndef S5_PLAN_H
#define S5_PLAN_H

#include <stdint.h>

#include "s5_profile.h"

#define S5_PLAN_OUT_MIN_DEG     (20)    /* Gentlest turn out.            */
#define S5_PLAN_OUT_MAX_DEG     (60)    /* Steeper: reverse first.       */

/** What to do about the obstacle. */
typedef enum
{
    S5_PLAN_CONTINUE = 0,   /* Not in the robot's path: drive on.        */
    S5_PLAN_LEFT,           /* Bypass on the left.                       */
    S5_PLAN_RIGHT,          /* Bypass on the right.                      */
    S5_PLAN_REVERSE         /* No side is wide enough: back off, rescan. */
} s5_plan_action_t;

/** The decision and the bypass legs. Lengths in mm, angles in degrees. */
typedef struct
{
    s5_plan_action_t    action;
    int16_t             turn_deg;       /* Step 1; + = left, - = right.  */
    int16_t             in_deg;         /* Step 8; equals turn_deg.      */
    uint16_t            out_mm;         /* Step 2.                       */
    uint16_t            tail_mm;        /* Step 5.                       */
    uint16_t            back_mm;        /* Step 7, before any search.    */
    uint16_t            clearance_mm;   /* Free gap on the chosen side.  */
    uint16_t            room_mm;        /* Step 0; 0 = no reverse.       */
} s5_plan_t;

void s5_plan_make(s5_profile_t const * p_profile, s5_plan_t * p_plan);

#endif /* S5_PLAN_H */

/*** end of file ***/
