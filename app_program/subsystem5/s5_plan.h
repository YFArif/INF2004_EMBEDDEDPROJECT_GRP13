/** @file s5_plan.h
 *
 * @brief Avoidance planning: from an obstacle profile, decide whether to
 *        continue, bypass left, bypass right or reverse and rescan, and
 *        work out the legs of the bypass (FR11, Week 6 report 6.4 PLAN).
 *
 * @par The bypass (seen from above, bypassing on the left)
 *   1. TURN +turn_deg (away from the obstacle).
 *   2. MOVE out_mm: the robot ends shift_mm to the side.
 *   3. TURN -turn_deg: parallel to the line again.
 *   4. MOVE alongside while obstacle_task watches the obstacle's side,
 *      until it has been passed (the obstacle's depth is measured here).
 *   5. MOVE tail_mm, so the robot's rear also clears the obstacle.
 *   6. TURN -turn_deg (toward the line).
 *   7. MOVE back_mm toward the line, looking for it.
 *   8. TURN +turn_deg: along the line, then line following resumes.
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

#define S5_PLAN_TURN_DEG        (45)    /* Week 6 report 6.4.            */

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
    uint16_t            out_mm;         /* Step 2.                       */
    uint16_t            tail_mm;        /* Step 5.                       */
    uint16_t            back_mm;        /* Step 7, before any search.    */
    uint16_t            clearance_mm;   /* Free gap on the chosen side.  */
} s5_plan_t;

void s5_plan_make(s5_profile_t const * p_profile, s5_plan_t * p_plan);

#endif /* S5_PLAN_H */

/*** end of file ***/
