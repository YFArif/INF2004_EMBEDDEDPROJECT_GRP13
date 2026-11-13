/** @file s5_avoid.h
 *
 * @brief Obstacle avoidance, run by vehicle_ctrl_task: scan, plan, then
 *        reverse and rescan, or bypass and rejoin the line (FR11; Week 6
 *        report 6.4 PLAN, BYPASS, REVERSE_RETRY and SEARCH_LINE).
 *
 * @par
 * s5_avoid_start() is called when obstacle_task has stopped the robot
 * (EV_OBSTACLE, MF_ESTOP set). s5_avoid_step() is then called with every
 * batch of mission events until it stops returning S5_AVOID_BUSY.
 */

#ifndef S5_AVOID_H
#define S5_AVOID_H

#include <stdbool.h>
#include <stdint.h>

#include "subsystem5.h"

/** Avoidance steps (logged as S5_EVENT_AVOID_STEP). */
typedef enum
{
    S5_AVOID_IDLE = 0,
    S5_AVOID_SCANNING,          /* Waiting for the obstacle profile.     */
    S5_AVOID_PROBE_TURN,        /* Turning to see past a corner.         */
    S5_AVOID_PROBE_BACK,        /* Turning back after that scan.         */
    S5_AVOID_REVERSING,         /* Backing off before a rescan.          */
    S5_AVOID_MAKE_ROOM,         /* Bypass step 0: room to turn out.      */
    S5_AVOID_TURN_OUT,          /* Bypass step 1.                        */
    S5_AVOID_CHECK_OUT,         /* Pinging the way out before driving it.*/
    S5_AVOID_CHECK_BACK,        /* Way out blocked: turning back.        */
    S5_AVOID_MOVE_OUT,          /* Step 2.                               */
    S5_AVOID_TURN_PARALLEL,     /* Step 3.                               */
    S5_AVOID_LOOK_STEP,         /* Step 4: alongside, to the next look.  */
    S5_AVOID_LOOK_TURN,         /* Turning towards the obstacle.         */
    S5_AVOID_LOOK_PINGS,        /* Listening for its side.               */
    S5_AVOID_LOOK_BACK,         /* Turning back along the track.         */
    S5_AVOID_RETURN,            /* Step 7: back to the line, searching.  */
    S5_AVOID_ONTO_LINE,         /* Line seen: rear axle onto it.         */
    S5_AVOID_ALIGN,             /* Turning forward until S3 sees it.     */
    S5_AVOID_REALIGN,           /* Step 8, floor tests only (no S3).     */
    S5_AVOID_DONE,
    S5_AVOID_FAILED
} s5_avoid_state_t;

/* Stored width (minimum-bit requirement): 21 states need 5 bits. */
#define S5_AVOID_STATE_BITS (5u)
#define S5_AVOID_STATE_MASK (0x1Fu)

/** What s5_avoid_step() reports. */
typedef enum
{
    S5_AVOID_BUSY = 0,          /* Still working.                        */
    S5_AVOID_RESUME,            /* Done: resume line following.          */
    S5_AVOID_PLAN_ONLY,         /* Bypass switched off: stop here.       */
    S5_AVOID_FAILURE,           /* Failed: see s5_avoid_fault().         */
    S5_AVOID_SEARCH             /* Back, but the line not found where it */
                                /* should be: search on the spot.        */
} s5_avoid_result_t;

void                    s5_avoid_set_bypass(bool b_enabled);
void                    s5_avoid_start(uint32_t now_ms);
s5_avoid_result_t       s5_avoid_step(uint32_t events, uint32_t now_ms);
subsystem5_ctrl_fault_t s5_avoid_fault(void);
bool                    s5_avoid_passed_left(void);
bool                    s5_avoid_forward_is_left(void);

#endif /* S5_AVOID_H */

/*** end of file ***/
