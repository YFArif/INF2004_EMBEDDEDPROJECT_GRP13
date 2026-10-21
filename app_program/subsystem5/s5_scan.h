/** @file s5_scan.h
 *
 * @brief Coarse-then-fine servo scan of an obstacle (FR10, Week 6
 *        report 6.4 COARSE_SCAN and FINE_SCAN).
 *
 * @par
 * The scan decides where to point the sensor; the caller decides how to
 * point it. Each reading goes through an s5_scan_measure_fn_t supplied
 * by the caller (on the robot: move the servo, wait, ping). This keeps
 * the scan free of hardware calls, so it can be tested on a PC with a
 * simulated obstacle.
 */

#ifndef S5_SCAN_H
#define S5_SCAN_H

#include <stdint.h>

#include "s5_profile.h"

#define S5_SCAN_COARSE_COUNT    (5u)    /* 30, 60, 90, 120, 150 degrees. */
#define S5_SCAN_MAX_POINTS      (20u)   /* 5 coarse + 2 x (6 fine + 1).  */

/** Result of one reading requested by the scan. */
typedef enum
{
    S5_SCAN_STEP_OK = 0,        /* Measured; the range may be no echo.   */
    S5_SCAN_STEP_ABORT,         /* A hold request cancelled the scan.    */
    S5_SCAN_STEP_FAULT          /* The sensor has failed.                */
} s5_scan_step_t;

/** Overall result of a scan. */
typedef enum
{
    S5_SCAN_DONE = 0,           /* Obstacle found; edges are filled in.  */
    S5_SCAN_CLEAR,              /* No hit in the coarse scan.            */
    S5_SCAN_ABORTED,            /* Cancelled; the result is incomplete.  */
    S5_SCAN_FAULT               /* Sensor fault; the result is invalid.  */
} s5_scan_outcome_t;

/** Everything one scan measured. */
typedef struct
{
    s5_profile_point_t  points[S5_SCAN_MAX_POINTS];
    uint32_t            count;      /* Valid entries in points.          */
    s5_profile_edge_t   left;       /* Left (higher angle) edge.         */
    s5_profile_edge_t   right;      /* Right (lower angle) edge.         */
} s5_scan_result_t;

/** Point the sensor at angle_deg and measure; NO_ECHO if nothing. */
typedef s5_scan_step_t (* s5_scan_measure_fn_t)(int16_t angle_deg,
                                                uint16_t * p_dist_mm);

s5_scan_outcome_t s5_scan_run(s5_scan_measure_fn_t p_measure,
                              s5_scan_result_t * p_result);

#endif /* S5_SCAN_H */

/*** end of file ***/
