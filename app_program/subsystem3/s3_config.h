/** @file s3_config.h
 *
 * @brief Subsystem 3 -- build-time settings: priorities, periods, limits
 *        and the test / debug switches. Student: Sean (Buddy 3).
 *
 * @par
 * Pins and ADC channels are NOT here: each driver header owns its own
 * (drv_ir_left.h, drv_ir_centre.h, drv_barcode.h). Line-following
 * thresholds and gains are in s3_line.h, barcode decoding limits in
 * s3_code39.h and s3_analog.h.
 *
 * @par Switches for the main branch
 * S3_DRIVE_TEST must be 0 in main. The debug prints may stay on: they
 * only print, they never change what the robot does.
 */

#ifndef S3_CONFIG_H
#define S3_CONFIG_H

#include "s3_line.h"        /* S3_LN_EDGE_RIGHT / S3_LN_EDGE_LEFT.      */

/* ---------------------------------------------------------------------
 *  Edge the Centre sensor rides (v19 STRADDLE layout, s3_line.h).
 *  The Centre rides the edge AWAY from the outer ("Left IR") sensor, and
 *  the outer sensor sits just outside the other edge. On our car (Sean,
 *  photo 28 Oct) the outer sensor is on the robot's RIGHT, so the Centre
 *  rides the LEFT edge:      C |##########| O
 *  If the outer sensor is moved to the robot's LEFT, set
 *  S3_LN_EDGE_RIGHT here. Nothing else changes.
 * --------------------------------------------------------------------- */
#ifndef S3_EDGE_SIDE
#define S3_EDGE_SIDE            (S3_LN_EDGE_LEFT)
#endif
#define S3_EDGE_TXT             ((S3_LN_EDGE_RIGHT == S3_EDGE_SIDE) ? \
                                 "RIGHT" : "LEFT")

/* ---------------------------------------------------------------------
 *  Tasks (Week 6 report section 3, TEAM.md section 4). Both run on core 1
 *  (TP_PRC2), where the sampler interrupt is enabled. Core 1 order is
 *  rate-monotonic, shorter period = higher priority (1 = highest):
 *    vehicle_ctrl_task 7 (S5, sporadic) > task_line 8 (10 ms)
 *    > motion_task 9 (S2, 20 ms) > task_barcode 12 (event-driven).
 *  task_barcode can be low: the time-critical part (edge and sample
 *  capture) is done in the interrupts, and a decode has more than 1 s
 *  before the junction.
 * --------------------------------------------------------------------- */
#define S3_PRI_LINE             (8)     /* task_line: every 10 ms.       */
#define S3_PRI_BARCODE          (12)    /* task_barcode: set-up, decode. */
#define S3_PRI_DRIVE_TEST       (17)    /* Test only, core 0.            */

/* ---------------------------------------------------------------------
 *  Line following
 * --------------------------------------------------------------------- */
#define S3_LINE_PERIOD_MS       (10u)   /* Cyclic handler period.        */
#define S3_STEER_MAX_MMS        (200)   /* Default steer_sp at full lock */
                                        /* (s3_set_steer_max() changes   */
                                        /* it; keep it near the base     */
                                        /* speed: S2 applies 50 %, so    */
                                        /* full lock = base -/+ half).   */
                                        /* (+-1000 permille); <= 511.    */
#define S3_STEER_LIMIT_MMS      (511)   /* 10-bit signed field limit.    */
#define S3_ASSUMED_SPEED_MMS    (100u)  /* Distance estimate used only   */
                                        /* until S2 calls                */
                                        /* s3_set_odometry().            */
#define S3_ODOM_MAX_STEP_MM     (1000)  /* A bigger jump is a reset.     */
#define S3_CAL_RELAX_MS         (1000u) /* Calibration relax period.     */
#define S3_STATUS_PRINT_MS      (200u)  /* "S3 LINE" console period.     */

/* ---------------------------------------------------------------------
 *  Barcode
 * --------------------------------------------------------------------- */
#define S3_BC_POLL_MS           (50u)   /* task_barcode checks for a     */
                                        /* finished pass this often.     */
#define S3_BC_COOLDOWN_MS       (1500u) /* Ignore further barcodes this  */
                                        /* long after publishing one.    */
#define S3_NAV_MBF_DEPTH        (4u)    /* mbf_nav holds 4 messages.     */
#define S3_NAV_MBF_HEADER       (4u)    /* Kernel header per message.    */

/* ---------------------------------------------------------------------
 *  Debug prints (0 = off, 1 = on). Console only, behaviour unchanged.
 * --------------------------------------------------------------------- */
#define S3_LINE_VERBOSE         (1)     /* "S3 LINE" status + events.    */
#define S3_JUNC_DEBUG           (1)     /* One "S3 JDBG" line per pass   */
                                        /* of the outer sensor on black. */
#define S3_BC_VERBOSE           (0)     /* 1 = every barcode attempt;    */
                                        /* 0 = only "S3 NAV" lines.      */
#ifndef S3_TIMING
#define S3_TIMING               (1)     /* Every 5 s, "S3 TIME": longest */
                                        /* sampler interrupt and line    */
                                        /* step, plus each decode time.  */
#endif
#define S3_TIMING_PRINT_MS      (5000u)

/* ---------------------------------------------------------------------
 *  TEST ONLY: line following on Buddy 2's motors before the Vehicle
 *  Controller exists (task_drive_test.c). Keep 0 in main.
 * --------------------------------------------------------------------- */
#ifndef S3_DRIVE_TEST
#define S3_DRIVE_TEST           (0)
#endif
#define S3_DRIVE_LOST_MM        (300u)  /* Stop after LOST this far.     */
#define S3_DRIVE_LOST_MS        (2000u) /* ... or LOST this long (a car  */
                                        /* that stopped never got far).  */
#define S3_DRIVE_STILL_MS       (2000u) /* Stop if the odometry has not  */
                                        /* changed this long.            */
#define S3_DRIVE_MAX_MM         (3000u) /* Stop after this distance.     */
#define S3_DRIVE_MAX_MS         (30000u)/* Stop after this time.         */
#define S3_DRIVE_BTN1_PIN       (20u)   /* Robo Pico Button 1: start /   */
                                        /* stop (read only).             */
#define S3_DRIVE_BTN2_PIN       (21u)   /* Button 2: next gain preset.   */
#define S3_DRIVE_RECAL_MS       (4000u) /* Hold Button 1 this long while */
                                        /* waiting = recalibrate.        */
#define S3_DRIVE_CAL_BLACK_MIN  (2500u) /* Refuse to start until BOTH    */
                                        /* sensors' black level is at    */
                                        /* least this (raw). Car 28 Oct: */
                                        /* full sweep 2955 ... 3939; a   */
                                        /* half sweep was "valid" at     */
                                        /* 2099 and kept rising while    */
                                        /* driving.                      */

#endif /* S3_CONFIG_H */

/*** end of file ***/
