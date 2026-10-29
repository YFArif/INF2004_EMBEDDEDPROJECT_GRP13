/** @file s3_config.h
 *
 * @brief Subsystem 3 -- build-time settings: priorities, periods, limits
 *        and the test / debug switches. Student: Sean (Buddy 3).
 *
 * @par
 * Pins and ADC channels are NOT here: each driver header owns its own
 * (drv_adc.h for the two line IR sensors, drv_barcode.h). Line-following
 * thresholds and gains are in s3_line.h, barcode decoding limits in
 * s3_code39.h and s3_analog.h.
 *
 * @par Switches for the main branch
 * S3_DRIVE_TEST must be 0 in main. The debug prints may stay on: they
 * only print, they never change what the robot does.
 */

#ifndef S3_CONFIG_H
#define S3_CONFIG_H

#include "s3_line.h"        /* S3_LN_L_ON_LEFT / S3_LN_L_ON_RIGHT.      */

/* ---------------------------------------------------------------------
 *  Line sensor layout (v29 EDGE PAIR, s3_line.h). One sensor on EACH
 *  edge of the 18 mm line, 18 mm apart centre to centre, the pair
 *  centred on the car. Centred, both read GREY (about 500):
 *
 *        white  L|##################|R  white     (looking down, car
 *                <------ 18 mm ----->              pointing up the page)
 *
 *  On our car the GP26 ("Left IR") sensor is on the robot's RIGHT and
 *  GP27 ("Centre IR") on its LEFT. If they are mounted the other way
 *  round, set S3_LN_L_ON_LEFT here. Nothing else changes. Wrong setting:
 *  the car steers AWAY from the line at once.
 * --------------------------------------------------------------------- */
#ifndef S3_IR_L_SIDE
#define S3_IR_L_SIDE            (S3_LN_L_ON_RIGHT)
#endif
#define S3_IR_SPACING_MM        (18)    /* Mounting: centre to centre.   */
#define S3_LAYOUT_TXT           ((S3_LN_L_ON_LEFT == S3_IR_L_SIDE) ? \
                                 "edge pair 18 mm, GP26 LEFT" :        \
                                 "edge pair 18 mm, GP26 RIGHT")

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
#ifndef S3_DRIVE_TEST
#define S3_DRIVE_TEST           (0)
#endif
#define S3_LINE_VERBOSE         (!S3_DRIVE_TEST) /* Drive task owns logs. */
#ifndef S3_BC_VERBOSE
#define S3_BC_VERBOSE           (0)     /* 1 = every barcode attempt;    */
                                        /* 0 = only "S3 NAV" lines.      */
#endif
#ifndef S3_TIMING
#define S3_TIMING               (!S3_DRIVE_TEST) /* Quiet drive prompts. */
                                        /* sampler interrupt and line    */
                                        /* step, plus each decode time.  */
#endif
#define S3_TIMING_PRINT_MS      (5000u)

/* ---------------------------------------------------------------------
 *  TEST ONLY: line following on Buddy 2's motors before the Vehicle
 *  Controller exists (task_drive_test.c). Keep 0 in main.
 * --------------------------------------------------------------------- */
#define S3_DRIVE_LOST_MM        (300u)  /* Stop after LOST this far.     */
#define S3_DRIVE_LOST_MS        (2000u) /* ... or LOST this long (a car  */
                                        /* that stopped never got far).  */
#define S3_DRIVE_STILL_MS       (2000u) /* Stop if the odometry has not  */
                                        /* changed this long.            */
#define S3_DRIVE_MAX_MM         (3000u) /* Stop after this distance.     */
#define S3_DRIVE_BTN1_PIN       (20u)   /* Button 1 (GP20): PRESS = start */
                                        /* (no hold); any press while    */
                                        /* driving = stop. Read only.    */
#define S3_DRIVE_BTN2_PIN       (21u)   /* Button 2 (GP21): press = next */
                                        /* gain preset.                  */
#define S3_DRIVE_RECAL_MS       (2000u) /* Hold Button 2 this long while */
                                        /* waiting = recalibrate (after  */
                                        /* the countdown below).         */
#define S3_DRIVE_CAL_COUNT_S    (10u)   /* Countdown before calibration  */
                                        /* starts (power-up, recal, or   */
                                        /* START while not READY).       */
#define S3_DRIVE_GO_COUNT_S     (3u)    /* Countdown after START.        */
#define S3_DRIVE_LOG_MS         (100u)  /* "S3 LOG" line while driving.  */
#define S3_DRIVE_CAL_BLACK_MIN  (2500u) /* Refuse to start until BOTH    */
                                        /* sensors' black level is at    */
                                        /* least this (raw). Car 28 Oct: */
                                        /* full sweep 2955 ... 3939; a   */
                                        /* half sweep was "valid" at     */
                                        /* 2099 and kept rising while    */
                                        /* driving.                      */

#endif /* S3_CONFIG_H */

/*** end of file ***/
