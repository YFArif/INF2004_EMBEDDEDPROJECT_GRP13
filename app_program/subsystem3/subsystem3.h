/** @file subsystem3.h
 *
 * @brief Subsystem 3 -- IR line following and barcode navigation.
 *        Student: Sean (Buddy 3).
 *
 * @par
 * Public interface of subsystem 3. Other subsystems include it as
 * "../subsystem3/subsystem3.h". Everything they may use is in s3_api.h;
 * task_consumer.h is the course template example (see below).
 *
 * @par Files (app_program/subsystem3/)
 * - s3_api.h / s3_api.c        public interface, kernel objects, status
 * - s3_config.h                priorities, periods, test / debug switches
 * - drv_ir_left.c / .h         outer ("Left") IR sensor (GP26, ADC0)
 * - drv_ir_centre.c / .h       Centre sensor (GP27, ADC1)
 * - drv_barcode.c / .h         barcode sensor (DOUT GP7, AOUT GP28)
 * - drv_adc.c / .h             ADC owner, 2 kHz sampler interrupt
 * - s3_ir_cal.c / .h           calibration of one line sensor (pure C)
 * - s3_line.c / .h             combiner: PD, junction, lost, search (pure C)
 * - s3_code39.c / .h           Code 39 decoder (pure C)
 * - s3_analog.c / .h           analogue barcode widths (pure C)
 * - s3_nav.c / .h              letter -> command map (pure C)
 * - task_line.c / .h           10 ms line task
 * - task_barcode.c / .h        set-up, barcode decode and publish
 * - task_drive_test.c / .h     TEST ONLY (S3_DRIVE_TEST)
 * - s3_private.h               helpers shared inside S3 only
 * "Pure C" files have no kernel calls and are unit-tested on a laptop
 * (testing/subsystem3/host/).
 *
 * @par Deviations from BARR-C:2018 (same kind as S2 and S5)
 * - micro T-Kernel types (ID, ER, INT, UW, H, UB, BOOL, T_xxx) appear only
 *   where a kernel call, a task entry, a kernel object ID or the interface
 *   to subsystem 2 requires them. s3_get_steer_sp() keeps BOOL / H / UB
 *   because subsystem2.c declares it with those types.
 * - The kernel object IDs mbf_nav, flg_line, flg_s3, mtx_s3 are declared
 *   extern in s3_api.h (rule 4.2c): the Vehicle Controller and Buddy 1
 *   must pass them to tk_rcv_mbf(), tk_set_flg() and tk_wai_flg().
 * - The packed bit-fields of s3_nav_msg_t and s3_line_status_t use UW /
 *   signed int (Change Requirement 1); their sizes are checked at compile
 *   time in s3_api.c.
 * - The template's build uses -std=gnu11 rather than C99 (rule 1.1a), and
 *   its file names subsystem1 ... subsystem5 are not unique in their first
 *   8 characters (rule 4.1b).
 * - task_consumer.c / .h and subsystem3_test.c are the course template's
 *   example, in the template's own style; they are deleted with the S2 /
 *   S4 examples.
 *
 * @par Abbreviations used in S3 (BARR-C 1.5b)
 * adc: analogue-to-digital converter; bc: barcode; c: Centre sensor; cal:
 * calibration; cmd: command; c39: Code 39; cyc: cyclic handler; dig:
 * digital; dur: duration; dv: wheel speed difference; flg: event flag; l:
 * Left IR; ln: line; mbf: message buffer; mms: mm/s; mtx: mutex; nav:
 * navigation; odom: odometry; pri: priority; sp: set point; um:
 * micrometre; us: microsecond; vc: Vehicle Controller.
 */

#ifndef SUBSYSTEM3_H
#define SUBSYSTEM3_H

#include "../app.h"
#include "s3_api.h"         /* Buddy 3 public interface.                */
#include "task_consumer.h"  /* Template example; see the note above.    */

#endif /* SUBSYSTEM3_H */

/*** end of file ***/
