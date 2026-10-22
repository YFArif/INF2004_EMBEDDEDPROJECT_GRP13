/** @file s5_event.h
 *
 * @brief Subsystem 5 event reporting (FR16) and console trace.
 *
 * @par Event record (S1 msg_layout.h, msg_event_t)
 * One code byte plus two argument bytes.
 * - Obstacle events 0x50-0x58 (s5_event_post): args[0] = distance in cm
 *   (0 to 255, clamped), args[1] = servo angle in degrees (0 to 180). The
 *   distance is the range, except the width for S5_EVENT_PROFILE and the
 *   depth for S5_EVENT_OBST_PASSED.
 * - Controller events 0x60-0x65 (s5_event_post_args): the two argument
 *   bytes are listed next to each code below.
 *
 * @par Code ranges proposed to the team
 * 0x10-0x1F S1, 0x20-0x2F S2, 0x30-0x3F S3, 0x40-0x4F S4, 0x50-0x6F S5.
 * 0x00 is never used.
 *
 * TODO: S1's mbf_event is not built yet (its Stage 3). Until then
 * s5_event_post() prints the event on the console; s5_event.c is the
 * only file to change when mbf_event arrives.
 */

#ifndef S5_EVENT_H
#define S5_EVENT_H

#include <stdint.h>

/** Event codes owned by subsystem 5. */
typedef enum
{
    S5_EVENT_OBST_NEAR    = 0x50,   /* Under 35 cm: slow down.           */
    S5_EVENT_OBST_CLEAR   = 0x51,   /* 40 cm or more again.              */
    S5_EVENT_OBST_STOP    = 0x52,   /* Under 15 cm: MF_ESTOP set.        */
    S5_EVENT_SCAN_START   = 0x53,   /* Coarse scan started.              */
    S5_EVENT_PROFILE      = 0x54,   /* Profile ready (width).            */
    S5_EVENT_SCAN_CLEAR   = 0x55,   /* Scan found nothing in range.      */
    S5_EVENT_SCAN_ABORT   = 0x56,   /* Scan cancelled by a hold.         */
    S5_EVENT_OBST_PASSED  = 0x57,   /* Side watch: obstacle passed.      */
    S5_EVENT_SENSOR_FAULT = 0x58,   /* Echo stuck high or no answer.     */

    /* vehicle_ctrl_task. Arguments: {args[0], args[1]}. */
    S5_EVENT_CTRL_STATE   = 0x60,   /* {new state, old state}.           */
    S5_EVENT_AVOID_PLAN   = 0x61,   /* {s5_plan_action_t, clearance cm}. */
    S5_EVENT_AVOID_STEP   = 0x62,   /* {s5_avoid_state_t, 0}.            */
    S5_EVENT_CTRL_FAULT   = 0x63,   /* {s5_mission_fault_t, 0}.          */
    S5_EVENT_RUN_DONE     = 0x64,   /* {barcodes, run time in s, max 255}*/
    S5_EVENT_COMMAND      = 0x65    /* {opcode, 1 accepted / 0 refused}. */
} s5_event_code_t;

void s5_event_post(s5_event_code_t code, uint32_t dist_mm, int32_t angle_deg);
void s5_event_post_args(s5_event_code_t code, uint32_t arg_a, uint32_t arg_b);
void s5_event_trace(char const * p_label, int32_t value_a, int32_t value_b);
void s5_event_log_ping(int32_t servo_deg, int32_t range_mm, uint32_t echo_us,
                       char const * p_why);

#endif /* S5_EVENT_H */

/*** end of file ***/
