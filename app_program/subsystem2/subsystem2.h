/** @file subsystem2.h
 *
 * @brief Subsystem 2 -- motion control. Student: Arif, 2501261.
 *
 * @par
 * Public interface of subsystem 2: what other subsystems may use. Other
 * subsystems include it as "../subsystem2/subsystem2.h". The section
 * marked "Private to subsystem 2" is for S2's own files only.
 *
 * @par Units and signs
 * Distance mm, angle degrees, speed mm/s, duty permille. A positive
 * distance drives forward. A positive TURN angle and a positive steering
 * correction both turn the robot left (the right wheel runs faster).
 *
 * @par Kernel objects (created in subsystem2_init)
 * mbf_motion (IF-04 commands), flg_motion (IF-05 MF_ESTOP latch, plus a
 * private 20 ms tick bit), mtx_odom (IF-06 odometry and status),
 * mtx_steer (IF-02 steering) and cyc_motion (20 ms period). Except for
 * the legacy flg_motion below, their IDs stay inside subsystem2.c;
 * other subsystems use the functions below (BARR-C 4.2c).
 *
 * @par Deviations from BARR-C:2018 (same as S5; approval: project
 *      manager, TBD)
 * - micro T-Kernel types (ID, ER, INT, UINT, T_xxx) appear only where a
 *   kernel call or the task entry signature requires them.
 * - The template's build uses -std=gnu11 rather than C99 (rule 1.1a).
 * - The template's file names subsystem1 ... subsystem5 are not unique in
 *   their first 8 characters (rule 4.1b).
 * - Legacy section: flg_motion is declared extern (rule 4.2c) and ODOM
 *   is not a lower-case _t name (rule 5.1a). Both stay only until S5's
 *   obstacle_task uses subsystem2_estop_set() and subsystem2_odom_get().
 * - task_producer.c / .h are the course template's example, unchanged
 *   apart from moving its objects there; they are deleted with the S3 /
 *   S4 examples.
 *
 * @par Abbreviations used in S2 (BARR-C 1.5b)
 * cmd: command; cyc: cyclic handler; dist: distance; enc: encoder;
 * ff: feed-forward; flg: event flag; mbf: message buffer; mtx: mutex;
 * mms: mm/s; odom: odometry; pri: priority; um: micrometre; us: microsecond.
 */

#ifndef SUBSYSTEM2_H
#define SUBSYSTEM2_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "task_producer.h"  /* Template example; see the note above.    */

/* ---------------------------------------------------------------------
 *  IF-04 mbf_motion: commands to motion_task, sent by vehicle_ctrl_task
 *  (S5) with subsystem2_motion_send(). The message buffer copies each
 *  command, so the sender may reuse its variable at once.
 * --------------------------------------------------------------------- */

/** Command types. */
typedef enum
{
    S2_CMD_STOP = 0,    /* Brake now and cancel any MOVE / TURN / FOLLOW. */
    S2_CMD_FOLLOW,      /* Drive at speed_mms, steered by line_task.      */
    S2_CMD_MOVE,        /* Drive value mm straight; negative = reverse.   */
    S2_CMD_TURN,        /* Turn on the spot value degrees; + = left.      */
    S2_CMD_CLEAR        /* Leave the E-stop, once MF_ESTOP is cleared.    */
} subsystem2_cmd_type_t;

/** One command. */
typedef struct
{
    uint8_t  type;      /* subsystem2_cmd_type_t.                          */
    int16_t  value;     /* MOVE: mm. TURN: degrees. Otherwise 0.           */
    uint16_t speed_mms; /* 0 = default. FOLLOW / MOVE: robot speed; TURN:  */
                        /* wheel speed. A new FOLLOW only changes speed.   */
} subsystem2_cmd_t;

#define S2_SPEED_CRUISE_MMS     (300)   /* FOLLOW / MOVE default (App. D).*/
#define S2_SPEED_TURN_MMS       (150)   /* TURN default wheel speed.      */
#define S2_SPEED_MAX_MMS        (500)   /* Larger requests are clipped.   */
#define S2_CMD_QUEUE_LEN        (8u)    /* Commands waiting at most.      */

/* ---------------------------------------------------------------------
 *  IF-05 flg_motion: the emergency-stop latch. S4 and S5 set it with
 *  subsystem2_estop_set(); motion_task brakes at once and stays braked
 *  (ESTOP_BRAKED). Only vehicle_ctrl_task clears it, then sends
 *  S2_CMD_CLEAR. Commands sent before the CLEAR are discarded; commands
 *  sent after it run in order.
 * --------------------------------------------------------------------- */
#define MF_ESTOP                (0x00000001u)

/* ---------------------------------------------------------------------
 *  IF-06 odometry, published by motion_task every 20 ms.
 * --------------------------------------------------------------------- */
typedef struct
{
    uint32_t count_l;       /* Encoder ticks since power-up (unsigned).   */
    uint32_t count_r;
    int32_t  dist_mm;       /* Signed distance of the robot centre.       */
    int32_t  speed_mms;     /* Robot speed, positive = forward.           */
    int32_t  speed_l_mms;   /* Signed wheel speeds, for the turn rate.    */
    int32_t  speed_r_mms;
    int32_t  dist_l_mm;     /* Signed distance of each wheel.             */
    int32_t  dist_r_mm;
    int32_t  heading_cdeg;  /* Heading from the wheels, 1/100 degree,     */
                            /* + = left of the power-up heading,          */
                            /* -17999 ... 18000.                          */
} subsystem2_odom_t;

/** motion_task states (Week 6 report 6.3). STALL_FAULT is not a state
 *  of its own: a stall sets MF_ESTOP and ends in S2_STATE_ESTOP. */
typedef enum
{
    S2_STATE_IDLE = 0,      /* IDLE_BRAKED.                                */
    S2_STATE_FOLLOWING,     /* FOLLOWING.                                  */
    S2_STATE_MOVING,        /* MOVING_DIST.                                */
    S2_STATE_TURNING,       /* TURNING.                                    */
    S2_STATE_ESTOP          /* ESTOP_BRAKED.                               */
} subsystem2_state_t;

/** Live status of motion_task, for telemetry (S1) and the S2 tests. */
typedef struct
{
    subsystem2_state_t state;
    int32_t  duty_l;            /* Last output, permille, signed.          */
    int32_t  duty_r;
    int32_t  target_l_mms;      /* Wheel speed set-points.                 */
    int32_t  target_r_mms;
    uint32_t done_count;        /* MOVE / TURN commands completed.         */
    uint32_t estop_count;       /* Entries into ESTOP_BRAKED.              */
    uint32_t estop_brake_us;    /* 1 MHz timer when it last braked for     */
                                /* the E-stop (S2-U05).                    */
    uint32_t stall_count;       /* Stalls detected.                        */
    uint32_t discarded_count;   /* Commands dropped in ESTOP_BRAKED.       */
    uint32_t cycle_count;       /* 20 ms cycles run.                       */
    uint32_t cycle_max_us;      /* Longest cycle (NFR11).                  */
} subsystem2_status_t;

/* Public functions: any task may call these, unless noted. */
ER   subsystem2_motion_send(subsystem2_cmd_t const * p_cmd);   /* S5.    */
ER   subsystem2_stop(void);                                     /* S5.    */
ER   subsystem2_follow(uint16_t speed_mms);                     /* S5.    */
ER   subsystem2_move(int16_t dist_mm, uint16_t speed_mms);      /* S5.    */
ER   subsystem2_turn(int16_t angle_deg, uint16_t speed_mms);    /* S5.    */
ER   subsystem2_clear(void);                                    /* S5.    */
void subsystem2_estop_set(void);                    /* S4, S5.           */
void subsystem2_estop_clear(void);                  /* S5 vehicle_ctrl.  */
bool subsystem2_estop_is_set(void);
void subsystem2_odom_get(subsystem2_odom_t * p_odom);
void subsystem2_status_get(subsystem2_status_t * p_status);
void subsystem2_steer_set(int16_t dv_mms);          /* S3 line_task.     */

/* Chassis. Distance per tick = circumference / ticks per revolution; only
 * that ratio matters, so the circumference is the calibration value.
 * Calibrated on the floor (S2-U11, 2026-10-23, log 2026-10-23_s2_u11_3):
 * with 63 mm and 639 / 629 ticks the robot went 5.4 % (left) and 3.9 %
 * (right) further than it counted, so it curved right (4-9 deg per metre).
 * Both wheels fit 639 ticks and a rolling diameter of about 66.5 mm. The
 * u11 run of s2_test.ps1 prints new values for these two lines.
 * Overflow: 50 ticks x 209010 and 639 x 209010 fit 32 bits easily. */
#define S2_TRACK_MM             (112)   /* Centre of tyre to centre.      */
#define S2_TICKS_PER_REV_L      (639)   /* Debounced rising edges.        */
#define S2_TICKS_PER_REV_R      (639)
#define S2_WHEEL_CIRC_UM_L      (208620)    /* 66.41 mm rolling diameter. */
#define S2_WHEEL_CIRC_UM_R      (209010)    /* 66.53 mm rolling diameter. */

/* ---------------------------------------------------------------------
 *  Legacy names used by S5 (origin/buddy5-ultrasonic, obstacle_task.c).
 *  TODO: Remove once S5 switches to subsystem2_estop_set() and
 *  subsystem2_odom_get().
 * --------------------------------------------------------------------- */
extern ID flg_motion;
typedef subsystem2_odom_t ODOM;
void s2_odom_get(subsystem2_odom_t * p_odom);

/* ---------------------------------------------------------------------
 *  Private to subsystem 2
 * --------------------------------------------------------------------- */
#define S2_PRI_MOTION           (9)     /* Week 6 report section 3.       */
#define S2_MOTION_PERIOD_MS     (20u)

/* motion_task's stack: TASK_STACK_SIZE (2048) plus the kernel's
 * DEFAULT_SYS_STKSZ (128 on the Cortex-M0+), as app_start_task() gives. */
#define S2_MOTION_STACK_BYTES   (2176u)

uint32_t subsystem2_stack_used(void);
void     subsystem2_floor_test(void);       /* make TEST=1 only.          */
void     subsystem2_step_ready(char const * p_next);    /* TEST=1 only.  */
void     subsystem2_cycle_wait(bool b_estop_wakes);
bool     subsystem2_cmd_receive(subsystem2_cmd_t * p_cmd);
void     subsystem2_odom_put(subsystem2_odom_t const * p_odom,
                             subsystem2_status_t const * p_status);
int16_t  subsystem2_steer_get(void);

#endif /* SUBSYSTEM2_H */

/*** end of file ***/
