/** @file subsystem5.h
 *
 * @brief Subsystem 5 -- obstacle scanning, avoidance and the vehicle
 *        controller. Student: Anna, 2502447.
 *
 * @par
 * Public interface of subsystem 5: what other subsystems may use. Other
 * subsystems include it as "../subsystem5/subsystem5.h". The section
 * marked "Private to subsystem 5" is for S5's own files only.
 *
 * @par Tasks
 * obstacle_task (11, core 0): ultrasonic, servo, scan, profile.
 * vehicle_ctrl_task (7, core 1): the mission state machine, obstacle
 * avoidance, buttons, and the commands from the dashboard.
 *
 * @par Kernel objects (created in subsystem5_init)
 * flg_mission (IF-03), flg_fault (S1 fault_mask), flg_obst (requests to
 * obstacle_task), mtx_obst (IF-09 and the controller state) and mbf_cmd
 * (commands from S1). Their IDs stay inside subsystem5.c; other
 * subsystems use the functions below instead, so no variable is declared
 * extern here (BARR-C 4.2c). S3's objects (flg_line, flg_s3, mbf_nav)
 * are used only in s5_links.c.
 *
 * @par Deviations from BARR-C:2018 (approval: project manager, TBD)
 * - micro T-Kernel types (ID, ER, INT, UINT, TMO, T_xxx) appear only
 *   where a kernel call or the task entry signature requires them.
 * - The template's build uses -std=gnu11 rather than C99 (rule 1.1a);
 *   S5 code uses only C99 features.
 * - The template's file names subsystem1 ... subsystem5 are not unique in
 *   their first 8 characters (rule 4.1b).
 * - The per-ping log in obstacle_task.c is switched by LOG_PINGS rather
 *   than #ifndef NDEBUG (rule 2.1c.ii): the template defines NDEBUG in
 *   every USB-console build, which would remove the log entirely.
 *
 * @par Abbreviations used in S5 (BARR-C 1.5b)
 * cm, mm, deg, ms, us: units; cal: calibration; corr: correction;
 * dist: distance; ev: event; flg: event flag; hw: hardware; idx: index;
 * mbf: message buffer; mtx: mutex; obst: obstacle; pri: priority;
 * q15: fixed point with 15 fraction bits; req: request.
 */

#ifndef SUBSYSTEM5_H
#define SUBSYSTEM5_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "s5_profile.h"

/* Task priorities (Week 6 report section 3; TEAM.md section 4). */
#define S5_PRI_VEHICLE      (7)     /* vehicle_ctrl_task, core 1.        */
#define S5_PRI_OBSTACLE     (11)    /* obstacle_task, core 0.            */

/* ---------------------------------------------------------------------
 *  IF-03 flg_mission: events for vehicle_ctrl_task. Any subsystem sets
 *  them with subsystem5_mission_signal(). S3's events arrive on its own
 *  flg_s3 and are turned into EV_xxx bits by s5_links_line_events().
 * --------------------------------------------------------------------- */
#define EV_START            (0x00000001u)   /* S1 or GP20: start a run.  */
#define EV_STOP             (0x00000002u)   /* S1: stop the run.         */
#define EV_RESET            (0x00000004u)   /* S1: leave FAULT.          */
#define EV_CAL              (0x00000008u)   /* GP21: calibrate.          */
#define EV_BARCODE          (0x00000010u)   /* S3: barcode in mbf_nav.   */
#define EV_JUNCTION         (0x00000020u)   /* S3: junction under robot. */
#define EV_LINE_LOST        (0x00000040u)   /* S3: no line for 40 mm.    */
#define EV_LINE_FOUND       (0x00000080u)   /* S3: a search found it.    */
#define EV_MOTION_DONE      (0x00000100u)   /* S2: MOVE or TURN done.    */
#define EV_IMPACT           (0x00000200u)   /* S4: collision. Being      */
                                            /* removed from the brief    */
                                            /* (2026-11-11): EV_LIFTED.  */
#define EV_FAULT            (0x00000400u)   /* Any: see flg_fault.       */
#define EV_OBSTACLE_NEAR    (0x00000800u)   /* S5: under 35 cm, slow.    */
#define EV_OBSTACLE_CLEAR   (0x00001000u)   /* S5: path clear again.     */
#define EV_OBSTACLE         (0x00002000u)   /* S5: stopped, MF_ESTOP.    */
#define EV_PROFILE_READY    (0x00004000u)   /* S5: profile published.    */
#define EV_OBST_PASSED      (0x00008000u)   /* S5: side watch finished.  */
#define EV_COMMAND          (0x00010000u)   /* S1: a command is queued.  */
#define EV_LIFTED           (0x00020000u)   /* S4: lifted off the ground:*/
                                            /* the run ends (FINISHED).  */
#define EV_OVERTURNED       (0x00040000u)   /* S4: tipped over: FAULT.   */
#define EV_OBST_AGAIN       (0x00080000u)   /* S5: side watch heard the  */
                                            /* obstacle again after it   */
                                            /* was passed: longer tail.  */
#define EV_ALL              (0x000FFFFFu)

/* ---------------------------------------------------------------------
 *  flg_fault: fault causes. Bit n is fault n, the numbering of S1's
 *  telemetry fault_mask (msg_layout.h MSG_FAULT_xxx), so S1 copies it
 *  with no remapping. Kernel event flag: tk_set_flg and tk_clr_flg are
 *  atomic on both cores, so no bit is lost (Tribunal 1). Nobody waits on
 *  it with TWF_CLR, which would clear every bit.
 *
 *  Bit  Name      Set by                    Kind     Cleared by
 *  0    STALL     S2 motion_task            latched  vehicle_ctrl_task
 *  1    ENCODER   S2 (reserved)             latched  vehicle_ctrl_task
 *  2    IMU       S4 imu_task               latched  vehicle_ctrl_task
 *  3    NO_ECHO   S5 obstacle_task          latched  vehicle_ctrl_task
 *                 (3 unanswered pings)               (RESET)
 *  4    WIFI      S1 telemetry_task         level    S1 itself
 *  Latched bits are cleared only on RESET (s5_mission.c FAULTS_LATCHED),
 *  one mask at a time with tk_clr_flg.
 * --------------------------------------------------------------------- */
#define S5_FAULT_STALL      (0x01u)     /* S2: wheel stall, latched.     */
#define S5_FAULT_ENCODER    (0x02u)     /* S2: reserved.                 */
#define S5_FAULT_IMU        (0x04u)     /* S4: IMU fault, latched.       */
#define S5_FAULT_NO_ECHO    (0x08u)     /* S5: ultrasonic failed, latched*/
#define S5_FAULT_WIFI       (0x10u)     /* S1: link down, level.         */

/* ---------------------------------------------------------------------
 *  Requests to obstacle_task (flg_obst), sent with
 *  subsystem5_obst_request(). If several are pending, OB_HOLD (or else
 *  OB_RESUME) is carried out first, then the first of the others.
 * --------------------------------------------------------------------- */
#define OB_HOLD             (0x01u)     /* Stop pinging, centre servo.   */
#define OB_RESUME           (0x02u)     /* Back to forward monitoring.   */
#define OB_SCAN_REQ         (0x04u)     /* Coarse + fine scan, profile.  */
#define OB_WATCH_LEFT       (0x08u)     /* Watch the obstacle on the     */
#define OB_WATCH_RIGHT      (0x10u)     /* left/right until passed.      */
#define OB_CAL_SWEEP        (0x20u)     /* Slow servo sweep, S5-U03.     */
#define OB_ALL_REQUESTS     (0x3Fu)

/** Controller states, numbered as S1's telemetry ctrl_state field. */
typedef enum
{
    S5_CTRL_INIT = 0,
    S5_CTRL_CALIBRATING,
    S5_CTRL_READY,
    S5_CTRL_LINE_FOLLOWING,
    S5_CTRL_APPROACH_JUNCTION,
    S5_CTRL_EXECUTING_COMMAND,
    S5_CTRL_REACQUIRE_LINE,
    S5_CTRL_LINE_RECOVERY,
    S5_CTRL_OBSTACLE_AVOIDANCE,
    S5_CTRL_FAULT,
    S5_CTRL_FINISHED
} subsystem5_ctrl_state_t;

/* Stored widths (minimum-bit requirement): 11 states need 4 bits. */
#define S5_CTRL_STATE_BITS  (4u)
#define S5_CTRL_STATE_MASK  (0xFu)

/** Why the controller entered FAULT (S5_EVENT_CTRL_FAULT). */
typedef enum
{
    S5_CTRL_FAULT_NONE = 0,
    S5_CTRL_FAULT_IMPACT,           /* S4 collision.                     */
    S5_CTRL_FAULT_STALL,            /* S2 wheel stall.                   */
    S5_CTRL_FAULT_SENSOR,           /* Ultrasonic failed.                */
    S5_CTRL_FAULT_NO_PATH,          /* No side clear after 2 retries.    */
    S5_CTRL_FAULT_LINE_NOT_FOUND,   /* Line not found again.             */
    S5_CTRL_FAULT_MOTION_TIMEOUT,   /* A MOVE or TURN never finished.    */
    S5_CTRL_FAULT_SCAN_TIMEOUT,     /* No profile after a scan request.  */
    S5_CTRL_FAULT_OTHER,
    S5_CTRL_FAULT_OVERTURNED        /* S4: the robot tipped over.        */
} subsystem5_ctrl_fault_t;

/* 10 fault reasons need 4 bits. */
#define S5_CTRL_FAULT_BITS  (4u)
#define S5_CTRL_FAULT_MASK  (0xFu)

/* ---------------------------------------------------------------------
 *  IF-01: barcode commands, converted from S3's mbf_nav messages by
 *  s5_links_nav_take().
 * --------------------------------------------------------------------- */
typedef enum
{
    S5_NAV_LEFT = 0,
    S5_NAV_RIGHT,
    S5_NAV_STRAIGHT,
    S5_NAV_UTURN
} subsystem5_nav_cmd_t;

typedef struct
{
    uint8_t     cmd;            /* subsystem5_nav_cmd_t.                 */
    char        letter;         /* Decoded Code 39 character.            */
    bool        b_reversed;     /* Barcode was read right to left.       */
    uint32_t    t_ms;           /* When it was decoded.                  */
} subsystem5_nav_msg_t;

/* ---------------------------------------------------------------------
 *  Commands from the dashboard (S1, subsystem5_command_post). The
 *  opcodes are S1's MSG_OP_xxx values. vehicle_ctrl_task answers each
 *  one through S1's acknowledgement.
 * --------------------------------------------------------------------- */
#define S5_CMD_START        (1u)
#define S5_CMD_STOP         (2u)
#define S5_CMD_RESET        (3u)

/** obstacle_task states (Week 6 report 6.4). */
typedef enum
{
    S5_OBST_HOLD = 0,       /* Idle; servo ahead; waiting for a request. */
    S5_OBST_MONITORING,     /* Pinging ahead every 60 ms.                */
    S5_OBST_SLOW,           /* Obstacle under 35 cm; still pinging.      */
    S5_OBST_SCANNING,       /* Coarse and fine scan in progress.         */
    S5_OBST_WATCH,          /* Watching the obstacle beside the robot.   */
    S5_OBST_FAULT           /* Ultrasonic failed; waiting for OB_RESUME. */
} subsystem5_obst_state_t;

/* 6 states need 3 bits. */
#define S5_OBST_STATE_BITS  (3u)
#define S5_OBST_STATE_MASK  (0x7u)

/** Live ultrasonic status, updated after every ping. Packed into one
 *  32-bit word (minimum-bit requirement; was 12 B). Read the fields by
 *  name and never take their address. servo_deg stays in plain degrees
 *  (8 bits, 0 ... 180) so S1 copies it without an offset.
 */
typedef struct
{
    unsigned int state         : 3;     /* subsystem5_obst_state_t.      */
    unsigned int b_range_valid : 1;     /* False: no echo.               */
    unsigned int range_mm      : 10;    /* Last range if valid, 0 ...    */
                                        /* 1023 (1023: that far or more).*/
    unsigned int servo_deg     : 8;     /* Servo angle of the ping.      */
    unsigned int ping_count    : 10;    /* Pings, wrapping at 1024.      */
} subsystem5_obst_status_t;

#define S5_OBST_DEG_MASK    (0xFFu)
#define S5_OBST_COUNT_MASK  (0x3FFu)

/* Public functions: any task may call these. */
void     subsystem5_mission_signal(uint32_t events);
void     subsystem5_fault_set(uint32_t faults);
uint32_t subsystem5_fault_get(void);
void     subsystem5_obst_request(uint32_t requests);
void     subsystem5_profile_get(s5_profile_t * p_profile);
void     subsystem5_obst_status_get(subsystem5_obst_status_t * p_status);
ER       subsystem5_command_post(uint8_t opcode, uint8_t cmd_id);  /* S1. */
subsystem5_ctrl_state_t subsystem5_ctrl_state_get(void);

/* ---------------------------------------------------------------------
 *  Private to subsystem 5
 * --------------------------------------------------------------------- */

/* obstacle_task waits this long after power-up before it starts, so a
 * serial monitor opened after plugging in sees its first lines.
 * TODO: Reduce for the final demo; a robot should not sit blind for 5 s.
 */
#define S5_BOOT_WAIT_MS     (5000u)

bool     subsystem5_mission_wait(uint32_t events, uint32_t timeout_ms,
                                 uint32_t * p_events);
void     subsystem5_fault_clear(uint32_t faults);
uint32_t subsystem5_obst_take(bool b_wait);
bool     subsystem5_obst_pending(uint32_t requests);
void     subsystem5_profile_put(s5_profile_t const * p_profile);
void     subsystem5_obst_status_put(subsystem5_obst_status_t const * p_status);
void     subsystem5_delay_ms(uint32_t delay_ms);
uint32_t subsystem5_now_ms(void);
bool     subsystem5_command_take(uint8_t * p_opcode, uint8_t * p_cmd_id);
void     subsystem5_ctrl_state_put(subsystem5_ctrl_state_t state);

#endif /* SUBSYSTEM5_H */

/*** end of file ***/
