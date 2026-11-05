/** @file s5_links.h
 *
 * @brief Subsystem 5's links to the other subsystems. Every call S5 makes
 *        into S1, S2, S3 or S4 goes through this one module, so a change
 *        in another subsystem changes only s5_links.c.
 *
 * @par Status of each link
 * - S2 motion (Arif): REAL. subsystem2_xxx().
 * - S3 line following (Sean): REAL. flg_line (IF-07), flg_s3 (IF-S3c),
 *   mbf_nav (IF-01) and the line status (IF-S3b) from s3_api.h. While
 *   S3's sensors are not calibrated, or the S5 floor tests switch the
 *   line off, s5_links_line_is_ready() is false and the controller takes
 *   the line as found where it is expected (s5_avoid.c, s5_mission.c).
 * - S4 IMU calibration (Kessler): STAND-IN. Reports "calibrated".
 * - S1 command acknowledgement (Caleb): STAND-IN. Prints the ack.
 */

#ifndef S5_LINKS_H
#define S5_LINKS_H

#include <stdbool.h>
#include <stdint.h>

#include "subsystem5.h"

/** Which side of the line the robot is on (the search direction). */
typedef enum
{
    S5_LINKS_SIDE_NONE = 0,     /* Not searching.                        */
    S5_LINKS_SIDE_LEFT,         /* Robot left of the line: it is right.  */
    S5_LINKS_SIDE_RIGHT         /* Robot right of the line: it is left.  */
} s5_links_side_t;

/* Start-up. */
bool    s5_links_tasks_allowed(void);

/* S2: motion. */
void    s5_links_motion_release(void);
void    s5_links_motion_stop(void);
void    s5_links_motion_settle(void);
void    s5_links_motion_follow(uint16_t speed_mms);
uint16_t s5_links_follow_speed_mms(void);
void    s5_links_motion_move(int32_t dist_mm, uint16_t speed_mms);
void    s5_links_motion_turn(int32_t angle_deg, uint16_t speed_mms);
void    s5_links_motion_estop(void);
int32_t s5_links_odom_dist_mm(void);

/* S3: line following and barcodes. */
void            s5_links_line_use(bool b_used);
bool            s5_links_line_is_used(void);
bool            s5_links_line_is_ready(void);
void            s5_links_line_recalibrate(void);
void            s5_links_line_enable(bool b_enable);
void            s5_links_line_search(s5_links_side_t side);
s5_links_side_t s5_links_line_side(void);
bool            s5_links_line_sees_black(void);
uint32_t        s5_links_line_events(void);
bool            s5_links_nav_take(subsystem5_nav_msg_t * p_msg);

/* S4: IMU. */
bool    s5_links_imu_cal_done(void);

/* S1: commands. */
void    s5_links_ack(uint8_t opcode, uint8_t cmd_id, bool b_accepted);

#endif /* S5_LINKS_H */

/*** end of file ***/
