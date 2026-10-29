/** @file s5_links.h
 *
 * @brief Subsystem 5's links to the other subsystems. Every call S5 makes
 *        into S1, S2, S3 or S4 goes through this one module, so a change
 *        in another subsystem changes only s5_links.c.
 *
 * @par Status of each link
 * - S2 motion (Arif): REAL. subsystem2_xxx() from origin/buddy2-motion.
 * - S3 line following (Sean): STAND-IN. Prints what it would ask for.
 *   s5_links_line_is_real() is false, and the controller then treats the
 *   line as found where it is expected (see s5_avoid.c and s5_mission.c).
 * - S4 IMU calibration (Kessler): STAND-IN. Reports "calibrated".
 * - S1 command acknowledgement (Caleb): STAND-IN. Prints the ack.
 */

#ifndef S5_LINKS_H
#define S5_LINKS_H

#include <stdbool.h>
#include <stdint.h>

/* S2: motion. */
void    s5_links_motion_release(void);
void    s5_links_motion_stop(void);
void    s5_links_motion_follow(uint16_t speed_mms);
void    s5_links_motion_move(int32_t dist_mm, uint16_t speed_mms);
void    s5_links_motion_turn(int32_t angle_deg);
void    s5_links_motion_estop(void);
int32_t s5_links_odom_dist_mm(void);

/* S3: line following. */
bool    s5_links_line_is_real(void);
void    s5_links_line_enable(bool b_enable);
void    s5_links_line_search(bool b_search);

/* S4: IMU. */
bool    s5_links_imu_cal_done(void);

/* S1: commands. */
void    s5_links_ack(uint8_t opcode, uint8_t cmd_id, bool b_accepted);

#endif /* S5_LINKS_H */

/*** end of file ***/
