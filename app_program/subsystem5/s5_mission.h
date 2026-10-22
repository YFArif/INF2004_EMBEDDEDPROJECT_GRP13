/** @file s5_mission.h
 *
 * @brief The mission state machine run by vehicle_ctrl_task (Week 6
 *        report section 5, Region A; FR5, FR6, FR11, FR13, FR15).
 *
 * @par States (subsystem5_ctrl_state_t)
 * INIT -> CALIBRATING -> READY -> LINE_FOLLOWING, then while running:
 * APPROACH_JUNCTION, EXECUTING_COMMAND and REACQUIRE_LINE after a
 * barcode; LINE_RECOVERY after line loss; OBSTACLE_AVOIDANCE after
 * obstacle_task stops the robot. FAULT and FINISHED end a run.
 */

#ifndef S5_MISSION_H
#define S5_MISSION_H

#include <stdbool.h>
#include <stdint.h>

#include "subsystem5.h"

void s5_mission_init(uint32_t now_ms);
void s5_mission_arm(bool b_armed);
void s5_mission_step(uint32_t events, uint32_t now_ms);
bool s5_mission_command(uint8_t opcode, uint32_t now_ms);
void s5_mission_calibrate(uint32_t now_ms);
bool s5_mission_is_running(void);

#endif /* S5_MISSION_H */

/*** end of file ***/
