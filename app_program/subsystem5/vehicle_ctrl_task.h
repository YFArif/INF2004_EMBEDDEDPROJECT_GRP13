/** @file vehicle_ctrl_task.h
 *
 * @brief vehicle_ctrl_task: the vehicle controller (Week 6 report
 *        section 3: priority 7, core 1, event-driven). Runs the mission
 *        state machine (s5_mission.c), reads the two Robo Pico buttons and
 *        answers the dashboard's commands. It is the only sender of motion
 *        commands to S2 (IF-04).
 */

#ifndef VEHICLE_CTRL_TASK_H
#define VEHICLE_CTRL_TASK_H

#include "../app.h"

void vehicle_ctrl_task(INT stacd, void * p_exinf);

#endif /* VEHICLE_CTRL_TASK_H */

/*** end of file ***/
