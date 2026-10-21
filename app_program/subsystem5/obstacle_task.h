/** @file obstacle_task.h
 *
 * @brief obstacle_task: owns the HC-SR04 (GP2/GP3, PWM slice 1) and the
 *        scanning servo (GP12, PWM slice 6). Detects obstacles ahead,
 *        stops the robot, profiles obstacles and watches them while the
 *        robot drives past (Week 6 report 6.4).
 *
 * @par
 * Other tasks never call this module. They send OB_xxx requests with
 * subsystem5_obst_request() and read results with
 * subsystem5_profile_get() and subsystem5_obst_status_get().
 */

#ifndef OBSTACLE_TASK_H
#define OBSTACLE_TASK_H

#include "../app.h"

void obstacle_task(INT stacd, void * p_exinf);

#endif /* OBSTACLE_TASK_H */

/*** end of file ***/
