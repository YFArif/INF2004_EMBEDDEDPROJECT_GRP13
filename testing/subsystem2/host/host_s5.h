/** @file host_s5.h
 *
 * @brief HOST TEST ONLY: the subsystem 5 names motion_task.c reports to,
 *        copied exactly from S5's subsystem5.h (origin/buddy5-ultrasonic),
 *        so the S2 host test (S2-U09) also checks the EV_MOTION_DONE and
 *        stall-fault hooks before S5's code is merged. Force-included with
 *        gcc -include.
 */

#ifndef HOST_S5_H
#define HOST_S5_H

#include <stdint.h>

#define EV_MOTION_DONE      (0x00000100u)   /* S2: MOVE or TURN done.    */
#define EV_FAULT            (0x00000400u)   /* Any: see flg_fault.       */
#define S5_FAULT_STALL      (0x01u)     /* S2: wheel stall, latched.     */

void subsystem5_mission_signal(uint32_t events);
void subsystem5_fault_set(uint32_t faults);

#endif /* HOST_S5_H */

/*** end of file ***/
