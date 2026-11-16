/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file telemetry_task.h
 *
 * @brief S1's telemetry_task, Stage 4a: dashboard commands (START, STOP,
 *        RESET) forwarded to S5 with an ack, and the 1 Hz heartbeat.
 *
 * @par Scheduling (Week 6 report section 3)
 * Priority 14 on core 0 (processor 1), with the radio service task: below
 * the WiFi task (4), imu_task (10) and obstacle_task (11), and off core 1
 * so that nothing S1 does around the network can delay a core-1 control
 * task (NFR9). Its own 2048-byte stack (NFR11: high-water at most 75 %).
 *
 * @par Loop
 * The task blocks in mqtt_link_receive_command() for at most
 * TELEMETRY_TASK_RX_WAIT_MSEC. A command wakes it at once, so the wait
 * adds nothing to a STOP's round trip (NFR15); it only bounds how late
 * the heartbeat deadline is checked when no command comes.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0), like the rest of
 * the MQTT link, so builds without MQTT are unchanged.
 */

#ifndef TELEMETRY_TASK_H
#define TELEMETRY_TASK_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdint.h>

#define TELEMETRY_TASK_PRI          14      /* Week 6 report section 3.  */
#define TELEMETRY_TASK_STACK_BYTES  2048U   /* app.h TASK_STACK_SIZE.    */
#define TELEMETRY_TASK_RX_WAIT_MSEC 50U     /* Longest wait per pass.    */

int32_t  telemetry_task_start(void);

#endif /* TM_MQTT */

#endif /* TELEMETRY_TASK_H */

/*** end of file ***/

