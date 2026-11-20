/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_event.h
 *
 * @brief S1's event channel inside the robot: mbf_event (FR16,
 *        docs/S1_binary_frame.md section 6).
 *
 * @par What it is
 * Any task, on either core, reports an event with subsystem1_event_post()
 * (declared in subsystem1.h, the public header). The call never waits: it
 * puts one 3-byte record (msg_layout_event_t) into mbf_event, or counts a
 * drop if the buffer is full. telemetry_task takes the records with
 * s1_event_take() and publishes each one on <prefix>/event with a time
 * stamp.
 *
 * @par Sizes
 * MSG_EVENT_RECORD_BYTES is 3; the kernel stores each record with a 4-byte
 * length header and rounds the record up to 4, so one record costs 8 bytes.
 * S1_EVENT_DEPTH records = 30 x 8 = 240 bytes, the figure in
 * docs/S1_binary_frame.md section 9.2.
 *
 * @par Build gate
 * NOTE: s1_event_init(), s1_event_take() and s1_event_drop_count() exist
 * only in MQTT=1 builds (TM_MQTT). subsystem1_event_post() exists in every
 * build, so another subsystem can call it unconditionally; without MQTT it
 * does nothing and returns false.
 */

#ifndef S1_EVENT_H
#define S1_EVENT_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#include <stdbool.h>
#include <stdint.h>

#include "msg_layout.h"

/* Records mbf_event holds before a post is dropped. */
#define S1_EVENT_DEPTH              30U

#if TM_MQTT

/*!
 * @brief Create mbf_event. Called once, from subsystem1_init().
 *
 * @return 0, or the negative kernel error from tk_cre_mbf().
 */
int32_t  s1_event_init(void);

/*!
 * @brief Take the oldest event record without waiting.
 *
 * Only telemetry_task calls this.
 *
 * @param[out] p_event Receives the record.
 *
 * @return True if a record was taken; false if none was waiting, if the
 *         channel does not exist, or if p_event is NULL.
 */
bool     s1_event_take(msg_layout_event_t * p_event);

/*!
 * @brief Count of posts dropped since boot (full buffer, or no buffer).
 *
 * @return The count; it wraps after 2^32 drops.
 */
uint32_t s1_event_drop_count(void);

#endif /* TM_MQTT */

#endif /* S1_EVENT_H */

/*** end of file ***/
