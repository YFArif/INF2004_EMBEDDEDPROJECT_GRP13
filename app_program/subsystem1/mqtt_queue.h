/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_queue.h
 *
 * @brief Kernel side of the MQTT link: the two message buffers.
 *
 * @par Why this is a separate module
 * lwIP's headers and <tk/tkernel.h> cannot be included in the same
 * translation unit, because they disagree about the type size_t. So
 * mqtt_link.c, which calls lwIP, includes no kernel header, and this
 * module, which calls the kernel, includes no lwIP header. They meet here,
 * through plain C types only.
 *
 * @par Private
 * Only mqtt_link.c uses this module. Applications use mqtt_link.h.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0) -- see mqtt_link.h.
 *
 * @par Channels
 * The publish channel carries messages from any application task to the
 * poll service. The command channel carries commands from the poll service
 * to the application dispatcher task. Each channel is a kernel message
 * buffer that holds exactly MQTT_LINK_QUEUE_DEPTH records. Every record is
 * stored at its full size, so the depth does not depend on how long each
 * payload is. No function here blocks except mqtt_queue_get_command(), and
 * only when the caller asks it to.
 *
 * @par Counting dropped publishes
 * A publish that finds its channel full is counted in a word that any task
 * on either core may increment. On a dual-core build the increment is
 * atomic_add(); on a single-core build it is made with task dispatching
 * disabled. The count is therefore exact on both.
 */

#ifndef MQTT_QUEUE_H
#define MQTT_QUEUE_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "mqtt_link.h"

/** One message waiting to be published.
 *
 * NOTE: This record never leaves the chip. Both cores use the same layout,
 * so any padding the compiler adds is harmless.
 */
typedef struct
{
    uint8_t     topic_id;                           /* mqtt_link_topic_t.   */
    uint16_t    payload_len;
    uint8_t     qos;                                /* 0 or 1.              */
    bool        b_retain;
    uint8_t     payload[MQTT_LINK_PAYLOAD_MAX];
} mqtt_queue_publish_t;

/**
 * @brief Create the two message buffers.
 *
 * Call once, from mqtt_link_init(). A later call after success does
 * nothing and returns success.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_queue_init(void);

/**
 * @brief Report whether mqtt_queue_init() has succeeded.
 *
 * Safe from any task on either core.
 *
 * @return True once the buffers exist.
 */
bool mqtt_queue_is_ready(void);

/**
 * @brief Put one message on the publish channel.
 *
 * Safe from any task on either core. It never blocks. When the channel is
 * full the message is dropped and counted; see mqtt_queue_full_count().
 *
 * @param[in] p_msg The message. Its payload_len must not exceed
 *                  MQTT_LINK_PAYLOAD_MAX.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_queue_put_publish(const mqtt_queue_publish_t * p_msg);

/**
 * @brief Take the oldest message from the publish channel.
 *
 * Called by the poll service only. It never blocks.
 *
 * @param[out] p_msg Receives the message.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_TIMEOUT when the channel is
 *         empty, or MQTT_LINK_ERR_NOT_READY or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_queue_get_publish(mqtt_queue_publish_t * p_msg);

/**
 * @brief Put one command on the command channel.
 *
 * Called by the poll service only. It never blocks. A full channel is
 * reported to the caller, which counts the drop in its own status.
 *
 * @param[in] p_command The command. Its payload_len must not exceed
 *                      MQTT_LINK_COMMAND_MAX.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_queue_put_command(const mqtt_link_command_t * p_command);

/**
 * @brief Take the oldest command from the command channel.
 *
 * Called by the application dispatcher task only.
 *
 * @param[out] p_command  Receives the command.
 * @param[in]  timeout_msec Wait limit in milliseconds. Zero polls without
 *                        waiting. MQTT_LINK_WAIT_FOREVER waits forever.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_TIMEOUT, MQTT_LINK_ERR_NOT_READY or
 *         MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_queue_get_command(mqtt_link_command_t * p_command,
                               uint32_t timeout_msec);

/**
 * @brief Read the number of publishes dropped because the channel was full.
 *
 * Safe from any task on either core.
 *
 * @return Drops since start-up.
 */
uint32_t mqtt_queue_full_count(void);

/**
 * @brief Read the kernel's own millisecond clock.
 *
 * Safe from any task on either core -- a plain kernel call (tk_get_otm()),
 * not an lwIP call, so it does not need to come from the radio service.
 * mqtt_link.c cannot include <tk/tkernel.h> (see this file's own "Why
 * this is a separate module" note), so this is where that call lives.
 * It wraps the exact same counter lwIP's own sys_now() wraps
 * (lib/liblwip/sysdepend/pico_rp2040/sys_arch.c), so a value from this
 * function and a value sys_now() returned are always directly comparable
 * -- there is one clock, read through two names depending on which side
 * of the lwIP/kernel split is calling.
 *
 * @return Milliseconds since boot, low 32 bits. Wraps about every 49.7
 *         days, the same documented behaviour as sys_now().
 */
uint32_t mqtt_queue_now_msec(void);

#endif /* TM_MQTT */

#endif /* MQTT_QUEUE_H */

/*** end of file ***/
