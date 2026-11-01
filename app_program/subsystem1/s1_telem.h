/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_telem.h
 *
 * @brief The part of telemetry_task that needs no kernel: a received
 *        command turned into its ack, the heartbeat packed, and the
 *        heartbeat schedule.
 *
 * @par Why it is separate
 * telemetry_task.c makes every kernel and driver call. This module only
 * works on the values it is given, so the host test S1-U14 can build and
 * run it with a stub in place of subsystem5_command_post().
 *
 * @par What an ack means (Stage 4a)
 * Result MSG_ACK_ACCEPTED means the command was queued for S5
 * (subsystem5_command_post() returned E_OK). vehicle_ctrl_task decides
 * later, on core 1, whether to carry it out, and S1 cannot see that
 * verdict. The dashboard labels this ack "queued", not "started".
 * Result MSG_ACK_REJECTED means the codec refused the command, or the
 * post failed (S5's command queue was full or missing).
 *
 * @par Thread safety
 * Every function works only on the structs it is given. The counters in
 * s1_telem_stats_t are written by telemetry_task alone.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0), like the rest of
 * the MQTT link, so builds without MQTT are unchanged.
 */

#ifndef S1_TELEM_H
#define S1_TELEM_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "msg_layout.h"
#include "mqtt_link.h"

/* Heartbeat period, in milliseconds (1 Hz, docs/S1_binary_frame.md
 * section 4). The schedule is absolute: deadline n is start + n periods.
 */
#define S1_TELEM_HB_PERIOD_MSEC     1000U

/* Payload room in one s1_telem_pub_t: the larger of the ack and the
 * heartbeat. s1_telem.c checks both fit at compile time.
 */
#define S1_TELEM_PUB_MAX            MSG_HB_BYTES

/** Posts one decoded command to S5. Returns 0 when it was queued, or a
 *  negative kernel error code when it was not.
 */
typedef int32_t (* s1_telem_post_fn_t)(uint8_t opcode, uint8_t cmd_id);

/** Counters kept by telemetry_task. Each one saturates at UINT32_MAX. */
typedef struct
{
    uint32_t    codec_reject;       /* Commands the codec refused.        */
    uint32_t    post_failure;       /* Posts S5's command queue refused.  */
    uint32_t    ack_publish_failed; /* Acks mqtt_link_publish() refused.  */
    uint32_t    hb_published;       /* Heartbeats queued for publishing.  */
    uint32_t    hb_publish_failed;  /* Heartbeats mqtt_link_publish()     */
                                    /* refused.                           */
    uint32_t    hb_skipped;         /* Heartbeats not sent: no session.   */
    uint32_t    rx_error;           /* Kernel errors while receiving.     */
} s1_telem_stats_t;

/** One message for mqtt_link_publish(): its topic, how it is sent and
 *  its bytes.
 */
typedef struct
{
    mqtt_link_topic_t   topic;
    mqtt_link_qos_t     qos;
    bool                b_retain;
    uint16_t            len;
    uint8_t             payload[S1_TELEM_PUB_MAX];
} s1_telem_pub_t;

/** What the heartbeat needs from outside this module. */
typedef struct
{
    uint32_t    uptime_s;           /* Seconds since boot.                */
    uint32_t    connect_count;      /* mqtt_link status connect_count.    */
    uint32_t    command_dropped;    /* mqtt_link status command_dropped.  */
} s1_telem_hb_in_t;

uint32_t s1_telem_sat_add(uint32_t a, uint32_t b);
bool     s1_telem_on_command(const mqtt_link_command_t * p_cmd,
                             s1_telem_post_fn_t post,
                             s1_telem_stats_t * p_stats,
                             s1_telem_pub_t * p_ack);
bool     s1_telem_heartbeat(uint8_t seq, const s1_telem_hb_in_t * p_in,
                            const s1_telem_stats_t * p_stats,
                            s1_telem_pub_t * p_hb);
bool     s1_telem_hb_due(uint32_t now_msec, uint32_t * p_next_msec);
bool     s1_telem_hb_should_publish(mqtt_link_state_t state, bool b_stale);

#endif /* TM_MQTT */

#endif /* S1_TELEM_H */

/*** end of file ***/
