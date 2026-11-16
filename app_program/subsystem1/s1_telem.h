/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_telem.h
 *
 * @brief The part of telemetry_task that needs no kernel: a received
 *        command or map turned into its ack, the heartbeat packed, the
 *        schedules,
 *        the rules for keeping room in the publish queue, the telemetry and
 *        live messages filled in, and the WiFi fault rule.
 *
 * @par Why it is separate
 * telemetry_task.c makes every kernel and driver call. This module only
 * works on the values it is given, so the host test S1-U14 can build and
 * run it with stubs in place of subsystem5_command_post(),
 * s3_set_nav_map(), subsystem5_fault_set() and subsystem5_fault_clear().
 * It also calls S3's s3_code39_char(), which needs no kernel either.
 *
 * @par What an ack means (Stage 4a)
 * Result MSG_ACK_ACCEPTED means the command was queued for S5
 * (subsystem5_command_post() returned E_OK). vehicle_ctrl_task decides
 * later, on core 1, whether to carry it out, and S1 cannot see that
 * verdict. The dashboard labels this ack "queued", not "started".
 * Result MSG_ACK_REJECTED means the codec refused the command, or the
 * post failed (S5's command queue was full or missing).
 *
 * @par What an ack means for a MAP
 * The ack's opcode is MSG_ACK_OP_MAP (0) and its cmd_id the MAP's.
 * MSG_ACK_ACCEPTED means S3 applied the map (s3_set_nav_map() returned
 * E_OK); MSG_ACK_REJECTED means S3 refused it or the MAP was malformed.
 * A MAP is taken in any state; S5 is not asked.
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

#include "msg_codec.h"
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

/* Telemetry frame at 2 Hz and live message at 5 Hz (docs/S1_binary_frame.md
 * sections 3 and 3A). Both schedules are absolute, like the heartbeat's.
 */
#define S1_TELEM_TEL_PERIOD_MSEC    500U
#define S1_TELEM_LIVE_PERIOD_MSEC   200U

/* How long after the first heartbeat the first frame and the first live
 * message are due. With no offsets all three messages fall due in the same
 * pass every 1000 ms; with these, no two schedules ever share a deadline.
 */
#define S1_TELEM_TEL_OFFSET_MSEC    250U
#define S1_TELEM_LIVE_OFFSET_MSEC   100U

/* How many records may already be in the publish queue for a periodic
 * message to join it. The queue is first in, first out and the driver sends
 * one message at a time, so each one queued delays an ack behind it. Keeping
 * the queue this short bounds that delay; a message refused by this rule is
 * counted in periodic_skipped. Acks are never held back by it.
 */

/* The queued count excludes the message in flight. */
#define S1_TELEM_TEL_QUEUED_MAX     1U
#define S1_TELEM_LIVE_QUEUED_MAX    1U
#define S1_TELEM_HB_QUEUED_MAX      3U

/* S3's UNCAL line state and S4's IMPACT motion class, the values sent when
 * S3 cannot be read and while an impact is latched. s1_telem.c does not
 * include the subsystem headers (so it builds on the host); the values are
 * checked at compile time by telemetry_task_check_map_t in telemetry_task.c.
 */
#define S1_TELEM_LINE_UNCAL         5U
#define S1_TELEM_CLASS_IMPACT       6U

/** Posts one decoded command to S5. Returns 0 when it was queued, or a
 *  negative kernel error code when it was not.
 */
typedef int32_t (* s1_telem_post_fn_t)(uint8_t opcode, uint8_t cmd_id);

/** Gives a received map to S3: telemetry_task passes a function that
 *  calls s3_set_nav_map(). p_letters holds the letters for LEFT, RIGHT,
 *  STRAIGHT and U-TURN, then a NUL. Returns 0 when S3 applied the map, or
 *  a negative kernel error code when it did not.
 */
typedef int32_t (* s1_telem_map_fn_t)(const char * p_letters);

/** Sets or clears fault bits in S5's flg_fault: subsystem5_fault_set() or
 *  subsystem5_fault_clear(), passed in so that s1_telem.c builds on the host.
 */
typedef void (* s1_telem_fault_function_t)(uint32_t faults);

/** Counters kept by telemetry_task. Each one saturates at UINT32_MAX. */
typedef struct
{
    uint32_t    codec_reject;       /* Commands the codec refused.        */
    uint32_t    post_failure;       /* Posts S5's command queue refused.  */
    uint32_t    map_reject;         /* MAPs S3 refused, or malformed.     */
    uint32_t    ack_publish_failed; /* Acks mqtt_link_publish() refused.  */
    uint32_t    hb_published;       /* Heartbeats queued for publishing.  */
    uint32_t    hb_publish_failed;  /* Heartbeats mqtt_link_publish()     */
                                    /* refused.                           */
    uint32_t    hb_skipped;         /* Heartbeats not sent: no session.   */
    uint32_t    periodic_skipped;   /* Telemetry, live and heartbeats not */
                                    /* sent: the queue was too full.      */
    uint32_t    periodic_failed;    /* Telemetry and live messages        */
                                    /* mqtt_link_publish() refused.       */
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

/** Everything the telemetry frame and the live message need from S2, S3,
 *  S4 and S5, as plain integers. telemetry_task.c copies the getters'
 *  results into this; s1_telem_fill_frame() and s1_telem_fill_live() do
 *  the rest, so that logic is host-tested without the kernel.
 */
typedef struct
{
    /* S2: subsystem2_status_t and subsystem2_odom_t. */
    uint8_t     motion_state;       /* subsystem2_state_t.                */
    bool        b_estop;            /* Emergency-stop latch set.          */
    int32_t     target_l_mms;       /* Wheel set-points.                  */
    int32_t     target_r_mms;
    uint32_t    count_l;            /* Encoder ticks since power-up.      */
    uint32_t    count_r;
    int32_t     dist_mm;
    int32_t     speed_l_mms;        /* Measured wheel speeds.             */
    int32_t     speed_r_mms;
    int32_t     heading_cdeg;       /* + = left of the power-up heading.  */

    /* S3: s3_line_status_t. b_s3_ok false: S3 could not be read. */
    bool        b_s3_ok;
    uint8_t     line_state;         /* s3_line_state_t.                   */
    int32_t     steer_dv_mms;
    uint8_t     outer_ir;           /* 0 white to 255 black.              */
    uint8_t     centre_ir;
    bool        b_overshoot;
    uint8_t     last_ch;            /* Code 39 index, 63 = none.          */
    uint8_t     last_cmd;           /* s3_cmd_t.                          */
    bool        b_last_rev;
    uint8_t     junc_turn;          /* S3_JUNC_xxx.                       */
    uint8_t     rejects;
    uint8_t     junctions;
    uint8_t     losts;
    uint8_t     barcodes;

    /* S4: subsystem4_terrain_t. */
    int16_t     pitch_hdeg;
    int16_t     roll_hdeg;
    int16_t     turn_rate_dps;
    uint8_t     peak_mm;
    uint8_t     hump_state;
    uint8_t     motion_class;       /* subsystem4_class_t.                */
    uint8_t     hump_count;
    bool        b_imu_ok;
    bool        b_cal_done;
    bool        b_impact;           /* Latched until the E-stop clears.   */

    /* S5: controller state, fault bits, last ping and last profile. */
    uint8_t     ctrl_state;         /* subsystem5_ctrl_state_t.           */
    uint32_t    fault_mask;         /* subsystem5_fault_get().            */
    bool        b_range_valid;      /* Last ping had an echo.             */
    uint16_t    range_mm;           /* Last ping's range.                 */
    int16_t     servo_deg;          /* Last ping's servo angle.           */
    bool        b_profile_valid;    /* A scan has been done.              */
    uint16_t    width_mm;           /* Last profile.                      */
    uint16_t    clear_left_mm;
    uint16_t    clear_right_mm;
} s1_telem_src_t;

/** What the heartbeat needs from outside this module. */
typedef struct
{
    uint32_t    uptime_s;           /* Seconds since boot.                */
    uint32_t    connect_count;      /* mqtt_link status connect_count.    */
    uint32_t    command_dropped;    /* mqtt_link status command_dropped.  */
} s1_telem_hb_in_t;

/** What the WiFi fault rule reads on each pass of telemetry_task. */
typedef struct
{
    bool                b_link_up;      /* Radio link is CYW43_LINK_UP.       */
    bool                b_status_read;  /* mqtt_link_get_status() returned    */
                                        /* true: the fields below are usable. */
    bool                b_stale;        /* mqtt_link status b_stale.          */
    mqtt_link_state_t   state;          /* mqtt_link status state.            */
    uint32_t            connect_count;  /* mqtt_link status connect_count.    */
} s1_telem_wifi_status_t;

uint32_t s1_telem_sat_add(uint32_t total, uint32_t amount);
bool     s1_telem_on_command(const mqtt_link_command_t * p_cmd,
                             s1_telem_post_fn_t p_post,
                             s1_telem_map_fn_t p_map,
                             s1_telem_stats_t * p_stats,
                             s1_telem_pub_t * p_ack);
bool     s1_telem_heartbeat(uint8_t seq, const s1_telem_hb_in_t * p_in,
                            const s1_telem_stats_t * p_stats,
                            s1_telem_pub_t * p_hb);
bool     s1_telem_hb_due(uint32_t now_msec, uint32_t * p_next_msec);
bool     s1_telem_hb_should_publish(mqtt_link_state_t state, bool b_stale);
bool     s1_telem_due(uint32_t now_msec, uint32_t period_msec,
                      uint32_t * p_next_msec);
bool     s1_telem_queue_allows(uint32_t free_slots, uint32_t max_queued);
bool     s1_telem_wifi_update(const s1_telem_wifi_status_t * p_status,
                              s1_telem_fault_function_t p_set,
                              s1_telem_fault_function_t p_clear,
                              bool * pb_fault_set);
void     s1_telem_fill_frame(const s1_telem_src_t * p_src, uint8_t seq,
                             msg_codec_telemetry_t * p_tel);
void     s1_telem_fill_live(const s1_telem_src_t * p_src, uint8_t seq,
                            msg_codec_live_t * p_live);

#endif /* TM_MQTT */

#endif /* S1_TELEM_H */

/*** end of file ***/

