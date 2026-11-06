/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file msg_codec.h
 *
 * @brief Encoder and decoder for the packed messages in msg_layout.h.
 *
 * @par Values in, bytes out
 * Each message has a plain C struct holding its fields in wire units
 * (for example pitch in half-degrees). The put functions clamp every
 * value to what its field can hold and pack it into a byte buffer; the
 * get functions check the length, version and type and unpack it again.
 * The structs are never sent as they are.
 *
 * @par What each field kind does on put
 * - Unsigned (states, mm, counts that saturate): clamped to the field's
 *   largest value, so an out-of-range value never spills into the next
 *   field.
 * - Signed (speeds, distance, angles, turn rate): clamped to the field's
 *   signed range and stored in offset binary.
 * - Modulo (seq, encoder counts, cmd_id, event time, fault_mask): only
 *   the low bits are kept. For fault_mask this drops unknown bits rather
 *   than setting every fault.
 * - Flags: false is 0, true is 1.
 *
 * @par Thread safety
 * Every function works only on the buffers and structs it is given, so
 * any task may call it. No function blocks or uses the kernel.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0), like the rest of
 * the MQTT link, so builds without MQTT are unchanged.
 */

#ifndef MSG_CODEC_H
#define MSG_CODEC_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "msg_layout.h"

/** Telemetry frame fields, in wire units. */
typedef struct
{
    uint8_t     seq;            /* Frame counter, modulo 256.           */
    uint8_t     ctrl_state;     /* S5 controller state, 0-15.           */
    uint8_t     motion_state;   /* S2 motion state, 0-7.                */
    uint8_t     fault_mask;     /* flg_fault & 0x1F (MSG_FAULT_xxx).    */
    uint8_t     line_state;     /* S3 line state, 0-7.                  */
    uint8_t     bc_letter;      /* 0 none, 1-43 Code 39 symbol.         */
    uint8_t     bc_cmd;         /* Barcode command, 0-3.                */
    bool        b_bc_dir;       /* NAV_MSG dir_reversed.                */
    int32_t     speed_l_mms;    /* -512 to 511 mm/s.                    */
    int32_t     speed_r_mms;    /* -512 to 511 mm/s.                    */
    uint32_t    count_l;        /* Full count in; low 16 bits out.      */
    uint32_t    count_r;        /* Full count in; low 16 bits out.      */
    int32_t     dist_mm;        /* -32768 to 32767 mm.                  */
    int16_t     pitch_hdeg;     /* Half-degrees, -64 to 63; from        */
    int16_t     roll_hdeg;      /* msg_codec_cdeg_to_hdeg().            */
    uint8_t     hump_state;     /* 0-3.                                 */
    uint16_t    peak_mm;        /* 0-255 mm.                            */
    uint8_t     hump_count;     /* 0-15, saturates.                     */
    uint8_t     motion_class;   /* 0-7.                                 */
    int16_t     turn_dps;       /* -512 to 511 deg/s (unit TBC).        */
    bool        b_imu_ok;
    bool        b_cal_done;
    uint16_t    closest_mm;     /* 0-1023 mm.                           */
    uint8_t     angle_idx;      /* Servo step index, 0-31.              */
    uint16_t    width_mm;       /* 0-1023 mm.                           */
    uint16_t    clear_l_mm;     /* 0-1023 mm.                           */
    uint16_t    clear_r_mm;     /* 0-1023 mm.                           */
} msg_codec_telemetry_t;

/** Heartbeat fields. */
typedef struct
{
    uint8_t     seq;            /* Heartbeat counter, modulo 256.       */
    uint32_t    uptime_s;       /* Saturates at 65535.                  */
    int16_t     rssi_dbm;       /* -127 to 0 dBm; above 0 sends 0.      */
    uint32_t    connect_count;  /* Saturates at 255.                    */
    uint32_t    tel_drop;       /* Saturates at 255.                    */
    uint32_t    evt_drop;       /* Saturates at 255.                    */
    uint32_t    cmd_reject;     /* Saturates at 255.                    */
} msg_codec_heartbeat_t;

/** Command fields (also the opcode and id echoed in an ack). */
typedef struct
{
    uint8_t     opcode;         /* MSG_OP_xxx.                          */
    uint8_t     cmd_id;         /* 0-63, set by the dashboard.          */
} msg_codec_command_t;

/** Ack fields. */
typedef struct
{
    uint8_t     opcode;         /* Echoed; 0 if the command was unread. */
    uint8_t     cmd_id;         /* Echoed; 0 if the command was unread. */
    uint8_t     result;         /* MSG_ACK_xxx.                         */
} msg_codec_ack_t;

/** Event as sent on the wire: the record plus a time stamp. */
typedef struct
{
    msg_event_t event;
    uint32_t    t_ms;           /* Uptime in ms, low 24 bits sent.      */
} msg_codec_event_t;

/*
 * Every put function returns the number of bytes written (the message's
 * fixed length), or 0 when a pointer is NULL or the buffer is too small.
 * Every get function returns true only when the length is exact and the
 * version and type match; full parameter notes are in msg_codec.c.
 */

/** @brief Convert pitch or roll from centidegrees to half-degrees
 *         (section 3.3): limit to +/-4000, round half away from zero,
 *         clamp to -64 ... 63. Any int32_t input is safe.
 */
int16_t  msg_codec_cdeg_to_hdeg(int32_t cdeg);

/** @brief Pack a telemetry frame (MSG_TEL_BYTES). */
uint32_t msg_codec_put_telemetry(const msg_codec_telemetry_t * p_in,
                                 uint8_t * p_buf, uint32_t buf_len);

/** @brief Unpack a telemetry frame; counts come back as their low 16 bits.
 */
bool     msg_codec_get_telemetry(const uint8_t * p_buf, uint32_t len,
                                 msg_codec_telemetry_t * p_out);

/** @brief Pack a heartbeat (MSG_HB_BYTES). */
uint32_t msg_codec_put_heartbeat(const msg_codec_heartbeat_t * p_in,
                                 uint8_t * p_buf, uint32_t buf_len);

/** @brief Unpack a heartbeat. */
bool     msg_codec_get_heartbeat(const uint8_t * p_buf, uint32_t len,
                                 msg_codec_heartbeat_t * p_out);

/** @brief Pack a command (MSG_CMD_BYTES); for tests and tools. */
uint32_t msg_codec_put_command(const msg_codec_command_t * p_in,
                               uint8_t * p_buf, uint32_t buf_len);

/** @brief Unpack and check a received command.
 *
 * Returns true only for a well-formed START, STOP or RESET. When the
 * length, version and type are right, p_out holds the opcode and id even
 * if the opcode is MSG_OP_INVALID, so the ack can echo them; otherwise
 * both are 0.
 */
bool     msg_codec_get_command(const uint8_t * p_buf, uint32_t len,
                               msg_codec_command_t * p_out);

/** @brief Pack an ack (MSG_ACK_BYTES). */
uint32_t msg_codec_put_ack(const msg_codec_ack_t * p_in,
                           uint8_t * p_buf, uint32_t buf_len);

/** @brief Unpack an ack; for tests and tools. */
bool     msg_codec_get_ack(const uint8_t * p_buf, uint32_t len,
                           msg_codec_ack_t * p_out);

/** @brief Pack an event for the wire (MSG_EVT_BYTES). */
uint32_t msg_codec_put_event(const msg_codec_event_t * p_in,
                             uint8_t * p_buf, uint32_t buf_len);

/** @brief Unpack an event; for tests and tools. */
bool     msg_codec_get_event(const uint8_t * p_buf, uint32_t len,
                             msg_codec_event_t * p_out);

#endif /* TM_MQTT */

#endif /* MSG_CODEC_H */

/*** end of file ***/
