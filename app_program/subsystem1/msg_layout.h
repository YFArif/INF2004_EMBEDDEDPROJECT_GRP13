/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file msg_layout.h
 *
 * @brief Shared layout of every packed message: telemetry, heartbeat,
 *        command, ack and event.
 *
 * @par Source of truth
 * docs/S1_binary_frame.md (draft 3). Every position and width below is a
 * row of that document. Change both together and raise
 * MSG_LAYOUT_VERSION whenever a position or width changes.
 *
 * @par Wire rules
 * A message is a bit stream, most significant bit first: bit position 0
 * is bit 7 of byte 0, position 8 is bit 7 of byte 1. A field that crosses
 * a byte boundary puts its high bits in the earlier byte. Signed fields
 * are stored in offset binary (value + 2^(bits - 1)). Padding bits are 0.
 * C structs and bit-fields are never sent on the wire; msg_codec.c
 * packs every field with masks and shifts.
 *
 * @par Who includes this
 * Any task that sends an event to mbf_event (msg_event_t and
 * MSG_EVENT_ARG_BYTES), and msg_codec.c. It holds constants and one type
 * only, so it is not gated on TM_MQTT.
 */

#ifndef MSG_LAYOUT_H
#define MSG_LAYOUT_H

#include <stdint.h>

/* Layout version, in the high 4 bits of every message. 0 is never valid. */
#define MSG_LAYOUT_VERSION          1U

/* Widest field in any message. The codec's bit helpers rely on it. */
#define MSG_FIELD_BITS_MAX          24U

/* Message types, in the low 4 bits of every message. 0 and 6 to 15 are
 * reserved.
 */
#define MSG_TYPE_TELEMETRY          1U
#define MSG_TYPE_HEARTBEAT          2U
#define MSG_TYPE_EVENT              3U
#define MSG_TYPE_COMMAND            4U
#define MSG_TYPE_ACK                5U

/* Header byte, common to every message. */
#define MSG_HDR_VERSION_POS         0U
#define MSG_HDR_VERSION_BITS        4U
#define MSG_HDR_TYPE_POS            4U
#define MSG_HDR_TYPE_BITS           4U

/* ---------------------------------------------------------------------
 * TELEMETRY, robot to dashboard, 2 Hz. Section 3 of the document.
 * ------------------------------------------------------------------- */
#define MSG_TEL_SEQ_POS             8U      /* 0-255, modulo.           */
#define MSG_TEL_SEQ_BITS            8U
#define MSG_TEL_CTRL_POS            16U     /* S5 controller state.     */
#define MSG_TEL_CTRL_BITS           4U
#define MSG_TEL_MOTION_POS          20U     /* S2 motion state.         */
#define MSG_TEL_MOTION_BITS         3U
#define MSG_TEL_FAULT_POS           23U     /* MSG_FAULT_xxx bits.      */
#define MSG_TEL_FAULT_BITS          5U
#define MSG_TEL_LINE_POS            28U     /* S3 line state.           */
#define MSG_TEL_LINE_BITS           3U
#define MSG_TEL_BC_LETTER_POS       31U     /* 0 none, 1-43 Code 39.    */
#define MSG_TEL_BC_LETTER_BITS      6U
#define MSG_TEL_BC_CMD_POS          37U
#define MSG_TEL_BC_CMD_BITS         2U
#define MSG_TEL_BC_DIR_POS          39U
#define MSG_TEL_BC_DIR_BITS         1U
#define MSG_TEL_SPEED_L_POS         40U     /* mm/s, signed.            */
#define MSG_TEL_SPEED_L_BITS        10U
#define MSG_TEL_SPEED_R_POS         50U     /* mm/s, signed.            */
#define MSG_TEL_SPEED_R_BITS        10U
#define MSG_TEL_COUNT_L_POS         60U     /* Low bits, modulo.        */
#define MSG_TEL_COUNT_L_BITS        16U
#define MSG_TEL_COUNT_R_POS         76U     /* Low bits, modulo.        */
#define MSG_TEL_COUNT_R_BITS        16U
#define MSG_TEL_DIST_POS            92U     /* mm, signed.              */
#define MSG_TEL_DIST_BITS           16U
#define MSG_TEL_PITCH_POS           108U    /* 0.5 deg steps, signed.   */
#define MSG_TEL_PITCH_BITS          7U
#define MSG_TEL_ROLL_POS            115U    /* 0.5 deg steps, signed.   */
#define MSG_TEL_ROLL_BITS           7U
#define MSG_TEL_HUMP_ST_POS         122U
#define MSG_TEL_HUMP_ST_BITS        2U
#define MSG_TEL_PEAK_POS            124U    /* mm.                      */
#define MSG_TEL_PEAK_BITS           8U
#define MSG_TEL_HUMP_CNT_POS        132U    /* Saturates at 15.         */
#define MSG_TEL_HUMP_CNT_BITS       4U
#define MSG_TEL_MCLASS_POS          136U
#define MSG_TEL_MCLASS_BITS         3U
#define MSG_TEL_TURN_POS            139U    /* deg/s, signed (unit TBC). */
#define MSG_TEL_TURN_BITS           10U
#define MSG_TEL_IMU_OK_POS          149U
#define MSG_TEL_IMU_OK_BITS         1U
#define MSG_TEL_CAL_DONE_POS        150U
#define MSG_TEL_CAL_DONE_BITS       1U
#define MSG_TEL_CLOSEST_POS         151U    /* mm, clamped at 1023.     */
#define MSG_TEL_CLOSEST_BITS        10U
#define MSG_TEL_ANGLE_POS           161U    /* Servo step index.        */
#define MSG_TEL_ANGLE_BITS          5U
#define MSG_TEL_WIDTH_POS           166U    /* mm, clamped at 1023.     */
#define MSG_TEL_WIDTH_BITS          10U
#define MSG_TEL_CLEAR_L_POS         176U    /* mm, clamped at 1023.     */
#define MSG_TEL_CLEAR_L_BITS        10U
#define MSG_TEL_CLEAR_R_POS         186U    /* mm, clamped at 1023.     */
#define MSG_TEL_CLEAR_R_BITS        10U
#define MSG_TEL_DATA_BITS           196U
#define MSG_TEL_BYTES               25U

/* fault_mask bits. The field is flg_fault & 0x1F, copied with no remap:
 * integer bit n is fault n. Like every field it is written MSB first, so
 * bit n lands at wire offset 27 - n (section 3.1 of the document).
 * NOTE: flg_fault (Buddy 5) must use this same numbering.
 */
#define MSG_FAULT_STALL             0x01U   /* Bit 0: S2, latched.      */
#define MSG_FAULT_ENCODER           0x02U   /* Bit 1. TODO: TBD.        */
#define MSG_FAULT_IMU               0x04U   /* Bit 2: S4, latched.      */
#define MSG_FAULT_NO_ECHO           0x08U   /* Bit 3. TODO: TBD.        */
#define MSG_FAULT_WIFI              0x10U   /* Bit 4: S1, level.        */

/* ---------------------------------------------------------------------
 * HEARTBEAT, robot to dashboard, 1 Hz. Section 4.
 * ------------------------------------------------------------------- */
#define MSG_HB_SEQ_POS              8U      /* 0-255, modulo.           */
#define MSG_HB_SEQ_BITS             8U
#define MSG_HB_UPTIME_POS           16U     /* Seconds, saturates.      */
#define MSG_HB_UPTIME_BITS          16U
#define MSG_HB_RSSI_POS             32U     /* -RSSI in dBm, clamped.   */
#define MSG_HB_RSSI_BITS            7U
#define MSG_HB_CONNECT_POS          39U     /* Saturates at 255.        */
#define MSG_HB_CONNECT_BITS         8U
#define MSG_HB_TEL_DROP_POS         47U     /* Saturates at 255.        */
#define MSG_HB_TEL_DROP_BITS        8U
#define MSG_HB_EVT_DROP_POS         55U     /* Saturates at 255.        */
#define MSG_HB_EVT_DROP_BITS        8U
#define MSG_HB_CMD_REJ_POS          63U     /* Saturates at 255.        */
#define MSG_HB_CMD_REJ_BITS         8U
#define MSG_HB_DATA_BITS            71U
#define MSG_HB_BYTES                9U

/* ---------------------------------------------------------------------
 * COMMAND, dashboard to robot. Section 5.
 * ------------------------------------------------------------------- */
#define MSG_CMD_OPCODE_POS          8U
#define MSG_CMD_OPCODE_BITS         2U
#define MSG_CMD_ID_POS              10U     /* Echoed in the ack.       */
#define MSG_CMD_ID_BITS             6U
#define MSG_CMD_DATA_BITS           16U
#define MSG_CMD_BYTES               2U

/* Opcodes. 0 is rejected, so an all-zero payload is never a command. */
#define MSG_OP_INVALID              0U
#define MSG_OP_START                1U
#define MSG_OP_STOP                 2U
#define MSG_OP_RESET                3U

/* ---------------------------------------------------------------------
 * ACK, robot to dashboard, on <prefix>/ack. Section 5.
 * ------------------------------------------------------------------- */
#define MSG_ACK_OPCODE_POS          8U
#define MSG_ACK_OPCODE_BITS         2U
#define MSG_ACK_ID_POS              10U
#define MSG_ACK_ID_BITS             6U
#define MSG_ACK_RESULT_POS          16U
#define MSG_ACK_RESULT_BITS         2U
#define MSG_ACK_DATA_BITS           18U
#define MSG_ACK_BYTES               3U

/* Ack results. 2 and 3 are reserved. */
#define MSG_ACK_ACCEPTED            0U
#define MSG_ACK_REJECTED            1U

/* ---------------------------------------------------------------------
 * EVENT. Section 6. Inside the robot an event is an msg_event_t record
 * in mbf_event; on the wire S1 adds the header and a time stamp.
 * ------------------------------------------------------------------- */

/* Argument bytes in every event. Each event code defines how it uses
 * them; unused bits are 0.
 */
#define MSG_EVENT_ARG_BYTES         2U
#define MSG_EVENT_RECORD_BYTES      (1U + MSG_EVENT_ARG_BYTES)

#define MSG_EVT_CODE_POS            8U
#define MSG_EVT_CODE_BITS           8U
#define MSG_EVT_ARGS_POS            16U
#define MSG_EVT_ARGS_BITS           (MSG_EVENT_ARG_BYTES * 8U)
#define MSG_EVT_TIME_POS            32U     /* ms uptime, modulo.       */
#define MSG_EVT_TIME_BITS           24U
#define MSG_EVT_DATA_BITS           56U
#define MSG_EVT_BYTES               7U

/* TODO: Event codes. The table belongs to Buddy 5 and is not written
 * yet; codes are added here once agreed.
 */

/* Largest message of any type, for buffers that hold any of them. */
#define MSG_MAX_BYTES               MSG_TEL_BYTES

/** One event record, as sent to mbf_event by any task.
 *
 * NOTE: Every member is a byte, so the record has no padding and its
 * size is MSG_EVENT_RECORD_BYTES; msg_codec.c checks that at compile
 * time. The record never leaves the chip as it is; msg_codec.c packs it
 * for the wire.
 */
typedef struct
{
    uint8_t     code;                           /* Event code.          */
    uint8_t     args[MSG_EVENT_ARG_BYTES];      /* Per-code arguments.  */
} msg_event_t;

#endif /* MSG_LAYOUT_H */

/*** end of file ***/
