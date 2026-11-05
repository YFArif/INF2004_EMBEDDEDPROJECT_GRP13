/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file msg_codec.c
 *
 * @brief Encoder and decoder for the packed messages in msg_layout.h.
 *
 * @par How fields are packed
 * put_field() writes a value one bit at a time, most significant bit
 * first, at a bit position counted from the start of the buffer; it only
 * shifts and masks unsigned values (Barr-C 5.3b). get_field() reads it
 * back the same way. The longest message is 200 bits, so a bit loop costs
 * an estimated tens of microseconds per message, to be measured on the
 * Pico; it was chosen because it is plainly correct at every byte
 * boundary.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0) -- see msg_codec.h.
 */

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "msg_codec.h"

/* Pitch and roll conversion, section 3.3 of docs/S1_binary_frame.md:
 * centidegrees in, half-degrees out. The input is limited to
 * +/-CDEG_LIMIT while still signed, so its magnitude never overflows
 * int32_t; every input beyond 3225 cdeg clamps to the same result anyway.
 */
#define CDEG_LIMIT              4000
#define CDEG_PER_HDEG           50
#define CDEG_HALF_STEP          25
#define HDEG_MIN                (-64)
#define HDEG_MAX                63

/* Compile-time checks. Each typedef has a negative array size, and so stops
 * the build, when its condition is false.
 *
 * 1. Every field starts where the one before it ends, and each message's
 *    data bits plus padding fill its bytes exactly (fewer than 8 padding
 *    bits). This pins the layout to the table in docs/S1_binary_frame.md.
 */
typedef uint8_t msg_codec_check_hdr_t[((MSG_HDR_TYPE_POS
    == (MSG_HDR_VERSION_POS + MSG_HDR_VERSION_BITS))
    && (8U == (MSG_HDR_TYPE_POS + MSG_HDR_TYPE_BITS))) ? 1 : -1];

typedef uint8_t msg_codec_check_tel_a_t[((MSG_TEL_SEQ_POS == 8U)
    && (MSG_TEL_CTRL_POS == (MSG_TEL_SEQ_POS + MSG_TEL_SEQ_BITS))
    && (MSG_TEL_MOTION_POS == (MSG_TEL_CTRL_POS + MSG_TEL_CTRL_BITS))
    && (MSG_TEL_FAULT_POS == (MSG_TEL_MOTION_POS + MSG_TEL_MOTION_BITS))
    && (MSG_TEL_LINE_POS == (MSG_TEL_FAULT_POS + MSG_TEL_FAULT_BITS))
    && (MSG_TEL_BC_LETTER_POS == (MSG_TEL_LINE_POS + MSG_TEL_LINE_BITS))
    && (MSG_TEL_BC_CMD_POS
        == (MSG_TEL_BC_LETTER_POS + MSG_TEL_BC_LETTER_BITS))
    && (MSG_TEL_BC_DIR_POS == (MSG_TEL_BC_CMD_POS + MSG_TEL_BC_CMD_BITS))
    && (MSG_TEL_SPEED_L_POS
        == (MSG_TEL_BC_DIR_POS + MSG_TEL_BC_DIR_BITS))
    && (MSG_TEL_SPEED_R_POS
        == (MSG_TEL_SPEED_L_POS + MSG_TEL_SPEED_L_BITS))
    && (MSG_TEL_COUNT_L_POS
        == (MSG_TEL_SPEED_R_POS + MSG_TEL_SPEED_R_BITS))
    && (MSG_TEL_COUNT_R_POS
        == (MSG_TEL_COUNT_L_POS + MSG_TEL_COUNT_L_BITS))
    && (MSG_TEL_DIST_POS == (MSG_TEL_COUNT_R_POS + MSG_TEL_COUNT_R_BITS))
    && (MSG_TEL_PITCH_POS == (MSG_TEL_DIST_POS + MSG_TEL_DIST_BITS)))
    ? 1 : -1];

typedef uint8_t msg_codec_check_tel_b_t[(
    (MSG_TEL_ROLL_POS == (MSG_TEL_PITCH_POS + MSG_TEL_PITCH_BITS))
    && (MSG_TEL_HUMP_ST_POS == (MSG_TEL_ROLL_POS + MSG_TEL_ROLL_BITS))
    && (MSG_TEL_PEAK_POS == (MSG_TEL_HUMP_ST_POS + MSG_TEL_HUMP_ST_BITS))
    && (MSG_TEL_HUMP_CNT_POS == (MSG_TEL_PEAK_POS + MSG_TEL_PEAK_BITS))
    && (MSG_TEL_MCLASS_POS
        == (MSG_TEL_HUMP_CNT_POS + MSG_TEL_HUMP_CNT_BITS))
    && (MSG_TEL_TURN_POS == (MSG_TEL_MCLASS_POS + MSG_TEL_MCLASS_BITS))
    && (MSG_TEL_IMU_OK_POS == (MSG_TEL_TURN_POS + MSG_TEL_TURN_BITS))
    && (MSG_TEL_CAL_DONE_POS
        == (MSG_TEL_IMU_OK_POS + MSG_TEL_IMU_OK_BITS))
    && (MSG_TEL_CLOSEST_POS
        == (MSG_TEL_CAL_DONE_POS + MSG_TEL_CAL_DONE_BITS))
    && (MSG_TEL_ANGLE_POS == (MSG_TEL_CLOSEST_POS + MSG_TEL_CLOSEST_BITS))
    && (MSG_TEL_WIDTH_POS == (MSG_TEL_ANGLE_POS + MSG_TEL_ANGLE_BITS))
    && (MSG_TEL_CLEAR_L_POS == (MSG_TEL_WIDTH_POS + MSG_TEL_WIDTH_BITS))
    && (MSG_TEL_CLEAR_R_POS
        == (MSG_TEL_CLEAR_L_POS + MSG_TEL_CLEAR_L_BITS))
    && (MSG_TEL_DATA_BITS
        == (MSG_TEL_CLEAR_R_POS + MSG_TEL_CLEAR_R_BITS))
    && (MSG_TEL_BYTES == ((MSG_TEL_DATA_BITS + 7U) / 8U))) ? 1 : -1];

typedef uint8_t msg_codec_check_hb_t[((MSG_HB_SEQ_POS == 8U)
    && (MSG_HB_UPTIME_POS == (MSG_HB_SEQ_POS + MSG_HB_SEQ_BITS))
    && (MSG_HB_RSSI_POS == (MSG_HB_UPTIME_POS + MSG_HB_UPTIME_BITS))
    && (MSG_HB_CONNECT_POS == (MSG_HB_RSSI_POS + MSG_HB_RSSI_BITS))
    && (MSG_HB_TEL_DROP_POS
        == (MSG_HB_CONNECT_POS + MSG_HB_CONNECT_BITS))
    && (MSG_HB_EVT_DROP_POS
        == (MSG_HB_TEL_DROP_POS + MSG_HB_TEL_DROP_BITS))
    && (MSG_HB_CMD_REJ_POS
        == (MSG_HB_EVT_DROP_POS + MSG_HB_EVT_DROP_BITS))
    && (MSG_HB_DATA_BITS == (MSG_HB_CMD_REJ_POS + MSG_HB_CMD_REJ_BITS))
    && (MSG_HB_BYTES == ((MSG_HB_DATA_BITS + 7U) / 8U))) ? 1 : -1];

typedef uint8_t msg_codec_check_cmd_t[((MSG_CMD_OPCODE_POS == 8U)
    && (MSG_CMD_ID_POS == (MSG_CMD_OPCODE_POS + MSG_CMD_OPCODE_BITS))
    && (MSG_CMD_DATA_BITS == (MSG_CMD_ID_POS + MSG_CMD_ID_BITS))
    && (MSG_CMD_BYTES == ((MSG_CMD_DATA_BITS + 7U) / 8U))
    && (MSG_ACK_OPCODE_POS == 8U)
    && (MSG_ACK_ID_POS == (MSG_ACK_OPCODE_POS + MSG_ACK_OPCODE_BITS))
    && (MSG_ACK_RESULT_POS == (MSG_ACK_ID_POS + MSG_ACK_ID_BITS))
    && (MSG_ACK_DATA_BITS == (MSG_ACK_RESULT_POS + MSG_ACK_RESULT_BITS))
    && (MSG_ACK_BYTES == ((MSG_ACK_DATA_BITS + 7U) / 8U))
    && (MSG_ACK_OPCODE_BITS == MSG_CMD_OPCODE_BITS)
    && (MSG_ACK_ID_BITS == MSG_CMD_ID_BITS)) ? 1 : -1];

typedef uint8_t msg_codec_check_evt_t[((MSG_EVT_CODE_POS == 8U)
    && (MSG_EVT_ARGS_POS == (MSG_EVT_CODE_POS + MSG_EVT_CODE_BITS))
    && (MSG_EVT_TIME_POS == (MSG_EVT_ARGS_POS + MSG_EVT_ARGS_BITS))
    && (MSG_EVT_DATA_BITS == (MSG_EVT_TIME_POS + MSG_EVT_TIME_BITS))
    && (MSG_EVT_BYTES == ((MSG_EVT_DATA_BITS + 7U) / 8U))) ? 1 : -1];

/* 2. The event record has no padding, so its size is exactly the bytes
 *    the layout gives it.
 */
typedef uint8_t msg_codec_check_rec_t[(sizeof(msg_event_t)
    == MSG_EVENT_RECORD_BYTES) ? 1 : -1];

/* 3. HDEG_MIN and HDEG_MAX are exactly the range of the 7-bit pitch and
 *    roll fields. HDEG_MAX is a positive constant (63), so converting it
 *    to uint32_t keeps its value.
 */
typedef uint8_t msg_codec_check_hdeg_t[((MSG_TEL_PITCH_BITS
    == MSG_TEL_ROLL_BITS)
    && (((uint32_t)HDEG_MAX + 1U) == (1U << (MSG_TEL_PITCH_BITS - 1U)))
    && (HDEG_MIN == (-HDEG_MAX - 1))) ? 1 : -1];

/* 4. Every field fits the bit helpers (1 to MSG_FIELD_BITS_MAX bits, so
 *    1U << bits never overflows), the fields read with get_s16() (pitch,
 *    roll and turn) are at most 16 bits wide, and MSG_MAX_BYTES holds
 *    every message.
 */
typedef uint8_t msg_codec_check_max_t[((MSG_FIELD_BITS_MAX < 32U)
    && (MSG_EVT_TIME_BITS <= MSG_FIELD_BITS_MAX)
    && (MSG_EVT_ARGS_BITS <= MSG_FIELD_BITS_MAX)
    && (MSG_TEL_PITCH_BITS <= 16U)
    && (MSG_TEL_ROLL_BITS <= 16U)
    && (MSG_TEL_TURN_BITS <= 16U)
    && (MSG_TEL_BYTES <= MSG_MAX_BYTES)
    && (MSG_HB_BYTES <= MSG_MAX_BYTES)
    && (MSG_EVT_BYTES <= MSG_MAX_BYTES)
    && (MSG_ACK_BYTES <= MSG_MAX_BYTES)
    && (MSG_CMD_BYTES <= MSG_MAX_BYTES)) ? 1 : -1];

static void     put_field(uint8_t * p_buf, uint32_t pos, uint32_t bits,
                          uint32_t value);
static uint32_t get_field(const uint8_t * p_buf, uint32_t pos,
                          uint32_t bits);
static void     put_unsigned(uint8_t * p_buf, uint32_t pos, uint32_t bits,
                             uint32_t value);
static void     put_modulo(uint8_t * p_buf, uint32_t pos, uint32_t bits,
                           uint32_t value);
static void     put_signed(uint8_t * p_buf, uint32_t pos, uint32_t bits,
                           int32_t value);
static void     put_flag(uint8_t * p_buf, uint32_t pos, bool b_value);
static uint8_t  get_u8(const uint8_t * p_buf, uint32_t pos, uint32_t bits);
static uint16_t get_u16(const uint8_t * p_buf, uint32_t pos,
                        uint32_t bits);
static int32_t  get_signed(const uint8_t * p_buf, uint32_t pos,
                           uint32_t bits);
static int16_t  get_s16(const uint8_t * p_buf, uint32_t pos,
                        uint32_t bits);
static bool     get_flag(const uint8_t * p_buf, uint32_t pos);
static uint32_t field_max(uint32_t bits);
static void     start_message(uint8_t * p_buf, uint32_t len, uint32_t type);
static bool     header_ok(const uint8_t * p_buf, uint32_t len,
                          uint32_t expected_len, uint32_t type);

/*!
 * @brief Convert centidegrees to the half-degrees sent for pitch and roll.
 *
 * Section 3.3 of docs/S1_binary_frame.md, in three steps, all on signed
 * integers: limit the input to +/-4000 cdeg, round half away from zero
 * (working on the magnitude, because C division truncates toward zero),
 * then clamp to the 7-bit field's range. The offset is added later, by
 * the encoder, on the unsigned value.
 *
 * @param[in] cdeg Angle in centidegrees, any int32_t value.
 *
 * @return Half-degrees, -64 to 63.
 */
int16_t
msg_codec_cdeg_to_hdeg (int32_t cdeg)
{
    int32_t limited = cdeg;
    int32_t hdeg;

    if (limited > CDEG_LIMIT)
    {
        limited = CDEG_LIMIT;
    }
    else if (limited < -CDEG_LIMIT)
    {
        limited = -CDEG_LIMIT;
    }
    else
    {
        /* Already within the limit. */
    }

    if (limited < 0)
    {
        hdeg = -((-limited + CDEG_HALF_STEP) / CDEG_PER_HDEG);
    }
    else
    {
        hdeg = (limited + CDEG_HALF_STEP) / CDEG_PER_HDEG;
    }

    if (hdeg > HDEG_MAX)
    {
        hdeg = HDEG_MAX;
    }
    else if (hdeg < HDEG_MIN)
    {
        hdeg = HDEG_MIN;
    }
    else
    {
        /* Already within the field's range. */
    }

    /* hdeg was just clamped to -64 ... 63, so it fits int16_t. */
    return ((int16_t)hdeg);
}

/*!
 * @brief Pack a telemetry frame.
 *
 * @param[in]  p_in    Field values; out-of-range values are clamped.
 * @param[out] p_buf   Receives MSG_TEL_BYTES bytes.
 * @param[in]  buf_len Size of p_buf.
 *
 * @return MSG_TEL_BYTES, or 0 if a pointer is NULL or p_buf is too small.
 */
uint32_t
msg_codec_put_telemetry (const msg_codec_telemetry_t * p_in,
                         uint8_t * p_buf, uint32_t buf_len)
{
    uint32_t written = 0U;

    if ((NULL != p_in) && (NULL != p_buf) && (buf_len >= MSG_TEL_BYTES))
    {
        start_message(p_buf, MSG_TEL_BYTES, MSG_TYPE_TELEMETRY);
        put_modulo(p_buf, MSG_TEL_SEQ_POS, MSG_TEL_SEQ_BITS, p_in->seq);
        put_unsigned(p_buf, MSG_TEL_CTRL_POS, MSG_TEL_CTRL_BITS,
                     p_in->ctrl_state);
        put_unsigned(p_buf, MSG_TEL_MOTION_POS, MSG_TEL_MOTION_BITS,
                     p_in->motion_state);
        put_modulo(p_buf, MSG_TEL_FAULT_POS, MSG_TEL_FAULT_BITS,
                   p_in->fault_mask);
        put_unsigned(p_buf, MSG_TEL_LINE_POS, MSG_TEL_LINE_BITS,
                     p_in->line_state);
        put_unsigned(p_buf, MSG_TEL_BC_LETTER_POS, MSG_TEL_BC_LETTER_BITS,
                     p_in->bc_letter);
        put_unsigned(p_buf, MSG_TEL_BC_CMD_POS, MSG_TEL_BC_CMD_BITS,
                     p_in->bc_cmd);
        put_flag(p_buf, MSG_TEL_BC_DIR_POS, p_in->b_bc_dir);
        put_signed(p_buf, MSG_TEL_SPEED_L_POS, MSG_TEL_SPEED_L_BITS,
                   p_in->speed_l_mms);
        put_signed(p_buf, MSG_TEL_SPEED_R_POS, MSG_TEL_SPEED_R_BITS,
                   p_in->speed_r_mms);
        put_modulo(p_buf, MSG_TEL_COUNT_L_POS, MSG_TEL_COUNT_L_BITS,
                   p_in->count_l);
        put_modulo(p_buf, MSG_TEL_COUNT_R_POS, MSG_TEL_COUNT_R_BITS,
                   p_in->count_r);
        put_signed(p_buf, MSG_TEL_DIST_POS, MSG_TEL_DIST_BITS,
                   p_in->dist_mm);
        put_signed(p_buf, MSG_TEL_PITCH_POS, MSG_TEL_PITCH_BITS,
                   p_in->pitch_hdeg);
        put_signed(p_buf, MSG_TEL_ROLL_POS, MSG_TEL_ROLL_BITS,
                   p_in->roll_hdeg);
        put_unsigned(p_buf, MSG_TEL_HUMP_ST_POS, MSG_TEL_HUMP_ST_BITS,
                     p_in->hump_state);
        put_unsigned(p_buf, MSG_TEL_PEAK_POS, MSG_TEL_PEAK_BITS,
                     p_in->peak_mm);
        put_unsigned(p_buf, MSG_TEL_HUMP_CNT_POS, MSG_TEL_HUMP_CNT_BITS,
                     p_in->hump_count);
        put_unsigned(p_buf, MSG_TEL_MCLASS_POS, MSG_TEL_MCLASS_BITS,
                     p_in->motion_class);
        put_signed(p_buf, MSG_TEL_TURN_POS, MSG_TEL_TURN_BITS,
                   p_in->turn_dps);
        put_flag(p_buf, MSG_TEL_IMU_OK_POS, p_in->b_imu_ok);
        put_flag(p_buf, MSG_TEL_CAL_DONE_POS, p_in->b_cal_done);
        put_unsigned(p_buf, MSG_TEL_CLOSEST_POS, MSG_TEL_CLOSEST_BITS,
                     p_in->closest_mm);
        put_unsigned(p_buf, MSG_TEL_ANGLE_POS, MSG_TEL_ANGLE_BITS,
                     p_in->angle_idx);
        put_unsigned(p_buf, MSG_TEL_WIDTH_POS, MSG_TEL_WIDTH_BITS,
                     p_in->width_mm);
        put_unsigned(p_buf, MSG_TEL_CLEAR_L_POS, MSG_TEL_CLEAR_L_BITS,
                     p_in->clear_l_mm);
        put_unsigned(p_buf, MSG_TEL_CLEAR_R_POS, MSG_TEL_CLEAR_R_BITS,
                     p_in->clear_r_mm);
        written = MSG_TEL_BYTES;
    }

    return (written);
}

/*!
 * @brief Unpack a telemetry frame.
 *
 * count_l and count_r receive only the low 16 bits that were sent.
 *
 * @param[in]  p_buf The received bytes.
 * @param[in]  len   Number of received bytes.
 * @param[out] p_out Receives the fields. Unchanged when false is returned.
 *
 * @return True if len, version and type are right.
 */
bool
msg_codec_get_telemetry (const uint8_t * p_buf, uint32_t len,
                         msg_codec_telemetry_t * p_out)
{
    bool b_ok = false;

    if ((NULL != p_out)
        && header_ok(p_buf, len, MSG_TEL_BYTES, MSG_TYPE_TELEMETRY))
    {
        p_out->seq = get_u8(p_buf, MSG_TEL_SEQ_POS, MSG_TEL_SEQ_BITS);
        p_out->ctrl_state = get_u8(p_buf, MSG_TEL_CTRL_POS,
                                   MSG_TEL_CTRL_BITS);
        p_out->motion_state = get_u8(p_buf, MSG_TEL_MOTION_POS,
                                     MSG_TEL_MOTION_BITS);
        p_out->fault_mask = get_u8(p_buf, MSG_TEL_FAULT_POS,
                                   MSG_TEL_FAULT_BITS);
        p_out->line_state = get_u8(p_buf, MSG_TEL_LINE_POS,
                                   MSG_TEL_LINE_BITS);
        p_out->bc_letter = get_u8(p_buf, MSG_TEL_BC_LETTER_POS,
                                  MSG_TEL_BC_LETTER_BITS);
        p_out->bc_cmd = get_u8(p_buf, MSG_TEL_BC_CMD_POS,
                               MSG_TEL_BC_CMD_BITS);
        p_out->b_bc_dir = get_flag(p_buf, MSG_TEL_BC_DIR_POS);
        p_out->speed_l_mms = get_signed(p_buf, MSG_TEL_SPEED_L_POS,
                                        MSG_TEL_SPEED_L_BITS);
        p_out->speed_r_mms = get_signed(p_buf, MSG_TEL_SPEED_R_POS,
                                        MSG_TEL_SPEED_R_BITS);
        p_out->count_l = get_field(p_buf, MSG_TEL_COUNT_L_POS,
                                   MSG_TEL_COUNT_L_BITS);
        p_out->count_r = get_field(p_buf, MSG_TEL_COUNT_R_POS,
                                   MSG_TEL_COUNT_R_BITS);
        p_out->dist_mm = get_signed(p_buf, MSG_TEL_DIST_POS,
                                    MSG_TEL_DIST_BITS);
        p_out->pitch_hdeg = get_s16(p_buf, MSG_TEL_PITCH_POS,
                                    MSG_TEL_PITCH_BITS);
        p_out->roll_hdeg = get_s16(p_buf, MSG_TEL_ROLL_POS,
                                   MSG_TEL_ROLL_BITS);
        p_out->hump_state = get_u8(p_buf, MSG_TEL_HUMP_ST_POS,
                                   MSG_TEL_HUMP_ST_BITS);
        p_out->peak_mm = get_u16(p_buf, MSG_TEL_PEAK_POS,
                                 MSG_TEL_PEAK_BITS);
        p_out->hump_count = get_u8(p_buf, MSG_TEL_HUMP_CNT_POS,
                                   MSG_TEL_HUMP_CNT_BITS);
        p_out->motion_class = get_u8(p_buf, MSG_TEL_MCLASS_POS,
                                     MSG_TEL_MCLASS_BITS);
        p_out->turn_dps = get_s16(p_buf, MSG_TEL_TURN_POS,
                                  MSG_TEL_TURN_BITS);
        p_out->b_imu_ok = get_flag(p_buf, MSG_TEL_IMU_OK_POS);
        p_out->b_cal_done = get_flag(p_buf, MSG_TEL_CAL_DONE_POS);
        p_out->closest_mm = get_u16(p_buf, MSG_TEL_CLOSEST_POS,
                                    MSG_TEL_CLOSEST_BITS);
        p_out->angle_idx = get_u8(p_buf, MSG_TEL_ANGLE_POS,
                                  MSG_TEL_ANGLE_BITS);
        p_out->width_mm = get_u16(p_buf, MSG_TEL_WIDTH_POS,
                                  MSG_TEL_WIDTH_BITS);
        p_out->clear_l_mm = get_u16(p_buf, MSG_TEL_CLEAR_L_POS,
                                    MSG_TEL_CLEAR_L_BITS);
        p_out->clear_r_mm = get_u16(p_buf, MSG_TEL_CLEAR_R_POS,
                                    MSG_TEL_CLEAR_R_BITS);
        b_ok = true;
    }

    return (b_ok);
}

/*!
 * @brief Pack a heartbeat.
 *
 * @param[in]  p_in    Field values; counters saturate, RSSI is clamped.
 * @param[out] p_buf   Receives MSG_HB_BYTES bytes.
 * @param[in]  buf_len Size of p_buf.
 *
 * @return MSG_HB_BYTES, or 0 if a pointer is NULL or p_buf is too small.
 */
uint32_t
msg_codec_put_heartbeat (const msg_codec_heartbeat_t * p_in,
                         uint8_t * p_buf, uint32_t buf_len)
{
    uint32_t written = 0U;
    uint32_t rssi_neg = 0U;

    if ((NULL != p_in) && (NULL != p_buf) && (buf_len >= MSG_HB_BYTES))
    {
        if (p_in->rssi_dbm < 0)
        {
            /* rssi_dbm is -32768 to -1 here; widening it to int32_t is
             * exact, and its negation, 1 to 32768, converts to uint32_t
             * unchanged.
             */
            rssi_neg = (uint32_t)(-(int32_t)p_in->rssi_dbm);
        }

        start_message(p_buf, MSG_HB_BYTES, MSG_TYPE_HEARTBEAT);
        put_modulo(p_buf, MSG_HB_SEQ_POS, MSG_HB_SEQ_BITS, p_in->seq);
        put_unsigned(p_buf, MSG_HB_UPTIME_POS, MSG_HB_UPTIME_BITS,
                     p_in->uptime_s);
        put_unsigned(p_buf, MSG_HB_RSSI_POS, MSG_HB_RSSI_BITS, rssi_neg);
        put_unsigned(p_buf, MSG_HB_CONNECT_POS, MSG_HB_CONNECT_BITS,
                     p_in->connect_count);
        put_unsigned(p_buf, MSG_HB_TEL_DROP_POS, MSG_HB_TEL_DROP_BITS,
                     p_in->tel_drop);
        put_unsigned(p_buf, MSG_HB_EVT_DROP_POS, MSG_HB_EVT_DROP_BITS,
                     p_in->evt_drop);
        put_unsigned(p_buf, MSG_HB_CMD_REJ_POS, MSG_HB_CMD_REJ_BITS,
                     p_in->cmd_reject);
        written = MSG_HB_BYTES;
    }

    return (written);
}

/*!
 * @brief Unpack a heartbeat.
 *
 * @param[in]  p_buf The received bytes.
 * @param[in]  len   Number of received bytes.
 * @param[out] p_out Receives the fields. Unchanged when false is returned.
 *
 * @return True if len, version and type are right.
 */
bool
msg_codec_get_heartbeat (const uint8_t * p_buf, uint32_t len,
                         msg_codec_heartbeat_t * p_out)
{
    bool     b_ok = false;
    uint32_t rssi_neg;

    if ((NULL != p_out)
        && header_ok(p_buf, len, MSG_HB_BYTES, MSG_TYPE_HEARTBEAT))
    {
        rssi_neg = get_field(p_buf, MSG_HB_RSSI_POS, MSG_HB_RSSI_BITS);

        p_out->seq = get_u8(p_buf, MSG_HB_SEQ_POS, MSG_HB_SEQ_BITS);
        p_out->uptime_s = get_field(p_buf, MSG_HB_UPTIME_POS,
                                    MSG_HB_UPTIME_BITS);
        /* rssi_neg is 0 to 127 (7-bit field); widening it to int32_t is
         * exact, and its negation, -127 to 0, fits int16_t.
         */
        p_out->rssi_dbm = (int16_t)(-(int32_t)rssi_neg);
        p_out->connect_count = get_field(p_buf, MSG_HB_CONNECT_POS,
                                         MSG_HB_CONNECT_BITS);
        p_out->tel_drop = get_field(p_buf, MSG_HB_TEL_DROP_POS,
                                    MSG_HB_TEL_DROP_BITS);
        p_out->evt_drop = get_field(p_buf, MSG_HB_EVT_DROP_POS,
                                    MSG_HB_EVT_DROP_BITS);
        p_out->cmd_reject = get_field(p_buf, MSG_HB_CMD_REJ_POS,
                                      MSG_HB_CMD_REJ_BITS);
        b_ok = true;
    }

    return (b_ok);
}

/*!
 * @brief Pack a command (used by tests and tools; the robot only reads
 *        commands).
 *
 * @param[in]  p_in    Opcode and id; the id is sent modulo 64.
 * @param[out] p_buf   Receives MSG_CMD_BYTES bytes.
 * @param[in]  buf_len Size of p_buf.
 *
 * @return MSG_CMD_BYTES, or 0 if a pointer is NULL or p_buf is too small.
 */
uint32_t
msg_codec_put_command (const msg_codec_command_t * p_in,
                       uint8_t * p_buf, uint32_t buf_len)
{
    uint32_t written = 0U;

    if ((NULL != p_in) && (NULL != p_buf) && (buf_len >= MSG_CMD_BYTES))
    {
        start_message(p_buf, MSG_CMD_BYTES, MSG_TYPE_COMMAND);
        put_unsigned(p_buf, MSG_CMD_OPCODE_POS, MSG_CMD_OPCODE_BITS,
                     p_in->opcode);
        put_modulo(p_buf, MSG_CMD_ID_POS, MSG_CMD_ID_BITS, p_in->cmd_id);
        written = MSG_CMD_BYTES;
    }

    return (written);
}

/*!
 * @brief Unpack and check a received command.
 *
 * @param[in]  p_buf The received payload.
 * @param[in]  len   Payload length.
 * @param[out] p_out Receives the opcode and id when len, version and type
 *                   are right (even if the opcode is MSG_OP_INVALID), so
 *                   the ack can echo them; otherwise both are set to 0.
 *
 * @return True only for a well-formed START, STOP or RESET. Answer false
 *         with an ack whose result is MSG_ACK_REJECTED.
 */
bool
msg_codec_get_command (const uint8_t * p_buf, uint32_t len,
                       msg_codec_command_t * p_out)
{
    bool b_ok = false;

    if (NULL != p_out)
    {
        p_out->opcode = MSG_OP_INVALID;
        p_out->cmd_id = 0U;

        if (header_ok(p_buf, len, MSG_CMD_BYTES, MSG_TYPE_COMMAND))
        {
            p_out->opcode = get_u8(p_buf, MSG_CMD_OPCODE_POS,
                                   MSG_CMD_OPCODE_BITS);
            p_out->cmd_id = get_u8(p_buf, MSG_CMD_ID_POS, MSG_CMD_ID_BITS);
            b_ok = (MSG_OP_INVALID != p_out->opcode);
        }
    }

    return (b_ok);
}

/*!
 * @brief Pack an ack.
 *
 * @param[in]  p_in    Echoed opcode and id, and the result.
 * @param[out] p_buf   Receives MSG_ACK_BYTES bytes.
 * @param[in]  buf_len Size of p_buf.
 *
 * @return MSG_ACK_BYTES, or 0 if a pointer is NULL or p_buf is too small.
 */
uint32_t
msg_codec_put_ack (const msg_codec_ack_t * p_in,
                   uint8_t * p_buf, uint32_t buf_len)
{
    uint32_t written = 0U;

    if ((NULL != p_in) && (NULL != p_buf) && (buf_len >= MSG_ACK_BYTES))
    {
        start_message(p_buf, MSG_ACK_BYTES, MSG_TYPE_ACK);
        put_unsigned(p_buf, MSG_ACK_OPCODE_POS, MSG_ACK_OPCODE_BITS,
                     p_in->opcode);
        put_modulo(p_buf, MSG_ACK_ID_POS, MSG_ACK_ID_BITS, p_in->cmd_id);
        put_unsigned(p_buf, MSG_ACK_RESULT_POS, MSG_ACK_RESULT_BITS,
                     p_in->result);
        written = MSG_ACK_BYTES;
    }

    return (written);
}

/*!
 * @brief Unpack an ack (used by tests and tools).
 *
 * @param[in]  p_buf The received bytes.
 * @param[in]  len   Number of received bytes.
 * @param[out] p_out Receives the fields. Unchanged when false is returned.
 *
 * @return True if len, version and type are right.
 */
bool
msg_codec_get_ack (const uint8_t * p_buf, uint32_t len,
                   msg_codec_ack_t * p_out)
{
    bool b_ok = false;

    if ((NULL != p_out)
        && header_ok(p_buf, len, MSG_ACK_BYTES, MSG_TYPE_ACK))
    {
        p_out->opcode = get_u8(p_buf, MSG_ACK_OPCODE_POS,
                               MSG_ACK_OPCODE_BITS);
        p_out->cmd_id = get_u8(p_buf, MSG_ACK_ID_POS, MSG_ACK_ID_BITS);
        p_out->result = get_u8(p_buf, MSG_ACK_RESULT_POS,
                               MSG_ACK_RESULT_BITS);
        b_ok = true;
    }

    return (b_ok);
}

/*!
 * @brief Pack an event for the wire.
 *
 * @param[in]  p_in    The event record and its time stamp (low 24 bits
 *                     sent).
 * @param[out] p_buf   Receives MSG_EVT_BYTES bytes.
 * @param[in]  buf_len Size of p_buf.
 *
 * @return MSG_EVT_BYTES, or 0 if a pointer is NULL or p_buf is too small.
 */
uint32_t
msg_codec_put_event (const msg_codec_event_t * p_in,
                     uint8_t * p_buf, uint32_t buf_len)
{
    uint32_t written = 0U;
    uint32_t idx;

    if ((NULL != p_in) && (NULL != p_buf) && (buf_len >= MSG_EVT_BYTES))
    {
        start_message(p_buf, MSG_EVT_BYTES, MSG_TYPE_EVENT);
        put_modulo(p_buf, MSG_EVT_CODE_POS, MSG_EVT_CODE_BITS,
                   p_in->event.code);

        for (idx = 0U; idx < MSG_EVENT_ARG_BYTES; idx++)
        {
            put_modulo(p_buf, MSG_EVT_ARGS_POS + (idx * 8U), 8U,
                       p_in->event.args[idx]);
        }

        put_modulo(p_buf, MSG_EVT_TIME_POS, MSG_EVT_TIME_BITS, p_in->t_ms);
        written = MSG_EVT_BYTES;
    }

    return (written);
}

/*!
 * @brief Unpack an event (used by tests and tools).
 *
 * @param[in]  p_buf The received bytes.
 * @param[in]  len   Number of received bytes.
 * @param[out] p_out Receives the fields. Unchanged when false is returned.
 *
 * @return True if len, version and type are right.
 */
bool
msg_codec_get_event (const uint8_t * p_buf, uint32_t len,
                     msg_codec_event_t * p_out)
{
    bool     b_ok = false;
    uint32_t idx;

    if ((NULL != p_out)
        && header_ok(p_buf, len, MSG_EVT_BYTES, MSG_TYPE_EVENT))
    {
        p_out->event.code = get_u8(p_buf, MSG_EVT_CODE_POS,
                                   MSG_EVT_CODE_BITS);

        for (idx = 0U; idx < MSG_EVENT_ARG_BYTES; idx++)
        {
            p_out->event.args[idx] = get_u8(p_buf,
                                            MSG_EVT_ARGS_POS + (idx * 8U),
                                            8U);
        }

        p_out->t_ms = get_field(p_buf, MSG_EVT_TIME_POS, MSG_EVT_TIME_BITS);
        b_ok = true;
    }

    return (b_ok);
}

/*!
 * @brief Write a value into a field, most significant bit first.
 *
 * The buffer must already be zeroed (start_message() does that), so only
 * the 1 bits are written.
 *
 * @param[out] p_buf Message buffer.
 * @param[in]  pos   Bit position of the field's first (most significant)
 *                   bit.
 * @param[in]  bits  Field width, 1 to MSG_FIELD_BITS_MAX.
 * @param[in]  value Value; must already fit in bits (callers make sure).
 */
static void
put_field (uint8_t * p_buf, uint32_t pos, uint32_t bits, uint32_t value)
{
    uint32_t idx;
    uint32_t bit_pos;
    uint32_t mask;

    for (idx = 0U; idx < bits; idx++)
    {
        bit_pos = pos + idx;
        mask = 0x80U >> (bit_pos % 8U);

        if (0U != ((value >> (bits - 1U - idx)) & 1U))
        {
            /* p_buf[] is 0 to 255, so widening it to uint32_t is exact and
             * the OR works on unsigned values; mask is 0x01 to 0x80, so the
             * result is 0 to 255 and narrows back to uint8_t unchanged.
             */
            p_buf[bit_pos / 8U] = (uint8_t)((uint32_t)p_buf[bit_pos / 8U]
                                            | mask);
        }
    }
}

/*!
 * @brief Read a field, most significant bit first.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the field's first bit.
 * @param[in] bits  Field width, 1 to MSG_FIELD_BITS_MAX.
 *
 * @return The field value, below 2^bits.
 */
static uint32_t
get_field (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    uint32_t idx;
    uint32_t bit_pos;
    uint32_t value = 0U;

    for (idx = 0U; idx < bits; idx++)
    {
        bit_pos = pos + idx;

        /* p_buf[] is 0 to 255, so widening it to uint32_t is exact and the
         * shift and mask work on unsigned values.
         */
        value = (value << 1U)
                | (((uint32_t)p_buf[bit_pos / 8U] >> (7U - (bit_pos % 8U)))
                   & 1U);
    }

    return (value);
}

/*!
 * @brief Write an unsigned value, clamped to the largest the field holds.
 *
 * @param[out] p_buf Message buffer.
 * @param[in]  pos   Bit position of the field.
 * @param[in]  bits  Field width.
 * @param[in]  value Value.
 */
static void
put_unsigned (uint8_t * p_buf, uint32_t pos, uint32_t bits, uint32_t value)
{
    const uint32_t max = field_max(bits);

    put_field(p_buf, pos, bits, (value > max) ? max : value);
}

/*!
 * @brief Write the low bits of a value (counters that wrap, bit masks).
 *
 * @param[out] p_buf Message buffer.
 * @param[in]  pos   Bit position of the field.
 * @param[in]  bits  Field width.
 * @param[in]  value Value; higher bits are dropped.
 */
static void
put_modulo (uint8_t * p_buf, uint32_t pos, uint32_t bits, uint32_t value)
{
    put_field(p_buf, pos, bits, value & field_max(bits));
}

/*!
 * @brief Write a signed value in offset binary, clamped to the field's
 *        range.
 *
 * A field of n bits holds -2^(n-1) to 2^(n-1) - 1 and stores
 * value + 2^(n-1), so no bitwise operator ever touches a signed value.
 *
 * @param[out] p_buf Message buffer.
 * @param[in]  pos   Bit position of the field.
 * @param[in]  bits  Field width, 2 to MSG_FIELD_BITS_MAX.
 * @param[in]  value Value.
 */
static void
put_signed (uint8_t * p_buf, uint32_t pos, uint32_t bits, int32_t value)
{
    /* Every signed field has 2 to MSG_FIELD_BITS_MAX (24) bits, so half
     * is 2 to 2^23 and fits int32_t.
     */
    const int32_t half = (int32_t)(1U << (bits - 1U));
    int32_t       clamped = value;

    if (clamped < -half)
    {
        clamped = -half;
    }
    else if (clamped > (half - 1))
    {
        clamped = half - 1;
    }
    else
    {
        /* Already in range. */
    }

    /* clamped + half is between 0 and 2^bits - 1, so it is not negative
     * and converts to uint32_t unchanged.
     */
    put_field(p_buf, pos, bits, (uint32_t)(clamped + half));
}

/*!
 * @brief Write a one-bit flag.
 *
 * @param[out] p_buf   Message buffer.
 * @param[in]  pos     Bit position of the flag.
 * @param[in]  b_value The flag.
 */
static void
put_flag (uint8_t * p_buf, uint32_t pos, bool b_value)
{
    put_field(p_buf, pos, 1U, b_value ? 1U : 0U);
}

/*!
 * @brief Read a field of at most 8 bits.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the field.
 * @param[in] bits  Field width, 1 to 8.
 *
 * @return The field value.
 */
static uint8_t
get_u8 (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    /* The mask leaves 0 to 255, which fits uint8_t. Callers pass at most
     * 8 bits, so get_field() returns 0 to 255 and the mask removes nothing.
     */
    return ((uint8_t)(get_field(p_buf, pos, bits) & 0xFFU));
}

/*!
 * @brief Read a field of at most 16 bits.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the field.
 * @param[in] bits  Field width, 1 to 16.
 *
 * @return The field value.
 */
static uint16_t
get_u16 (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    /* The mask leaves 0 to 65535, which fits uint16_t. Callers pass at
     * most 16 bits, so get_field() returns 0 to 65535 and the mask removes
     * nothing.
     */
    return ((uint16_t)(get_field(p_buf, pos, bits) & 0xFFFFU));
}

/*!
 * @brief Read a signed field stored in offset binary.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the field.
 * @param[in] bits  Field width, 2 to MSG_FIELD_BITS_MAX.
 *
 * @return The value, between -2^(bits-1) and 2^(bits-1) - 1.
 */
static int32_t
get_signed (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    /* The stored value is below 2^24 and half is at most 2^23, so both
     * fit int32_t.
     */
    const int32_t stored = (int32_t)get_field(p_buf, pos, bits);
    const int32_t half = (int32_t)(1U << (bits - 1U));

    return (stored - half);
}

/*!
 * @brief Read a signed field of at most 16 bits.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the field.
 * @param[in] bits  Field width, 2 to 16.
 *
 * @return The value.
 */
static int16_t
get_s16 (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    /* Every caller passes at most 16 bits (compile-time check 4), so the
     * value is between -32768 and 32767 and fits int16_t.
     */
    return ((int16_t)get_signed(p_buf, pos, bits));
}

/*!
 * @brief Read a one-bit flag.
 *
 * @param[in] p_buf Message buffer.
 * @param[in] pos   Bit position of the flag.
 *
 * @return The flag.
 */
static bool
get_flag (const uint8_t * p_buf, uint32_t pos)
{
    return (0U != get_field(p_buf, pos, 1U));
}

/*!
 * @brief Largest value a field holds.
 *
 * @param[in] bits Field width, 1 to MSG_FIELD_BITS_MAX.
 *
 * @return 2^bits - 1.
 */
static uint32_t
field_max (uint32_t bits)
{
    return ((1U << bits) - 1U);
}

/*!
 * @brief Zero a message buffer and write its header byte.
 *
 * @param[out] p_buf Message buffer.
 * @param[in]  len   Message length in bytes.
 * @param[in]  type  MSG_TYPE_xxx.
 */
static void
start_message (uint8_t * p_buf, uint32_t len, uint32_t type)
{
    uint32_t idx;

    for (idx = 0U; idx < len; idx++)
    {
        p_buf[idx] = 0U;
    }

    put_field(p_buf, MSG_HDR_VERSION_POS, MSG_HDR_VERSION_BITS,
              MSG_LAYOUT_VERSION);
    put_field(p_buf, MSG_HDR_TYPE_POS, MSG_HDR_TYPE_BITS, type);
}

/*!
 * @brief Check a received message's length and header byte.
 *
 * @param[in] p_buf        The received bytes (may be NULL).
 * @param[in] len          Number of received bytes.
 * @param[in] expected_len Exact length for this type.
 * @param[in] type         Expected MSG_TYPE_xxx.
 *
 * @return True if the length is exact and version and type match.
 */
static bool
header_ok (const uint8_t * p_buf, uint32_t len, uint32_t expected_len,
           uint32_t type)
{
    bool b_ok = false;

    if ((NULL != p_buf) && (expected_len == len))
    {
        b_ok = ((MSG_LAYOUT_VERSION
                 == get_field(p_buf, MSG_HDR_VERSION_POS,
                              MSG_HDR_VERSION_BITS))
                && (type == get_field(p_buf, MSG_HDR_TYPE_POS,
                                      MSG_HDR_TYPE_BITS)));
    }

    return (b_ok);
}

#endif /* TM_MQTT */

/*** end of file ***/
