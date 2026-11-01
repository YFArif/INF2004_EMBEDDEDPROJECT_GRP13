/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file codec_test_main.c
 *
 * @brief S1-U05: host test of the minimum-bit message codec.
 *
 * @par What it tests
 * app_program/subsystem1/msg_codec.c and msg_layout.h, through the public
 * API only, against docs/S1_binary_frame.md:
 * - the 18 golden vectors of section 10.2 (encode and decode), and the
 *   MSG_FAULT_xxx constants against the one-hot vectors;
 * - the 17 pitch/roll rounding cases of section 3.3;
 * - every field at its minimum and maximum;
 * - the wrap, reconstruction, clamp and reject cases of section 10.1.
 *
 * @par How
 * Every encode writes into a buffer of exactly the message length, and
 * every decode reads from one, both from malloc(), so AddressSanitizer
 * reports any access past the end. Stored bits are read back with this
 * file's own bit reader, not with the codec's helpers.
 *
 * @par Build and run
 * Use testing/subsystem1/run_codec_test.sh from the repository root; it
 * builds this file with strict warnings and UBSan/ASan, runs it, and then
 * runs golden.py --check, the mutation runs and the target compile.
 * This program prints one line per failed check and one line per group,
 * then "RESULT: all passed (n checks)" and exits 0, or
 * "RESULT: m FAILED (n checks)" and exits 1.
 *
 * @par Test code
 * This module is never built into the firmware. int main(void) is the
 * form the C standard requires (Barr-C 5.2.a does not apply to it).
 */

#include "codec_test_main.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "codec_vectors.h"
#include "msg_codec.h"

/* Byte written into buffers and output structs before a call, so that a
 * write the call should not make can be seen afterwards.
 */
#define MARK_BYTE               0xA5U

/* Number of golden vectors in section 10.2 of the document. */
#define GOLDEN_COUNT            18U

/* Version nibble values used by the reject cases. */
#define BAD_VERSION_LOW         0U
#define BAD_VERSION_HIGH        2U
#define RESERVED_TYPE           6U

/** Telemetry fields, so that tests can set or read one by number. */
typedef enum
{
    TF_SEQ = 0,
    TF_CTRL,
    TF_MOTION,
    TF_FAULT,
    TF_LINE,
    TF_LETTER,
    TF_BC_CMD,
    TF_BC_DIR,
    TF_SPEED_L,
    TF_SPEED_R,
    TF_COUNT_L,
    TF_COUNT_R,
    TF_DIST,
    TF_PITCH,
    TF_ROLL,
    TF_HUMP_ST,
    TF_PEAK,
    TF_HUMP_CNT,
    TF_MCLASS,
    TF_TURN,
    TF_IMU_OK,
    TF_CAL_DONE,
    TF_CLOSEST,
    TF_ANGLE,
    TF_WIDTH,
    TF_CLEAR_L,
    TF_CLEAR_R,
    TF_NONE                     /* Number of fields; also "no field". */
} tel_field_t;

/** One case that sets a telemetry field and expects a decoded value. */
typedef struct
{
    const char *    p_label;
    tel_field_t     field;
    int64_t         value;
    int64_t         expect;
} tel_case_t;

/** Room for the decoded output of any message type. */
typedef union
{
    msg_codec_telemetry_t   tel;
    msg_codec_heartbeat_t   hb;
    msg_codec_event_t       evt;
    msg_codec_command_t     cmd;
    msg_codec_ack_t         ack;
} any_out_t;

/** Calls into the codec for one message type, with untyped pointers. */
typedef bool     (*get_fn_t)(const uint8_t * p_buf, uint32_t len,
                             void * p_out);
typedef uint32_t (*put_fn_t)(const void * p_in, uint8_t * p_buf,
                             uint32_t buf_len);

/** What the reject cases need to know about one message type. */
typedef struct
{
    const char *        p_name;
    uint32_t            len;
    uint32_t            type;
    get_fn_t            get;
    put_fn_t            put;
    bool                b_zero_on_reject;   /* get_command zeroes out. */
    const void *        p_valid_in;
    const uint8_t *     p_valid_bytes;
} msg_desc_t;

/* Pitch/roll rounding, section 3.3 of the document: input, half-degrees
 * after rounding and clamping, and the 7-bit stored value.
 */
typedef struct
{
    int32_t     cdeg;
    int16_t     hdeg;
    uint32_t    stored;
} round_case_t;

static const round_case_t g_round_cases[] =
{
    { 0, 0, 64U },              { 24, 0, 64U },
    { -24, 0, 64U },            { 25, 1, 65U },
    { -25, -1, 63U },           { 26, 1, 65U },
    { -26, -1, 63U },           { 75, 2, 66U },
    { -75, -2, 62U },           { 3150, 63, 127U },
    { 3175, 63, 127U },         { -3175, -64, 0U },
    { -3225, -64, 0U },         { 9000, 63, 127U },
    { -9000, -64, 0U },         { INT32_MAX, 63, 127U },
    { INT32_MIN, -64, 0U },
};

/* Telemetry clamp cases, section 10.1, plus the codec's own clamp of
 * pitch and roll given directly in half-degrees.
 */
static const tel_case_t g_tel_clamps[] =
{
    { "ctrl_state 16", TF_CTRL, 16, 15 },
    { "ctrl_state 255", TF_CTRL, 255, 15 },
    { "motion_state 8", TF_MOTION, 8, 7 },
    { "line_state 8", TF_LINE, 8, 7 },
    { "motion_class 8", TF_MCLASS, 8, 7 },
    { "bc_letter 64", TF_LETTER, 64, 63 },
    { "bc_cmd 4", TF_BC_CMD, 4, 3 },
    { "hump_state 4", TF_HUMP_ST, 4, 3 },
    { "hump_count 16", TF_HUMP_CNT, 16, 15 },
    { "hump_count 99", TF_HUMP_CNT, 99, 15 },
    { "peak_mm 256", TF_PEAK, 256, 255 },
    { "peak_mm 999", TF_PEAK, 999, 255 },
    { "angle_idx 32", TF_ANGLE, 32, 31 },
    { "closest_mm 1024", TF_CLOSEST, 1024, 1023 },
    { "closest_mm 4000", TF_CLOSEST, 4000, 1023 },
    { "width_mm 1024", TF_WIDTH, 1024, 1023 },
    { "width_mm 4000", TF_WIDTH, 4000, 1023 },
    { "clear_l_mm 1024", TF_CLEAR_L, 1024, 1023 },
    { "clear_l_mm 4000", TF_CLEAR_L, 4000, 1023 },
    { "clear_r_mm 1024", TF_CLEAR_R, 1024, 1023 },
    { "clear_r_mm 4000", TF_CLEAR_R, 4000, 1023 },
    { "speed_l_mms 512", TF_SPEED_L, 512, 511 },
    { "speed_l_mms -513", TF_SPEED_L, -513, -512 },
    { "speed_l_mms 100000", TF_SPEED_L, 100000, 511 },
    { "speed_l_mms -100000", TF_SPEED_L, -100000, -512 },
    { "speed_r_mms 512", TF_SPEED_R, 512, 511 },
    { "speed_r_mms -513", TF_SPEED_R, -513, -512 },
    { "speed_r_mms 100000", TF_SPEED_R, 100000, 511 },
    { "speed_r_mms -100000", TF_SPEED_R, -100000, -512 },
    { "dist_mm 32768", TF_DIST, 32768, 32767 },
    { "dist_mm -32769", TF_DIST, -32769, -32768 },
    { "turn 512", TF_TURN, 512, 511 },
    { "turn -513", TF_TURN, -513, -512 },
    { "pitch_hdeg 64", TF_PITCH, 64, 63 },
    { "pitch_hdeg -65", TF_PITCH, -65, -64 },
    { "roll_hdeg 64", TF_ROLL, 64, 63 },
    { "roll_hdeg -65", TF_ROLL, -65, -64 },
};

/* Telemetry wrap (modulo) cases, section 10.1. */
static const tel_case_t g_tel_wraps[] =
{
    { "count_l 65535", TF_COUNT_L, 65535, 65535 },
    { "count_l 65536", TF_COUNT_L, 65536, 0 },
    { "count_l 0x12345678", TF_COUNT_L, 0x12345678, 0x5678 },
    { "count_r 65535", TF_COUNT_R, 65535, 65535 },
    { "count_r 65536", TF_COUNT_R, 65536, 0 },
    { "count_r 0x12345678", TF_COUNT_R, 0x12345678, 0x5678 },
    { "fault_mask 0xE5", TF_FAULT, 0xE5, 0x05 },
};

/* Check counters: all groups, and the group that is running. */
static uint32_t     g_checks = 0U;
static uint32_t     g_failures = 0U;
static uint32_t     g_group_checks = 0U;
static uint32_t     g_group_failures = 0U;
static const char * gp_group = "";

static void     group_begin(const char * p_name);
static void     group_end(void);
static void     check(bool b_ok, const char * p_what, const char * p_label);
static uint8_t * alloc_copy(const uint8_t * p_src, uint32_t size,
                            uint32_t copy_len);
static uint32_t read_bits(const uint8_t * p_buf, uint32_t pos,
                          uint32_t bits);
static void     tel_to_array(const msg_codec_telemetry_t * p_tel,
                             int64_t * p_vals);
static void     tel_from_array(const int64_t * p_vals,
                               msg_codec_telemetry_t * p_tel);
static int64_t  tel_get(const msg_codec_telemetry_t * p_tel,
                        tel_field_t field);
static void     tel_set(msg_codec_telemetry_t * p_tel, tel_field_t field,
                        int64_t value);
static bool     put_matches(put_fn_t put, const void * p_in, uint32_t len,
                            const uint8_t * p_expect);
static bool     get_exact(get_fn_t get, const uint8_t * p_bytes,
                          uint32_t len, void * p_out);
static bool     tel_same(const msg_codec_telemetry_t * p_a,
                         const msg_codec_telemetry_t * p_b,
                         tel_field_t skip);
static void     tel_input(const codec_vectors_tel_t * p_vec,
                          msg_codec_telemetry_t * p_in);
static const codec_vectors_tel_t * tel_find(const char * p_label);
static bool     tel_round_trip(const msg_codec_telemetry_t * p_in,
                               msg_codec_telemetry_t * p_out,
                               uint8_t * p_bytes);
static bool     hb_round_trip(const msg_codec_heartbeat_t * p_in,
                              msg_codec_heartbeat_t * p_out);
static bool     evt_round_trip(const msg_codec_event_t * p_in,
                               msg_codec_event_t * p_out);
static bool     cmd_round_trip(const msg_codec_command_t * p_in,
                               msg_codec_command_t * p_out);
static bool     ack_round_trip(const msg_codec_ack_t * p_in,
                               msg_codec_ack_t * p_out);
static bool     hb_same(const msg_codec_heartbeat_t * p_a,
                        const msg_codec_heartbeat_t * p_b);
static bool     evt_same(const msg_codec_event_t * p_a,
                         const msg_codec_event_t * p_b);
static bool     ack_same(const msg_codec_ack_t * p_a,
                         const msg_codec_ack_t * p_b);
static uint32_t unwrap16(uint32_t prev_full, uint32_t low16);
static bool     get_tel(const uint8_t * p_buf, uint32_t len, void * p_out);
static bool     get_hb(const uint8_t * p_buf, uint32_t len, void * p_out);
static bool     get_evt(const uint8_t * p_buf, uint32_t len, void * p_out);
static bool     get_cmd(const uint8_t * p_buf, uint32_t len, void * p_out);
static bool     get_ack(const uint8_t * p_buf, uint32_t len, void * p_out);
static uint32_t put_tel(const void * p_in, uint8_t * p_buf,
                        uint32_t buf_len);
static uint32_t put_hb(const void * p_in, uint8_t * p_buf,
                       uint32_t buf_len);
static uint32_t put_evt(const void * p_in, uint8_t * p_buf,
                        uint32_t buf_len);
static uint32_t put_cmd(const void * p_in, uint8_t * p_buf,
                        uint32_t buf_len);
static uint32_t put_ack(const void * p_in, uint8_t * p_buf,
                        uint32_t buf_len);
static void     test_golden_tel(void);
static void     test_golden_other(void);
static void     test_fault_constants(void);
static void     test_rounding(void);
static void     test_min_max(void);
static void     test_wrap(void);
static void     test_reconstruction(void);
static void     test_tel_cases(const tel_case_t * p_cases, uint32_t count);
static void     test_clamps_other(void);
static void     reject_get(const msg_desc_t * p_msg, const uint8_t * p_buf,
                           uint32_t len, const char * p_what);
static void     reject_put(const msg_desc_t * p_msg);
static void     test_rejects_one(const msg_desc_t * p_msg);
static void     test_rejects(void);

/*!
 * @brief Run every test group and report the result.
 *
 * @return EXIT_SUCCESS if every check passed, otherwise EXIT_FAILURE.
 */
int
main (void)
{
    int result = EXIT_FAILURE;

    (void)printf("S1-U05 msg_codec host test (layout version %u)\n",
                 MSG_LAYOUT_VERSION);

    group_begin("golden vectors");
    test_golden_tel();
    test_golden_other();
    test_fault_constants();
    group_end();

    group_begin("rounding");
    test_rounding();
    group_end();

    group_begin("min and max");
    test_min_max();
    group_end();

    group_begin("wrap");
    test_wrap();
    test_tel_cases(g_tel_wraps,
                   (uint32_t)(sizeof(g_tel_wraps) / sizeof(g_tel_wraps[0])));
    group_end();

    group_begin("reconstruction");
    test_reconstruction();
    group_end();

    group_begin("clamps");
    test_tel_cases(g_tel_clamps,
                   (uint32_t)(sizeof(g_tel_clamps) / sizeof(g_tel_clamps[0])));
    test_clamps_other();
    group_end();

    group_begin("rejects");
    test_rejects();
    group_end();

    if (0U == g_failures)
    {
        (void)printf("RESULT: all passed (%" PRIu32 " checks)\n", g_checks);
        result = EXIT_SUCCESS;
    }
    else
    {
        (void)printf("RESULT: %" PRIu32 " FAILED (%" PRIu32 " checks)\n",
                     g_failures, g_checks);
    }

    return (result);
}

/*!
 * @brief Start counting the checks of one group.
 *
 * @param[in] p_name Group name, printed with its totals.
 */
static void
group_begin (const char * p_name)
{
    gp_group = p_name;
    g_group_checks = 0U;
    g_group_failures = 0U;
}

/*!
 * @brief Print the totals of the group that is running.
 */
static void
group_end (void)
{
    (void)printf("group %s: %" PRIu32 " checks, %" PRIu32 " failed\n",
                 gp_group, g_group_checks, g_group_failures);
}

/*!
 * @brief Count one check, and print it if it failed.
 *
 * @param[in] b_ok    True if the check passed.
 * @param[in] p_what  What was checked.
 * @param[in] p_label Which case (vector label or case name).
 */
static void
check (bool b_ok, const char * p_what, const char * p_label)
{
    g_checks++;
    g_group_checks++;

    if (!b_ok)
    {
        g_failures++;
        g_group_failures++;
        (void)printf("FAIL %s: %s [%s]\n", gp_group, p_what, p_label);
    }
}

/*!
 * @brief Allocate a buffer of exactly size bytes, fill it with MARK_BYTE,
 *        and copy the first copy_len bytes of p_src into it.
 *
 * @param[in] p_src    Source bytes (may be NULL when copy_len is 0).
 * @param[in] size     Buffer size, at least 1.
 * @param[in] copy_len Bytes to copy, at most size.
 *
 * @return The buffer (free it with free()), or NULL if malloc failed.
 */
static uint8_t *
alloc_copy (const uint8_t * p_src, uint32_t size, uint32_t copy_len)
{
    uint8_t * p_buf = malloc(size);

    if (NULL != p_buf)
    {
        (void)memset(p_buf, (int)MARK_BYTE, size);

        if ((NULL != p_src) && (copy_len > 0U))
        {
            (void)memcpy(p_buf, p_src, copy_len);
        }
    }

    return (p_buf);
}

/*!
 * @brief Read a field MSB first, independently of the codec's helpers.
 *
 * @param[in] p_buf Message bytes.
 * @param[in] pos   Bit position of the field's first bit.
 * @param[in] bits  Field width, 1 to 31.
 *
 * @return The field value.
 */
static uint32_t
read_bits (const uint8_t * p_buf, uint32_t pos, uint32_t bits)
{
    uint32_t value = 0U;
    uint32_t idx;
    uint32_t byte;

    for (idx = pos; idx < (pos + bits); idx++)
    {
        /* p_buf[] is 0 to 255, so widening it to uint32_t is exact. */
        byte = (uint32_t)p_buf[idx / 8U];
        value = (value << 1U) | ((byte >> (7U - (idx % 8U))) & 1U);
    }

    return (value);
}

/*!
 * @brief Copy every telemetry field into an array indexed by tel_field_t.
 *
 * @param[in]  p_tel  The fields.
 * @param[out] p_vals Receives TF_NONE values.
 */
static void
tel_to_array (const msg_codec_telemetry_t * p_tel, int64_t * p_vals)
{
    p_vals[TF_SEQ] = p_tel->seq;
    p_vals[TF_CTRL] = p_tel->ctrl_state;
    p_vals[TF_MOTION] = p_tel->motion_state;
    p_vals[TF_FAULT] = p_tel->fault_mask;
    p_vals[TF_LINE] = p_tel->line_state;
    p_vals[TF_LETTER] = p_tel->bc_letter;
    p_vals[TF_BC_CMD] = p_tel->bc_cmd;
    p_vals[TF_BC_DIR] = p_tel->b_bc_dir ? 1 : 0;
    p_vals[TF_SPEED_L] = p_tel->speed_l_mms;
    p_vals[TF_SPEED_R] = p_tel->speed_r_mms;
    p_vals[TF_COUNT_L] = p_tel->count_l;
    p_vals[TF_COUNT_R] = p_tel->count_r;
    p_vals[TF_DIST] = p_tel->dist_mm;
    p_vals[TF_PITCH] = p_tel->pitch_hdeg;
    p_vals[TF_ROLL] = p_tel->roll_hdeg;
    p_vals[TF_HUMP_ST] = p_tel->hump_state;
    p_vals[TF_PEAK] = p_tel->peak_mm;
    p_vals[TF_HUMP_CNT] = p_tel->hump_count;
    p_vals[TF_MCLASS] = p_tel->motion_class;
    p_vals[TF_TURN] = p_tel->turn_dps;
    p_vals[TF_IMU_OK] = p_tel->b_imu_ok ? 1 : 0;
    p_vals[TF_CAL_DONE] = p_tel->b_cal_done ? 1 : 0;
    p_vals[TF_CLOSEST] = p_tel->closest_mm;
    p_vals[TF_ANGLE] = p_tel->angle_idx;
    p_vals[TF_WIDTH] = p_tel->width_mm;
    p_vals[TF_CLEAR_L] = p_tel->clear_l_mm;
    p_vals[TF_CLEAR_R] = p_tel->clear_r_mm;
}

/*!
 * @brief Set every telemetry field from an array indexed by tel_field_t.
 *
 * NOTE: Every value here either came from tel_to_array() of a real
 * struct, or is a case value from g_tel_clamps or g_tel_wraps, and those
 * tables only hold values within the range of the member's own C type
 * (for example 999 for the uint16_t peak_mm, 0x12345678 for the uint32_t
 * count_l). Each cast below therefore keeps the value unchanged; the
 * codec, not this conversion, is what must clamp or wrap it.
 *
 * @param[in]  p_vals TF_NONE values, each within its member's C range.
 * @param[out] p_tel  Receives the fields.
 */
static void
tel_from_array (const int64_t * p_vals, msg_codec_telemetry_t * p_tel)
{
    p_tel->seq = (uint8_t)p_vals[TF_SEQ];
    p_tel->ctrl_state = (uint8_t)p_vals[TF_CTRL];
    p_tel->motion_state = (uint8_t)p_vals[TF_MOTION];
    p_tel->fault_mask = (uint8_t)p_vals[TF_FAULT];
    p_tel->line_state = (uint8_t)p_vals[TF_LINE];
    p_tel->bc_letter = (uint8_t)p_vals[TF_LETTER];
    p_tel->bc_cmd = (uint8_t)p_vals[TF_BC_CMD];
    p_tel->b_bc_dir = (0 != p_vals[TF_BC_DIR]);
    p_tel->speed_l_mms = (int32_t)p_vals[TF_SPEED_L];
    p_tel->speed_r_mms = (int32_t)p_vals[TF_SPEED_R];
    p_tel->count_l = (uint32_t)p_vals[TF_COUNT_L];
    p_tel->count_r = (uint32_t)p_vals[TF_COUNT_R];
    p_tel->dist_mm = (int32_t)p_vals[TF_DIST];
    p_tel->pitch_hdeg = (int16_t)p_vals[TF_PITCH];
    p_tel->roll_hdeg = (int16_t)p_vals[TF_ROLL];
    p_tel->hump_state = (uint8_t)p_vals[TF_HUMP_ST];
    p_tel->peak_mm = (uint16_t)p_vals[TF_PEAK];
    p_tel->hump_count = (uint8_t)p_vals[TF_HUMP_CNT];
    p_tel->motion_class = (uint8_t)p_vals[TF_MCLASS];
    p_tel->turn_dps = (int16_t)p_vals[TF_TURN];
    p_tel->b_imu_ok = (0 != p_vals[TF_IMU_OK]);
    p_tel->b_cal_done = (0 != p_vals[TF_CAL_DONE]);
    p_tel->closest_mm = (uint16_t)p_vals[TF_CLOSEST];
    p_tel->angle_idx = (uint8_t)p_vals[TF_ANGLE];
    p_tel->width_mm = (uint16_t)p_vals[TF_WIDTH];
    p_tel->clear_l_mm = (uint16_t)p_vals[TF_CLEAR_L];
    p_tel->clear_r_mm = (uint16_t)p_vals[TF_CLEAR_R];
}

/*!
 * @brief Read one telemetry field as a signed 64-bit value.
 *
 * @param[in] p_tel The fields.
 * @param[in] field Which field.
 *
 * @return The value (0 for TF_NONE).
 */
static int64_t
tel_get (const msg_codec_telemetry_t * p_tel, tel_field_t field)
{
    int64_t vals[TF_NONE];
    int64_t value = 0;

    tel_to_array(p_tel, vals);

    if (field < TF_NONE)
    {
        value = vals[field];
    }

    return (value);
}

/*!
 * @brief Set one telemetry field from a signed 64-bit value.
 *
 * @param[out] p_tel The fields.
 * @param[in]  field Which field (TF_NONE changes nothing).
 * @param[in]  value New value, within the member's C type range (see
 *                   tel_from_array()).
 */
static void
tel_set (msg_codec_telemetry_t * p_tel, tel_field_t field, int64_t value)
{
    int64_t vals[TF_NONE];

    tel_to_array(p_tel, vals);

    if (field < TF_NONE)
    {
        vals[field] = value;
    }

    tel_from_array(vals, p_tel);
}

/*!
 * @brief Compare two sets of telemetry fields, one field excepted.
 *
 * @param[in] p_a  First set.
 * @param[in] p_b  Second set.
 * @param[in] skip Field not compared, or TF_NONE to compare all.
 *
 * @return True if every compared field is equal.
 */
static bool
tel_same (const msg_codec_telemetry_t * p_a,
          const msg_codec_telemetry_t * p_b, tel_field_t skip)
{
    int64_t  vals_a[TF_NONE];
    int64_t  vals_b[TF_NONE];
    bool     b_same = true;
    uint32_t idx;

    tel_to_array(p_a, vals_a);
    tel_to_array(p_b, vals_b);

    for (idx = 0U; idx < (uint32_t)TF_NONE; idx++)
    {
        if ((idx != (uint32_t)skip) && (vals_a[idx] != vals_b[idx]))
        {
            b_same = false;
        }
    }

    return (b_same);
}

/*!
 * @brief Build a vector's input: its fields, with pitch and roll converted
 *        from centidegrees by the codec.
 *
 * @param[in]  p_vec The vector.
 * @param[out] p_in  Receives the input fields.
 */
static void
tel_input (const codec_vectors_tel_t * p_vec, msg_codec_telemetry_t * p_in)
{
    *p_in = p_vec->in;
    p_in->pitch_hdeg = msg_codec_cdeg_to_hdeg(p_vec->pitch_cdeg);
    p_in->roll_hdeg = msg_codec_cdeg_to_hdeg(p_vec->roll_cdeg);
}

/*!
 * @brief Find a telemetry vector by its label.
 *
 * @param[in] p_label Label, for example "TEL-2".
 *
 * @return The vector, or NULL if there is none.
 */
static const codec_vectors_tel_t *
tel_find (const char * p_label)
{
    uint32_t                    count = 0U;
    const codec_vectors_tel_t * p_vec = codec_vectors_tel(&count);
    const codec_vectors_tel_t * p_found = NULL;
    uint32_t                    idx;

    for (idx = 0U; idx < count; idx++)
    {
        if ((NULL == p_found) && (0 == strcmp(p_vec[idx].p_label, p_label)))
        {
            p_found = &p_vec[idx];
        }
    }

    return (p_found);
}

/*!
 * @brief Encode into an exactly sized buffer, then decode it again.
 *
 * @param[in]  p_in    Fields to encode.
 * @param[out] p_out   Receives the decoded fields.
 * @param[out] p_bytes Receives the encoded bytes (MSG_TEL_BYTES), or NULL.
 *
 * @return True if both calls succeeded.
 */
static bool
tel_round_trip (const msg_codec_telemetry_t * p_in,
                msg_codec_telemetry_t * p_out, uint8_t * p_bytes)
{
    bool      b_ok = false;
    uint8_t * p_buf = alloc_copy(NULL, MSG_TEL_BYTES, 0U);

    if (NULL != p_buf)
    {
        b_ok = ((MSG_TEL_BYTES
                 == msg_codec_put_telemetry(p_in, p_buf, MSG_TEL_BYTES))
                && msg_codec_get_telemetry(p_buf, MSG_TEL_BYTES, p_out));

        if (NULL != p_bytes)
        {
            (void)memcpy(p_bytes, p_buf, MSG_TEL_BYTES);
        }

        free(p_buf);
    }

    return (b_ok);
}

/*!
 * @brief Heartbeat round trip through an exactly sized buffer.
 *
 * @param[in]  p_in  Fields to encode.
 * @param[out] p_out Receives the decoded fields.
 *
 * @return True if both calls succeeded.
 */
static bool
hb_round_trip (const msg_codec_heartbeat_t * p_in,
               msg_codec_heartbeat_t * p_out)
{
    bool      b_ok = false;
    uint8_t * p_buf = alloc_copy(NULL, MSG_HB_BYTES, 0U);

    if (NULL != p_buf)
    {
        b_ok = ((MSG_HB_BYTES
                 == msg_codec_put_heartbeat(p_in, p_buf, MSG_HB_BYTES))
                && msg_codec_get_heartbeat(p_buf, MSG_HB_BYTES, p_out));
        free(p_buf);
    }

    return (b_ok);
}

/*!
 * @brief Event round trip through an exactly sized buffer.
 *
 * @param[in]  p_in  Fields to encode.
 * @param[out] p_out Receives the decoded fields.
 *
 * @return True if both calls succeeded.
 */
static bool
evt_round_trip (const msg_codec_event_t * p_in, msg_codec_event_t * p_out)
{
    bool      b_ok = false;
    uint8_t * p_buf = alloc_copy(NULL, MSG_EVT_BYTES, 0U);

    if (NULL != p_buf)
    {
        b_ok = ((MSG_EVT_BYTES
                 == msg_codec_put_event(p_in, p_buf, MSG_EVT_BYTES))
                && msg_codec_get_event(p_buf, MSG_EVT_BYTES, p_out));
        free(p_buf);
    }

    return (b_ok);
}

/*!
 * @brief Command round trip through an exactly sized buffer.
 *
 * @param[in]  p_in  Fields to encode.
 * @param[out] p_out Receives the decoded fields.
 *
 * @return True if the put succeeded and get accepted the command.
 */
static bool
cmd_round_trip (const msg_codec_command_t * p_in,
                msg_codec_command_t * p_out)
{
    bool      b_ok = false;
    uint8_t * p_buf = alloc_copy(NULL, MSG_CMD_BYTES, 0U);

    if (NULL != p_buf)
    {
        b_ok = ((MSG_CMD_BYTES
                 == msg_codec_put_command(p_in, p_buf, MSG_CMD_BYTES))
                && msg_codec_get_command(p_buf, MSG_CMD_BYTES, p_out));
        free(p_buf);
    }

    return (b_ok);
}

/*!
 * @brief Ack round trip through an exactly sized buffer.
 *
 * @param[in]  p_in  Fields to encode.
 * @param[out] p_out Receives the decoded fields.
 *
 * @return True if both calls succeeded.
 */
static bool
ack_round_trip (const msg_codec_ack_t * p_in, msg_codec_ack_t * p_out)
{
    bool      b_ok = false;
    uint8_t * p_buf = alloc_copy(NULL, MSG_ACK_BYTES, 0U);

    if (NULL != p_buf)
    {
        b_ok = ((MSG_ACK_BYTES
                 == msg_codec_put_ack(p_in, p_buf, MSG_ACK_BYTES))
                && msg_codec_get_ack(p_buf, MSG_ACK_BYTES, p_out));
        free(p_buf);
    }

    return (b_ok);
}

/*!
 * @brief Compare two heartbeats field by field.
 *
 * @param[in] p_a First.
 * @param[in] p_b Second.
 *
 * @return True if every field is equal.
 */
static bool
hb_same (const msg_codec_heartbeat_t * p_a, const msg_codec_heartbeat_t * p_b)
{
    return ((p_a->seq == p_b->seq) && (p_a->uptime_s == p_b->uptime_s)
            && (p_a->rssi_dbm == p_b->rssi_dbm)
            && (p_a->connect_count == p_b->connect_count)
            && (p_a->tel_drop == p_b->tel_drop)
            && (p_a->evt_drop == p_b->evt_drop)
            && (p_a->cmd_reject == p_b->cmd_reject));
}

/*!
 * @brief Compare two events field by field.
 *
 * @param[in] p_a First.
 * @param[in] p_b Second.
 *
 * @return True if every field is equal.
 */
static bool
evt_same (const msg_codec_event_t * p_a, const msg_codec_event_t * p_b)
{
    return ((p_a->event.code == p_b->event.code)
            && (0 == memcmp(p_a->event.args, p_b->event.args,
                            MSG_EVENT_ARG_BYTES))
            && (p_a->t_ms == p_b->t_ms));
}

/*!
 * @brief Compare two acks field by field.
 *
 * @param[in] p_a First.
 * @param[in] p_b Second.
 *
 * @return True if every field is equal.
 */
static bool
ack_same (const msg_codec_ack_t * p_a, const msg_codec_ack_t * p_b)
{
    return ((p_a->opcode == p_b->opcode) && (p_a->cmd_id == p_b->cmd_id)
            && (p_a->result == p_b->result));
}

/*!
 * @brief Reference rebuild of a full encoder count from its low 16 bits,
 *        the rule the dashboard decoder must also use (doc section 10.1).
 *
 * The count only goes up, so the step is the difference modulo 65536.
 *
 * @param[in] prev_full The previous full count.
 * @param[in] low16     The low 16 bits just received.
 *
 * @return The new full count.
 */
static uint32_t
unwrap16 (uint32_t prev_full, uint32_t low16)
{
    return (prev_full + ((low16 - (prev_full & 0xFFFFU)) & 0xFFFFU));
}

/*!
 * @brief Put into a buffer of exactly len bytes and compare the result.
 *
 * @param[in] put      The put adapter.
 * @param[in] p_in     Fields to encode.
 * @param[in] len      Message length.
 * @param[in] p_expect Expected bytes.
 *
 * @return True if put returned len and wrote exactly p_expect.
 */
static bool
put_matches (put_fn_t put, const void * p_in, uint32_t len,
             const uint8_t * p_expect)
{
    uint8_t * p_buf = alloc_copy(NULL, len, 0U);
    bool      b_ok = (NULL != p_buf)
                     && (len == put(p_in, p_buf, len))
                     && (0 == memcmp(p_buf, p_expect, len));

    free(p_buf);

    return (b_ok);
}

/*!
 * @brief Get from a copy of the bytes in a buffer of exactly len bytes.
 *
 * @param[in]  get     The get adapter.
 * @param[in]  p_bytes Bytes to decode.
 * @param[in]  len     Message length.
 * @param[out] p_out   Receives the fields.
 *
 * @return What get returned (false if malloc failed).
 */
static bool
get_exact (get_fn_t get, const uint8_t * p_bytes, uint32_t len,
           void * p_out)
{
    uint8_t * p_buf = alloc_copy(p_bytes, len, len);
    bool      b_ok = (NULL != p_buf) && get(p_buf, len, p_out);

    free(p_buf);

    return (b_ok);
}

/*!
 * @brief Adapter: telemetry get with an untyped output pointer.
 */
static bool
get_tel (const uint8_t * p_buf, uint32_t len, void * p_out)
{
    msg_codec_telemetry_t * p_tel = p_out;

    return (msg_codec_get_telemetry(p_buf, len, p_tel));
}

/*!
 * @brief Adapter: heartbeat get with an untyped output pointer.
 */
static bool
get_hb (const uint8_t * p_buf, uint32_t len, void * p_out)
{
    msg_codec_heartbeat_t * p_hb = p_out;

    return (msg_codec_get_heartbeat(p_buf, len, p_hb));
}

/*!
 * @brief Adapter: event get with an untyped output pointer.
 */
static bool
get_evt (const uint8_t * p_buf, uint32_t len, void * p_out)
{
    msg_codec_event_t * p_evt = p_out;

    return (msg_codec_get_event(p_buf, len, p_evt));
}

/*!
 * @brief Adapter: command get with an untyped output pointer.
 */
static bool
get_cmd (const uint8_t * p_buf, uint32_t len, void * p_out)
{
    msg_codec_command_t * p_cmd = p_out;

    return (msg_codec_get_command(p_buf, len, p_cmd));
}

/*!
 * @brief Adapter: ack get with an untyped output pointer.
 */
static bool
get_ack (const uint8_t * p_buf, uint32_t len, void * p_out)
{
    msg_codec_ack_t * p_ack = p_out;

    return (msg_codec_get_ack(p_buf, len, p_ack));
}

/*!
 * @brief Adapter: telemetry put with an untyped input pointer.
 */
static uint32_t
put_tel (const void * p_in, uint8_t * p_buf, uint32_t buf_len)
{
    const msg_codec_telemetry_t * p_tel = p_in;

    return (msg_codec_put_telemetry(p_tel, p_buf, buf_len));
}

/*!
 * @brief Adapter: heartbeat put with an untyped input pointer.
 */
static uint32_t
put_hb (const void * p_in, uint8_t * p_buf, uint32_t buf_len)
{
    const msg_codec_heartbeat_t * p_hb = p_in;

    return (msg_codec_put_heartbeat(p_hb, p_buf, buf_len));
}

/*!
 * @brief Adapter: event put with an untyped input pointer.
 */
static uint32_t
put_evt (const void * p_in, uint8_t * p_buf, uint32_t buf_len)
{
    const msg_codec_event_t * p_evt = p_in;

    return (msg_codec_put_event(p_evt, p_buf, buf_len));
}

/*!
 * @brief Adapter: command put with an untyped input pointer.
 */
static uint32_t
put_cmd (const void * p_in, uint8_t * p_buf, uint32_t buf_len)
{
    const msg_codec_command_t * p_cmd = p_in;

    return (msg_codec_put_command(p_cmd, p_buf, buf_len));
}

/*!
 * @brief Adapter: ack put with an untyped input pointer.
 */
static uint32_t
put_ack (const void * p_in, uint8_t * p_buf, uint32_t buf_len)
{
    const msg_codec_ack_t * p_ack = p_in;

    return (msg_codec_put_ack(p_ack, p_buf, buf_len));
}

/*!
 * @brief Golden telemetry vectors: exact bytes out, exact fields back.
 */
static void
test_golden_tel (void)
{
    uint32_t                    count = 0U;
    const codec_vectors_tel_t * p_vec = codec_vectors_tel(&count);
    msg_codec_telemetry_t       in;
    any_out_t                   out;
    uint32_t                    idx;

    check(9U == count, "telemetry vector count is 9", "TEL");

    for (idx = 0U; idx < count; idx++)
    {
        tel_input(&p_vec[idx], &in);
        check(put_matches(put_tel, &in, MSG_TEL_BYTES, p_vec[idx].bytes),
              "put gives the document's bytes", p_vec[idx].p_label);
        check(get_exact(get_tel, p_vec[idx].bytes, MSG_TEL_BYTES, &out)
              && tel_same(&out.tel, &p_vec[idx].out, TF_NONE),
              "get gives the expected fields", p_vec[idx].p_label);
    }
}

/*!
 * @brief Golden heartbeat, event, command and ack vectors.
 */
static void
test_golden_other (void)
{
    uint32_t                    n_hb = 0U;
    uint32_t                    n_evt = 0U;
    uint32_t                    n_cmd = 0U;
    uint32_t                    n_ack = 0U;
    uint32_t                    n_tel = 0U;
    const codec_vectors_hb_t *  p_hb = codec_vectors_hb(&n_hb);
    const codec_vectors_evt_t * p_evt = codec_vectors_evt(&n_evt);
    const codec_vectors_cmd_t * p_cmd = codec_vectors_cmd(&n_cmd);
    const codec_vectors_ack_t * p_ack = codec_vectors_ack(&n_ack);
    any_out_t                   out;
    uint32_t                    idx;

    (void)codec_vectors_tel(&n_tel);
    check(GOLDEN_COUNT == (n_tel + n_hb + n_evt + n_cmd + n_ack),
          "18 golden vectors in total", "all");

    for (idx = 0U; idx < n_hb; idx++)
    {
        check(put_matches(put_hb, &p_hb[idx].in, MSG_HB_BYTES,
                          p_hb[idx].bytes)
              && get_exact(get_hb, p_hb[idx].bytes, MSG_HB_BYTES, &out)
              && hb_same(&out.hb, &p_hb[idx].out),
              "bytes and fields match", p_hb[idx].p_label);
    }

    for (idx = 0U; idx < n_evt; idx++)
    {
        check(put_matches(put_evt, &p_evt[idx].in, MSG_EVT_BYTES,
                          p_evt[idx].bytes)
              && get_exact(get_evt, p_evt[idx].bytes, MSG_EVT_BYTES, &out)
              && evt_same(&out.evt, &p_evt[idx].out),
              "bytes and fields match", p_evt[idx].p_label);
    }

    for (idx = 0U; idx < n_cmd; idx++)
    {
        check(put_matches(put_cmd, &p_cmd[idx].in, MSG_CMD_BYTES,
                          p_cmd[idx].bytes)
              && (p_cmd[idx].b_get_ok
                  == get_exact(get_cmd, p_cmd[idx].bytes, MSG_CMD_BYTES,
                               &out))
              && (out.cmd.opcode == p_cmd[idx].out.opcode)
              && (out.cmd.cmd_id == p_cmd[idx].out.cmd_id),
              "bytes, result and fields match", p_cmd[idx].p_label);
    }

    for (idx = 0U; idx < n_ack; idx++)
    {
        check(put_matches(put_ack, &p_ack[idx].in, MSG_ACK_BYTES,
                          p_ack[idx].bytes)
              && get_exact(get_ack, p_ack[idx].bytes, MSG_ACK_BYTES, &out)
              && ack_same(&out.ack, &p_ack[idx].out),
              "bytes and fields match", p_ack[idx].p_label);
    }
}

/*!
 * @brief The MSG_FAULT_xxx constants reproduce TEL-F0 to TEL-F4.
 */
static void
test_fault_constants (void)
{
    static const uint8_t      faults[5] =
    {
        MSG_FAULT_STALL, MSG_FAULT_ENCODER, MSG_FAULT_IMU,
        MSG_FAULT_NO_ECHO, MSG_FAULT_WIFI
    };
    static const char * const labels[5] =
    {
        "TEL-F0", "TEL-F1", "TEL-F2", "TEL-F3", "TEL-F4"
    };
    const codec_vectors_tel_t * p_base = tel_find("TEL-2");
    const codec_vectors_tel_t * p_vec;
    msg_codec_telemetry_t       in;
    msg_codec_telemetry_t       out;
    uint8_t                     bytes[MSG_TEL_BYTES];
    uint32_t                    idx;

    check(NULL != p_base, "TEL-2 vector present", "TEL-2");

    for (idx = 0U; (NULL != p_base) && (idx < 5U); idx++)
    {
        p_vec = tel_find(labels[idx]);
        tel_input(p_base, &in);
        in.fault_mask = faults[idx];
        check((NULL != p_vec) && tel_round_trip(&in, &out, bytes)
              && (0 == memcmp(bytes, p_vec->bytes, MSG_TEL_BYTES))
              && (out.fault_mask == (1U << idx)),
              "MSG_FAULT constant gives the one-hot vector", labels[idx]);
    }
}

/*!
 * @brief The 17 rounding cases of section 3.3, through pitch and roll.
 */
static void
test_rounding (void)
{
    const codec_vectors_tel_t * p_base = tel_find("TEL-2");
    msg_codec_telemetry_t       in;
    msg_codec_telemetry_t       out;
    uint8_t                     bytes[MSG_TEL_BYTES];
    const uint32_t              count =
        (uint32_t)(sizeof(g_round_cases) / sizeof(g_round_cases[0]));
    int16_t                     hdeg;
    uint32_t                    idx;

    check(17U == count, "17 rounding cases", "3.3");

    for (idx = 0U; (NULL != p_base) && (idx < count); idx++)
    {
        hdeg = msg_codec_cdeg_to_hdeg(g_round_cases[idx].cdeg);
        check(hdeg == g_round_cases[idx].hdeg, "cdeg_to_hdeg result",
              "3.3 table row");

        tel_input(p_base, &in);
        in.pitch_hdeg = hdeg;
        in.roll_hdeg = hdeg;
        check(tel_round_trip(&in, &out, bytes)
              && (g_round_cases[idx].stored
                  == read_bits(bytes, MSG_TEL_PITCH_POS, MSG_TEL_PITCH_BITS))
              && (g_round_cases[idx].stored
                  == read_bits(bytes, MSG_TEL_ROLL_POS, MSG_TEL_ROLL_BITS))
              && (out.pitch_hdeg == g_round_cases[idx].hdeg)
              && (out.roll_hdeg == g_round_cases[idx].hdeg),
              "stored 7-bit value and decoded value", "3.3 table row");
    }
}

/*!
 * @brief Every heartbeat, event, command and ack field at its minimum and
 *        maximum (telemetry: vectors TEL-2 and TEL-3).
 */
static void
test_min_max (void)
{
    static const msg_codec_heartbeat_t hb[2] =
    {
        { 0U, 0U, 0, 0U, 0U, 0U, 0U },
        { 255U, 65535U, -127, 255U, 255U, 255U, 255U }
    };
    static const msg_codec_event_t     evt[2] =
    {
        { { 0U, { 0U, 0U } }, 0U },
        { { 255U, { 255U, 255U } }, 0xFFFFFFU }
    };
    any_out_t           out;
    msg_codec_command_t cmd;
    msg_codec_ack_t     ack;
    uint32_t            idx;

    for (idx = 0U; idx < 2U; idx++)
    {
        check(hb_round_trip(&hb[idx], &out.hb) && hb_same(&out.hb, &hb[idx]),
              "heartbeat round trip", (0U == idx) ? "min" : "max");
        check(evt_round_trip(&evt[idx], &out.evt)
              && evt_same(&out.evt, &evt[idx]),
              "event round trip", (0U == idx) ? "min" : "max");
    }

    for (idx = 0U; idx < 6U; idx++)
    {
        /* idx / 2 is 0 to 2, so the opcode is 1 to 3 and fits uint8_t. */
        cmd.opcode = (uint8_t)(1U + (idx / 2U));
        cmd.cmd_id = (0U == (idx % 2U)) ? 0U : 63U;
        check(cmd_round_trip(&cmd, &out.cmd)
              && (out.cmd.opcode == cmd.opcode)
              && (out.cmd.cmd_id == cmd.cmd_id),
              "command opcode 1 to 3, id 0 and 63", "CMD");
    }

    for (idx = 0U; idx < 32U; idx++)
    {
        /* idx is 0 to 31: opcode and result are 0 to 3, the id 0 or 63,
         * so every value fits uint8_t.
         */
        ack.opcode = (uint8_t)(idx % 4U);
        ack.result = (uint8_t)((idx / 4U) % 4U);
        ack.cmd_id = (idx < 16U) ? 0U : 63U;
        check(ack_round_trip(&ack, &out.ack) && ack_same(&out.ack, &ack),
              "ack opcode, id and result range", "ACK");
    }
}

/*!
 * @brief Wrap cases for seq, t_ms, cmd_id and the uptime saturation
 *        (count and fault_mask cases are in g_tel_wraps).
 */
static void
test_wrap (void)
{
    const codec_vectors_tel_t * p_base = tel_find("TEL-1");
    static const uint32_t       t_in[3] = { 16777215U, 16777216U,
                                            0x01ABCDEFU };
    static const uint32_t       t_out[3] = { 16777215U, 0U, 0xABCDEFU };
    static const uint32_t       up_in[3] = { 65535U, 65536U, 70000U };
    msg_codec_telemetry_t       tel;
    msg_codec_heartbeat_t       hb = { 255U, 0U, 0, 0U, 0U, 0U, 0U };
    msg_codec_event_t           evt = { { 1U, { 2U, 3U } }, 0U };
    msg_codec_command_t         cmd = { MSG_OP_STOP, 63U };
    msg_codec_ack_t             ack = { MSG_OP_STOP, 63U, MSG_ACK_ACCEPTED };
    any_out_t                   out;
    uint32_t                    idx;

    check(NULL != p_base, "TEL-1 vector present", "TEL-1");
    if (NULL != p_base)
    {
        tel_input(p_base, &tel);
        tel.seq = 255U;
        check(tel_round_trip(&tel, &out.tel, NULL) && (255U == out.tel.seq),
              "telemetry seq 255", "seq");
        /* seq is 255, so seq + 1 is 256 and the frame counter wraps to 0
         * when it is narrowed back to uint8_t, as on the robot.
         */
        tel.seq = (uint8_t)(tel.seq + 1U);
        check(tel_round_trip(&tel, &out.tel, NULL) && (0U == out.tel.seq),
              "telemetry seq after 255 is 0", "seq");
    }

    check(hb_round_trip(&hb, &out.hb) && (255U == out.hb.seq),
          "heartbeat seq 255", "seq");
    /* Same wrap as the telemetry counter above: 255 + 1 narrows to 0. */
    hb.seq = (uint8_t)(hb.seq + 1U);
    check(hb_round_trip(&hb, &out.hb) && (0U == out.hb.seq),
          "heartbeat seq after 255 is 0", "seq");

    for (idx = 0U; idx < 3U; idx++)
    {
        evt.t_ms = t_in[idx];
        check(evt_round_trip(&evt, &out.evt) && (t_out[idx] == out.evt.t_ms),
              "t_ms keeps its low 24 bits", "t_ms");
        hb.uptime_s = up_in[idx];
        check(hb_round_trip(&hb, &out.hb) && (65535U == out.hb.uptime_s),
              "uptime_s saturates at 65535", "uptime_s");
    }

    check(cmd_round_trip(&cmd, &out.cmd) && (63U == out.cmd.cmd_id),
          "command cmd_id 63", "cmd_id");
    cmd.cmd_id = 64U;
    check(cmd_round_trip(&cmd, &out.cmd) && (0U == out.cmd.cmd_id),
          "command cmd_id 64 wraps to 0", "cmd_id");
    check(ack_round_trip(&ack, &out.ack) && (63U == out.ack.cmd_id),
          "ack cmd_id 63", "cmd_id");
    ack.cmd_id = 64U;
    check(ack_round_trip(&ack, &out.ack) && (0U == out.ack.cmd_id),
          "ack cmd_id 64 wraps to 0", "cmd_id");
}

/*!
 * @brief Rebuilding full encoder counts from the 16 bits sent.
 */
static void
test_reconstruction (void)
{
    const codec_vectors_tel_t * p_base = tel_find("TEL-1");
    static const uint32_t       prev[2] = { 65535U, 0x0001FFF0U };
    static const uint32_t       next[2] = { 65536U, 0x00020010U };
    static const uint32_t       low[2] = { 0U, 0x0010U };
    msg_codec_telemetry_t       tel;
    msg_codec_telemetry_t       out;
    uint32_t                    idx;

    for (idx = 0U; idx < 2U; idx++)
    {
        check(next[idx] == unwrap16(prev[idx], low[idx]),
              "reference unwrap gives the next full count", "unwrap16");
    }

    for (idx = 0U; (NULL != p_base) && (idx < 2U); idx++)
    {
        tel_input(p_base, &tel);
        tel.count_l = next[idx];
        tel.count_r = next[idx];
        check(tel_round_trip(&tel, &out, NULL)
              && (low[idx] == out.count_l) && (low[idx] == out.count_r)
              && (next[idx] == unwrap16(prev[idx], out.count_l))
              && (next[idx] == unwrap16(prev[idx], out.count_r)),
              "sent low bits rebuild the full count", "count_l/count_r");
    }
}

/*!
 * @brief Run telemetry cases that change one field of TEL-1: the field
 *        must decode as expected and every other field must not change.
 *
 * @param[in] p_cases The cases.
 * @param[in] count   How many.
 */
static void
test_tel_cases (const tel_case_t * p_cases, uint32_t count)
{
    const codec_vectors_tel_t * p_base = tel_find("TEL-1");
    msg_codec_telemetry_t       in;
    msg_codec_telemetry_t       out;
    uint32_t                    idx;

    for (idx = 0U; (NULL != p_base) && (idx < count); idx++)
    {
        tel_input(p_base, &in);
        tel_set(&in, p_cases[idx].field, p_cases[idx].value);
        check(tel_round_trip(&in, &out, NULL)
              && (p_cases[idx].expect == tel_get(&out, p_cases[idx].field))
              && tel_same(&out, &p_base->out, p_cases[idx].field),
              "field decodes as expected, others unchanged",
              p_cases[idx].p_label);
    }
}

/*!
 * @brief Heartbeat, command and ack clamp cases.
 */
static void
test_clamps_other (void)
{
    static const uint32_t counts[2] = { 256U, 0xFFFFFFFFU };
    msg_codec_heartbeat_t hb = { 42U, 1234U, -67, 3U, 1U, 0U, 2U };
    msg_codec_heartbeat_t expect;
    msg_codec_command_t   cmd = { 4U, 5U };
    msg_codec_ack_t       ack = { 2U, 5U, 4U };
    any_out_t             out;
    uint32_t              idx;

    expect = hb;
    hb.rssi_dbm = 5;
    expect.rssi_dbm = 0;
    check(hb_round_trip(&hb, &out.hb) && hb_same(&out.hb, &expect),
          "rssi +5 dBm sends 0", "rssi");
    hb.rssi_dbm = -128;
    expect.rssi_dbm = -127;
    check(hb_round_trip(&hb, &out.hb) && hb_same(&out.hb, &expect),
          "rssi -128 dBm clamps to -127", "rssi");
    hb.rssi_dbm = -67;
    expect.rssi_dbm = -67;

    for (idx = 0U; idx < 2U; idx++)
    {
        hb.connect_count = counts[idx];
        hb.tel_drop = counts[idx];
        hb.evt_drop = counts[idx];
        hb.cmd_reject = counts[idx];
        expect.connect_count = 255U;
        expect.tel_drop = 255U;
        expect.evt_drop = 255U;
        expect.cmd_reject = 255U;
        check(hb_round_trip(&hb, &out.hb) && hb_same(&out.hb, &expect),
              "heartbeat counters saturate at 255", "counters");
    }

    check(cmd_round_trip(&cmd, &out.cmd) && (3U == out.cmd.opcode)
          && (5U == out.cmd.cmd_id),
          "command put clamps opcode 4 to 3", "opcode");
    check(ack_round_trip(&ack, &out.ack) && (3U == out.ack.result)
          && (2U == out.ack.opcode) && (5U == out.ack.cmd_id),
          "ack result 4 clamps to 3", "result");
    ack.opcode = 4U;
    ack.result = 0U;
    check(ack_round_trip(&ack, &out.ack) && (3U == out.ack.opcode)
          && (0U == out.ack.result),
          "ack opcode 4 clamps to 3", "opcode");
}

/*!
 * @brief One get that must be rejected: false, and the output left alone
 *        (or zeroed, for a command).
 *
 * @param[in] p_msg  The message type.
 * @param[in] p_buf  Bytes to decode (may be NULL).
 * @param[in] len    Length passed to get.
 * @param[in] p_what Case name.
 */
static void
reject_get (const msg_desc_t * p_msg, const uint8_t * p_buf, uint32_t len,
            const char * p_what)
{
    any_out_t                 out;
    any_out_t                 before;
    static const msg_codec_command_t zero = { 0U, 0U };
    bool                      b_untouched;

    (void)memset(&out, (int)MARK_BYTE, sizeof(out));
    before = out;

    if (p_msg->b_zero_on_reject)
    {
        b_untouched = !p_msg->get(p_buf, len, &out)
                      && (out.cmd.opcode == zero.opcode)
                      && (out.cmd.cmd_id == zero.cmd_id);
    }
    else
    {
        b_untouched = !p_msg->get(p_buf, len, &out)
                      && (0 == memcmp(&out, &before, sizeof(out)));
    }

    check(b_untouched, p_what, p_msg->p_name);
}

/*!
 * @brief Puts that must write nothing: NULL input, NULL buffer, and a
 *        buffer one byte short, whose marker bytes must stay intact.
 *
 * @param[in] p_msg The message type.
 */
static void
reject_put (const msg_desc_t * p_msg)
{
    uint8_t * p_short = alloc_copy(NULL, p_msg->len - 1U, 0U);
    uint8_t * p_full = alloc_copy(NULL, p_msg->len, 0U);
    uint32_t  idx;
    bool      b_marks = (NULL != p_short);

    check(0U == p_msg->put(NULL, p_full, p_msg->len), "put with NULL input",
          p_msg->p_name);
    check(0U == p_msg->put(p_msg->p_valid_in, NULL, p_msg->len),
          "put with NULL buffer", p_msg->p_name);
    check((NULL != p_short)
          && (0U == p_msg->put(p_msg->p_valid_in, p_short, p_msg->len - 1U)),
          "put into a buffer one byte short returns 0", p_msg->p_name);

    for (idx = 0U; (NULL != p_short) && (idx < (p_msg->len - 1U)); idx++)
    {
        b_marks = b_marks && (MARK_BYTE == p_short[idx]);
    }

    check(b_marks, "short buffer left untouched", p_msg->p_name);
    free(p_short);
    free(p_full);
}

/*!
 * @brief Every reject case of section 10.1 for one message type.
 *
 * @param[in] p_msg The message type.
 */
static void
test_rejects_one (const msg_desc_t * p_msg)
{
    const uint32_t len = p_msg->len;
    uint8_t *      p_short = alloc_copy(p_msg->p_valid_bytes, len - 1U,
                                        len - 1U);
    uint8_t *      p_long = alloc_copy(p_msg->p_valid_bytes, len + 1U, len);
    uint8_t *      p_buf = alloc_copy(p_msg->p_valid_bytes, len, len);
    any_out_t      out;

    check((NULL != p_short) && (NULL != p_long) && (NULL != p_buf),
          "buffers allocated", p_msg->p_name);

    if ((NULL != p_short) && (NULL != p_long) && (NULL != p_buf))
    {
        reject_get(p_msg, p_short, len - 1U, "one byte short");
        reject_get(p_msg, p_long, len + 1U, "one byte long");

        /* version and type are 4-bit values, so the header byte built
         * from them is at most 0xFF.
         */
        p_buf[0] = (uint8_t)((BAD_VERSION_LOW << 4U) | p_msg->type);
        reject_get(p_msg, p_buf, len, "version 0");
        p_buf[0] = (uint8_t)((BAD_VERSION_HIGH << 4U) | p_msg->type);
        reject_get(p_msg, p_buf, len, "version 2");
        p_buf[0] = (uint8_t)((MSG_LAYOUT_VERSION << 4U) | RESERVED_TYPE);
        reject_get(p_msg, p_buf, len, "wrong type");
        reject_get(p_msg, NULL, len, "NULL buffer");

        (void)memcpy(p_buf, p_msg->p_valid_bytes, len);
        check(!p_msg->get(p_buf, len, NULL), "NULL output", p_msg->p_name);
        check(p_msg->get(p_buf, len, &out), "valid bytes still accepted",
              p_msg->p_name);
    }

    free(p_short);
    free(p_long);
    free(p_buf);
    reject_put(p_msg);
}

/*!
 * @brief Reject cases for all five message types, plus a command with
 *        opcode 0 (rejected, but its cmd_id is still returned).
 */
static void
test_rejects (void)
{
    uint32_t                    n = 0U;
    const codec_vectors_tel_t * p_tel = codec_vectors_tel(&n);
    const codec_vectors_hb_t *  p_hb = codec_vectors_hb(&n);
    const codec_vectors_evt_t * p_evt = codec_vectors_evt(&n);
    const codec_vectors_cmd_t * p_cmd = codec_vectors_cmd(&n);
    const codec_vectors_ack_t * p_ack = codec_vectors_ack(&n);
    const msg_desc_t            msgs[5] =
    {
        { "telemetry", MSG_TEL_BYTES, MSG_TYPE_TELEMETRY, get_tel, put_tel,
          false, &p_tel[0].in, p_tel[0].bytes },
        { "heartbeat", MSG_HB_BYTES, MSG_TYPE_HEARTBEAT, get_hb, put_hb,
          false, &p_hb[0].in, p_hb[0].bytes },
        { "event", MSG_EVT_BYTES, MSG_TYPE_EVENT, get_evt, put_evt,
          false, &p_evt[0].in, p_evt[0].bytes },
        { "command", MSG_CMD_BYTES, MSG_TYPE_COMMAND, get_cmd, put_cmd,
          true, &p_cmd[0].in, p_cmd[0].bytes },
        { "ack", MSG_ACK_BYTES, MSG_TYPE_ACK, get_ack, put_ack,
          false, &p_ack[0].in, p_ack[0].bytes },
    };
    static const uint8_t        op0[MSG_CMD_BYTES] = { 0x14U, 0x05U };
    msg_codec_command_t         cmd;
    uint32_t                    idx;

    for (idx = 0U; idx < 5U; idx++)
    {
        test_rejects_one(&msgs[idx]);
    }

    check(!msg_codec_get_command(op0, MSG_CMD_BYTES, &cmd)
          && (MSG_OP_INVALID == cmd.opcode) && (5U == cmd.cmd_id),
          "opcode 0 rejected, cmd_id still returned", "command");
}

/*** end of file ***/
