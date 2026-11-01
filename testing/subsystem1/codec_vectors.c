/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file codec_vectors.c
 *
 * @brief Golden vectors for the S1-U05 host test of msg_codec.
 *
 * @par
 * One entry per row of section 10.2 of docs/S1_binary_frame.md. "in" is
 * what the test passes to put (pitch and roll are given in centidegrees
 * and converted with msg_codec_cdeg_to_hdeg()); "out" is what get must
 * return, which differs from "in" wherever a vector clamps or wraps a
 * value; "bytes" is copied from the document. golden.py --check compares
 * these bytes with the document and with its own independent packing.
 *
 * @par Test code
 * This module is part of a host test and is never built into the
 * firmware.
 */

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "codec_vectors.h"

/* Telemetry: TEL-1, TEL-2, TEL-3, TEL-F0 to TEL-F4, TEL-INDEP. */
static const codec_vectors_tel_t g_tel[] =
{
    {
        .p_label = "TEL-1",
        .in =
        {
            .seq = 7U, .ctrl_state = 3U, .motion_state = 1U,
            .fault_mask = 0x04U, .line_state = 0U, .bc_letter = 11U,
            .bc_cmd = 2U, .b_bc_dir = true, .speed_l_mms = 300,
            .speed_r_mms = -120, .count_l = 0x00011170U,
            .count_r = 0x0001FFFFU, .dist_mm = -1234, .hump_state = 2U,
            .peak_mm = 37U, .hump_count = 3U, .motion_class = 4U,
            .turn_dps = -45, .b_imu_ok = true, .b_cal_done = true,
            .closest_mm = 350U, .angle_idx = 12U, .width_mm = 210U,
            .clear_l_mm = 400U, .clear_r_mm = 1023U,
        },
        .pitch_cdeg = -75,
        .roll_cdeg = 25,
        .out =
        {
            .seq = 7U, .ctrl_state = 3U, .motion_state = 1U,
            .fault_mask = 0x04U, .line_state = 0U, .bc_letter = 11U,
            .bc_cmd = 2U, .b_bc_dir = true, .speed_l_mms = 300,
            .speed_r_mms = -120, .count_l = 0x00001170U,
            .count_r = 0x0000FFFFU, .dist_mm = -1234, .pitch_hdeg = -2,
            .roll_hdeg = 1, .hump_state = 2U, .peak_mm = 37U,
            .hump_count = 3U, .motion_class = 4U, .turn_dps = -45,
            .b_imu_ok = true, .b_cal_done = true, .closest_mm = 350U,
            .angle_idx = 12U, .width_mm = 210U, .clear_l_mm = 400U,
            .clear_r_mm = 1023U,
        },
        .bytes =
        {
            0x11U, 0x07U, 0x32U, 0x40U, 0x5DU, 0xCBU, 0x18U, 0x81U, 0x17U,
            0x0FU, 0xFFU, 0xF7U, 0xB2U, 0xE7U, 0xD0U, 0x62U, 0x53U, 0x8EU,
            0x9EU, 0xAFU, 0x30U, 0xD2U, 0x64U, 0x3FU, 0xF0U,
        },
    },
    {
        .p_label = "TEL-2",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x00U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x00U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-3",
        .in =
        {
            .seq = 255U, .ctrl_state = 15U, .motion_state = 7U,
            .fault_mask = 0x1FU, .line_state = 7U, .bc_letter = 63U,
            .bc_cmd = 3U, .b_bc_dir = true, .speed_l_mms = 511,
            .speed_r_mms = 511, .count_l = 0x0000FFFFU,
            .count_r = 0x0000FFFFU, .dist_mm = 32767, .hump_state = 3U,
            .peak_mm = 255U, .hump_count = 15U, .motion_class = 7U,
            .turn_dps = 511, .b_imu_ok = true, .b_cal_done = true,
            .closest_mm = 1023U, .angle_idx = 31U, .width_mm = 1023U,
            .clear_l_mm = 1023U, .clear_r_mm = 1023U,
        },
        .pitch_cdeg = 3150,
        .roll_cdeg = 3150,
        .out =
        {
            .seq = 255U, .ctrl_state = 15U, .motion_state = 7U,
            .fault_mask = 0x1FU, .line_state = 7U, .bc_letter = 63U,
            .bc_cmd = 3U, .b_bc_dir = true, .speed_l_mms = 511,
            .speed_r_mms = 511, .count_l = 0x0000FFFFU,
            .count_r = 0x0000FFFFU, .dist_mm = 32767, .pitch_hdeg = 63,
            .roll_hdeg = 63, .hump_state = 3U, .peak_mm = 255U,
            .hump_count = 15U, .motion_class = 7U, .turn_dps = 511,
            .b_imu_ok = true, .b_cal_done = true, .closest_mm = 1023U,
            .angle_idx = 31U, .width_mm = 1023U, .clear_l_mm = 1023U,
            .clear_r_mm = 1023U,
        },
        .bytes =
        {
            0x11U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
            0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU,
            0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xF0U,
        },
    },
    {
        .p_label = "TEL-F0",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x01U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x01U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x00U, 0x10U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-F1",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x02U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x02U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x00U, 0x20U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-F2",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x04U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x04U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x00U, 0x40U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-F3",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x08U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x08U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x00U, 0x80U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-F4",
        .in =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x10U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .hump_state = 0U, .peak_mm = 0U,
            .hump_count = 0U, .motion_class = 0U, .turn_dps = -512,
            .b_imu_ok = false, .b_cal_done = false, .closest_mm = 0U,
            .angle_idx = 0U, .width_mm = 0U, .clear_l_mm = 0U,
            .clear_r_mm = 0U,
        },
        .pitch_cdeg = -3200,
        .roll_cdeg = -3200,
        .out =
        {
            .seq = 0U, .ctrl_state = 0U, .motion_state = 0U,
            .fault_mask = 0x10U, .line_state = 0U, .bc_letter = 0U,
            .bc_cmd = 0U, .b_bc_dir = false, .speed_l_mms = -512,
            .speed_r_mms = -512, .count_l = 0x00U, .count_r = 0x00U,
            .dist_mm = -32768, .pitch_hdeg = -64, .roll_hdeg = -64,
            .hump_state = 0U, .peak_mm = 0U, .hump_count = 0U,
            .motion_class = 0U, .turn_dps = -512, .b_imu_ok = false,
            .b_cal_done = false, .closest_mm = 0U, .angle_idx = 0U,
            .width_mm = 0U, .clear_l_mm = 0U, .clear_r_mm = 0U,
        },
        .bytes =
        {
            0x11U, 0x00U, 0x01U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
            0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U, 0x00U,
        },
    },
    {
        .p_label = "TEL-INDEP",
        .in =
        {
            .seq = 165U, .ctrl_state = 9U, .motion_state = 5U,
            .fault_mask = 0x01U, .line_state = 6U, .bc_letter = 37U,
            .bc_cmd = 2U, .b_bc_dir = true, .speed_l_mms = -400,
            .speed_r_mms = 400, .count_l = 0x00071234U,
            .count_r = 0x0009ABCDU, .dist_mm = -1500, .hump_state = 2U,
            .peak_mm = 60U, .hump_count = 3U, .motion_class = 4U,
            .turn_dps = -90, .b_imu_ok = true, .b_cal_done = false,
            .closest_mm = 1500U, .angle_idx = 12U, .width_mm = 250U,
            .clear_l_mm = 0U, .clear_r_mm = 1023U,
        },
        .pitch_cdeg = 75,
        .roll_cdeg = -25,
        .out =
        {
            .seq = 165U, .ctrl_state = 9U, .motion_state = 5U,
            .fault_mask = 0x01U, .line_state = 6U, .bc_letter = 37U,
            .bc_cmd = 2U, .b_bc_dir = true, .speed_l_mms = -400,
            .speed_r_mms = 400, .count_l = 0x00001234U,
            .count_r = 0x0000ABCDU, .dist_mm = -1500, .pitch_hdeg = 2,
            .roll_hdeg = -1, .hump_state = 2U, .peak_mm = 60U,
            .hump_count = 3U, .motion_class = 4U, .turn_dps = -90,
            .b_imu_ok = true, .b_cal_done = false, .closest_mm = 1023U,
            .angle_idx = 12U, .width_mm = 250U, .clear_l_mm = 0U,
            .clear_r_mm = 1023U,
        },
        .bytes =
        {
            0x11U, 0xA5U, 0x9AU, 0x1DU, 0x2DU, 0x1CU, 0x39U, 0x01U, 0x23U,
            0x4AU, 0xBCU, 0xD7U, 0xA2U, 0x48U, 0x4FU, 0xE3U, 0xC3U, 0x8DU,
            0x35U, 0xFFU, 0xB0U, 0xFAU, 0x00U, 0x3FU, 0xF0U,
        },
    },
};

/* Heartbeat: HB-1. */
static const codec_vectors_hb_t g_hb[] =
{
    {
        .p_label = "HB-1",
        .in =
        {
            .seq = 42U, .uptime_s = 1234U, .rssi_dbm = -67,
            .connect_count = 3U, .tel_drop = 1U, .evt_drop = 0U,
            .cmd_reject = 2U,
        },
        .out =
        {
            .seq = 42U, .uptime_s = 1234U, .rssi_dbm = -67,
            .connect_count = 3U, .tel_drop = 1U, .evt_drop = 0U,
            .cmd_reject = 2U,
        },
        .bytes =
        {
            0x12U, 0x2AU, 0x04U, 0xD2U, 0x86U, 0x06U, 0x02U, 0x00U, 0x04U,
        },
    },
};

/* Event: EVT-1. */
static const codec_vectors_evt_t g_evt[] =
{
    {
        .p_label = "EVT-1",
        .in =
        {
            .event = { .code = 0x21U, .args = { 0x25U, 0x30U } },
            .t_ms = 123456U,
        },
        .out =
        {
            .event = { .code = 0x21U, .args = { 0x25U, 0x30U } },
            .t_ms = 123456U,
        },
        .bytes =
        {
            0x13U, 0x21U, 0x25U, 0x30U, 0x01U, 0xE2U, 0x40U,
        },
    },
};

/* Command: CMD-1 to CMD-4. CMD-4 has opcode 0, so get returns
 * false but still gives the cmd_id for the ack.
 */
static const codec_vectors_cmd_t g_cmd[] =
{
    {
        .p_label = "CMD-1",
        .in = { .opcode = 2U, .cmd_id = 5U },
        .b_get_ok = true,
        .out = { .opcode = 2U, .cmd_id = 5U },
        .bytes =
        {
            0x14U, 0x85U,
        },
    },
    {
        .p_label = "CMD-2",
        .in = { .opcode = 1U, .cmd_id = 0U },
        .b_get_ok = true,
        .out = { .opcode = 1U, .cmd_id = 0U },
        .bytes =
        {
            0x14U, 0x40U,
        },
    },
    {
        .p_label = "CMD-3",
        .in = { .opcode = 3U, .cmd_id = 63U },
        .b_get_ok = true,
        .out = { .opcode = 3U, .cmd_id = 63U },
        .bytes =
        {
            0x14U, 0xFFU,
        },
    },
    {
        .p_label = "CMD-4",
        .in = { .opcode = 0U, .cmd_id = 5U },
        .b_get_ok = false,
        .out = { .opcode = 0U, .cmd_id = 5U },
        .bytes =
        {
            0x14U, 0x05U,
        },
    },
};

/* Ack: ACK-1 to ACK-3. */
static const codec_vectors_ack_t g_ack[] =
{
    {
        .p_label = "ACK-1",
        .in = { .opcode = 2U, .cmd_id = 5U, .result = 0U },
        .out = { .opcode = 2U, .cmd_id = 5U, .result = 0U },
        .bytes =
        {
            0x15U, 0x85U, 0x00U,
        },
    },
    {
        .p_label = "ACK-2",
        .in = { .opcode = 0U, .cmd_id = 5U, .result = 1U },
        .out = { .opcode = 0U, .cmd_id = 5U, .result = 1U },
        .bytes =
        {
            0x15U, 0x05U, 0x40U,
        },
    },
    {
        .p_label = "ACK-3",
        .in = { .opcode = 0U, .cmd_id = 0U, .result = 1U },
        .out = { .opcode = 0U, .cmd_id = 0U, .result = 1U },
        .bytes =
        {
            0x15U, 0x00U, 0x40U,
        },
    },
};

/*!
 * @brief Return the telemetry vectors.
 *
 * @param[out] p_count Receives the number of vectors (ignored if NULL).
 *
 * @return The first vector.
 */
const codec_vectors_tel_t *
codec_vectors_tel (uint32_t * p_count)
{
    if (NULL != p_count)
    {
        /* The table holds a handful of entries, so its count fits
         * uint32_t.
         */
        *p_count = (uint32_t)(sizeof(g_tel) / sizeof(g_tel[0]));
    }

    return (g_tel);
}

/*!
 * @brief Return the heartbeat vectors.
 *
 * @param[out] p_count Receives the number of vectors (ignored if NULL).
 *
 * @return The first vector.
 */
const codec_vectors_hb_t *
codec_vectors_hb (uint32_t * p_count)
{
    if (NULL != p_count)
    {
        /* The table holds a handful of entries, so its count fits
         * uint32_t.
         */
        *p_count = (uint32_t)(sizeof(g_hb) / sizeof(g_hb[0]));
    }

    return (g_hb);
}

/*!
 * @brief Return the event vectors.
 *
 * @param[out] p_count Receives the number of vectors (ignored if NULL).
 *
 * @return The first vector.
 */
const codec_vectors_evt_t *
codec_vectors_evt (uint32_t * p_count)
{
    if (NULL != p_count)
    {
        /* The table holds a handful of entries, so its count fits
         * uint32_t.
         */
        *p_count = (uint32_t)(sizeof(g_evt) / sizeof(g_evt[0]));
    }

    return (g_evt);
}

/*!
 * @brief Return the command vectors.
 *
 * @param[out] p_count Receives the number of vectors (ignored if NULL).
 *
 * @return The first vector.
 */
const codec_vectors_cmd_t *
codec_vectors_cmd (uint32_t * p_count)
{
    if (NULL != p_count)
    {
        /* The table holds a handful of entries, so its count fits
         * uint32_t.
         */
        *p_count = (uint32_t)(sizeof(g_cmd) / sizeof(g_cmd[0]));
    }

    return (g_cmd);
}

/*!
 * @brief Return the ack vectors.
 *
 * @param[out] p_count Receives the number of vectors (ignored if NULL).
 *
 * @return The first vector.
 */
const codec_vectors_ack_t *
codec_vectors_ack (uint32_t * p_count)
{
    if (NULL != p_count)
    {
        /* The table holds a handful of entries, so its count fits
         * uint32_t.
         */
        *p_count = (uint32_t)(sizeof(g_ack) / sizeof(g_ack[0]));
    }

    return (g_ack);
}

/*** end of file ***/
