/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file codec_vectors.h
 *
 * @brief Golden vectors for the S1-U05 host test of msg_codec.
 *
 * @par Source
 * Each vector is a row of section 10.2 of docs/S1_binary_frame.md: the
 * field values and the exact bytes. The bytes are copied from that table;
 * testing/subsystem1/golden.py --check confirms that the document, this
 * file and its own independent packing all agree.
 *
 * @par Test code
 * This module is part of a host test and is never built into the
 * firmware.
 */

#ifndef CODEC_VECTORS_H
#define CODEC_VECTORS_H

#include <stdbool.h>
#include <stdint.h>

#include "msg_codec.h"

/** One telemetry vector. */
typedef struct
{
    const char *            p_label;    /* Short ID from doc 10.2.      */
    msg_codec_telemetry_t   in;         /* pitch/roll set from cdeg.    */
    int32_t                 pitch_cdeg; /* Input to cdeg_to_hdeg().     */
    int32_t                 roll_cdeg;  /* Input to cdeg_to_hdeg().     */
    msg_codec_telemetry_t   out;        /* Expected from get.           */
    uint8_t                 bytes[MSG_TEL_BYTES];
} codec_vectors_tel_t;

/** One heartbeat vector. */
typedef struct
{
    const char *            p_label;
    msg_codec_heartbeat_t   in;
    msg_codec_heartbeat_t   out;
    uint8_t                 bytes[MSG_HB_BYTES];
} codec_vectors_hb_t;

/** One event vector. */
typedef struct
{
    const char *            p_label;
    msg_codec_event_t       in;
    msg_codec_event_t       out;
    uint8_t                 bytes[MSG_EVT_BYTES];
} codec_vectors_evt_t;

/** One command vector. */
typedef struct
{
    const char *            p_label;
    msg_codec_command_t     in;
    bool                    b_get_ok;   /* Expected get result.         */
    msg_codec_command_t     out;
    uint8_t                 bytes[MSG_CMD_BYTES];
} codec_vectors_cmd_t;

/** One ack vector. */
typedef struct
{
    const char *            p_label;
    msg_codec_ack_t         in;
    msg_codec_ack_t         out;
    uint8_t                 bytes[MSG_ACK_BYTES];
} codec_vectors_ack_t;

/** @brief Telemetry vectors; *p_count receives how many. */
const codec_vectors_tel_t * codec_vectors_tel(uint32_t * p_count);

/** @brief Heartbeat vectors; *p_count receives how many. */
const codec_vectors_hb_t *  codec_vectors_hb(uint32_t * p_count);

/** @brief Event vectors; *p_count receives how many. */
const codec_vectors_evt_t * codec_vectors_evt(uint32_t * p_count);

/** @brief Command vectors; *p_count receives how many. */
const codec_vectors_cmd_t * codec_vectors_cmd(uint32_t * p_count);

/** @brief Ack vectors; *p_count receives how many. */
const codec_vectors_ack_t * codec_vectors_ack(uint32_t * p_count);

#endif /* CODEC_VECTORS_H */

/*** end of file ***/
