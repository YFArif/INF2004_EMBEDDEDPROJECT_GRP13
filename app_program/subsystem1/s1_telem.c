/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_telem.c
 *
 * @brief Command to ack, heartbeat packing and the heartbeat schedule for
 *        telemetry_task (see s1_telem.h). No kernel or driver calls.
 */

#include "s1_telem.h"

#if TM_MQTT

#include <stddef.h>

#include "msg_codec.h"

/* Both messages this module builds must fit one s1_telem_pub_t, and a
 * length must fit its uint16_t.
 */
typedef uint8_t s1_telem_check_pub_t[((MSG_ACK_BYTES <= S1_TELEM_PUB_MAX)
    && (MSG_HB_BYTES <= S1_TELEM_PUB_MAX)
    && (S1_TELEM_PUB_MAX <= UINT16_MAX)) ? 1 : -1];

/* Half the range of a 32-bit millisecond clock: a deadline less than this
 * far behind "now" has passed; one further away is still ahead.
 */
#define HALF_RANGE_MSEC             0x80000000U

/*!
 * @brief Add two counts without wrapping.
 *
 * @param[in] a First count.
 * @param[in] b Second count.
 *
 * @return a + b, or UINT32_MAX if the sum does not fit.
 */
uint32_t
s1_telem_sat_add (uint32_t a, uint32_t b)
{
    return ((b > (UINT32_MAX - a)) ? UINT32_MAX : (a + b));
}

/*!
 * @brief Decode one received command, post it to S5 if it is valid, and
 *        build its ack.
 *
 * A command the codec refuses (wrong length, version or type, or opcode
 * 0) is not posted; its ack is MSG_ACK_REJECTED and codec_reject goes up.
 * A valid command is posted once; the ack is MSG_ACK_ACCEPTED ("queued")
 * if the post returned 0, otherwise MSG_ACK_REJECTED and post_failure
 * goes up. The ack echoes the opcode and id whenever the codec could read
 * them (msg_codec_get_command()), otherwise both are 0.
 *
 * @param[in]     p_cmd   The command from mqtt_link_receive_command().
 * @param[in]     post    Posts a command to S5 (see s1_telem_post_fn_t).
 * @param[in,out] p_stats Counters to update.
 * @param[out]    p_ack   Receives the ack and how to publish it: topic
 *                        MQTT_LINK_TOPIC_ACK, QoS 1, not retained.
 *
 * @return True if p_ack holds an ack to publish; false only for a NULL
 *         argument.
 */
bool
s1_telem_on_command (const mqtt_link_command_t * p_cmd,
                     s1_telem_post_fn_t post, s1_telem_stats_t * p_stats,
                     s1_telem_pub_t * p_ack)
{
    msg_codec_command_t cmd = {MSG_OP_INVALID, 0U};
    msg_codec_ack_t     ack = {MSG_OP_INVALID, 0U, MSG_ACK_REJECTED};
    bool                b_valid = false;
    bool                b_ok = false;

    if ((NULL != p_cmd) && (NULL != post) && (NULL != p_stats)
        && (NULL != p_ack))
    {
        if (p_cmd->payload_len <= sizeof(p_cmd->payload))
        {
            b_valid = msg_codec_get_command(p_cmd->payload,
                                            p_cmd->payload_len, &cmd);
        }

        if (!b_valid)
        {
            p_stats->codec_reject = s1_telem_sat_add(p_stats->codec_reject,
                                                     1U);
        }
        else if (0 == post(cmd.opcode, cmd.cmd_id))
        {
            ack.result = MSG_ACK_ACCEPTED;
        }
        else
        {
            p_stats->post_failure = s1_telem_sat_add(p_stats->post_failure,
                                                     1U);
        }

        ack.opcode = cmd.opcode;
        ack.cmd_id = cmd.cmd_id;
        p_ack->topic = MQTT_LINK_TOPIC_ACK;
        p_ack->qos = MQTT_LINK_QOS_1;
        p_ack->b_retain = false;
        /* The ack is MSG_ACK_BYTES long, which the compile-time check
         * above keeps within uint16_t.
         */
        p_ack->len = (uint16_t)msg_codec_put_ack(&ack, p_ack->payload,
                                                 sizeof(p_ack->payload));
        b_ok = (MSG_ACK_BYTES == p_ack->len);
    }

    return (b_ok);
}

/*!
 * @brief Pack one heartbeat (docs/S1_binary_frame.md section 4).
 *
 * rssi is sent as 0: no RSSI can be read outside the radio service task
 * yet (assumption A15), and 0 dBm never occurs on a real link, so it
 * reads as "not measured". tel_drop and evt_drop are 0 in Stage 4a, which
 * sends no telemetry frames and has no mbf_event. cmd_reject is the sum
 * of the codec rejections, the post failures and the driver's
 * command_dropped. The codec saturates every count at its field's
 * largest value.
 *
 * @param[in]  seq     Heartbeat counter, modulo 256.
 * @param[in]  p_in    Uptime and driver counts.
 * @param[in]  p_stats telemetry_task's counters.
 * @param[out] p_hb    Receives the heartbeat and how to publish it:
 *                     topic MQTT_LINK_TOPIC_TELEMETRY, QoS 0, not retained.
 *
 * @return True if p_hb holds a heartbeat; false only for a NULL argument.
 */
bool
s1_telem_heartbeat (uint8_t seq, const s1_telem_hb_in_t * p_in,
                    const s1_telem_stats_t * p_stats, s1_telem_pub_t * p_hb)
{
    msg_codec_heartbeat_t hb = {0U, 0U, 0, 0U, 0U, 0U, 0U};
    bool                  b_ok = false;

    if ((NULL != p_in) && (NULL != p_stats) && (NULL != p_hb))
    {
        hb.seq = seq;
        hb.uptime_s = p_in->uptime_s;
        hb.rssi_dbm = 0;
        hb.connect_count = p_in->connect_count;
        hb.tel_drop = 0U;
        hb.evt_drop = 0U;
        hb.cmd_reject = s1_telem_sat_add(
            s1_telem_sat_add(p_stats->codec_reject, p_stats->post_failure),
            p_in->command_dropped);

        p_hb->topic = MQTT_LINK_TOPIC_TELEMETRY;
        p_hb->qos = MQTT_LINK_QOS_0;
        p_hb->b_retain = false;
        /* The heartbeat is MSG_HB_BYTES long, which the compile-time
         * check above keeps within uint16_t.
         */
        p_hb->len = (uint16_t)msg_codec_put_heartbeat(&hb, p_hb->payload,
                                                      sizeof(p_hb->payload));
        b_ok = (MSG_HB_BYTES == p_hb->len);
    }

    return (b_ok);
}

/*!
 * @brief Say whether a heartbeat is due, and move the deadline on.
 *
 * The schedule is absolute: deadlines are S1_TELEM_HB_PERIOD_MSEC apart
 * from the first one, so late wake-ups do not add up. After a stall that
 * missed several deadlines, one heartbeat is due and the next deadline is
 * the first one still ahead, so missed heartbeats are not sent in a burst.
 * The clock may wrap; deadlines up to half its range ahead or behind are
 * handled.
 *
 * @param[in]     now_msec    The clock now, in milliseconds.
 * @param[in,out] p_next_msec The next deadline; moved on when one is due.
 *
 * @return True if a heartbeat is due now.
 */
bool
s1_telem_hb_due (uint32_t now_msec, uint32_t * p_next_msec)
{
    uint32_t late_msec = 0U;
    bool     b_due = false;

    if (NULL != p_next_msec)
    {
        late_msec = now_msec - *p_next_msec;    /* Modulo 2^32. */

        if (late_msec < HALF_RANGE_MSEC)
        {
            b_due = true;
            *p_next_msec += ((late_msec / S1_TELEM_HB_PERIOD_MSEC) + 1U)
                            * S1_TELEM_HB_PERIOD_MSEC;
        }
    }

    return (b_due);
}

/*!
 * @brief Say whether a heartbeat should be published now.
 *
 * Only while the session is up and the driver snapshot is fresh: during
 * an outage, queued heartbeats would only fill the publish queue that the
 * acks need, and be stale when they finally went out.
 *
 * @param[in] state   mqtt_link status state.
 * @param[in] b_stale mqtt_link status b_stale.
 *
 * @return True to publish.
 */
bool
s1_telem_hb_should_publish (mqtt_link_state_t state, bool b_stale)
{
    return ((MQTT_LINK_STATE_CONNECTED == state) && (!b_stale));
}

#endif /* TM_MQTT */

/*** end of file ***/
