/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_telem.c
 *
 * @brief Command and map to ack, heartbeat packing, schedules, the queue
 *        room rule, the WiFi fault rule and message filling for
 *        telemetry_task (see s1_telem.h). No kernel or driver calls.
 */

#include "s1_telem.h"

#if TM_MQTT

#include <stddef.h>

#include "../subsystem3/s3_code39.h"
#include "msg_codec.h"

/* A MAP's letters, in the order s3_set_nav_map() takes them, and the text
 * passed to it: the four letters and a NUL.
 */
#define MAP_SLOT_LEFT               0U
#define MAP_SLOT_RIGHT              1U
#define MAP_SLOT_STRAIGHT           2U
#define MAP_SLOT_UTURN              3U
#define MAP_LETTERS                 4U
#define MAP_TEXT_BYTES              (MAP_LETTERS + 1U)

/* Both messages this module builds must fit one s1_telem_pub_t, and a
 * length must fit its uint16_t.
 */
typedef uint8_t s1_telem_check_pub_t[((MSG_ACK_BYTES <= S1_TELEM_PUB_MAX)
    && (MSG_HB_BYTES <= S1_TELEM_PUB_MAX)
    && (S1_TELEM_PUB_MAX <= UINT16_MAX)) ? 1 : -1];

/* S3 takes exactly MAP_LETTERS letters (S3_NAV_MAP_LEN). */
typedef uint8_t s1_telem_check_map_t[(MAP_LETTERS == S3_NAV_MAP_LEN)
                                     ? 1 : -1];

static msg_codec_ack_t on_opcode(const mqtt_link_command_t * p_cmd,
                                 s1_telem_post_fn_t p_post,
                                 s1_telem_stats_t * p_stats);
static msg_codec_ack_t on_map(const mqtt_link_command_t * p_cmd,
                              s1_telem_map_fn_t p_map,
                              s1_telem_stats_t * p_stats);

/* Half the range of a 32-bit millisecond clock: a deadline less than this
 * far behind "now" has passed; one further away is still ahead.
 */
#define HALF_RANGE_MSEC             0x80000000U

/* Servo angle to the telemetry angle_idx: 7.5 degree steps over 0 to 180
 * degrees (angle_idx = degrees x 2 / 15, 0 to 24).
 */
#define S1_TELEM_SERVO_MAX_DEG      180
#define S1_TELEM_ANGLE_NUMERATOR    2
#define S1_TELEM_ANGLE_DENOMINATOR  15

/*!
 * @brief Add two counts without wrapping.
 *
 * @param[in] total  The count so far.
 * @param[in] amount The count to add.
 *
 * @return total + amount, or UINT32_MAX if the sum does not fit.
 */
uint32_t
s1_telem_sat_add (uint32_t total, uint32_t amount)
{
    return ((amount > (UINT32_MAX - total)) ? UINT32_MAX : (total + amount));
}

/*!
 * @brief Decode one received message, act on it, and build its ack.
 *
 * The type field alone picks the path (msg_codec_get_type()): type 7 is
 * a MAP (on_map()); any other type takes the command path (on_opcode()),
 * whose codec refuses everything but a well-formed COMMAND, as before the
 * MAP existed.
 *
 * @param[in]     p_cmd   The message from mqtt_link_receive_command().
 * @param[in]     p_post  Posts a command to S5 (see s1_telem_post_fn_t).
 * @param[in]     p_map   Gives a map to S3 (see s1_telem_map_fn_t).
 * @param[in,out] p_stats Counters to update.
 * @param[out]    p_ack   Receives the ack and how to publish it: topic
 *                        MQTT_LINK_TOPIC_ACK, QoS 1, not retained.
 *
 * @return True if p_ack holds an ack to publish; false only for a NULL
 *         argument.
 */
bool
s1_telem_on_command (const mqtt_link_command_t * p_cmd,
                     s1_telem_post_fn_t p_post, s1_telem_map_fn_t p_map,
                     s1_telem_stats_t * p_stats, s1_telem_pub_t * p_ack)
{
    msg_codec_ack_t ack = {MSG_OP_INVALID, 0U, MSG_ACK_REJECTED};
    bool            b_ok = false;

    if ((NULL != p_cmd) && (NULL != p_post) && (NULL != p_map)
        && (NULL != p_stats) && (NULL != p_ack))
    {
        /* get_type() reads byte 0 only, which the payload always has. */
        if (MSG_TYPE_MAP == msg_codec_get_type(p_cmd->payload,
                                               p_cmd->payload_len))
        {
            ack = on_map(p_cmd, p_map, p_stats);
        }
        else
        {
            ack = on_opcode(p_cmd, p_post, p_stats);
        }

        p_ack->topic    = MQTT_LINK_TOPIC_ACK;
        p_ack->qos      = MQTT_LINK_QOS_1;
        p_ack->b_retain = false;
        /* The ack is MSG_ACK_BYTES long, which the compile-time check
         * above keeps within uint16_t.
         */
        p_ack->len = (uint16_t)msg_codec_put_ack(&ack, p_ack->payload,
                                                 sizeof(p_ack->payload));
        b_ok       = (MSG_ACK_BYTES == p_ack->len);
    }

    return (b_ok);
}

/*!
 * @brief Pack one heartbeat (docs/S1_binary_frame.md section 4).
 *
 * rssi is sent as 0: no RSSI can be read outside the radio service task
 * yet (assumption A15), and 0 dBm never occurs on a real link, so it
 * reads as "not measured". tel_drop is the sum of the periodic messages
 * the queue rule skipped and the ones the driver refused (telemetry, live
 * and heartbeats). cmd_reject is the sum of the codec rejections, the post
 * failures, the refused or malformed MAPs and the driver's command_dropped.
 * The codec saturates every count at its field's largest value.
 *
 * TODO: evt_drop is 0: there is no mbf_event yet. Count its drops when
 * Stage 4b adds it.
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
    msg_codec_heartbeat_t hb_fields = {0U, 0U, 0, 0U, 0U, 0U, 0U};
    bool                  b_ok = false;

    if ((NULL != p_in) && (NULL != p_stats) && (NULL != p_hb))
    {
        hb_fields.seq           = seq;
        hb_fields.uptime_s      = p_in->uptime_s;
        hb_fields.rssi_dbm      = 0;
        hb_fields.connect_count = p_in->connect_count;
        hb_fields.tel_drop      = s1_telem_sat_add(
            s1_telem_sat_add(p_stats->periodic_skipped,
                             p_stats->periodic_failed),
            p_stats->hb_publish_failed);
        hb_fields.evt_drop      = 0U;
        hb_fields.cmd_reject    = s1_telem_sat_add(
            s1_telem_sat_add(
                s1_telem_sat_add(p_stats->codec_reject,
                                 p_stats->post_failure),
                p_stats->map_reject),
            p_in->command_dropped);

        p_hb->topic    = MQTT_LINK_TOPIC_TELEMETRY;
        p_hb->qos      = MQTT_LINK_QOS_0;
        p_hb->b_retain = false;
        /* The heartbeat is MSG_HB_BYTES long, which the compile-time
         * check above keeps within uint16_t.
         */
        p_hb->len = (uint16_t)msg_codec_put_heartbeat(
            &hb_fields, p_hb->payload, sizeof(p_hb->payload));
        b_ok      = (MSG_HB_BYTES == p_hb->len);
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
    return (s1_telem_due(now_msec, S1_TELEM_HB_PERIOD_MSEC, p_next_msec));
}

/*!
 * @brief Say whether a periodic message is due, on an absolute schedule.
 *
 * The same rule as s1_telem_hb_due(), for any period: deadlines are
 * period_msec apart from the first one, a stall that missed several
 * deadlines gives one due message and not a burst, and the clock may wrap.
 *
 * @param[in]     now_msec    The clock now, in milliseconds.
 * @param[in]     period_msec The period; 0 is never due.
 * @param[in,out] p_next_msec The next deadline; moved on when one is due.
 *
 * @return True if the message is due now.
 */
bool
s1_telem_due (uint32_t now_msec, uint32_t period_msec,
              uint32_t * p_next_msec)
{
    uint32_t late_msec = 0U;
    bool     b_due = false;

    if ((NULL != p_next_msec) && (period_msec > 0U))
    {
        late_msec = now_msec - *p_next_msec;    /* Modulo 2^32. */

        if (late_msec < HALF_RANGE_MSEC)
        {
            b_due        = true;
            *p_next_msec += ((late_msec / period_msec) + 1U) * period_msec;
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

/*!
 * @brief Say whether the publish queue is short enough for a periodic
 *        message to join it.
 *
 * @param[in] free_slots From mqtt_link_publish_free(). More than
 *                       MQTT_LINK_QUEUE_DEPTH is read as an empty queue.
 * @param[in] max_queued S1_TELEM_TEL_QUEUED_MAX, S1_TELEM_LIVE_QUEUED_MAX
 *                       or S1_TELEM_HB_QUEUED_MAX.
 *
 * @return True if at most max_queued records are in the queue.
 */
bool
s1_telem_queue_allows (uint32_t free_slots, uint32_t max_queued)
{
    uint32_t queued = 0U;

    if (free_slots < MQTT_LINK_QUEUE_DEPTH)
    {
        queued = MQTT_LINK_QUEUE_DEPTH - free_slots;
    }

    return (queued <= max_queued);
}

/*!
 * @brief Apply the WiFi fault rule: set or clear fault bit 4 when it changes.
 *
 * The bit means "dashboard link lost": the radio is not CYW43_LINK_UP or the
 * MQTT session is down (docs/S1_binary_frame.md section 3.1). The session
 * test is the one that gates the heartbeat, s1_telem_hb_should_publish(), so
 * a stale driver status counts as down. Nothing happens before the first
 * successful connection: connect_count is 0 until then and is never reset, so
 * once armed the rule stays armed. An inconsistent status read leaves the
 * level as it is; the next pass reads it again.
 *
 * @param[in]     p_status     The radio and driver status just read.
 * @param[in]     p_set        Sets fault bits (subsystem5_fault_set()).
 * @param[in]     p_clear      Clears fault bits (subsystem5_fault_clear()).
 * @param[in,out] pb_fault_set The level last applied; written on a change.
 *
 * @return True if the level changed, so exactly one of p_set and p_clear was
 *         called; false otherwise, and for a NULL argument.
 */
bool
s1_telem_wifi_update (const s1_telem_wifi_status_t * p_status,
                      s1_telem_fault_function_t p_set,
                      s1_telem_fault_function_t p_clear, bool * pb_fault_set)
{
    s1_telem_fault_function_t p_change = NULL;
    bool                      b_fault_wanted = false;
    bool                      b_changed = false;

    if ((NULL != p_status) && (NULL != p_set) && (NULL != p_clear)
        && (NULL != pb_fault_set) && (p_status->b_status_read)
        && (0U != p_status->connect_count))
    {
        b_fault_wanted = (!p_status->b_link_up)
                         || (!s1_telem_hb_should_publish(p_status->state,
                                                         p_status->b_stale));

        if (b_fault_wanted != *pb_fault_set)
        {
            p_change = b_fault_wanted ? p_set : p_clear;
            p_change(MSG_FAULT_WIFI);

            *pb_fault_set = b_fault_wanted;
            b_changed     = true;
        }
    }

    return (b_changed);
}

/*!
 * @brief Fill the 2 Hz telemetry frame from the subsystem values.
 *
 * Mapping, section 3.4 of docs/S1_binary_frame.md: S2 gives the motion
 * state, wheel speeds, counts and distance; S3 the line state and the last
 * barcode's letter and direction (the barcode command is in the live
 * message, so bc_cmd is 0); S4 the terrain fields, with motion_class
 * forced to IMPACT while the impact is latched; S5 the controller state,
 * the fault bits, the last ping (closest_mm, angle_idx) and the last
 * profile (width, clearances). Anything a subsystem cannot give is 0.
 *
 * @param[in]  p_src Values; a NULL pointer leaves p_tel unchanged.
 * @param[in]  seq   Frame counter.
 * @param[out] p_tel Receives the fields.
 */
void
s1_telem_fill_frame (const s1_telem_src_t * p_src, uint8_t seq,
                     msg_codec_telemetry_t * p_tel)
{
    int32_t angle_deg = 0;

    if ((NULL != p_src) && (NULL != p_tel))
    {
        angle_deg = p_src->servo_deg;

        if (angle_deg < 0)
        {
            angle_deg = 0;
        }
        else if (angle_deg > S1_TELEM_SERVO_MAX_DEG)
        {
            angle_deg = S1_TELEM_SERVO_MAX_DEG;
        }
        else
        {
            /* Already within the servo's range. */
        }

        *p_tel = (msg_codec_telemetry_t){0U};

        p_tel->seq          = seq;
        p_tel->ctrl_state   = p_src->ctrl_state;
        p_tel->motion_state = p_src->motion_state;
        /* The mask keeps 5 bits, which fit uint8_t. */
        p_tel->fault_mask   = (uint8_t)(p_src->fault_mask
                                        & MSG_FAULT_MASK_ALL);
        p_tel->line_state   = p_src->b_s3_ok ? p_src->line_state
                                             : S1_TELEM_LINE_UNCAL;
        p_tel->bc_letter    = p_src->b_s3_ok ? p_src->last_ch
                                             : MSG_BC_LETTER_NONE;
        p_tel->b_bc_dir     = (p_src->b_s3_ok) && (p_src->b_last_rev);
        p_tel->speed_l_mms  = p_src->speed_l_mms;
        p_tel->speed_r_mms  = p_src->speed_r_mms;
        p_tel->count_l      = p_src->count_l;
        p_tel->count_r      = p_src->count_r;
        p_tel->dist_mm      = p_src->dist_mm;
        p_tel->pitch_hdeg   = p_src->pitch_hdeg;
        p_tel->roll_hdeg    = p_src->roll_hdeg;
        p_tel->hump_state   = p_src->hump_state;
        p_tel->peak_mm      = p_src->peak_mm;
        p_tel->hump_count   = p_src->hump_count;
        p_tel->motion_class = p_src->b_impact ? S1_TELEM_CLASS_IMPACT
                                              : p_src->motion_class;
        p_tel->turn_dps     = p_src->turn_rate_dps;
        p_tel->b_imu_ok     = p_src->b_imu_ok;
        p_tel->b_cal_done   = p_src->b_cal_done;
        p_tel->closest_mm   = p_src->b_range_valid ? p_src->range_mm : 0U;
        /* angle_deg is 0 to 180, so the result is 0 to 24 and fits
         * uint8_t.
         */
        p_tel->angle_idx    = (uint8_t)((angle_deg * S1_TELEM_ANGLE_NUMERATOR)
                                        / S1_TELEM_ANGLE_DENOMINATOR);
        p_tel->width_mm     = p_src->b_profile_valid ? p_src->width_mm : 0U;
        p_tel->clear_l_mm   = p_src->b_profile_valid ? p_src->clear_left_mm
                                                     : 0U;
        p_tel->clear_r_mm   = p_src->b_profile_valid ? p_src->clear_right_mm
                                                     : 0U;
    }
}

/*!
 * @brief Fill the 5 Hz live message from the subsystem values.
 *
 * Mapping, section 3A of docs/S1_binary_frame.md. cmd_speed_mms is the
 * mean of the two wheel set-points (rounded toward zero); heading is
 * converted from centidegrees to half-degrees. When S3 could not be read,
 * line_state is UNCAL, bc_letter is 63 and every other S3 field is 0.
 *
 * @param[in]  p_src  Values; a NULL pointer leaves p_live unchanged.
 * @param[in]  seq    Message counter.
 * @param[out] p_live Receives the fields.
 */
void
s1_telem_fill_live (const s1_telem_src_t * p_src, uint8_t seq,
                    msg_codec_live_t * p_live)
{
    if ((NULL != p_src) && (NULL != p_live))
    {
        *p_live = (msg_codec_live_t){0U};

        p_live->seq           = seq;
        p_live->motion_state  = p_src->motion_state;
        p_live->b_estop       = p_src->b_estop;
        /* Each set-point is a few hundred mm/s, so the sum cannot overflow
         * int32_t.
         */
        p_live->cmd_speed_mms = (p_src->target_l_mms + p_src->target_r_mms)
                                / 2;
        p_live->heading_hdeg  = msg_codec_heading_to_hdeg(
                                    p_src->heading_cdeg);
        p_live->line_state    = S1_TELEM_LINE_UNCAL;
        p_live->bc_letter     = MSG_BC_LETTER_NONE;

        if (p_src->b_s3_ok)
        {
            p_live->line_state   = p_src->line_state;
            p_live->steer_dv_mms = p_src->steer_dv_mms;
            p_live->outer_ir     = p_src->outer_ir;
            p_live->centre_ir    = p_src->centre_ir;
            p_live->b_overshoot  = p_src->b_overshoot;
            p_live->bc_letter    = p_src->last_ch;
            p_live->bc_cmd       = p_src->last_cmd;
            p_live->b_bc_rev     = p_src->b_last_rev;
            p_live->junc_turn    = p_src->junc_turn;
            p_live->rejects      = p_src->rejects;
            p_live->junctions    = p_src->junctions;
            p_live->losts        = p_src->losts;
            p_live->barcodes     = p_src->barcodes;
        }
    }
}

/*!
 * @brief The command path: decode a COMMAND, post it to S5 if it is valid,
 *        and say what its ack is. Unchanged from before the MAP existed.
 *
 * A command the codec refuses (wrong length, version or type, or opcode
 * 0) is not posted; its ack is MSG_ACK_REJECTED and codec_reject goes up.
 * A valid command is posted once; the ack is MSG_ACK_ACCEPTED ("queued")
 * if the post returned 0, otherwise MSG_ACK_REJECTED and post_failure
 * goes up. The ack echoes the opcode and id whenever the codec could read
 * them (msg_codec_get_command()), otherwise both are 0.
 *
 * @param[in]     p_cmd   The message (not NULL).
 * @param[in]     p_post  Posts a command to S5 (not NULL).
 * @param[in,out] p_stats Counters to update (not NULL).
 *
 * @return The ack's opcode, id and result.
 */
static msg_codec_ack_t
on_opcode (const mqtt_link_command_t * p_cmd, s1_telem_post_fn_t p_post,
           s1_telem_stats_t * p_stats)
{
    msg_codec_command_t cmd = {MSG_OP_INVALID, 0U};
    msg_codec_ack_t     ack = {MSG_OP_INVALID, 0U, MSG_ACK_REJECTED};
    bool                b_valid = false;

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
    else if (0 == p_post(cmd.opcode, cmd.cmd_id))
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

    return (ack);
}

/*!
 * @brief The map path: decode a MAP, give its letters to S3, and say what
 *        its ack is.
 *
 * Each 6-bit index becomes its character with S3's own s3_code39_char(),
 * which gives '?' above 43, so S3 alone judges the map. A MAP the codec
 * refuses is not passed on. The ack's opcode is MSG_ACK_OP_MAP and its id
 * the MAP's (0 when the codec refused it); its result is
 * MSG_ACK_ACCEPTED only when the map function returned 0. Otherwise
 * map_reject goes up.
 *
 * @param[in]     p_cmd   The message (not NULL), type field 7.
 * @param[in]     p_map   Gives the map to S3 (not NULL).
 * @param[in,out] p_stats Counters to update (not NULL).
 *
 * @return The ack's opcode, id and result.
 */
static msg_codec_ack_t
on_map (const mqtt_link_command_t * p_cmd, s1_telem_map_fn_t p_map,
        s1_telem_stats_t * p_stats)
{
    msg_codec_map_t map = {0U, 0U, 0U, 0U, 0U};
    msg_codec_ack_t answer = {MSG_ACK_OP_MAP, 0U, MSG_ACK_REJECTED};
    char            letters[MAP_TEXT_BYTES];
    bool            b_valid = false;
    bool            b_applied = false;

    if (p_cmd->payload_len <= sizeof(p_cmd->payload))
    {
        b_valid = msg_codec_get_map(p_cmd->payload, p_cmd->payload_len,
                                    &map);
    }

    if (b_valid)
    {
        letters[MAP_SLOT_LEFT]     = s3_code39_char(map.letter_left);
        letters[MAP_SLOT_RIGHT]    = s3_code39_char(map.letter_right);
        letters[MAP_SLOT_STRAIGHT] = s3_code39_char(map.letter_straight);
        letters[MAP_SLOT_UTURN]    = s3_code39_char(map.letter_uturn);
        letters[MAP_LETTERS]       = '\0';
        b_applied = (0 == p_map(letters));
    }

    if (b_applied)
    {
        answer.result = MSG_ACK_ACCEPTED;
    }
    else
    {
        p_stats->map_reject = s1_telem_sat_add(p_stats->map_reject, 1U);
    }

    answer.cmd_id = map.cmd_id;

    return (answer);
}

#endif /* TM_MQTT */

/*** end of file ***/

