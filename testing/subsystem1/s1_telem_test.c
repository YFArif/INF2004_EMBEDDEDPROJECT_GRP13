/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_telem_test.c
 *
 * @brief S1-U14: host test of telemetry_task's logic (s1_telem.c).
 *
 * @par What it tests
 * app_program/subsystem1/s1_telem.c through its public API, with a stub in
 * place of subsystem5_command_post() (the post function telemetry_task
 * passes in):
 * - a valid START, STOP or RESET is posted once, and acked "queued"
 *   (result 0) when the post returns 0;
 * - a post failure (E_TMOUT, E_ID, E_NOEXS) is acked "rejected" and
 *   counted in post_failure;
 * - a command the codec refuses is never posted, is acked "rejected"
 *   with the echo rules of docs/S1_binary_frame.md section 5.1, and is
 *   counted in codec_reject;
 * - the ack goes to the ack topic at QoS 1, not retained; the heartbeat
 *   to the telemetry topic at QoS 0, not retained;
 * - the heartbeat bytes, its cmd_reject sum and its saturation;
 * - the absolute 1 Hz schedule: no drift, no burst after a stall, and the
 *   32-bit clock wrap;
 * - the heartbeat is published only while the session is up and fresh;
 * - the telemetry frame and the live message are filled from the S2, S3,
 *   S4 and S5 values as section 3.4 and 3A of the document say: mean of
 *   the wheel set-points, heading in half-degrees, servo angle to
 *   angle_idx, the defaults when S3 cannot be read, the latched impact,
 *   and the 5 Hz schedule;
 * - the publish queue rules (telemetry and live with at most 1 record
 *   queued, the heartbeat with at most 3) and tel_drop in the heartbeat;
 * - the three schedules, started with their offsets, never have two
 *   messages due in the same millisecond;
 * - the WiFi fault rule, with stubs in place of subsystem5_fault_set() and
 *   subsystem5_fault_clear(): armed by the first connection, set by a radio
 *   or session loss, cleared only when both are good, one call per change;
 * - S1-U24 part C, in a group of its own ("map"): the MAP path, with a
 *   stub in place of the function that calls s3_set_nav_map(). The stub
 *   applies S3's own rules with s3_nav_set_map() from s3_code39.c, which
 *   the runner links in.
 *
 * @par How
 * Expected bytes are written out by hand from the document's tables,
 * not produced with the codec.
 *
 * @par Build and run
 * Use testing/subsystem1/run_telemetry_test.sh from the repository root.
 * This program prints one line per failed check and one line per group,
 * then "RESULT: all passed (n checks)" and exits 0, or
 * "RESULT: m FAILED (n checks)" and exits 1.
 *
 * @par Test code
 * This module is never built into the firmware. int main(void) is the
 * form the C standard requires (Barr-C 5.2.a does not apply to it).
 */

#include "s1_telem_test.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../subsystem3/s3_code39.h"
#include "msg_layout.h"
#include "mqtt_link.h"
#include "s1_telem.h"

/* Kernel error codes the stub returns (include/tk/errno.h). */
#define E_OK_VALUE              0
#define E_PAR_VALUE             (-17)
#define E_ID_VALUE              (-18)
#define E_NOEXS_VALUE           (-42)
#define E_TMOUT_VALUE           (-50)

/* Header bytes: version 1 in the high nibble, the type in the low one. */
#define HDR_COMMAND             0x14U
#define HDR_MAP                 0x17U
#define HDR_ACK                 0x15U
#define HDR_HEARTBEAT           0x12U

/* Third ack byte: the 2-bit result at the top, padding below. */
#define ACK_BYTE2_QUEUED        0x00U
#define ACK_BYTE2_REJECTED      0x40U

/** One servo angle and the angle_idx it must give. */
typedef struct
{
    int16_t     servo_deg;
    uint8_t     idx;
} angle_case_t;

/** Free slots in the publish queue and what each rule must say. */
typedef struct
{
    uint32_t    free_slots;
    bool        b_tel_allowed;
    bool        b_live_allowed;
    bool        b_hb_allowed;
} queue_case_t;

/** One input on which the link counts as lost, once the rule is armed. */
typedef struct
{
    const char *        p_name;
    bool                b_link_up;
    mqtt_link_state_t   state;
    bool                b_stale;
} wifi_lost_case_t;

/** One pass of a replayed drop: the input, then the stubs' running totals
 *  and the level it must leave.
 */
typedef struct
{
    bool                b_link_up;
    mqtt_link_state_t   state;
    bool                b_stale;
    uint32_t            connect_count;
    uint32_t            set_calls;
    uint32_t            clear_calls;
    bool                b_fault_set;
} wifi_replay_t;

/* S5_FAULT_WIFI, written out by hand from docs/S1_binary_frame.md 3.1. */
#define FAULT_BIT_WIFI          0x10U

/* Any connect count above 0 arms the WiFi fault rule. */
#define WIFI_ARMED_COUNT        1U

/* test_period_schedule(): 10 s of 10 ms polls. */
#define PERIOD_TEST_START_MSEC  1000U
#define PERIOD_TEST_END_MSEC    11000U
#define PERIOD_TEST_STEP_MSEC   10U

/* test_schedule_stagger(): 10 s of 1 ms steps. */
#define STAGGER_TEST_START_MSEC 1000U
#define STAGGER_TEST_SPAN_MSEC  10000U
#define STAGGER_TEST_END_MSEC   (STAGGER_TEST_START_MSEC + \
                                 STAGGER_TEST_SPAN_MSEC)
#define STAGGER_TEST_STEP_MSEC  1U

static uint32_t g_checks;
static uint32_t g_failures;
static uint32_t g_group_failures;

/* The stub's call log and scripted return value. */
static uint32_t g_post_calls;
static uint8_t  g_post_opcode;
static uint8_t  g_post_cmd_id;
static int32_t  g_post_return;

/* The map stub's call log, the letters it was given, and a forced
 * return value (0: apply S3's rules).
 */
static uint32_t g_map_calls;
static char     g_map_text[8];
static int32_t  g_map_force;

/* The fault stubs' call log. */
static uint32_t g_set_calls;
static uint32_t g_clear_calls;
static uint32_t g_set_mask;
static uint32_t g_clear_mask;

/* Every way the link can count as lost. */
static const wifi_lost_case_t g_wifi_lost[] =
{
    {"wifi: radio down",           false, MQTT_LINK_STATE_CONNECTED,  false},
    {"wifi: radio down and stale", false, MQTT_LINK_STATE_CONNECTED,  true},
    {"wifi: radio down, backoff",  false, MQTT_LINK_STATE_BACKOFF,    false},
    {"wifi: session idle",         true,  MQTT_LINK_STATE_IDLE,       false},
    {"wifi: session resolving",    true,  MQTT_LINK_STATE_RESOLVING,  false},
    {"wifi: session connecting",   true,  MQTT_LINK_STATE_CONNECTING, false},
    {"wifi: session in backoff",   true,  MQTT_LINK_STATE_BACKOFF,    false},
    {"wifi: connected but stale",  true,  MQTT_LINK_STATE_CONNECTED,  true}
};

/* A boot, a drop where the session ends first, a drop where the radio goes
 * first and the session survives, then a broker close: each one set and
 * clear, in that order, and nothing before the first connection.
 */
static const wifi_replay_t g_wifi_replay[] =
{
    {false, MQTT_LINK_STATE_IDLE,       false, 0U, 0U, 0U, false},
    {true,  MQTT_LINK_STATE_CONNECTING, false, 0U, 0U, 0U, false},
    {true,  MQTT_LINK_STATE_CONNECTED,  false, 1U, 0U, 0U, false},
    {true,  MQTT_LINK_STATE_BACKOFF,    false, 1U, 1U, 0U, true},
    {false, MQTT_LINK_STATE_BACKOFF,    false, 1U, 1U, 0U, true},
    {false, MQTT_LINK_STATE_BACKOFF,    true,  1U, 1U, 0U, true},
    {true,  MQTT_LINK_STATE_BACKOFF,    false, 1U, 1U, 0U, true},
    {true,  MQTT_LINK_STATE_CONNECTING, false, 1U, 1U, 0U, true},
    {true,  MQTT_LINK_STATE_CONNECTED,  false, 2U, 1U, 1U, false},
    {false, MQTT_LINK_STATE_CONNECTED,  false, 2U, 2U, 1U, true},
    {false, MQTT_LINK_STATE_CONNECTED,  true,  2U, 2U, 1U, true},
    {true,  MQTT_LINK_STATE_CONNECTED,  false, 2U, 2U, 2U, false},
    {true,  MQTT_LINK_STATE_BACKOFF,    false, 2U, 3U, 2U, true},
    {true,  MQTT_LINK_STATE_CONNECTED,  false, 3U, 3U, 3U, false}
};

/* sizeof is a small compile-time constant here, so the casts keep the value. */
#define WIFI_LOST_COUNT     ((uint32_t)(sizeof(g_wifi_lost) \
                                        / sizeof(g_wifi_lost[0])))
#define WIFI_REPLAY_COUNT   ((uint32_t)(sizeof(g_wifi_replay) \
                                        / sizeof(g_wifi_replay[0])))

static void check(bool b_ok, const char * p_what, uint32_t detail);
static void group_begin(const char * p_name);
static void group_end(void);
static int32_t stub_post(uint8_t opcode, uint8_t cmd_id);
static void stub_reset(int32_t ret);
static int32_t stub_map(const char * p_letters);
static void map_reset(int32_t force);
static mqtt_link_command_t make_map(const uint8_t * p_bytes, uint16_t len);
static bool map_ack_is(const mqtt_link_command_t * p_cmd,
                       s1_telem_stats_t * p_stats, const uint8_t * p_want);
static void test_map_type4_unchanged(void);
static void test_map_applied(void);
static void test_map_refused(void);
static void test_map_malformed(void);
static void test_map_duplicate(void);
static void test_map_letters(void);
static void test_map_null_and_sum(void);
static mqtt_link_command_t make_cmd(uint8_t byte0, uint8_t byte1,
                                    uint16_t len);
static bool bytes_equal(const uint8_t * p_a, const uint8_t * p_b,
                        uint32_t len);
static void test_valid_commands(void);
static void test_post_failures(void);
static void test_codec_rejects(void);
static void test_null_arguments(void);
static void test_heartbeat_bytes(void);
static void test_heartbeat_saturation(void);
static void test_schedule_steady(void);
static void test_schedule_jitter(void);
static void test_schedule_stall(void);
static void test_should_publish(void);
static void test_fill_live(void);
static void test_fill_frame(void);
static void test_period_schedule(void);
static void test_queue_rules(void);
static void test_heartbeat_tel_drop(void);
static void test_schedule_stagger(void);
static s1_telem_src_t make_src(void);
static void stub_fault_set(uint32_t faults);
static void stub_fault_clear(uint32_t faults);
static void stub_fault_reset(void);
static s1_telem_wifi_status_t make_wifi_status(bool b_link_up,
                                               mqtt_link_state_t state,
                                               bool b_stale,
                                               uint32_t connect_count);
static bool wifi_step(const s1_telem_wifi_status_t * p_status,
                      bool * pb_fault_set);
static void test_wifi_not_armed(void);
static void test_wifi_set(void);
static void test_wifi_clear(void);
static void test_wifi_held_read(void);
static void test_wifi_drop_replay(void);
static void test_wifi_null_arguments(void);

/*!
 * @brief Count one check; print it if it failed.
 */
static void
check (bool b_ok, const char * p_what, uint32_t detail)
{
    g_checks++;

    if (!b_ok)
    {
        g_failures++;
        g_group_failures++;
        (void)printf("  FAIL %s (%" PRIu32 ")\n", p_what, detail);
    }
}

static void
group_begin (const char * p_name)
{
    g_group_failures = 0U;
    (void)printf("%s\n", p_name);
}

static void
group_end (void)
{
    (void)printf("  %s\n", (0U == g_group_failures) ? "ok" : "FAILED");
}

/*!
 * @brief Stand-in for subsystem5_command_post(): records the call and
 *        returns the scripted value.
 */
static int32_t
stub_post (uint8_t opcode, uint8_t cmd_id)
{
    g_post_calls++;
    g_post_opcode = opcode;
    g_post_cmd_id = cmd_id;

    return (g_post_return);
}

static void
stub_reset (int32_t ret)
{
    g_post_calls = 0U;
    g_post_opcode = 0xFFU;
    g_post_cmd_id = 0xFFU;
    g_post_return = ret;
}

static mqtt_link_command_t
make_cmd (uint8_t byte0, uint8_t byte1, uint16_t len)
{
    mqtt_link_command_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    cmd.payload[0] = byte0;
    cmd.payload[1] = byte1;
    cmd.payload_len = len;

    return (cmd);
}

static bool
bytes_equal (const uint8_t * p_a, const uint8_t * p_b, uint32_t len)
{
    return (0 == memcmp(p_a, p_b, len));
}

/*!
 * @brief START, STOP and RESET with ids 0, 1 and 63: posted once, acked
 *        "queued".
 */
static void
test_valid_commands (void)
{
    static const uint8_t opcodes[3] = {MSG_OP_START, MSG_OP_STOP,
                                       MSG_OP_RESET};
    static const uint8_t ids[3] = {0U, 1U, 63U};
    uint32_t op_idx = 0U;
    uint32_t id_idx = 0U;

    for (op_idx = 0U; op_idx < 3U; op_idx++)
    {
        for (id_idx = 0U; id_idx < 3U; id_idx++)
        {
            /* Opcode in the top 2 bits of byte 1, id in the low 6; the
             * result is at most 0xFF, so the cast keeps it.
             */
            uint8_t             cmd_byte1 =
                (uint8_t)(((uint32_t)opcodes[op_idx] << 6U) | ids[id_idx]);
            mqtt_link_command_t cmd = make_cmd(HDR_COMMAND, cmd_byte1, 2U);
            s1_telem_stats_t    stats;
            s1_telem_pub_t      ack;
            const uint8_t       want[3] = {HDR_ACK, cmd_byte1,
                                           ACK_BYTE2_QUEUED};
            bool                b_ok = false;

            (void)memset(&stats, 0, sizeof(stats));
            (void)memset(&ack, 0xA5, sizeof(ack));
            stub_reset(E_OK_VALUE);
            b_ok = s1_telem_on_command(&cmd, stub_post, stub_map, &stats, &ack);

            check(b_ok, "valid: returns true", cmd_byte1);
            check(1U == g_post_calls, "valid: posted once", cmd_byte1);
            check(opcodes[op_idx] == g_post_opcode, "valid: opcode posted",
                  cmd_byte1);
            check(ids[id_idx] == g_post_cmd_id, "valid: id posted", cmd_byte1);
            check(MSG_ACK_BYTES == ack.len, "valid: ack length", ack.len);
            check(bytes_equal(ack.payload, want, 3U), "valid: ack bytes",
                  cmd_byte1);
            check(MQTT_LINK_TOPIC_ACK == ack.topic, "valid: ack topic",
                  cmd_byte1);
            check(MQTT_LINK_QOS_1 == ack.qos, "valid: ack QoS 1", cmd_byte1);
            check(!ack.b_retain, "valid: ack not retained", cmd_byte1);
            check((0U == stats.codec_reject) && (0U == stats.post_failure),
                  "valid: nothing counted", cmd_byte1);
        }
    }
}

/*!
 * @brief A valid command whose post fails: acked "rejected", counted.
 */
static void
test_post_failures (void)
{
    static const int32_t errors[3] = {E_TMOUT_VALUE, E_ID_VALUE,
                                      E_NOEXS_VALUE};
    s1_telem_stats_t stats;
    uint32_t         err_idx = 0U;

    (void)memset(&stats, 0, sizeof(stats));

    for (err_idx = 0U; err_idx < 3U; err_idx++)
    {
        /* STOP, id 5: golden vector CMD-1 (14 85). */
        mqtt_link_command_t cmd = make_cmd(HDR_COMMAND, 0x85U, 2U);
        s1_telem_pub_t      ack;
        const uint8_t       want[3] = {HDR_ACK, 0x85U, ACK_BYTE2_REJECTED};

        stub_reset(errors[err_idx]);
        check(s1_telem_on_command(&cmd, stub_post, stub_map, &stats, &ack),
              "post failure: returns true", err_idx);
        check(1U == g_post_calls, "post failure: posted once", err_idx);
        check(bytes_equal(ack.payload, want, 3U),
              "post failure: ack rejected, opcode and id echoed", err_idx);
        check((err_idx + 1U) == stats.post_failure, "post failure: counted",
              stats.post_failure);
        check(0U == stats.codec_reject, "post failure: not a codec reject",
              stats.codec_reject);
    }
}

/*!
 * @brief Commands the codec refuses: never posted, acked "rejected".
 */
static void
test_codec_rejects (void)
{
    /* byte0, byte1, length, expected ack byte 1. */
    static const uint8_t cases[6][4] = {
        {HDR_COMMAND, 0x09U, 2U, 0x09U},  /* Opcode 0, id 9: echoed.     */
        {0x24U,       0x85U, 2U, 0x00U},  /* Version 2.                  */
        {0x04U,       0x85U, 2U, 0x00U},  /* Version 0.                  */
        {HDR_ACK,     0x85U, 2U, 0x00U},  /* Type 5 (ack), not command.  */
        {HDR_COMMAND, 0x85U, 1U, 0x00U},  /* One byte.                   */
        {HDR_COMMAND, 0x85U, 0U, 0x00U},  /* Empty (a cleared retain).   */
    };
    s1_telem_stats_t stats;
    uint32_t         case_idx = 0U;

    (void)memset(&stats, 0, sizeof(stats));

    for (case_idx = 0U; case_idx < 6U; case_idx++)
    {
        mqtt_link_command_t cmd = make_cmd(cases[case_idx][0],
                                           cases[case_idx][1],
                                           cases[case_idx][2]);
        s1_telem_pub_t      ack;
        const uint8_t       want[3] = {HDR_ACK, cases[case_idx][3],
                                       ACK_BYTE2_REJECTED};

        stub_reset(E_OK_VALUE);
        check(s1_telem_on_command(&cmd, stub_post, stub_map, &stats, &ack),
              "codec reject: returns true", case_idx);
        check(0U == g_post_calls, "codec reject: not posted", case_idx);
        check(bytes_equal(ack.payload, want, 3U),
              "codec reject: ack bytes", case_idx);
        check((case_idx + 1U) == stats.codec_reject, "codec reject: counted",
              stats.codec_reject);
        check(0U == stats.post_failure, "codec reject: no post failure",
              stats.post_failure);
    }

    {
        /* A length the payload buffer cannot hold is refused unread. */
        mqtt_link_command_t cmd = make_cmd(HDR_COMMAND, 0x85U, 3U);
        s1_telem_pub_t      ack;
        const uint8_t       want[3] = {HDR_ACK, 0x00U, ACK_BYTE2_REJECTED};

        stub_reset(E_OK_VALUE);
        check(s1_telem_on_command(&cmd, stub_post, stub_map, &stats, &ack)
              && (0U == g_post_calls)
              && bytes_equal(ack.payload, want, 3U),
              "codec reject: over-long length refused, not posted", 3U);
    }
}

/*!
 * @brief Every NULL argument is refused without a crash.
 */
static void
test_null_arguments (void)
{
    mqtt_link_command_t cmd = make_cmd(HDR_COMMAND, 0x85U, 2U);
    s1_telem_stats_t    stats;
    s1_telem_pub_t      pub;
    s1_telem_hb_in_t    hb_in = {0U, 0U, 0U};

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    check(!s1_telem_on_command(NULL, stub_post, stub_map, &stats, &pub),
          "null cmd", 0U);
    check(!s1_telem_on_command(&cmd, NULL, stub_map, &stats, &pub),
          "null post", 0U);
    check(!s1_telem_on_command(&cmd, stub_post, stub_map, NULL, &pub),
          "null stats", 0U);
    check(!s1_telem_on_command(&cmd, stub_post, stub_map, &stats, NULL),
          "null ack", 0U);
    check(0U == g_post_calls, "null: never posted", g_post_calls);
    check(!s1_telem_heartbeat(0U, NULL, &stats, &pub), "null hb in", 0U);
    check(!s1_telem_heartbeat(0U, &hb_in, NULL, &pub), "null hb stats", 0U);
    check(!s1_telem_heartbeat(0U, &hb_in, &stats, NULL), "null hb out", 0U);
    check(!s1_telem_hb_due(0U, NULL), "null deadline", 0U);
}

/*!
 * @brief One heartbeat, byte for byte (section 4), and how it is sent.
 */
static void
test_heartbeat_bytes (void)
{
    /* seq 1, uptime 2, rssi 0, connect 3, tel 0, evt 0, cmd_reject 4.
     * Bits 32-38 rssi, 39-46 connect: byte 4 is 0, byte 5 is 0000 0110.
     * Bits 63-70 cmd_reject: byte 8 is 0000 1000.
     */
    static const uint8_t want[9] = {HDR_HEARTBEAT, 0x01U, 0x00U, 0x02U,
                                    0x00U, 0x06U, 0x00U, 0x00U, 0x08U};
    s1_telem_hb_in_t     hb_in = {2U, 3U, 1U};
    s1_telem_stats_t     stats;
    s1_telem_pub_t       hb_msg;

    (void)memset(&stats, 0, sizeof(stats));
    stats.codec_reject = 1U;
    stats.post_failure = 2U;

    check(s1_telem_heartbeat(1U, &hb_in, &stats, &hb_msg), "hb: returns true",
          0U);
    check(MSG_HB_BYTES == hb_msg.len, "hb: length", hb_msg.len);
    check(bytes_equal(hb_msg.payload, want, 9U),
          "hb: bytes (cmd_reject = codec + post failures + dropped)", 0U);
    check(MQTT_LINK_TOPIC_TELEMETRY == hb_msg.topic, "hb: telemetry topic", 0U);
    check(MQTT_LINK_QOS_0 == hb_msg.qos, "hb: QoS 0", 0U);
    check(!hb_msg.b_retain, "hb: not retained", 0U);

    hb_in.uptime_s = 0U;
    hb_in.connect_count = 0U;
    hb_in.command_dropped = 0U;
    (void)memset(&stats, 0, sizeof(stats));
    check(s1_telem_heartbeat(255U, &hb_in, &stats, &hb_msg)
          && (HDR_HEARTBEAT == hb_msg.payload[0])
          && (0xFFU == hb_msg.payload[1])
          && (0x00U == hb_msg.payload[8]), "hb: seq 255, all else 0", 0U);
}

/*!
 * @brief Counts above a field's range saturate; sums never wrap.
 */
static void
test_heartbeat_saturation (void)
{
    s1_telem_hb_in_t hb_in = {70000U, 300U, 10U};
    s1_telem_stats_t stats;
    s1_telem_pub_t   hb_msg;

    (void)memset(&stats, 0, sizeof(stats));
    stats.codec_reject = 200U;
    stats.post_failure = 50U;

    check(s1_telem_heartbeat(0U, &hb_in, &stats, &hb_msg), "sat: returns true",
          0U);
    /* uptime 65535 in bytes 2-3; connect 255 across bits 39-46; cmd_reject
     * 255 across bits 63-70: byte 7 bit 0 and byte 8 bits 7-1.
     */
    check((0xFFU == hb_msg.payload[2]) && (0xFFU == hb_msg.payload[3]),
          "sat: uptime 70000 -> 65535", 0U);
    check((0x01U == (hb_msg.payload[4] & 0x01U))
          && (0xFEU == hb_msg.payload[5]), "sat: connect 300 -> 255", 0U);
    check((0x01U == (hb_msg.payload[7] & 0x01U))
          && (0xFEU == hb_msg.payload[8]), "sat: cmd_reject 260 -> 255", 0U);

    check(UINT32_MAX == s1_telem_sat_add(UINT32_MAX, 1U), "sat_add max+1",
          0U);
    check(UINT32_MAX == s1_telem_sat_add(1U, UINT32_MAX), "sat_add 1+max",
          0U);
    check(3U == s1_telem_sat_add(1U, 2U), "sat_add 1+2", 0U);
    check(0U == s1_telem_sat_add(0U, 0U), "sat_add 0+0", 0U);
}

/*!
 * @brief 10 s in 50 ms steps across the clock wrap: 10 heartbeats,
 *        exactly 1000 ms apart.
 */
static void
test_schedule_steady (void)
{
    const uint32_t start_msec = 0xFFFFF000U;    /* Wraps 4096 ms in. */
    uint32_t       next = start_msec;
    uint32_t       dues = 0U;
    uint32_t       last = 0U;
    uint32_t       step = 0U;
    bool           b_spaced = true;

    for (step = 0U; step < 200U; step++)
    {
        uint32_t now = start_msec + (50U * step);

        if (s1_telem_hb_due(now, &next))
        {
            if ((dues > 0U) && (1000U != (now - last)))
            {
                b_spaced = false;
            }
            last = now;
            dues++;
        }
    }

    check(10U == dues, "steady: 10 heartbeats in 10 s", dues);
    check(b_spaced, "steady: 1000 ms apart, across the wrap", 0U);
}

/*!
 * @brief 20 s in 70 ms steps: still 20 heartbeats, each within one step
 *        of its absolute deadline (no drift).
 */
static void
test_schedule_jitter (void)
{
    uint32_t next = 0U;
    uint32_t dues = 0U;
    uint32_t step = 0U;
    bool     b_on_grid = true;

    for (step = 0U; step < 286U; step++)
    {
        uint32_t now = 70U * step;

        if (s1_telem_hb_due(now, &next))
        {
            if ((now - (1000U * dues)) >= 70U)
            {
                b_on_grid = false;
            }
            dues++;
        }
    }

    check(20U == dues, "jitter: 20 heartbeats in 20 s", dues);
    check(b_on_grid, "jitter: each within 70 ms of its deadline", 0U);
}

/*!
 * @brief A 3.5 s stall: one heartbeat, then back on the original grid.
 */
static void
test_schedule_stall (void)
{
    uint32_t next = 5000U;

    check(!s1_telem_hb_due(4999U, &next), "stall: 1 ms early, not due",
          next);
    check(s1_telem_hb_due(5000U, &next) && (6000U == next),
          "stall: due on time, next +1000", next);
    check(s1_telem_hb_due(9500U, &next) && (10000U == next),
          "stall: one due after 3.5 s, next on the grid", next);
    check(!s1_telem_hb_due(9550U, &next), "stall: no burst", next);
    check(!s1_telem_hb_due(9999U, &next), "stall: not before the grid",
          next);
    check(s1_telem_hb_due(10000U, &next) && (11000U == next),
          "stall: due on the grid again", next);
}

/*!
 * @brief Publish only while CONNECTED and fresh.
 */
static void
test_should_publish (void)
{
    check(s1_telem_hb_should_publish(MQTT_LINK_STATE_CONNECTED, false),
          "publish: connected and fresh", 0U);
    check(!s1_telem_hb_should_publish(MQTT_LINK_STATE_CONNECTED, true),
          "publish: connected but stale", 0U);
    check(!s1_telem_hb_should_publish(MQTT_LINK_STATE_IDLE, false),
          "publish: idle", 0U);
    check(!s1_telem_hb_should_publish(MQTT_LINK_STATE_RESOLVING, false),
          "publish: resolving", 0U);
    check(!s1_telem_hb_should_publish(MQTT_LINK_STATE_CONNECTING, false),
          "publish: connecting", 0U);
    check(!s1_telem_hb_should_publish(MQTT_LINK_STATE_BACKOFF, false),
          "publish: backoff", 0U);
}

int
main (void)
{
    int      result = EXIT_FAILURE;
    uint32_t map_checks = 0U;

    (void)printf("S1-U14 s1_telem host test (layout version %u)\n",
                 MSG_LAYOUT_VERSION);

    group_begin("commands");
    test_valid_commands();
    test_post_failures();
    test_codec_rejects();
    test_null_arguments();
    group_end();

    group_begin("heartbeat");
    test_heartbeat_bytes();
    test_heartbeat_saturation();
    test_heartbeat_tel_drop();
    group_end();

    group_begin("schedule");
    test_schedule_steady();
    test_schedule_jitter();
    test_schedule_stall();
    test_should_publish();
    test_queue_rules();
    test_schedule_stagger();
    group_end();

    group_begin("fill");
    test_fill_live();
    test_fill_frame();
    test_period_schedule();
    group_end();

    group_begin("wifi fault");
    test_wifi_not_armed();
    test_wifi_set();
    test_wifi_clear();
    test_wifi_held_read();
    test_wifi_drop_replay();
    test_wifi_null_arguments();
    group_end();

    /* S1-U24 part C; its own count is printed so the totals of the groups
     * above stay readable.
     */
    map_checks = g_checks;
    group_begin("map");
    test_map_type4_unchanged();
    test_map_applied();
    test_map_refused();
    test_map_malformed();
    test_map_duplicate();
    test_map_letters();
    test_map_null_and_sum();
    group_end();
    (void)printf("  map group: %" PRIu32 " checks\n", g_checks - map_checks);

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
 * @brief A source with distinctive values in every field, S3 readable.
 *
 * @return The filled-in source.
 */
static s1_telem_src_t
make_src (void)
{
    s1_telem_src_t src = {0U};

    src.motion_state    = 2U;
    src.b_estop         = false;
    src.target_l_mms    = 300;
    src.target_r_mms    = 201;
    src.count_l         = 70000U;
    src.count_r         = 65535U;
    src.dist_mm         = -1234;
    src.speed_l_mms     = 290;
    src.speed_r_mms     = -120;
    src.heading_cdeg    = -4575;
    src.b_s3_ok         = true;
    src.line_state      = 1U;
    src.steer_dv_mms    = -87;
    src.outer_ir        = 12U;
    src.centre_ir       = 230U;
    src.b_overshoot     = true;
    src.last_ch         = 11U;
    src.last_cmd        = 4U;
    src.b_last_rev      = true;
    src.junc_turn       = 3U;
    src.rejects         = 3U;
    src.junctions       = 44U;
    src.losts           = 1U;
    src.barcodes        = 5U;
    src.pitch_hdeg      = -2;
    src.roll_hdeg       = 1;
    src.turn_rate_dps   = -45;
    src.peak_mm         = 37U;
    src.hump_state      = 2U;
    src.motion_class    = 4U;
    src.hump_count      = 3U;
    src.b_imu_ok        = true;
    src.b_cal_done      = true;
    src.b_impact        = false;
    src.ctrl_state      = 3U;
    src.fault_mask      = 0x1E5U;
    src.b_range_valid   = true;
    src.range_mm        = 350U;
    src.servo_deg       = 90;
    src.b_profile_valid = true;
    src.width_mm        = 210U;
    src.clear_left_mm   = 400U;
    src.clear_right_mm  = 2000U;
    return (src);
}

/*!
 * @brief The live message: every field comes from the right source.
 */
static void
test_fill_live (void)
{
    s1_telem_src_t   src = make_src();
    msg_codec_live_t live;
    uint8_t          buf[MSG_LIV_BYTES] = {0U};

    s1_telem_fill_live(&src, 9U, &live);
    check(9U == live.seq, "live: seq", live.seq);
    check(2U == live.motion_state, "live: motion state", live.motion_state);
    check(!live.b_estop, "live: estop clear", 0U);
    /* (300 + 201) / 2 = 250, rounded toward zero. The casts here only pass
     * a signed value to check()'s number for the failure message.
     */
    check(250 == live.cmd_speed_mms, "live: mean of set-points",
          (uint32_t)live.cmd_speed_mms);

    /* -4575 cdeg = -91.5 hdeg, half away from zero = -92. */
    check(-92 == live.heading_hdeg, "live: heading",
          (uint32_t)live.heading_hdeg);
    check(1U == live.line_state, "live: line state", live.line_state);
    check(-87 == live.steer_dv_mms, "live: steer", 0U);
    check((12U == live.outer_ir) && (230U == live.centre_ir),
          "live: sensors", live.outer_ir);
    check(live.b_overshoot, "live: overshoot", 0U);
    check((11U == live.bc_letter) && (4U == live.bc_cmd) && (live.b_bc_rev),
          "live: barcode", live.bc_letter);
    check(3U == live.junc_turn, "live: junction turn", live.junc_turn);
    check((3U == live.rejects) && (44U == live.junctions)
          && (1U == live.losts) && (5U == live.barcodes),
          "live: counters", live.barcodes);
    check(MSG_LIV_BYTES == msg_codec_put_live(&live, buf, sizeof(buf)),
          "live: packs", 0U);

    /* Negative set-points average toward zero as well: (-301 - 200) / 2. */
    src.target_l_mms = -301;
    src.target_r_mms = -200;
    s1_telem_fill_live(&src, 0U, &live);
    /* The cast only passes a signed value to check()'s number for the
     * failure message.
     */
    check(-250 == live.cmd_speed_mms, "live: negative mean",
          (uint32_t)live.cmd_speed_mms);

    src.b_estop = true;
    s1_telem_fill_live(&src, 0U, &live);
    check(live.b_estop, "live: estop set", 0U);

    /* S3 unreadable: UNCAL, no letter, every other S3 field 0; the S2
     * fields are still sent.
     */
    src.b_s3_ok = false;
    s1_telem_fill_live(&src, 1U, &live);
    check((S1_TELEM_LINE_UNCAL == live.line_state)
          && (MSG_BC_LETTER_NONE == live.bc_letter),
          "live: S3 unreadable defaults", live.line_state);
    check((0 == live.steer_dv_mms) && (0U == live.outer_ir)
          && (0U == live.centre_ir) && (!live.b_overshoot)
          && (0U == live.bc_cmd) && (!live.b_bc_rev)
          && (0U == live.junc_turn) && (0U == live.rejects)
          && (0U == live.junctions) && (0U == live.losts)
          && (0U == live.barcodes),
          "live: S3 unreadable, rest zero", 0U);
    check((2U == live.motion_state) && (live.b_estop),
          "live: S2 still sent", live.motion_state);

    s1_telem_fill_live(NULL, 0U, &live);
    s1_telem_fill_live(&src, 0U, NULL);
    check(true, "live: NULL arguments survive", 0U);
}

/*!
 * @brief The 2 Hz frame: every field comes from the right source.
 */
static void
test_fill_frame (void)
{
    static const angle_case_t angles[] =
    {
        { -5, 0U }, { 0, 0U }, { 7, 0U }, { 8, 1U }, { 90, 12U },
        { 172, 22U }, { 173, 23U }, { 180, 24U }, { 181, 24U },
        { 32767, 24U },
    };
    s1_telem_src_t            src = make_src();
    msg_codec_telemetry_t     tel;
    uint8_t                   buf[MSG_TEL_BYTES] = {0U};
    uint32_t                  idx;

    s1_telem_fill_frame(&src, 7U, &tel);
    check(7U == tel.seq, "frame: seq", tel.seq);
    check(3U == tel.ctrl_state, "frame: controller state", tel.ctrl_state);
    check(2U == tel.motion_state, "frame: motion state", tel.motion_state);
    /* 0x1E5 & 0x1F = 0x05: stall and IMU. */
    check(0x05U == tel.fault_mask, "frame: fault mask keeps 5 bits",
          tel.fault_mask);
    check(1U == tel.line_state, "frame: line state", tel.line_state);
    check((11U == tel.bc_letter) && (tel.b_bc_dir) && (0U == tel.bc_cmd),
          "frame: barcode letter and direction, command left to live",
          tel.bc_letter);
    check((290 == tel.speed_l_mms) && (-120 == tel.speed_r_mms),
          "frame: wheel speeds", 0U);
    check((70000U == tel.count_l) && (65535U == tel.count_r)
          && (-1234 == tel.dist_mm), "frame: counts and distance", 0U);
    check((-2 == tel.pitch_hdeg) && (1 == tel.roll_hdeg)
          && (-45 == tel.turn_dps), "frame: terrain angles", 0U);
    check((2U == tel.hump_state) && (37U == tel.peak_mm)
          && (3U == tel.hump_count) && (4U == tel.motion_class),
          "frame: hump fields", tel.motion_class);
    check((tel.b_imu_ok) && (tel.b_cal_done), "frame: IMU flags", 0U);
    check(350U == tel.closest_mm, "frame: closest is the live ping",
          tel.closest_mm);
    check(12U == tel.angle_idx, "frame: angle index", tel.angle_idx);
    check((210U == tel.width_mm) && (400U == tel.clear_l_mm)
          && (2000U == tel.clear_r_mm), "frame: profile", 0U);
    check(MSG_TEL_BYTES == msg_codec_put_telemetry(&tel, buf, sizeof(buf)),
          "frame: packs", 0U);

    for (idx = 0U; idx < (sizeof(angles) / sizeof(angles[0])); idx++)
    {
        src.servo_deg = angles[idx].servo_deg;
        s1_telem_fill_frame(&src, 0U, &tel);
        /* The cast only passes a signed value to check()'s number for the
         * failure message, and the mask keeps its low 16 bits.
         */
        check(angles[idx].idx == tel.angle_idx, "frame: servo angle to index",
              ((uint32_t)angles[idx].servo_deg) & 0xFFFFU);
    }

    src          = make_src();
    src.b_impact = true;
    s1_telem_fill_frame(&src, 0U, &tel);
    check(S1_TELEM_CLASS_IMPACT == tel.motion_class,
          "frame: latched impact forces class IMPACT", tel.motion_class);

    src                 = make_src();
    src.b_range_valid   = false;
    src.b_profile_valid = false;
    s1_telem_fill_frame(&src, 0U, &tel);
    check((0U == tel.closest_mm) && (0U == tel.width_mm)
          && (0U == tel.clear_l_mm) && (0U == tel.clear_r_mm),
          "frame: no echo and no profile give 0", tel.closest_mm);

    src         = make_src();
    src.b_s3_ok = false;
    s1_telem_fill_frame(&src, 0U, &tel);
    check((S1_TELEM_LINE_UNCAL == tel.line_state)
          && (MSG_BC_LETTER_NONE == tel.bc_letter) && (!tel.b_bc_dir),
          "frame: S3 unreadable defaults", tel.line_state);

    s1_telem_fill_frame(NULL, 0U, &tel);
    s1_telem_fill_frame(&src, 0U, NULL);
    check(true, "frame: NULL arguments survive", 0U);
}

/*!
 * @brief The absolute schedule for the 2 Hz and 5 Hz messages.
 */
static void
test_period_schedule (void)
{
    uint32_t next_live = PERIOD_TEST_START_MSEC;
    uint32_t next_tel = PERIOD_TEST_START_MSEC;
    uint32_t n_live = 0U;
    uint32_t n_tel = 0U;
    uint32_t now;

    /* 10 s of 10 ms polls: exactly 50 live messages and 20 frames, with
     * no drift.
     */
    for (now = PERIOD_TEST_START_MSEC; now < PERIOD_TEST_END_MSEC;
         now += PERIOD_TEST_STEP_MSEC)
    {
        if (s1_telem_due(now, S1_TELEM_LIVE_PERIOD_MSEC, &next_live))
        {
            n_live++;
        }

        if (s1_telem_due(now, S1_TELEM_TEL_PERIOD_MSEC, &next_tel))
        {
            n_tel++;
        }
    }

    check(50U == n_live, "period: 5 Hz count in 10 s", n_live);
    check(20U == n_tel, "period: 2 Hz count in 10 s", n_tel);
    check(PERIOD_TEST_END_MSEC == next_live,
          "period: live deadline stays on the grid",
          next_live);

    /* A stall of 1 s gives one message, not five, and stays on the grid. */
    next_live = 1000U;
    check(s1_telem_due(2000U, S1_TELEM_LIVE_PERIOD_MSEC, &next_live),
          "period: due after a stall", 0U);
    check(2200U == next_live, "period: next deadline after a stall",
          next_live);
    check(!s1_telem_due(2000U, S1_TELEM_LIVE_PERIOD_MSEC, &next_live),
          "period: not due twice", 0U);

    /* The clock wraps. */
    next_live = 0xFFFFFF00U;
    check(s1_telem_due(0x00000010U, S1_TELEM_LIVE_PERIOD_MSEC, &next_live),
          "period: due across the clock wrap", 0U);
    check(0x00000090U == next_live, "period: deadline across the wrap",
          next_live);

    next_live = 0U;
    check(!s1_telem_due(100U, 0U, &next_live), "period: 0 is never due", 0U);
    check(!s1_telem_due(100U, S1_TELEM_LIVE_PERIOD_MSEC, NULL),
          "period: NULL deadline", 0U);
}

/*!
 * @brief tel_drop is the skipped and the refused periodic messages.
 */
static void
test_heartbeat_tel_drop (void)
{
    /* tel_drop 9 is bits 47-54: byte 5 bit 0 is 0, byte 6 bits 7-1 are
     * 0001001. Everything else is 0.
     */
    static const uint8_t   want[9] = {HDR_HEARTBEAT, 0x00U, 0x00U, 0x00U,
                                      0x00U, 0x00U, 0x12U, 0x00U, 0x00U};
    const s1_telem_hb_in_t hb_in = {0U, 0U, 0U};
    s1_telem_stats_t       stats;
    s1_telem_pub_t         hb_msg;

    /* The cast discards memset()'s return value, which is only its first
     * argument.
     */
    (void)memset(&stats, 0, sizeof(stats));
    stats.periodic_skipped   = 2U;
    stats.periodic_failed    = 3U;
    stats.hb_publish_failed  = 4U;
    /* These do not count: no session, or not a periodic message. */
    stats.hb_skipped         = 50U;
    stats.ack_publish_failed = 60U;

    check((s1_telem_heartbeat(0U, &hb_in, &stats, &hb_msg))
          && (bytes_equal(hb_msg.payload, want, 9U)),
          "tel_drop: skipped + refused telemetry, live and heartbeats", 0U);

    /* 260 saturates at 255: byte 5 bit 0 and byte 6 bits 7-1 all set. */
    stats.periodic_skipped  = 200U;
    stats.periodic_failed   = 50U;
    stats.hb_publish_failed = 10U;
    check((s1_telem_heartbeat(0U, &hb_in, &stats, &hb_msg))
          && (0x01U == (hb_msg.payload[5] & 0x01U))
          && (0xFEU == hb_msg.payload[6]), "tel_drop: 260 -> 255", 0U);

    /* A sum that would wrap 32 bits must not read as a small count. */
    stats.periodic_skipped  = UINT32_MAX;
    stats.periodic_failed   = 1U;
    stats.hb_publish_failed = 1U;
    check((s1_telem_heartbeat(0U, &hb_in, &stats, &hb_msg))
          && (0xFEU == hb_msg.payload[6]), "tel_drop: no 32-bit wrap", 0U);
}

/*!
 * @brief Telemetry and live each join the queue only with at most 1 record
 *        in it, the heartbeat only with at most 3. Depth 8, so 8 free is
 *        empty.
 */
static void
test_queue_rules (void)
{
    static const queue_case_t cases[] =
    {
        { 8U, true, true, true }, { 7U, true, true, true },
        { 6U, false, false, true }, { 5U, false, false, true },
        { 4U, false, false, false }, { 1U, false, false, false },
        { 0U, false, false, false }, { 9U, true, true, true },
        { UINT32_MAX, true, true, true },
    };
    uint32_t                  idx;

    check(8U == MQTT_LINK_QUEUE_DEPTH, "queue: rules assume depth 8", 0U);

    for (idx = 0U; idx < (sizeof(cases) / sizeof(cases[0])); idx++)
    {
        check(cases[idx].b_tel_allowed
              == s1_telem_queue_allows(cases[idx].free_slots,
                                       S1_TELEM_TEL_QUEUED_MAX),
              "queue: telemetry rule", cases[idx].free_slots);
        check(cases[idx].b_live_allowed
              == s1_telem_queue_allows(cases[idx].free_slots,
                                       S1_TELEM_LIVE_QUEUED_MAX),
              "queue: live rule", cases[idx].free_slots);
        check(cases[idx].b_hb_allowed
              == s1_telem_queue_allows(cases[idx].free_slots,
                                       S1_TELEM_HB_QUEUED_MAX),
              "queue: heartbeat rule", cases[idx].free_slots);
    }
}

/*!
 * @brief The three schedules, started the way telemetry_task starts them,
 *        never have two messages due in the same millisecond.
 */
static void
test_schedule_stagger (void)
{
    uint32_t next_hb = STAGGER_TEST_START_MSEC;
    uint32_t next_tel = STAGGER_TEST_START_MSEC + S1_TELEM_TEL_OFFSET_MSEC;
    uint32_t next_live = STAGGER_TEST_START_MSEC + S1_TELEM_LIVE_OFFSET_MSEC;
    uint32_t n_hb = 0U;
    uint32_t n_tel = 0U;
    uint32_t n_live = 0U;
    uint32_t n_due = 0U;
    uint32_t n_together = 0U;
    uint32_t now;

    for (now = STAGGER_TEST_START_MSEC; now < STAGGER_TEST_END_MSEC;
         now += STAGGER_TEST_STEP_MSEC)
    {
        n_due = 0U;

        if (s1_telem_due(now, S1_TELEM_HB_PERIOD_MSEC, &next_hb))
        {
            n_hb++;
            n_due++;
        }

        if (s1_telem_due(now, S1_TELEM_TEL_PERIOD_MSEC, &next_tel))
        {
            n_tel++;
            n_due++;
        }

        if (s1_telem_due(now, S1_TELEM_LIVE_PERIOD_MSEC, &next_live))
        {
            n_live++;
            n_due++;
        }

        if (n_due > 1U)
        {
            n_together++;
        }
    }

    check(0U == n_together, "stagger: two or more due in one millisecond",
          n_together);
    check((STAGGER_TEST_SPAN_MSEC / S1_TELEM_HB_PERIOD_MSEC) == n_hb,
          "stagger: 1 Hz count in 10 s", n_hb);
    check((STAGGER_TEST_SPAN_MSEC / S1_TELEM_TEL_PERIOD_MSEC) == n_tel,
          "stagger: 2 Hz count in 10 s", n_tel);
    check((STAGGER_TEST_SPAN_MSEC / S1_TELEM_LIVE_PERIOD_MSEC) == n_live,
          "stagger: 5 Hz count in 10 s", n_live);
}

/*!
 * @brief Stand-in for subsystem5_fault_set(): records the call.
 *
 * @param[in] faults The fault bits passed.
 */
static void
stub_fault_set (uint32_t faults)
{
    g_set_calls++;
    g_set_mask = faults;
}

/*!
 * @brief Stand-in for subsystem5_fault_clear(): records the call.
 *
 * @param[in] faults The fault bits passed.
 */
static void
stub_fault_clear (uint32_t faults)
{
    g_clear_calls++;
    g_clear_mask = faults;
}

/*!
 * @brief Forget the fault stubs' calls.
 */
static void
stub_fault_reset (void)
{
    g_set_calls   = 0U;
    g_clear_calls = 0U;
    g_set_mask    = 0U;
    g_clear_mask  = 0U;
}

/*!
 * @brief Build one input for the WiFi fault rule, with a consistent read.
 *
 * @param[in] b_link_up     The radio link is up.
 * @param[in] state         The driver state.
 * @param[in] b_stale       The driver status is stale.
 * @param[in] connect_count Sessions established so far.
 *
 * @return The input.
 */
static s1_telem_wifi_status_t
make_wifi_status (bool b_link_up, mqtt_link_state_t state, bool b_stale,
                  uint32_t connect_count)
{
    const s1_telem_wifi_status_t wifi_status = {b_link_up, true, b_stale, state,
                                                connect_count};

    return (wifi_status);
}

/*!
 * @brief Run the WiFi fault rule once with the stubs.
 *
 * @param[in]     p_status     The input.
 * @param[in,out] pb_fault_set The level.
 *
 * @return What s1_telem_wifi_update() returned.
 */
static bool
wifi_step (const s1_telem_wifi_status_t * p_status, bool * pb_fault_set)
{
    return (s1_telem_wifi_update(p_status, stub_fault_set, stub_fault_clear,
                                 pb_fault_set));
}

/*!
 * @brief Before the first connection, no input changes the bit.
 */
static void
test_wifi_not_armed (void)
{
    uint32_t idx = 0U;
    bool     b_fault_set = false;

    stub_fault_reset();

    for (idx = 0U; idx < WIFI_LOST_COUNT; idx++)
    {
        const s1_telem_wifi_status_t lost =
            make_wifi_status(g_wifi_lost[idx].b_link_up,
                             g_wifi_lost[idx].state,
                             g_wifi_lost[idx].b_stale, 0U);

        check(!wifi_step(&lost, &b_fault_set), g_wifi_lost[idx].p_name, idx);
    }

    check((0U == g_set_calls) && (0U == g_clear_calls),
          "wifi: not armed, no calls", g_set_calls + g_clear_calls);
    check(!b_fault_set, "wifi: not armed, level stays clear", 0U);
}

/*!
 * @brief Once armed, each way of losing the link sets the bit exactly once.
 */
static void
test_wifi_set (void)
{
    const s1_telem_wifi_status_t good =
        make_wifi_status(true, MQTT_LINK_STATE_CONNECTED, false,
                         WIFI_ARMED_COUNT);
    uint32_t idx = 0U;
    bool     b_fault_set = false;
    bool     b_changed = false;

    stub_fault_reset();
    check(!wifi_step(&good, &b_fault_set), "wifi: first connection, no change",
          0U);
    check((0U == g_set_calls) && (0U == g_clear_calls) && (!b_fault_set),
          "wifi: first connection, no calls", g_set_calls + g_clear_calls);

    for (idx = 0U; idx < WIFI_LOST_COUNT; idx++)
    {
        const s1_telem_wifi_status_t lost =
            make_wifi_status(g_wifi_lost[idx].b_link_up,
                             g_wifi_lost[idx].state,
                             g_wifi_lost[idx].b_stale, WIFI_ARMED_COUNT);

        stub_fault_reset();
        b_fault_set = false;
        b_changed   = wifi_step(&lost, &b_fault_set);
        check(b_changed && (1U == g_set_calls) && (0U == g_clear_calls)
              && (FAULT_BIT_WIFI == g_set_mask) && b_fault_set,
              g_wifi_lost[idx].p_name, g_set_calls);

        b_changed = wifi_step(&lost, &b_fault_set);
        check((!b_changed) && (1U == g_set_calls) && (0U == g_clear_calls),
              "wifi: same input again, no second call", idx);
    }
}

/*!
 * @brief The bit clears only when the radio and the session are both good.
 */
static void
test_wifi_clear (void)
{
    const s1_telem_wifi_status_t good =
        make_wifi_status(true, MQTT_LINK_STATE_CONNECTED, false,
                         WIFI_ARMED_COUNT);
    uint32_t idx = 0U;
    bool     b_fault_set = true;
    bool     b_changed = false;

    stub_fault_reset();

    for (idx = 0U; idx < WIFI_LOST_COUNT; idx++)
    {
        const s1_telem_wifi_status_t lost =
            make_wifi_status(g_wifi_lost[idx].b_link_up,
                             g_wifi_lost[idx].state,
                             g_wifi_lost[idx].b_stale, WIFI_ARMED_COUNT);

        check((!wifi_step(&lost, &b_fault_set)) && b_fault_set,
              g_wifi_lost[idx].p_name, idx);
    }

    check((0U == g_set_calls) && (0U == g_clear_calls),
          "wifi: still lost, no calls", g_set_calls + g_clear_calls);

    b_changed = wifi_step(&good, &b_fault_set);
    check(b_changed && (0U == g_set_calls) && (1U == g_clear_calls)
          && (FAULT_BIT_WIFI == g_clear_mask) && (!b_fault_set),
          "wifi: both good, one clear", g_clear_calls);

    b_changed = wifi_step(&good, &b_fault_set);
    check((!b_changed) && (0U == g_set_calls) && (1U == g_clear_calls),
          "wifi: still good, no second call", g_clear_calls);
}

/*!
 * @brief An inconsistent status read holds the level, whichever it is.
 */
static void
test_wifi_held_read (void)
{
    s1_telem_wifi_status_t wifi_status =
        make_wifi_status(false, MQTT_LINK_STATE_CONNECTED, false,
                         WIFI_ARMED_COUNT);
    bool                   b_fault_set = false;

    wifi_status.b_status_read = false;
    stub_fault_reset();
    check((!wifi_step(&wifi_status, &b_fault_set)) && (!b_fault_set),
          "wifi: read failed, radio down, level stays clear", 0U);

    wifi_status.b_link_up = true;
    b_fault_set           = true;
    check((!wifi_step(&wifi_status, &b_fault_set)) && b_fault_set,
          "wifi: read failed, all good, level stays set", 0U);

    check((0U == g_set_calls) && (0U == g_clear_calls),
          "wifi: read failed, no calls", g_set_calls + g_clear_calls);
}

/*!
 * @brief Replay whole drops: the running totals are exact after every pass.
 */
static void
test_wifi_drop_replay (void)
{
    uint32_t idx = 0U;
    bool     b_fault_set = false;

    stub_fault_reset();

    for (idx = 0U; idx < WIFI_REPLAY_COUNT; idx++)
    {
        const wifi_replay_t * const  p_pass = &g_wifi_replay[idx];
        const s1_telem_wifi_status_t wifi_status =
            make_wifi_status(p_pass->b_link_up, p_pass->state, p_pass->b_stale,
                             p_pass->connect_count);

        (void)wifi_step(&wifi_status, &b_fault_set);
        check((p_pass->set_calls == g_set_calls)
              && (p_pass->clear_calls == g_clear_calls)
              && (p_pass->b_fault_set == b_fault_set),
              "wifi: replay totals", idx);
    }
}

/*!
 * @brief A NULL argument makes no call and leaves the level alone.
 */
static void
test_wifi_null_arguments (void)
{
    const s1_telem_wifi_status_t lost =
        make_wifi_status(false, MQTT_LINK_STATE_CONNECTED, false,
                         WIFI_ARMED_COUNT);
    bool b_fault_set = false;

    stub_fault_reset();
    check(!s1_telem_wifi_update(NULL, stub_fault_set, stub_fault_clear,
                                &b_fault_set), "wifi: null status", 0U);
    check(!s1_telem_wifi_update(&lost, NULL, stub_fault_clear, &b_fault_set),
          "wifi: null set", 0U);
    check(!s1_telem_wifi_update(&lost, stub_fault_set, NULL, &b_fault_set),
          "wifi: null clear", 0U);
    check(!s1_telem_wifi_update(&lost, stub_fault_set, stub_fault_clear, NULL),
          "wifi: null level", 0U);
    check((0U == g_set_calls) && (0U == g_clear_calls) && (!b_fault_set),
          "wifi: null arguments, no calls", g_set_calls + g_clear_calls);
}

/* S1-U24 part C: the MAP path. ---------------------------------------- */

/* Golden vectors MAP-1 ("DCBA", id 9), MAP-3 (all 63, id 63) and MAP-4
 * ("09-Z", id 33), docs/S1_binary_frame.md 10.2.
 */
static const uint8_t g_map1[MSG_MAP_BYTES] = {0x17U, 0x24U, 0xD3U, 0x0BU,
                                              0x28U};
static const uint8_t g_map3[MSG_MAP_BYTES] = {0x17U, 0xFFU, 0xFFU, 0xFFU,
                                              0xFCU};
static const uint8_t g_map4[MSG_MAP_BYTES] = {0x17U, 0x84U, 0x02U, 0x64U,
                                              0x8CU};

/* "DDBA", id 9: MAP-1 with RIGHT changed from C (12) to D (13). */
static const uint8_t g_map_ddba[MSG_MAP_BYTES] = {0x17U, 0x24U, 0xD3U,
                                                  0x4BU, 0x28U};

/* Acks written out by hand: opcode 0, then the id, then the result. */
static const uint8_t g_ack_map1_ok[MSG_ACK_BYTES] = {HDR_ACK, 0x09U,
                                                     ACK_BYTE2_QUEUED};
static const uint8_t g_ack_map1_no[MSG_ACK_BYTES] = {HDR_ACK, 0x09U,
                                                     ACK_BYTE2_REJECTED};
static const uint8_t g_ack_unread[MSG_ACK_BYTES] = {HDR_ACK, 0x00U,
                                                    ACK_BYTE2_REJECTED};

/*!
 * @brief Stand-in for the function telemetry_task passes, which calls
 *        s3_set_nav_map(): records the letters and applies S3's own rules
 *        with s3_nav_set_map(), or returns the forced value.
 *
 * @param[in] p_letters The letters for LEFT, RIGHT, STRAIGHT and U-TURN.
 *
 * @return E_OK_VALUE if S3's rules accept the map, otherwise E_PAR_VALUE;
 *         g_map_force instead when it is not 0.
 */
static int32_t
stub_map (const char * p_letters)
{
    int32_t ret = E_PAR_VALUE;

    g_map_calls++;
    (void)memset(g_map_text, 0, sizeof(g_map_text));
    (void)strncpy(g_map_text, p_letters, sizeof(g_map_text) - 1U);

    if (0 != g_map_force)
    {
        ret = g_map_force;
    }
    else if (0 == s3_nav_set_map(p_letters))
    {
        ret = E_OK_VALUE;
    }
    else
    {
        /* S3 refused it and kept the old map. */
    }

    return (ret);
}

/*!
 * @brief Clear the map stub's log, set its forced return value, and put
 *        S3's map back to ABCD.
 */
static void
map_reset (int32_t force)
{
    g_map_calls = 0U;
    (void)memset(g_map_text, 0, sizeof(g_map_text));
    g_map_force = force;
    s3_nav_reset();
}

/*!
 * @brief A received message of up to MSG_MAP_BYTES bytes; len may claim
 *        more, as a corrupt record would.
 */
static mqtt_link_command_t
make_map (const uint8_t * p_bytes, uint16_t len)
{
    mqtt_link_command_t cmd;

    (void)memset(&cmd, 0, sizeof(cmd));
    (void)memcpy(cmd.payload, p_bytes, MSG_MAP_BYTES);
    cmd.payload_len = len;

    return (cmd);
}

/*!
 * @brief Run one message through s1_telem_on_command() and compare its ack
 *        with the bytes expected; the ack must go out like any other.
 */
static bool
map_ack_is (const mqtt_link_command_t * p_cmd, s1_telem_stats_t * p_stats,
            const uint8_t * p_want)
{
    s1_telem_pub_t ack;

    (void)memset(&ack, 0xA5, sizeof(ack));

    return (s1_telem_on_command(p_cmd, stub_post, stub_map, p_stats, &ack)
            && (MSG_ACK_BYTES == ack.len)
            && bytes_equal(ack.payload, p_want, MSG_ACK_BYTES)
            && (MQTT_LINK_TOPIC_ACK == ack.topic)
            && (MQTT_LINK_QOS_1 == ack.qos) && (!ack.b_retain));
}

/*!
 * @brief The command path is unchanged by the map path: START, STOP and
 *        RESET still posted and acked "queued", and the map stub is never
 *        called; a type 5 message is still a codec reject.
 */
static void
test_map_type4_unchanged (void)
{
    static const uint8_t want_stop[MSG_ACK_BYTES] = {HDR_ACK, 0x85U,
                                                     ACK_BYTE2_QUEUED};
    static const uint8_t want_start[MSG_ACK_BYTES] = {HDR_ACK, 0x40U,
                                                      ACK_BYTE2_QUEUED};
    static const uint8_t want_reset[MSG_ACK_BYTES] = {HDR_ACK, 0xFFU,
                                                      ACK_BYTE2_QUEUED};
    mqtt_link_command_t  stop = make_cmd(HDR_COMMAND, 0x85U, 2U);
    mqtt_link_command_t  start = make_cmd(HDR_COMMAND, 0x40U, 2U);
    mqtt_link_command_t  reset = make_cmd(HDR_COMMAND, 0xFFU, 2U);
    mqtt_link_command_t  type5 = make_cmd(HDR_ACK, 0x85U, 2U);
    s1_telem_stats_t     stats;

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(map_ack_is(&stop, &stats, want_stop) && (1U == g_post_calls)
          && (MSG_OP_STOP == g_post_opcode) && (5U == g_post_cmd_id),
          "map: STOP still posted and acked queued", 0U);
    check(map_ack_is(&start, &stats, want_start),
          "map: START id 0 still acked 15 40 00", 0U);
    check(map_ack_is(&reset, &stats, want_reset),
          "map: RESET id 63 still acked 15 FF 00", 0U);
    check(3U == g_post_calls, "map: START, STOP, RESET posted",
          g_post_calls);
    check(map_ack_is(&type5, &stats, g_ack_unread) && (3U == g_post_calls),
          "map: type 5 still a codec reject", 0U);
    check((0U == g_map_calls) && (1U == stats.codec_reject)
          && (0U == stats.map_reject),
          "map: the map stub never called on the command path",
          g_map_calls);
}

/*!
 * @brief MAP-1 ("DCBA", id 9) applied: ack 15 09 00, the stub called once
 *        with "DCBA", nothing posted, no counter changed.
 */
static void
test_map_applied (void)
{
    mqtt_link_command_t cmd = make_map(g_map1, MSG_MAP_BYTES);
    s1_telem_stats_t    stats;
    char                now[S3_NAV_MAP_LEN + 1U];

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(map_ack_is(&cmd, &stats, g_ack_map1_ok),
          "map: DCBA applied, ack 15 09 00", 0U);
    check((1U == g_map_calls) && (0 == strcmp(g_map_text, "DCBA")),
          "map: the stub given DCBA once", g_map_calls);
    s3_nav_get_map(now);
    check(0 == strcmp(now, "DCBA"), "map: S3's map is DCBA", 0U);
    check(0U == g_post_calls, "map: nothing posted to S5", g_post_calls);
    check((0U == stats.codec_reject) && (0U == stats.post_failure)
          && (0U == stats.map_reject), "map: no counter changed", 0U);
}

/*!
 * @brief Refused maps: by the forced stub, and by S3's rules ("DDBA"):
 *        ack 15 09 40, map_reject +1 each, S3's map kept.
 */
static void
test_map_refused (void)
{
    mqtt_link_command_t cmd = make_map(g_map1, MSG_MAP_BYTES);
    mqtt_link_command_t ddba = make_map(g_map_ddba, MSG_MAP_BYTES);
    s1_telem_stats_t    stats;
    char                now[S3_NAV_MAP_LEN + 1U];

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(E_PAR_VALUE);
    check(map_ack_is(&cmd, &stats, g_ack_map1_no)
          && (1U == stats.map_reject),
          "map: stub refuses, ack 15 09 40, map_reject 1", stats.map_reject);
    map_reset(0);
    check(map_ack_is(&ddba, &stats, g_ack_map1_no)
          && (2U == stats.map_reject)
          && (0 == strcmp(g_map_text, "DDBA")),
          "map: DDBA refused by S3's rules, map_reject 2", stats.map_reject);
    s3_nav_get_map(now);
    check(0 == strcmp(now, "ABCD"), "map: S3 kept its map", 0U);
    check((0U == stats.codec_reject) && (0U == stats.post_failure)
          && (0U == g_post_calls), "map: refusals counted only once", 0U);
}

/*!
 * @brief Malformed MAPs: length 4, length 6 (more than the record holds)
 *        and type 7 with version 2; none applied, map_reject +1 each,
 *        ack 15 00 40.
 */
static void
test_map_malformed (void)
{
    uint8_t             version2_bytes[MSG_MAP_BYTES];
    mqtt_link_command_t len4 = make_map(g_map1, MSG_MAP_BYTES - 1U);
    mqtt_link_command_t len6 = make_map(g_map1, MSG_MAP_BYTES + 1U);
    mqtt_link_command_t version2;
    s1_telem_stats_t    stats;

    (void)memcpy(version2_bytes, g_map1, MSG_MAP_BYTES);
    version2_bytes[0] = 0x27U;
    version2 = make_map(version2_bytes, MSG_MAP_BYTES);
    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(map_ack_is(&len4, &stats, g_ack_unread)
          && (1U == stats.map_reject), "map: length 4 refused", 4U);
    check(map_ack_is(&len6, &stats, g_ack_unread)
          && (2U == stats.map_reject), "map: length 6 refused", 6U);
    check(map_ack_is(&version2, &stats, g_ack_unread)
          && (3U == stats.map_reject), "map: version 2 refused", 2U);
    check((0U == g_map_calls) && (0U == g_post_calls)
          && (0U == stats.codec_reject),
          "map: malformed ones never reach S3 or S5", g_map_calls);
}

/*!
 * @brief The same MAP delivered twice (a QoS 1 duplicate): two acks with
 *        the same result, two calls; applied or refused alike.
 */
static void
test_map_duplicate (void)
{
    mqtt_link_command_t cmd = make_map(g_map1, MSG_MAP_BYTES);
    s1_telem_stats_t    stats;

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(map_ack_is(&cmd, &stats, g_ack_map1_ok)
          && map_ack_is(&cmd, &stats, g_ack_map1_ok)
          && (2U == g_map_calls) && (0U == stats.map_reject),
          "map: applied twice, two equal acks", g_map_calls);
    map_reset(E_PAR_VALUE);
    check(map_ack_is(&cmd, &stats, g_ack_map1_no)
          && map_ack_is(&cmd, &stats, g_ack_map1_no)
          && (2U == g_map_calls) && (2U == stats.map_reject),
          "map: refused twice, two equal acks", g_map_calls);
}

/*!
 * @brief The letters reach S3 in the right order: MAP-4 ("09-Z", id 33)
 *        as "09-Z", and MAP-3 (every index 63) as "????", which S3
 *        refuses.
 */
static void
test_map_letters (void)
{
    static const uint8_t want4[MSG_ACK_BYTES] = {HDR_ACK, 0x21U,
                                                 ACK_BYTE2_QUEUED};
    static const uint8_t want3[MSG_ACK_BYTES] = {HDR_ACK, 0x3FU,
                                                 ACK_BYTE2_REJECTED};
    mqtt_link_command_t  map4 = make_map(g_map4, MSG_MAP_BYTES);
    mqtt_link_command_t  map3 = make_map(g_map3, MSG_MAP_BYTES);
    s1_telem_stats_t     stats;

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(map_ack_is(&map4, &stats, want4)
          && (0 == strcmp(g_map_text, "09-Z")),
          "map: 09-Z reaches S3 as 09-Z, applied", 0U);
    check(map_ack_is(&map3, &stats, want3)
          && (0 == strcmp(g_map_text, "????"))
          && (1U == stats.map_reject),
          "map: index 63 reaches S3 as ?, refused", 0U);
}

/*!
 * @brief A NULL map function gives no ack; the heartbeat's cmd_reject
 *        adds map_reject to the other three counts.
 */
static void
test_map_null_and_sum (void)
{
    mqtt_link_command_t cmd = make_map(g_map1, MSG_MAP_BYTES);
    s1_telem_hb_in_t    hb_in = {2U, 3U, 4U};
    s1_telem_stats_t    stats;
    s1_telem_pub_t      pub;

    (void)memset(&stats, 0, sizeof(stats));
    stub_reset(E_OK_VALUE);
    map_reset(0);
    check(!s1_telem_on_command(&cmd, stub_post, NULL, &stats, &pub)
          && (0U == g_map_calls), "map: null map function", 0U);

    /* cmd_reject = 1 + 2 + 3 + 4 = 10 = 0000 1010, at bits 63 to 70:
     * bit 63 (the 0 at the top) ends byte 7, and byte 8 is the other
     * seven bits and a padding 0, 0001 0100.
     */
    stats.codec_reject = 1U;
    stats.post_failure = 2U;
    stats.map_reject = 3U;
    check(s1_telem_heartbeat(1U, &hb_in, &stats, &pub)
          && (0x14U == pub.payload[8]),
          "map: cmd_reject = codec + post + map + dropped", pub.payload[8]);
}

/*** end of file ***/

