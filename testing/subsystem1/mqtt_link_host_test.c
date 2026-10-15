/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_link_host_test.c
 *
 * @brief Host test for the public half of mqtt_link.c.
 *
 * @par What it tests
 * The real app_program/subsystem1/mqtt_link.c is compiled on the build
 * machine, not on the Pico, and its public functions are called directly:
 * topic validation in mqtt_link_publish(), the seqlock reader in
 * mqtt_link_get_status(), the pass-through in mqtt_link_receive_command(),
 * the order of work in mqtt_link_init(), the last_poll_msec / b_stale
 * staleness fields mqtt_link_poll() and mqtt_link_get_status() share, and
 * the single-writer gate between mqtt_link_poll() and start_link() (see
 * mqtt_link.c's own "Single writer" note). Two file-local statics are
 * also called directly, to reach state the public API cannot drive
 * without a real broker: enter_backoff()/next_backoff() (test_backoff(),
 * the reconnect delay sequence and its cap) and start_connect()
 * (test_connect_info(), the keep-alive placed in the CONNECT info). The
 * rest of the state machine and the lwIP callbacks are not exercised here
 * -- every other test drives mqtt_link_poll() with a zeroed struct netif,
 * so link_ready() is always false and run_state() never leaves
 * MQTT_LINK_STATE_IDLE.
 *
 * @par What the stubs replace
 * mqtt_queue.c is replaced by stubs that record what they are given and
 * return a result chosen by the test, so no kernel is needed. That
 * includes mqtt_queue_now_msec(): the stub returns a test-controlled
 * g_stub_now_msec instead of reading the kernel's clock, so staleness
 * behaviour (including a 32-bit wraparound) can be driven exactly. The
 * lwIP functions mqtt_link.c calls (sys_now, dns_gethostbyname,
 * ip4addr_aton, mqtt_client_new, mqtt_set_inpub_callback,
 * mqtt_client_connect, mqtt_disconnect, mqtt_publish and mqtt_sub_unsub)
 * are replaced by stubs that are never reached by these tests. The lwIP
 * headers themselves are the real ones.
 *
 * @par Build gate
 * mqtt_link.c's whole body is gated on TM_MQTT (see its own "Build gate"
 * note), so this file must be built with -DTM_MQTT=1 or the #include
 * below pulls in an empty translation unit and nothing here compiles.
 *
 * @par Build and run, from the repository root
 * PICO_SDK_PATH must point at a Pico SDK with lib/lwip checked out.
 * In zsh (this Mac's shell), an unquoted $VAR holding several
 * space-separated flags is passed as one argument, not several -- every
 * command below that builds -I flags into a variable needs "eval" in
 * front of the compiler invocation to split them correctly. (A shell
 * that word-splits unquoted variables by default, e.g. bash, does not
 * need it, but eval is harmless there too.)
 *
 *   T=${TMPDIR:-/tmp}
 *   I="-Iapp_program/subsystem1"
 *   I="$I -Ilib/liblwip/sysdepend/pico_rp2040/include"
 *   I="$I -Ilib/liblwip/sysdepend/pico_rp2040"
 *   I="$I -I$PICO_SDK_PATH/lib/lwip/src/include"
 *   eval gcc -std=gnu11 -O0 -w -DTM_MQTT=1 $I \
 *       -o $T/mqtt_link_host_test testing/subsystem1/mqtt_link_host_test.c
 *   $T/mqtt_link_host_test
 *
 * It prints one line per failed check, then "RESULT: all passed" and exits
 * with 0, or "RESULT: n FAILED" and exits with 1. The
 * app_program/subsystem1/mqtt_config.h file must exist (copy it from
 * mqtt_config.example.h in the same folder), because mqtt_link.c includes
 * it.
 *
 * @par Checking the configuration helper
 * The build-time prefix decides what mqtt_link_init() does, so that check
 * needs a second build per prefix. Put a changed copy of mqtt_config.h in a
 * directory searched before app_program/subsystem1, and run with "bad" for
 * a prefix that init must refuse, or "good" for one it must accept:
 *
 *   C=app_program/subsystem1/mqtt_config.h
 *   D=$T/mqtt_cfg_test
 *   mkdir -p $D
 *   sed 's|"robocar/car1"|"/robocar"|' $C > $D/mqtt_config.h
 *   eval gcc -I$D $I -std=gnu11 -O0 -w -DTM_MQTT=1 \
 *       -o $D/test testing/subsystem1/mqtt_link_host_test.c
 *   $D/test bad
 *
 * The prefixes that must be refused are one with a leading slash, one with
 * a trailing slash, and one containing # or +.
 *
 * @par Deviations from BARR-C:2018
 * This file is test code and is never part of the firmware build. Each
 * deviation is proposed by Buddy 1, pending lecturer confirmation.
 * - Rule 4.3.f: the file includes mqtt_link.c, so that the tests can read
 *   its static variables and prove the order of work in mqtt_link_init()
 *   and the single-writer gate in mqtt_link_poll().
 * - Rule 4.1.b: the module name starts with the same 8 characters as
 *   mqtt_link.
 * - Rule 4.1.d: this module contains main() and its name has no "main",
 *   because the file name was fixed by the project owner.
 * - Rule 1.1.c: __builtin_printf() is used instead of printf(), because
 *   including stdio.h here clashes with lwIP's definition of ssize_t.
 */

#include "../../app_program/subsystem1/mqtt_link.c"

/* Pins CONNECT_TIMEOUT_MSEC at compile time. No test here reaches
 * MQTT_LINK_STATE_CONNECTING (every test keeps the stub netif down, so
 * link_ready() is always false), so a regression in this constant would
 * not be caught at run time -- this typedef fails to compile instead.
 */
typedef uint8_t host_test_check_connect_timeout_t[
    (CONNECT_TIMEOUT_MSEC == 5000U) ? 1 : -1];

/* Stub state. The tests set the results and read what the stubs recorded. */
static bool                 gb_stub_ready = false;
static int32_t              g_stub_init_result = 0;
static int32_t              g_stub_init_calls = 0;
static int32_t              g_stub_put_result = 0;
static int32_t              g_stub_put_calls = 0;
static int32_t              g_stub_command_result = 0;
static uint32_t             g_stub_command_timeout = 0U;
static uint32_t             g_stub_full_count = 0U;
static uint32_t             g_stub_now_msec = 0U;
static mqtt_queue_publish_t g_stub_last_msg;
static mqtt_link_status_t   g_stub_snapshot;
static bool                 gb_stub_init_status_ok = false;
static bool                 gb_stub_init_topics_ok = false;
static uint16_t             g_stub_init_port = 0U;

/* What mqtt_client_connect() last received, read by test_connect_info(). */
static uint16_t             g_stub_connect_keep_alive = 0U;

/* A netif with every field zero. netif_is_up() reads only p_netif->flags
 * (0, so "not up"), and link_ready()'s "&&" chain short-circuits there
 * without touching the IP address field -- so this is safe to pass to
 * mqtt_link_poll() without ever bringing the state machine out of
 * MQTT_LINK_STATE_IDLE, which is outside what this file tests (see "What
 * it tests" above).
 */
static struct netif g_stub_netif;

/* Test data and the count of failed checks. */
static const uint8_t g_data[8] = { 1U, 2U, 3U, 4U, 5U, 6U, 7U, 8U };
static char          g_topic_max[MQTT_LINK_TOPIC_MAX + 1U];
static char          g_topic_over[MQTT_LINK_TOPIC_MAX + 2U];

/* A source buffer as large as the longest payload mqtt_link_publish()
 * accepts. g_data is only 8 bytes, so the "longest payload" case in
 * test_publish_accepted() must not read it as the source -- that would
 * read past its end.
 */
static uint8_t       g_data_max[MQTT_LINK_PAYLOAD_MAX];

static int32_t       g_failures = 0;

static void say(const char * p_text);
static void say_failure(int line);
static void check(bool b_ok, int line);
static void expect_param(const char * p_topic, int line);
static void expect_ok(const char * p_topic, int line);
static void test_publish_rejections(void);
static void test_publish_accepted(void);
static void test_get_status(void);
static void test_receive_command(void);
static void test_init_order(void);
static void test_init_second_call(void);
static void test_init_queue_failure(void);
static void test_staleness(void);
static void test_poll_before_init(void);
static void test_poll_after_init(void);
static void test_connect_info(void);
static void test_backoff(void);
static int run_config_mode(bool b_expect_refusal);

/* The queue stubs. The casts to void only silence the unused-parameter
 * warning. A cast to void discards the value, so there is no range to
 * check.
 */

int32_t
mqtt_queue_init (void)
{
    g_stub_init_calls++;

    /* What the poll step would see at the moment the queue becomes ready. */
    mqtt_link_get_status(&g_stub_snapshot);
    gb_stub_init_status_ok = g_stub_snapshot.b_initialised;
    g_stub_init_port = g_stub_snapshot.broker_port;
    gb_stub_init_topics_ok =
        ((0 == strcmp(g_status_topic,
                      MQTT_LINK_CFG_TOPIC_PREFIX "/status"))
         && (0 == strcmp(g_cmd_filter,
                         MQTT_LINK_CFG_TOPIC_PREFIX "/cmd/#")));

    if (MQTT_LINK_OK == g_stub_init_result)
    {
        gb_stub_ready = true;
    }

    return (g_stub_init_result);
}

bool
mqtt_queue_is_ready (void)
{
    return (gb_stub_ready);
}

int32_t
mqtt_queue_put_publish (const mqtt_queue_publish_t * p_msg)
{
    g_stub_put_calls++;
    g_stub_last_msg = *p_msg;

    return (g_stub_put_result);
}

int32_t
mqtt_queue_get_publish (mqtt_queue_publish_t * p_msg)
{
    /* The stub never has a message, and does not need the argument. */
    (void)p_msg;

    return (MQTT_LINK_ERR_TIMEOUT);
}

int32_t
mqtt_queue_put_command (const mqtt_link_command_t * p_command)
{
    /* The stub accepts every command, and does not need the argument. */
    (void)p_command;

    return (MQTT_LINK_OK);
}

int32_t
mqtt_queue_get_command (mqtt_link_command_t * p_command,
                        uint32_t timeout_msec)
{
    /* The stub records the time-out and does not need the command. */
    (void)p_command;
    g_stub_command_timeout = timeout_msec;

    return (g_stub_command_result);
}

uint32_t
mqtt_queue_full_count (void)
{
    return (g_stub_full_count);
}

uint32_t
mqtt_queue_now_msec (void)
{
    return (g_stub_now_msec);
}

/* The lwIP stubs. Every test but test_connect_info() leaves these
 * unreached, and they ignore their arguments, so the casts to void only
 * silence the compiler -- a cast to void discards the value, so there is
 * no range to check. mqtt_client_connect() is the one exception: it
 * records the keep-alive it was given, for that test to read.
 */

u32_t
sys_now (void)
{
    return (0U);
}

err_t
dns_gethostbyname (const char * p_name, ip_addr_t * p_addr,
                   dns_found_callback found, void * p_arg)
{
    (void)p_name;
    (void)p_addr;
    (void)found;
    (void)p_arg;

    return (ERR_VAL);
}

int
ip4addr_aton (const char * p_text, ip4_addr_t * p_addr)
{
    (void)p_text;
    (void)p_addr;

    return (0);
}

mqtt_client_t *
mqtt_client_new (void)
{
    return (NULL);
}

void
mqtt_set_inpub_callback (mqtt_client_t * p_client,
                         mqtt_incoming_publish_cb_t publish_cb,
                         mqtt_incoming_data_cb_t data_cb, void * p_arg)
{
    (void)p_client;
    (void)publish_cb;
    (void)data_cb;
    (void)p_arg;
}

err_t
mqtt_client_connect (mqtt_client_t * p_client, const ip_addr_t * p_addr,
                     u16_t port, mqtt_connection_cb_t connection_cb,
                     void * p_arg,
                     const struct mqtt_connect_client_info_t * p_info)
{
    (void)p_client;
    (void)p_addr;
    (void)port;
    (void)connection_cb;
    (void)p_arg;

    /* Recorded for test_connect_info(); every other field is unused. */
    g_stub_connect_keep_alive = p_info->keep_alive;

    return (ERR_VAL);
}

void
mqtt_disconnect (mqtt_client_t * p_client)
{
    (void)p_client;
}

err_t
mqtt_publish (mqtt_client_t * p_client, const char * p_topic,
              const void * p_payload, u16_t length, u8_t qos, u8_t retain,
              mqtt_request_cb_t done_cb, void * p_arg)
{
    (void)p_client;
    (void)p_topic;
    (void)p_payload;
    (void)length;
    (void)qos;
    (void)retain;
    (void)done_cb;
    (void)p_arg;

    return (ERR_VAL);
}

err_t
mqtt_sub_unsub (mqtt_client_t * p_client, const char * p_topic, u8_t qos,
                mqtt_request_cb_t done_cb, void * p_arg, u8_t sub)
{
    (void)p_client;
    (void)p_topic;
    (void)qos;
    (void)done_cb;
    (void)p_arg;
    (void)sub;

    return (ERR_VAL);
}

/* The test helpers. */

static void
say (const char * p_text)
{
    __builtin_printf("%s", p_text);
}

static void
say_failure (int line)
{
    __builtin_printf("  FAIL line %d\n", line);
}

static void
check (bool b_ok, int line)
{
    if (!b_ok)
    {
        g_failures++;
        say_failure(line);
    }
}

/* Publish one byte to a topic and expect it to be refused. */
static void
expect_param (const char * p_topic, int line)
{
    const int32_t calls = g_stub_put_calls;

    check((MQTT_LINK_ERR_PARAM
           == mqtt_link_publish(p_topic, g_data, 1U, MQTT_LINK_QOS_0, false))
          && (calls == g_stub_put_calls), line);
}

/* Publish one byte to a topic and expect it to reach the queue. */
static void
expect_ok (const char * p_topic, int line)
{
    check(MQTT_LINK_OK
          == mqtt_link_publish(p_topic, g_data, 1U, MQTT_LINK_QOS_0, false),
          line);
}

/* The tests. */

static void
test_publish_rejections (void)
{
    const int32_t calls = g_stub_put_calls;

    say("-- publish rejections\n");
    g_stub_put_result = MQTT_LINK_OK;

    expect_param(NULL, __LINE__);
    expect_param("", __LINE__);
    expect_param("/leading", __LINE__);
    expect_param("status", __LINE__);
    expect_param("a#", __LINE__);
    expect_param("#", __LINE__);
    expect_param("a/+/b", __LINE__);
    expect_param(g_topic_over, __LINE__);
    expect_param("cmd/x", __LINE__);
    expect_param("cmd/", __LINE__);
    expect_param("cmd/mission/stop", __LINE__);

    check(MQTT_LINK_ERR_PARAM
          == mqtt_link_publish("t", g_data, MQTT_LINK_PAYLOAD_MAX + 1U,
                               MQTT_LINK_QOS_0, false), __LINE__);
    /* Literal 161U, so raising MQTT_LINK_PAYLOAD_MAX without also raising
     * the boundary here still gets caught.
     */
    check(MQTT_LINK_ERR_PARAM
          == mqtt_link_publish("t", g_data_max, 161U,
                               MQTT_LINK_QOS_0, false), __LINE__);
    check(MQTT_LINK_ERR_PARAM
          == mqtt_link_publish("t", NULL, 5U, MQTT_LINK_QOS_0, false),
          __LINE__);
    check(MQTT_LINK_ERR_PARAM
          == mqtt_link_publish("t", g_data, 1U, (mqtt_link_qos_t)2, false),
          __LINE__);

    /* No rejected call may have reached the queue. */
    check(calls == g_stub_put_calls, __LINE__);
}

static void
test_publish_accepted (void)
{
    say("-- publish accepted cases\n");
    g_stub_put_result = MQTT_LINK_OK;

    check(MQTT_LINK_OK
          == mqtt_link_publish("telemetry/state", g_data, 8U,
                               MQTT_LINK_QOS_1, true), __LINE__);
    check(0 == strcmp(g_stub_last_msg.topic, "telemetry/state"), __LINE__);
    check((8U == g_stub_last_msg.payload_len)
          && (0 == memcmp(g_stub_last_msg.payload, g_data, 8U)), __LINE__);
    check((1U == g_stub_last_msg.qos) && (g_stub_last_msg.b_retain),
          __LINE__);

    /* Exactly the longest topic. */
    expect_ok(g_topic_max, __LINE__);
    check(0 == strcmp(g_stub_last_msg.topic, g_topic_max), __LINE__);

    /* An empty payload, and the longest payload. */
    check(MQTT_LINK_OK
          == mqtt_link_publish("t", NULL, 0U, MQTT_LINK_QOS_0, false),
          __LINE__);
    check(MQTT_LINK_OK
          == mqtt_link_publish("t", g_data_max, MQTT_LINK_PAYLOAD_MAX,
                               MQTT_LINK_QOS_0, false), __LINE__);
    /* Literal 160U, so lowering MQTT_LINK_PAYLOAD_MAX below the agreed
     * limit without also lowering this call still gets caught.
     */
    check(MQTT_LINK_OK
          == mqtt_link_publish("t", g_data_max, 160U,
                               MQTT_LINK_QOS_0, false), __LINE__);

    /* Only the exact status topic is reserved, and only a leading cmd/ is
     * refused.
     */
    expect_ok("status/extra", __LINE__);
    expect_ok("cmd", __LINE__);
    expect_ok("xcmd/y", __LINE__);
    expect_ok("telemetry/cmd/x", __LINE__);

    /* Whatever the queue says is passed back. */
    g_stub_put_result = MQTT_LINK_ERR_FULL;
    check(MQTT_LINK_ERR_FULL
          == mqtt_link_publish("t", g_data, 1U, MQTT_LINK_QOS_0, false),
          __LINE__);
    g_stub_put_result = MQTT_LINK_ERR_NOT_READY;
    check(MQTT_LINK_ERR_NOT_READY
          == mqtt_link_publish("t", g_data, 1U, MQTT_LINK_QOS_0, false),
          __LINE__);
    g_stub_put_result = MQTT_LINK_OK;
}

static void
test_get_status (void)
{
    mqtt_link_status_t status;

    say("-- get_status\n");

    /* A stable copy, with the queue's drop count filled in. */
    g_stub_full_count = 7U;
    check(mqtt_link_get_status(&status), __LINE__);
    check(7U == status.publish_queue_full, __LINE__);

    g_stub_full_count = 9U;
    check(mqtt_link_get_status(&status), __LINE__);
    check(9U == status.publish_queue_full, __LINE__);

    /* An odd version means an update is in progress. The reader gives up,
     * says so, and still fills in the drop count.
     */
    g_version = 1U;
    g_stub_full_count = 11U;
    check(!mqtt_link_get_status(&status), __LINE__);
    check(11U == status.publish_queue_full, __LINE__);
    check(!mqtt_link_get_status(NULL), __LINE__);

    g_version = 2U;
    check(mqtt_link_get_status(&status), __LINE__);
}

static void
test_receive_command (void)
{
    mqtt_link_command_t command;

    say("-- receive_command\n");

    check(MQTT_LINK_ERR_PARAM == mqtt_link_receive_command(NULL, 5U),
          __LINE__);

    g_stub_command_result = MQTT_LINK_ERR_TIMEOUT;
    check(MQTT_LINK_ERR_TIMEOUT == mqtt_link_receive_command(&command, 123U),
          __LINE__);
    check(123U == g_stub_command_timeout, __LINE__);

    g_stub_command_result = MQTT_LINK_OK;
    check(MQTT_LINK_OK
          == mqtt_link_receive_command(&command, MQTT_LINK_WAIT_FOREVER),
          __LINE__);
    check(MQTT_LINK_WAIT_FOREVER == g_stub_command_timeout, __LINE__);
}

static void
test_init_order (void)
{
    mqtt_link_status_t status;

    say("-- init order, good configuration\n");
    gb_stub_ready = false;
    g_status.b_initialised = false;
    g_status.broker_port = 0U;
    g_stub_init_calls = 0;
    g_stub_init_result = MQTT_LINK_OK;

    check(MQTT_LINK_OK == mqtt_link_init(), __LINE__);
    check(1 == g_stub_init_calls, __LINE__);

    /* When the queue went ready, everything was already in place. */
    check(gb_stub_init_status_ok, __LINE__);
    check(MQTT_LINK_CFG_BROKER_PORT == g_stub_init_port, __LINE__);
    check(gb_stub_init_topics_ok, __LINE__);

    /* Cast to void discards the bool; no range to check. The test reads
     * the snapshot, not this result.
     */
    (void)mqtt_link_get_status(&status);
    check((0 == status.last_error) && (status.b_initialised), __LINE__);
}

static void
test_init_second_call (void)
{
    const int32_t calls = g_stub_init_calls;
    char          saved[sizeof(g_status_topic)];

    say("-- a second init after success does nothing\n");
    memcpy(saved, g_status_topic, sizeof(saved));
    g_status.b_initialised = false;

    check(MQTT_LINK_OK == mqtt_link_init(), __LINE__);
    check(calls == g_stub_init_calls, __LINE__);
    check(!g_status.b_initialised, __LINE__);
    check(0 == memcmp(saved, g_status_topic, sizeof(saved)), __LINE__);

    g_status.b_initialised = true;
}

static void
test_init_queue_failure (void)
{
    mqtt_link_status_t status;
    uint32_t            published_last_poll;

    say("-- init when the queue cannot be created\n");
    gb_stub_ready = false;
    g_stub_init_result = MQTT_LINK_ERR_KERNEL;

    check(MQTT_LINK_ERR_KERNEL == mqtt_link_init(), __LINE__);
    /* Cast to void discards the bool; no range to check. The test reads
     * the snapshot, not this result.
     */
    (void)mqtt_link_get_status(&status);

    /* Rolled back, with the reason left where the status shows it. This is
     * start_link()'s own failure-path publish_status() call -- it always
     * runs, and is safe, because gb_ready (mqtt_queue.c) is never set when
     * mqtt_queue_init() itself fails.
     */
    check(!status.b_initialised, __LINE__);
    check(MQTT_LINK_ERR_KERNEL == status.last_error, __LINE__);

    /* The queue never becomes ready after a failed init, so every poll
     * from here on stays gated: last_poll_msec is stamped internally (see
     * test_poll_before_init() for that half), but nothing is published
     * again -- the error start_link() left stands forever.
     */
    check(!mqtt_queue_is_ready(), __LINE__);
    published_last_poll = g_published.last_poll_msec;
    g_stub_now_msec = published_last_poll + 999U;
    mqtt_link_poll(&g_stub_netif);
    check(published_last_poll == g_published.last_poll_msec, __LINE__);
    check(MQTT_LINK_ERR_KERNEL == g_published.last_error, __LINE__);
}

static void
test_staleness (void)
{
    mqtt_link_status_t status;

    say("-- last_poll_msec / b_stale\n");

    /* Never polled (last_poll_msec at its zero default): once the clock
     * has advanced past the threshold, the snapshot reads stale even
     * though nothing has gone wrong -- there is simply no fresh
     * information yet.
     */
    gb_stub_ready = true;
    g_status.last_poll_msec = 0U;
    g_published.last_poll_msec = 0U;
    g_stub_now_msec = MQTT_LINK_STALE_MSEC;
    check(mqtt_link_get_status(&status), __LINE__);
    check(status.b_stale, __LINE__);

    /* Right after a poll (queue ready, so publish_status() runs), the
     * snapshot is fresh.
     */
    mqtt_link_poll(&g_stub_netif);
    check(mqtt_link_get_status(&status), __LINE__);
    check(!status.b_stale, __LINE__);
    check(g_stub_now_msec == status.last_poll_msec, __LINE__);

    /* One tick before the threshold: still fresh. */
    g_stub_now_msec += (MQTT_LINK_STALE_MSEC - 1U);
    check(mqtt_link_get_status(&status), __LINE__);
    check(!status.b_stale, __LINE__);

    /* Exactly at the threshold: now stale. */
    g_stub_now_msec += 1U;
    check(mqtt_link_get_status(&status), __LINE__);
    check(status.b_stale, __LINE__);

    /* A 32-bit clock wraparound, small distance: poll 100 ms before the
     * wrap, then check 50 ms after it. True elapsed is 150 ms, well under
     * the threshold, so this must read fresh. (With a distance-to-wrap
     * this much smaller than MQTT_LINK_STALE_MSEC, last + MQTT_LINK_
     * STALE_MSEC wraps too, so a naive "now >= last + MQTT_LINK_STALE_MSEC"
     * check would -- by coincidence, not correctness -- give the same
     * answer as the real subtraction here. The next case below is chosen
     * so that coincidence cannot happen.)
     */
    g_stub_now_msec = 0U - 100U;
    mqtt_link_poll(&g_stub_netif);
    g_stub_now_msec = 50U;
    check(mqtt_link_get_status(&status), __LINE__);
    check(!status.b_stale, __LINE__);

    /* A 32-bit clock wraparound, large distance: poll 2000 ms before the
     * wrap -- more than MQTT_LINK_STALE_MSEC away, so last + MQTT_LINK_
     * STALE_MSEC does NOT itself overflow -- then check 500 ms after the
     * wrap. True elapsed is 2500 ms, over the threshold, so this must
     * read stale. A naive "now >= last + MQTT_LINK_STALE_MSEC" check gets
     * this one wrong: last + MQTT_LINK_STALE_MSEC is a large value just
     * under UINT32_MAX, and the small post-wrap "now" (500) is never
     * greater than or equal to it, so that check would wrongly read
     * fresh. Only the correct unsigned-subtraction form reads this as
     * stale.
     */
    g_stub_now_msec = 0U - 2000U;
    mqtt_link_poll(&g_stub_netif);
    g_stub_now_msec = 500U;
    check(mqtt_link_get_status(&status), __LINE__);
    check(status.b_stale, __LINE__);
}

static void
test_poll_before_init (void)
{
    say("-- poll before init: no crash, no publish, stamp still happens\n");

    gb_stub_ready = false;
    g_published.last_poll_msec = 0U;
    g_published.b_initialised = false;

    g_stub_now_msec = 12345U;
    mqtt_link_poll(&g_stub_netif);        /* must not crash */

    /* The internal stamp happens even though the queue is not ready. */
    check(12345U == g_status.last_poll_msec, __LINE__);

    /* publish_status() did not run: the published snapshot is untouched. */
    check(0U == g_published.last_poll_msec, __LINE__);
    check(!g_published.b_initialised, __LINE__);
}

static void
test_poll_after_init (void)
{
    say("-- poll after a successful init publishes\n");

    gb_stub_ready = true;
    g_published.last_poll_msec = 0U;

    g_stub_now_msec = 54321U;
    mqtt_link_poll(&g_stub_netif);

    check(54321U == g_status.last_poll_msec, __LINE__);
    check(54321U == g_published.last_poll_msec, __LINE__);
}

static void
test_connect_info (void)
{
    say("-- start_connect() carries the configured keep-alive\n");

    g_stub_connect_keep_alive = 0U;
    start_connect(0U);

    /* Literal 5U, so reverting KEEPALIVE_SECONDS still gets caught. */
    check(5U == g_stub_connect_keep_alive, __LINE__);
}

static void
test_backoff (void)
{
    say("-- reconnect backoff doubles and then holds its cap\n");

    g_status.b_connected = false;
    g_backoff_msec = 0U;

    enter_backoff(0U);
    check(1000U == g_backoff_msec, __LINE__);
    enter_backoff(0U);
    check(2000U == g_backoff_msec, __LINE__);
    enter_backoff(0U);
    check(4000U == g_backoff_msec, __LINE__);
    enter_backoff(0U);
    check(8000U == g_backoff_msec, __LINE__);
    enter_backoff(0U);
    check(8000U == g_backoff_msec, __LINE__);
}

/* The configuration check, run once per build of this file. */
static int
run_config_mode (bool b_expect_refusal)
{
    mqtt_link_status_t status;
    int32_t            result = MQTT_LINK_OK;

    gb_stub_ready = false;
    g_stub_init_result = MQTT_LINK_OK;
    g_stub_init_calls = 0;
    result = mqtt_link_init();
    /* Cast to void discards the bool; no range to check. The test reads
     * the snapshot, not this result.
     */
    (void)mqtt_link_get_status(&status);

    __builtin_printf("  prefix \"%s\": init returned %d, "
                     "mqtt_queue_init called %d time(s)\n",
                     MQTT_LINK_CFG_TOPIC_PREFIX, result,
                     g_stub_init_calls);

    if (b_expect_refusal)
    {
        check((MQTT_LINK_ERR_PARAM == result) && (0 == g_stub_init_calls),
              __LINE__);
        check((MQTT_LINK_ERR_PARAM == status.last_error)
              && (!status.b_initialised), __LINE__);
    }
    else
    {
        check((MQTT_LINK_OK == result) && (1 == g_stub_init_calls),
              __LINE__);
        check((0 == status.last_error) && (status.b_initialised), __LINE__);
    }

    return ((0 == g_failures) ? 0 : 1);
}

int
main (int argc, char * * pp_argv)
{
    int exit_code = 0;

    memset(g_topic_max, 'a', MQTT_LINK_TOPIC_MAX);
    g_topic_max[MQTT_LINK_TOPIC_MAX] = '\0';
    memset(g_topic_over, 'a', MQTT_LINK_TOPIC_MAX + 1U);
    g_topic_over[MQTT_LINK_TOPIC_MAX + 1U] = '\0';
    memset(g_data_max, 'p', MQTT_LINK_PAYLOAD_MAX);

    if (argc > 1)
    {
        exit_code = run_config_mode(0 == strcmp(pp_argv[1], "bad"));
    }
    else
    {
        test_publish_rejections();
        test_publish_accepted();
        test_get_status();
        test_receive_command();
        test_init_order();
        test_init_second_call();
        test_init_queue_failure();
        test_staleness();
        test_poll_before_init();
        test_poll_after_init();
        test_connect_info();
        test_backoff();

        if (0 == g_failures)
        {
            say("RESULT: all passed\n");
        }
        else
        {
            __builtin_printf("RESULT: %d FAILED\n", g_failures);
            exit_code = 1;
        }
    }

    return (exit_code);
}

/*** end of file ***/
