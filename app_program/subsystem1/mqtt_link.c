/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_link.c
 *
 * @brief MQTT session driver for the NO_SYS lwIP owner task.
 *
 * @par
 * See mqtt_link.h for the threading rules and mqtt_queue.h for why the
 * kernel calls live in another module. This file includes lwIP headers and
 * must never include a kernel header.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0) -- see mqtt_link.h.
 *
 * @par Deviation from BARR-C:2018 rule 1.1.c
 * memory_barrier() uses the GCC builtin __atomic_thread_fence(), which is a
 * compiler extension. It is the only one in this module and it is confined
 * to that one function. On the Cortex-M0+ it compiles to a single dmb
 * instruction and needs no library. The port's own barrier,
 * mp_memory_barrir(), cannot be used here: it is declared by a kernel
 * header, which this file must not include, and on a single-core build it
 * does not exist at all. Proposed by Buddy 1, pending lecturer
 * confirmation.
 * TODO: Record the approver's name here, as rule 1.1 of the deviation
 * procedure requires.
 *
 * @par Port assert hook
 * NOTE: lwip_utk_assert() (lib/liblwip/sysdepend/pico_rp2040/sys_arch.c)
 * prints the message, then loops forever in the calling task -- it does
 * not disable interrupts or halt the core. The calling task is always
 * cyw43_utk_task here (see mqtt_link_poll()'s contract below), so an
 * lwIP assertion still stops the radio service for good; the limits
 * checked at compile time below are therefore not optional.
 */

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

#include "lwip/apps/mqtt.h"
#include "lwip/dns.h"
#include "lwip/err.h"
#include "lwip/ip4_addr.h"
#include "lwip/ip_addr.h"
#include "lwip/netif.h"
#include "lwip/sys.h"

#include "mqtt_config.h"
#include "mqtt_link.h"
#include "mqtt_queue.h"

#if (MQTT_LINK_CFG_BROKER_PORT < 1U) || (MQTT_LINK_CFG_BROKER_PORT > 65535U)
#error "mqtt_config.h: MQTT_LINK_CFG_BROKER_PORT must be 1 to 65535"
#endif

/* Limits on the configured strings. The 32-character prefix cap is derived
 * from lwIP's 256-byte output ring in the checks further down.
 */
#define PREFIX_MAX_LEN          32U
#define CLIENT_ID_MAX_LEN       32U
#define LOGIN_MAX_LEN           32U

/* Lengths of the configured strings, without the terminating NUL. sizeof is
 * a compile-time constant of a few dozen bytes, so converting it to
 * uint32_t cannot lose any bits.
 */
#define HOST_LEN        ((uint32_t)(sizeof(MQTT_LINK_CFG_BROKER_HOST) - 1U))
#define PREFIX_LEN      ((uint32_t)(sizeof(MQTT_LINK_CFG_TOPIC_PREFIX) - 1U))
#define CLIENT_ID_LEN   ((uint32_t)(sizeof(MQTT_LINK_CFG_CLIENT_ID) - 1U))
#define USER_LEN        ((uint32_t)(sizeof(MQTT_LINK_CFG_USERNAME) - 1U))
#define PASS_LEN        ((uint32_t)(sizeof(MQTT_LINK_CFG_PASSWORD) - 1U))
#define STATUS_LEN      ((uint32_t)(sizeof(MQTT_LINK_TOPIC_STATUS) - 1U))
#define ONLINE_LEN      ((uint32_t)(sizeof(MQTT_LINK_STATUS_ONLINE) - 1U))
#define OFFLINE_LEN     ((uint32_t)(sizeof(MQTT_LINK_STATUS_OFFLINE) - 1U))

/* Reconnect backoff. The first retry waits 1 s and every failed attempt
 * doubles the wait, up to a cap of 8 s: 1, 2, 4, 8, 8, and so on.
 * The cap keeps a car that lost WiFi briefly from sitting idle for long
 * once it is back, and keeps a dead broker from being hammered. A session
 * that stayed up for at least STABLE_SESSION_MSEC resets the wait to 1 s.
 */
#define BACKOFF_MIN_MSEC        1000U
#define BACKOFF_MAX_MSEC        8000U
#define STABLE_SESSION_MSEC     10000U

/* lwIP gives a connect attempt 100 s, which is far too long for a car, so
 * the poll step gives up after this much time.
 */
#define CONNECT_TIMEOUT_MSEC    5000U

/* MQTT keep-alive in seconds. lwIP drops a silent broker after 1.5 times
 * this, which is 7.5 s.
 */
#define KEEPALIVE_SECONDS       5U

/* Commands are subscribed at QoS 1 so that lwIP acknowledges each one: it
 * sends a PUBACK after the last fragment (mqtt_message_received() in lwIP's
 * mqtt.c). If that acknowledgement is lost the broker sends the command
 * again, so a command can arrive twice. The session is clean, so commands
 * sent while the car is offline are not kept.
 */
#define COMMAND_QOS             1U

/* A publish that is not acknowledged is sent again, up to this many tries
 * in all, before it is dropped and counted as failed. A lost connection
 * does not count as a try.
 */
#define PUBLISH_MAX_ATTEMPTS    3U

#define STATUS_READ_ATTEMPTS    8U

/* Suffix of each topic ID (mqtt_link_topic_t). None is
 * MQTT_LINK_TOPIC_STATUS or under "cmd/", so no task can publish there.
 */
#define TOPIC_TELEMETRY         "telemetry"
#define TOPIC_EVENT             "event"
#define TOPIC_ACK               "ack"

/* lwIP's value for the last part of an incoming message, as an unsigned
 * mask. The check further down makes sure it still matches lwIP.
 */
#define DATA_FLAG_LAST_MASK     1U

/* Size of a buffer that holds "<prefix>/<suffix>" and its NUL. */
#define FULL_TOPIC_BYTES        (PREFIX_MAX_LEN + 1U + MQTT_LINK_TOPIC_MAX \
                                 + 1U)

/* Worst-case sizes of the packets this driver builds, measured in lwIP's
 * output ring. A packet is its type byte, a length field of one byte below
 * 128 and two bytes from there, and then the remaining bytes. A packet that
 * fills the ring exactly cannot be told from an empty ring, so each one
 * must be strictly smaller than the ring.
 *
 * PUBLISH: topic length (2), the topic (prefix, a slash and a suffix), the
 * packet ID of a QoS 1 message (2) and the payload.
 */
#define PUBLISH_REMAINING_MAX   (2U + (PREFIX_MAX_LEN + 1U \
                                 + MQTT_LINK_TOPIC_MAX) + 2U \
                                 + MQTT_LINK_PAYLOAD_MAX)
#define PUBLISH_WIRE_MAX        (1U + ((PUBLISH_REMAINING_MAX < 128U) \
                                 ? 1U : 2U) + PUBLISH_REMAINING_MAX)

/* CONNECT: 10 fixed bytes, then the will topic ("<prefix>/status") and the
 * will message, the client ID, and the user name and password, each with a
 * 2-byte length.
 */
#define CONNECT_REMAINING_MAX   (10U + (2U + PREFIX_MAX_LEN + 1U \
                                 + STATUS_LEN) + (2U + OFFLINE_LEN) \
                                 + (2U + CLIENT_ID_MAX_LEN) \
                                 + (2U + LOGIN_MAX_LEN) \
                                 + (2U + LOGIN_MAX_LEN))
#define CONNECT_WIRE_MAX        (1U + ((CONNECT_REMAINING_MAX < 128U) \
                                 ? 1U : 2U) + CONNECT_REMAINING_MAX)

/* An incoming command topic must fit lwIP's header buffer, which also holds
 * the fixed header (at most 5 bytes), the topic length (2) and a packet ID
 * (2).
 */
#define INCOMING_HEADER_MAX     (5U + 2U + (PREFIX_MAX_LEN + 1U \
                                 + MQTT_LINK_TOPIC_MAX) + 2U)

/* Compile-time checks. Each typedef has a negative array size, and so stops
 * the build, when its condition is false.
 */
typedef uint8_t mqtt_link_check_host_t[(HOST_LEN >= 1U) ? 1 : -1];
typedef uint8_t mqtt_link_check_prefix_t[((PREFIX_LEN >= 1U) \
    && (PREFIX_LEN <= PREFIX_MAX_LEN)) ? 1 : -1];
typedef uint8_t mqtt_link_check_client_t[((CLIENT_ID_LEN >= 1U) \
    && (CLIENT_ID_LEN <= CLIENT_ID_MAX_LEN)) ? 1 : -1];
typedef uint8_t mqtt_link_check_login_t[((USER_LEN <= LOGIN_MAX_LEN) \
    && (PASS_LEN <= LOGIN_MAX_LEN)) ? 1 : -1];
typedef uint8_t mqtt_link_check_status_t[(STATUS_LEN <= MQTT_LINK_TOPIC_MAX) \
    ? 1 : -1];
typedef uint8_t mqtt_link_check_publish_t[(PUBLISH_WIRE_MAX \
    < MQTT_OUTPUT_RINGBUF_SIZE) ? 1 : -1];
typedef uint8_t mqtt_link_check_connect_t[(CONNECT_WIRE_MAX \
    < MQTT_OUTPUT_RINGBUF_SIZE) ? 1 : -1];
typedef uint8_t mqtt_link_check_incoming_t[(INCOMING_HEADER_MAX \
    <= MQTT_VAR_HEADER_BUFFER_LEN) ? 1 : -1];
typedef uint8_t mqtt_link_check_flag_t[(MQTT_DATA_FLAG_LAST \
    == DATA_FLAG_LAST_MASK) ? 1 : -1];
/* Every topic suffix fits MQTT_LINK_TOPIC_MAX, which sizes the topic buffer
 * and the publish packet check above.
 */
typedef uint8_t mqtt_link_check_suffix_t[ \
    (((sizeof(TOPIC_TELEMETRY) - 1U) <= MQTT_LINK_TOPIC_MAX) \
     && ((sizeof(TOPIC_EVENT) - 1U) <= MQTT_LINK_TOPIC_MAX) \
     && ((sizeof(TOPIC_ACK) - 1U) <= MQTT_LINK_TOPIC_MAX)) ? 1 : -1];
/* The caps come from msg_layout.h: every message the robot publishes must
 * fit the publish cap, and the command cap must be exactly one COMMAND
 * message (docs/S1_binary_frame.md, section 9.2).
 */
typedef uint8_t mqtt_link_check_caps_t[((MSG_TEL_BYTES \
    <= MQTT_LINK_PAYLOAD_MAX) && (MSG_HB_BYTES <= MQTT_LINK_PAYLOAD_MAX) \
    && (MSG_EVT_BYTES <= MQTT_LINK_PAYLOAD_MAX) \
    && (MSG_ACK_BYTES <= MQTT_LINK_PAYLOAD_MAX) \
    && (MQTT_LINK_COMMAND_MAX == MSG_CMD_BYTES)) ? 1 : -1];

/** What the one publish that lwIP may hold at a time is. */
typedef enum
{
    INFLIGHT_NONE = 0,
    INFLIGHT_BIRTH,
    INFLIGHT_MESSAGE
} mqtt_link_inflight_t;

/** Progress of the broker name lookup. */
typedef enum
{
    DNS_IDLE = 0,
    DNS_PENDING,
    DNS_OK,
    DNS_FAILED
} mqtt_link_dns_t;

/* Everything below is touched only by the radio service task, from
 * mqtt_link_poll() and from the lwIP callbacks that run inside it, with two
 * exceptions: g_published and g_version, which the seqlock protects.
 *
 * WARNING: start_link() (called from mqtt_link_init(), on whatever task
 * calls it -- not necessarily the radio service task) is the one other
 * writer of g_status and the one other caller of publish_status(), on
 * both its success and failure paths. Both finish every such write and
 * call strictly before mqtt_queue_init() sets mqtt_queue.c's gb_ready --
 * on the success path, because everything start_link() writes happens
 * before it even calls mqtt_queue_init(); on the failure path, because
 * gb_ready is never set at all. mqtt_link_poll() depends on this: it
 * must not call publish_status() until mqtt_queue_is_ready() is true,
 * or two tasks could write the seqlock at once.
 */
static mqtt_link_status_t       g_status = { MQTT_LINK_STATE_IDLE };
static mqtt_link_status_t       g_published = { MQTT_LINK_STATE_IDLE };
static volatile uint32_t        g_version = 0U;
static mqtt_client_t *          gp_client = NULL;
static ip_addr_t                g_broker_addr;
static mqtt_queue_publish_t     g_pending;
static mqtt_link_command_t      g_command;
static char                     g_status_topic[FULL_TOPIC_BYTES];
static char                     g_cmd_filter[FULL_TOPIC_BYTES];
static mqtt_link_inflight_t     g_inflight = INFLIGHT_NONE;
static mqtt_link_dns_t          g_dns = DNS_IDLE;
static bool                     gb_pending_valid = false;
static bool                     gb_sub_sent = false;
static bool                     gb_sub_failed = false;
static bool                     gb_birth_done = false;
static bool                     gb_command_drop = true;
static uint32_t                 g_pending_attempts = 0U;
static uint32_t                 g_step_start_msec = 0U;
static uint32_t                 g_session_start_msec = 0U;
static uint32_t                 g_backoff_msec = 0U;

/* Suffix of each topic ID, indexed by the ID. */
static const char * const       g_topic_suffix[] =
{
    [MQTT_LINK_TOPIC_TELEMETRY] = TOPIC_TELEMETRY,
    [MQTT_LINK_TOPIC_EVENT]     = TOPIC_EVENT,
    [MQTT_LINK_TOPIC_ACK]       = TOPIC_ACK
};

/* One suffix for each topic ID: an ID added without a suffix stops the
 * build here.
 */
typedef uint8_t mqtt_link_check_table_t[((sizeof(g_topic_suffix) \
    / sizeof(g_topic_suffix[0])) == MQTT_LINK_TOPIC_COUNT) ? 1 : -1];

static void memory_barrier(void);
static void publish_status(void);
static void build_topics(void);
static int32_t check_config(void);
static int32_t start_link(void);
static void run_state(struct netif * p_netif);
static void step_idle(struct netif * p_netif, uint32_t now_msec);
static void step_resolving(uint32_t now_msec);
static void step_connecting(struct netif * p_netif, uint32_t now_msec);
static void step_connected(struct netif * p_netif, uint32_t now_msec);
static void step_backoff(uint32_t now_msec);
static void step_subscribe(uint32_t now_msec);
static void step_publish(uint32_t now_msec);
static void resolve_broker(uint32_t now_msec);
static void start_connect(uint32_t now_msec);
static void send_birth(uint32_t now_msec);
static void send_message(uint32_t now_msec);
static void take_pending(void);
static void submit_pending(uint32_t now_msec);
static void build_topic(char * p_dest, const char * p_suffix);
static void enter_state(mqtt_link_state_t state, uint32_t now_msec);
static void enter_backoff(uint32_t now_msec);
static void abort_session(uint32_t now_msec);
static uint32_t next_backoff(bool b_stable);
static bool link_ready(const struct netif * p_netif);
static bool ensure_client(void);
static const char * login_or_null(const char * p_login);
static void finish_birth(err_t result);
static void finish_message(err_t result);
static bool topic_has_prefix(const char * p_topic);
static void append_payload(const u8_t * p_data, u16_t length);
static void finish_command(void);
static void dns_found_cb(const char * p_name, const ip_addr_t * p_addr,
                         void * p_arg);
static void connection_cb(mqtt_client_t * p_client, void * p_arg,
                          mqtt_connection_status_t status);
static void subscribe_done_cb(void * p_arg, err_t result);
static void publish_done_cb(void * p_arg, err_t result);
static void incoming_publish_cb(void * p_arg, const char * p_topic,
                                u32_t total_len);
static void incoming_data_cb(void * p_arg, const u8_t * p_data, u16_t length,
                             u8_t flags);

/*!
 * @brief Create the kernel objects the driver needs.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM when the configuration is
 *         unusable, or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_link_init (void)
{
    int32_t result = MQTT_LINK_OK;

    /* After a successful start the poll step is running, so a second call
     * must not touch anything.
     */
    if (!mqtt_queue_is_ready())
    {
        result = start_link();
    }

    return (result);
}

/*!
 * @brief Advance the session by one non-blocking step.
 *
 * @param[in] p_netif The station netif.
 */
void
mqtt_link_poll (struct netif * p_netif)
{
    /* Stamped unconditionally, even before mqtt_link_init() has succeeded
     * or p_netif is usable, so mqtt_link_get_status()'s b_stale always
     * reflects how long ago this function last actually ran -- see
     * MQTT_LINK_STALE_MSEC's comment in mqtt_link.h.
     */
    g_status.last_poll_msec = mqtt_queue_now_msec();

    /* Gated on mqtt_queue_is_ready(), not just to skip a not-yet-usable
     * queue: start_link() also calls publish_status(), and (see the
     * "Single writer" note above) always finishes before this flag is
     * set. Calling publish_status() here before the flag is set would
     * let two tasks write the seqlock at once, which it is not built to
     * survive.
     */
    if ((NULL != p_netif) && (mqtt_queue_is_ready()))
    {
        run_state(p_netif);
        publish_status();
    }
}

/*!
 * @brief Copy the driver status.
 *
 * Reads the status under a seqlock and tries up to STATUS_READ_ATTEMPTS
 * times. Every try can fail only if the radio service is in the middle of
 * an update, and that lasts a few microseconds unless this task preempted
 * the service while it was updating. Then no retry can succeed, which is
 * why the number of tries is bounded.
 *
 * @param[out] p_status Receives the snapshot.
 *
 * @return True if p_status holds a consistent snapshot. False if p_status
 *         is NULL, or if no try gave a consistent copy; the copy is then
 *         the last one read and may be torn, so it must not be acted on.
 *         publish_queue_full is filled in either way.
 */
bool
mqtt_link_get_status (mqtt_link_status_t * p_status)
{
    uint32_t attempt = 0U;
    uint32_t before = 0U;
    bool     b_steady = false;

    if (NULL != p_status)
    {
        for (attempt = 0U; (attempt < STATUS_READ_ATTEMPTS) && (!b_steady);
             attempt++)
        {
            /* The version is odd while an update is in progress, and it
             * changes if one happens during the copy.
             */
            before = g_version;
            memory_barrier();
            *p_status = g_published;
            memory_barrier();
            b_steady = ((0U == (before & 1U)) && (before == g_version));
        }

        p_status->publish_queue_full = mqtt_queue_full_count();

        /* Same reasoning as publish_queue_full just above: filled in
         * either way, even after a torn read, since a torn read only
         * happens while the poll service is actively updating the
         * snapshot -- which on its own proves polling is not stale.
         */
        p_status->b_stale = ((mqtt_queue_now_msec() - p_status->last_poll_msec)
                             >= MQTT_LINK_STALE_MSEC);
    }

    return (b_steady);
}

/*!
 * @brief Queue one message for publication.
 *
 * @param[in] topic       Topic ID.
 * @param[in] p_payload   Payload bytes.
 * @param[in] payload_len Payload size.
 * @param[in] qos         Quality of service.
 * @param[in] b_retain    True to ask the broker to retain the message.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_FULL or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_link_publish (mqtt_link_topic_t topic, const uint8_t * p_payload,
                   uint16_t payload_len, mqtt_link_qos_t qos, bool b_retain)
{
    int32_t              result = MQTT_LINK_OK;
    mqtt_queue_publish_t msg = { .topic_id = 0U, .payload_len = 0U,
                                 .qos = 0U, .b_retain = false,
                                 .payload = { 0U } };

    /* Whether the enum is signed or unsigned is implementation-defined.
     * The cast to uint32_t keeps a value of 0 or more as it is and turns a
     * negative value into one of at least 2^31, far above
     * MQTT_LINK_TOPIC_COUNT, so this one comparison refuses both.
     */
    if ((uint32_t)topic >= MQTT_LINK_TOPIC_COUNT)
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else if ((payload_len > MQTT_LINK_PAYLOAD_MAX)
             || ((NULL == p_payload) && (payload_len > 0U))
             || ((MQTT_LINK_QOS_0 != qos) && (MQTT_LINK_QOS_1 != qos)))
    {
        result = MQTT_LINK_ERR_PARAM;
    }
    else
    {
        /* The first check above refused every topic of
         * MQTT_LINK_TOPIC_COUNT (3) or more, so here it is 0 to 2 and the
         * cast to uint8_t cannot lose any bits.
         */
        msg.topic_id = (uint8_t)topic;

        if (payload_len > 0U)
        {
            memcpy(msg.payload, p_payload, payload_len);
        }

        msg.payload_len = payload_len;
        /* qos was already validated to be MQTT_LINK_QOS_0 or
         * MQTT_LINK_QOS_1 by the parameter checks earlier in this
         * function, so the cast to uint8_t cannot lose any bits.
         */
        msg.qos = (uint8_t)qos;
        msg.b_retain = b_retain;
        result = mqtt_queue_put_publish(&msg);
    }

    return (result);
}

/*!
 * @brief Take the next command received from the broker.
 *
 * @param[out] p_command   Receives the command.
 * @param[in]  timeout_msec Wait limit in milliseconds, or
 *                          MQTT_LINK_WAIT_FOREVER.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_TIMEOUT or MQTT_LINK_ERR_KERNEL.
 */
int32_t
mqtt_link_receive_command (mqtt_link_command_t * p_command,
                           uint32_t timeout_msec)
{
    int32_t result = MQTT_LINK_ERR_PARAM;

    if (NULL != p_command)
    {
        result = mqtt_queue_get_command(p_command, timeout_msec);
    }

    return (result);
}

/*!
 * @brief Order this core's writes before the other core's reads.
 *
 * NOTE: This is the one compiler extension in the module; see the
 * deviation in the file header.
 */
static void
memory_barrier (void)
{
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
}

/*!
 * @brief Copy the working status to the copy that readers see.
 *
 * The version is odd while the copy is in progress. Only the radio service
 * task calls this, so there is one writer.
 */
static void
publish_status (void)
{
    g_version++;
    memory_barrier();
    g_published = g_status;
    memory_barrier();
    g_version++;
}

/*!
 * @brief Build the status topic and the command filter from the prefix.
 */
static void
build_topics (void)
{
    memcpy(g_status_topic, MQTT_LINK_CFG_TOPIC_PREFIX, PREFIX_LEN);
    g_status_topic[PREFIX_LEN] = '/';
    memcpy(&g_status_topic[PREFIX_LEN + 1U], MQTT_LINK_TOPIC_STATUS,
           STATUS_LEN + 1U);

    memcpy(g_cmd_filter, MQTT_LINK_CFG_TOPIC_PREFIX, PREFIX_LEN);
    memcpy(&g_cmd_filter[PREFIX_LEN], "/cmd/#", sizeof("/cmd/#"));
}

/*!
 * @brief Check the parts of the configuration the compiler cannot.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_PARAM.
 */
static int32_t
check_config (void)
{
    int32_t result = MQTT_LINK_OK;

    if (('/' == MQTT_LINK_CFG_TOPIC_PREFIX[0])
        || ('/' == MQTT_LINK_CFG_TOPIC_PREFIX[PREFIX_LEN - 1U])
        || (NULL != strpbrk(MQTT_LINK_CFG_TOPIC_PREFIX, "#+")))
    {
        result = MQTT_LINK_ERR_PARAM;
    }

    return (result);
}

/*!
 * @brief Set everything up, then make the queue ready.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM when the configuration is
 *         unusable, or MQTT_LINK_ERR_KERNEL.
 */
static int32_t
start_link (void)
{
    int32_t result = check_config();

    if (MQTT_LINK_OK == result)
    {
        /* Everything the poll step reads is set up first, because
         * mqtt_queue_init() makes the poll step start working the moment it
         * succeeds. After a success this function must not touch the
         * status.
         */
        build_topics();
        /* MQTT_LINK_CFG_BROKER_PORT is already range-checked (1 to
         * 65535) by the #if near the top of this file, so it fits in
         * uint16_t.
         */
        g_status.broker_port = (uint16_t)MQTT_LINK_CFG_BROKER_PORT;
        g_status.b_initialised = true;
        publish_status();

        result = mqtt_queue_init();
    }

    if (MQTT_LINK_OK != result)
    {
        /* The poll step never started, because it waits for the queue, so
         * this is still safe. Leave the reason where the status shows it.
         */
        g_status.b_initialised = false;
        g_status.last_error = result;
        publish_status();
    }

    return (result);
}

/*!
 * @brief Run the step for the current state.
 *
 * @param[in] p_netif The station netif.
 */
static void
run_state (struct netif * p_netif)
{
    const uint32_t now_msec = sys_now();

    switch (g_status.state)
    {
        case MQTT_LINK_STATE_IDLE:
            step_idle(p_netif, now_msec);
        break;

        case MQTT_LINK_STATE_RESOLVING:
            step_resolving(now_msec);
        break;

        case MQTT_LINK_STATE_CONNECTING:
            step_connecting(p_netif, now_msec);
        break;

        case MQTT_LINK_STATE_CONNECTED:
            step_connected(p_netif, now_msec);
        break;

        case MQTT_LINK_STATE_BACKOFF:
            step_backoff(now_msec);
        break;

        default:
            /* An unknown state can only be a bug; start over safely. */
            enter_backoff(now_msec);
        break;
    }
}

/*!
 * @brief IDLE: wait for the link and an address, then find the broker.
 */
static void
step_idle (struct netif * p_netif, uint32_t now_msec)
{
    if (link_ready(p_netif))
    {
        if (ensure_client())
        {
            resolve_broker(now_msec);
        }
        else
        {
            g_status.last_error = ERR_MEM;
            enter_backoff(now_msec);
        }
    }
}

/*!
 * @brief RESOLVING: wait for the name lookup to finish.
 *
 * lwIP always calls dns_found_cb(), with an address or without one, so
 * there is no time limit here.
 */
static void
step_resolving (uint32_t now_msec)
{
    if (DNS_OK == g_dns)
    {
        start_connect(now_msec);
    }
    else if (DNS_FAILED == g_dns)
    {
        /* lwIP does not say why a lookup failed, so call it a time-out. */
        g_status.last_error = ERR_TIMEOUT;
        enter_backoff(now_msec);
    }
    else if (DNS_PENDING != g_dns)
    {
        /* No lookup is running, which can only be a bug. */
        enter_backoff(now_msec);
    }
    else
    {
        /* Still waiting for the answer. */
    }
}

/*!
 * @brief CONNECTING: wait for the broker, within a time limit.
 */
static void
step_connecting (struct netif * p_netif, uint32_t now_msec)
{
    if (!link_ready(p_netif))
    {
        abort_session(now_msec);
    }
    else if ((now_msec - g_step_start_msec) >= CONNECT_TIMEOUT_MSEC)
    {
        g_status.last_error = ERR_TIMEOUT;
        abort_session(now_msec);
    }
    else
    {
        /* Still waiting; connection_cb() ends this state. */
    }
}

/*!
 * @brief CONNECTED: subscribe, announce, then send queued messages.
 */
static void
step_connected (struct netif * p_netif, uint32_t now_msec)
{
    g_status.connected_msec = now_msec - g_session_start_msec;

    if ((!link_ready(p_netif)) || (gb_sub_failed))
    {
        abort_session(now_msec);
    }
    else
    {
        step_subscribe(now_msec);
        step_publish(now_msec);
    }
}

/*!
 * @brief BACKOFF: wait out the retry delay, then try again from IDLE.
 */
static void
step_backoff (uint32_t now_msec)
{
    if ((now_msec - g_step_start_msec) >= g_backoff_msec)
    {
        enter_state(MQTT_LINK_STATE_IDLE, now_msec);
    }
}

/*!
 * @brief Send the one SUBSCRIBE for the command filter.
 */
static void
step_subscribe (uint32_t now_msec)
{
    err_t result = ERR_OK;

    if (!gb_sub_sent)
    {
        result = mqtt_subscribe(gp_client, g_cmd_filter, COMMAND_QOS,
                                subscribe_done_cb, NULL);

        if (ERR_OK == result)
        {
            gb_sub_sent = true;
        }
        else if (ERR_MEM == result)
        {
            /* The output ring is busy. Try again on the next pass. */
        }
        else
        {
            g_status.last_error = result;
            abort_session(now_msec);
        }
    }
}

/*!
 * @brief Send at most one publish, and only when none is in flight.
 *
 * The birth message comes first, then the queued messages. A message is
 * taken from the queue only here, which is only reached when the session is
 * up and nothing is in flight, so a disconnect cannot discard the queue.
 */
static void
step_publish (uint32_t now_msec)
{
    if (INFLIGHT_NONE == g_inflight)
    {
        if (!gb_birth_done)
        {
            send_birth(now_msec);
        }
        else
        {
            send_message(now_msec);
        }
    }
}

/*!
 * @brief Look up the broker: use a dotted address as it is, else ask DNS.
 */
static void
resolve_broker (uint32_t now_msec)
{
    ip4_addr_t literal;
    err_t      result = ERR_OK;

    if (0 != ip4addr_aton(MQTT_LINK_CFG_BROKER_HOST, &literal))
    {
        /* A dotted address needs no lookup. */
        ip_addr_copy_from_ip4(g_broker_addr, literal);
        start_connect(now_msec);
    }
    else
    {
        g_dns = DNS_PENDING;
        result = dns_gethostbyname(MQTT_LINK_CFG_BROKER_HOST, &g_broker_addr,
                                   dns_found_cb, NULL);

        if (ERR_OK == result)
        {
            /* The answer was already known. */
            g_dns = DNS_IDLE;
            start_connect(now_msec);
        }
        else if (ERR_INPROGRESS == result)
        {
            enter_state(MQTT_LINK_STATE_RESOLVING, now_msec);
        }
        else
        {
            g_dns = DNS_IDLE;
            g_status.last_error = result;
            enter_backoff(now_msec);
        }
    }
}

/*!
 * @brief Open the TCP connection and send CONNECT with the last will.
 */
static void
start_connect (uint32_t now_msec)
{
    struct mqtt_connect_client_info_t info = { 0 };
    err_t                             result = ERR_OK;

    info.client_id = MQTT_LINK_CFG_CLIENT_ID;
    info.client_user = login_or_null(MQTT_LINK_CFG_USERNAME);
    info.client_pass = login_or_null(MQTT_LINK_CFG_PASSWORD);
    info.keep_alive = KEEPALIVE_SECONDS;
    info.will_topic = g_status_topic;
    info.will_msg = MQTT_LINK_STATUS_OFFLINE;
    info.will_qos = 1U;
    info.will_retain = 1U;

    /* The port is range-checked at compile time, so it fits 16 bits. */
    result = mqtt_client_connect(gp_client, &g_broker_addr,
                                 (u16_t)MQTT_LINK_CFG_BROKER_PORT,
                                 connection_cb, NULL, &info);

    if (ERR_OK == result)
    {
        gb_sub_sent = false;
        gb_sub_failed = false;
        gb_birth_done = false;
        g_inflight = INFLIGHT_NONE;
        g_dns = DNS_IDLE;
        enter_state(MQTT_LINK_STATE_CONNECTING, now_msec);
    }
    else
    {
        g_status.last_error = result;
        enter_backoff(now_msec);
    }
}

/*!
 * @brief Publish the retained "online" message.
 */
static void
send_birth (uint32_t now_msec)
{
    /* ONLINE_LEN is sizeof(MQTT_LINK_STATUS_ONLINE) - 1: the length of
     * the string literal "online", 6 bytes, fixed at build time.
     */
    const err_t result = mqtt_publish(gp_client, g_status_topic,
                                      MQTT_LINK_STATUS_ONLINE,
                                      (u16_t)ONLINE_LEN, 1U, 1U,
                                      publish_done_cb, NULL);

    if (ERR_OK == result)
    {
        g_inflight = INFLIGHT_BIRTH;
    }
    else if (ERR_MEM == result)
    {
        /* The output ring or the request list is busy. Try again later. */
    }
    else
    {
        g_status.last_error = result;
        abort_session(now_msec);
    }
}

/*!
 * @brief Send the pending message, taking one from the queue if there is
 *        none.
 */
static void
send_message (uint32_t now_msec)
{
    if (!gb_pending_valid)
    {
        take_pending();
    }

    if (gb_pending_valid)
    {
        submit_pending(now_msec);
    }
}

/*!
 * @brief Move the oldest queued message into the pending slot.
 */
static void
take_pending (void)
{
    const int32_t result = mqtt_queue_get_publish(&g_pending);

    if ((MQTT_LINK_OK == result)
        && (g_pending.topic_id < MQTT_LINK_TOPIC_COUNT))
    {
        gb_pending_valid = true;
        g_pending_attempts = 0U;
    }
    else if (MQTT_LINK_ERR_TIMEOUT == result)
    {
        /* The queue is empty. */
    }
    else
    {
        /* The record is lost, and the read already removed it. A topic ID
         * out of range, which mqtt_link_publish() never queues, is lost
         * the same way rather than used as a table index.
         */
        g_status.publish_failed++;
    }
}

/*!
 * @brief Hand the pending message to lwIP.
 *
 * The message stays in the pending slot until lwIP reports it complete, so
 * a lost connection never loses it.
 */
static void
submit_pending (uint32_t now_msec)
{
    char  topic[FULL_TOPIC_BYTES];
    err_t result = ERR_OK;

    /* take_pending() lets only a topic ID below MQTT_LINK_TOPIC_COUNT into
     * g_pending, so the index is inside g_topic_suffix.
     */
    build_topic(topic, g_topic_suffix[g_pending.topic_id]);

    /* The retain flag is 0 or 1, so it fits lwIP's 8-bit argument. */
    result = mqtt_publish(gp_client, topic, g_pending.payload,
                          g_pending.payload_len, g_pending.qos,
                          (u8_t)(g_pending.b_retain ? 1U : 0U),
                          publish_done_cb, NULL);

    if (ERR_OK == result)
    {
        g_inflight = INFLIGHT_MESSAGE;
    }
    else if (ERR_MEM == result)
    {
        /* The output ring or the request list is busy. Try again later. */
    }
    else if (ERR_CONN == result)
    {
        /* The connection is gone. The message stays pending. */
        g_status.last_error = result;
        abort_session(now_msec);
    }
    else
    {
        /* lwIP will never accept this message, so drop it. */
        g_status.last_error = result;
        g_status.publish_failed++;
        gb_pending_valid = false;
    }
}

/*!
 * @brief Build "<prefix>/<suffix>" in a buffer of FULL_TOPIC_BYTES.
 */
static void
build_topic (char * p_dest, const char * p_suffix)
{
    memcpy(p_dest, MQTT_LINK_CFG_TOPIC_PREFIX, PREFIX_LEN);
    p_dest[PREFIX_LEN] = '/';
    strcpy(&p_dest[PREFIX_LEN + 1U], p_suffix);
}

/*!
 * @brief Move to a new state and note when it began.
 */
static void
enter_state (mqtt_link_state_t state, uint32_t now_msec)
{
    g_status.state = state;
    g_step_start_msec = now_msec;
}

/*!
 * @brief Give up the session and wait before the next attempt.
 *
 * The pending message is kept, and so is everything still in the queue.
 * Only what lwIP held in flight is forgotten: lwIP discards its requests
 * without calling them back when a connection closes.
 */
static void
enter_backoff (uint32_t now_msec)
{
    const bool b_stable = (g_status.b_connected
                           && ((now_msec - g_session_start_msec)
                               >= STABLE_SESSION_MSEC));

    g_status.b_connected = false;
    g_status.b_subscribed = false;
    g_status.disconnect_count++;
    g_inflight = INFLIGHT_NONE;
    g_dns = DNS_IDLE;

    g_backoff_msec = next_backoff(b_stable);
    g_status.backoff_msec = g_backoff_msec;
    enter_state(MQTT_LINK_STATE_BACKOFF, now_msec);
}

/*!
 * @brief Close the connection ourselves, then back off.
 *
 * mqtt_disconnect() does not call connection_cb(), so the state change is
 * made here.
 */
static void
abort_session (uint32_t now_msec)
{
    mqtt_disconnect(gp_client);
    enter_backoff(now_msec);
}

/*!
 * @brief Work out the next retry delay.
 *
 * @param[in] b_stable True if the session that just ended stayed up long
 *                     enough to count as working.
 *
 * @return The delay in milliseconds, from BACKOFF_MIN_MSEC to
 *         BACKOFF_MAX_MSEC.
 */
static uint32_t
next_backoff (bool b_stable)
{
    uint32_t delay_msec = BACKOFF_MIN_MSEC;

    if ((!b_stable) && (g_backoff_msec >= BACKOFF_MIN_MSEC))
    {
        delay_msec = g_backoff_msec * 2U;

        if (delay_msec > BACKOFF_MAX_MSEC)
        {
            delay_msec = BACKOFF_MAX_MSEC;
        }
    }

    return (delay_msec);
}

/*!
 * @brief Report whether the netif is up with a link and an address.
 */
static bool
link_ready (const struct netif * p_netif)
{
    return ((0U != netif_is_up(p_netif))
            && (0U != netif_is_link_up(p_netif))
            && (!ip4_addr_isany_val(*netif_ip4_addr(p_netif))));
}

/*!
 * @brief Create the lwIP client the first time it is needed.
 *
 * @return True if a client exists.
 */
static bool
ensure_client (void)
{
    if (NULL == gp_client)
    {
        gp_client = mqtt_client_new();

        if (NULL != gp_client)
        {
            mqtt_set_inpub_callback(gp_client, incoming_publish_cb,
                                    incoming_data_cb, NULL);
        }
    }

    return (NULL != gp_client);
}

/*!
 * @brief Return NULL for an empty login, which lwIP would reject.
 */
static const char *
login_or_null (const char * p_login)
{
    return (('\0' == p_login[0]) ? NULL : p_login);
}

/*!
 * @brief Note the result of the birth publish.
 */
static void
finish_birth (err_t result)
{
    if (ERR_OK == result)
    {
        gb_birth_done = true;
    }
    else
    {
        /* Not acknowledged. It is sent again on the next pass. */
        g_status.last_error = result;
    }
}

/*!
 * @brief Note the result of a queued message's publish.
 */
static void
finish_message (err_t result)
{
    if (ERR_OK == result)
    {
        g_status.publish_sent++;
        gb_pending_valid = false;
    }
    else
    {
        g_status.last_error = result;
        g_pending_attempts++;

        if (g_pending_attempts >= PUBLISH_MAX_ATTEMPTS)
        {
            g_status.publish_failed++;
            gb_pending_valid = false;
        }
    }
}

/*!
 * @brief Report whether an incoming topic starts with "<prefix>/".
 */
static bool
topic_has_prefix (const char * p_topic)
{
    return ((0 == strncmp(p_topic, MQTT_LINK_CFG_TOPIC_PREFIX, PREFIX_LEN))
            && ('/' == p_topic[PREFIX_LEN]));
}

/*!
 * @brief Add one fragment of an incoming payload to the command.
 */
static void
append_payload (const u8_t * p_data, u16_t length)
{
    /* Both values are 16 bits, so the sum cannot overflow 32 bits. */
    const uint32_t total = (uint32_t)g_command.payload_len
                           + (uint32_t)length;

    if (total > MQTT_LINK_COMMAND_MAX)
    {
        gb_command_drop = true;
    }
    else if (length > 0U)
    {
        memcpy(&g_command.payload[g_command.payload_len], p_data, length);
        /* total was already checked against MQTT_LINK_COMMAND_MAX by
         * the guard just above, so it fits in uint16_t here.
         */
        g_command.payload_len = (uint16_t)total;
    }
    else
    {
        /* An empty fragment adds nothing. */
    }
}

/*!
 * @brief Hand a complete command to the queue, or count it as dropped.
 */
static void
finish_command (void)
{
    if (gb_command_drop)
    {
        g_status.command_dropped++;
    }
    else if (MQTT_LINK_OK == mqtt_queue_put_command(&g_command))
    {
        g_status.command_received++;
    }
    else
    {
        g_status.command_dropped++;
    }

    gb_command_drop = true;
    g_command.payload_len = 0U;
}

/*!
 * @brief lwIP calls this when the broker name lookup ends.
 */
static void
dns_found_cb (const char * p_name, const ip_addr_t * p_addr, void * p_arg)
{
    /* The name and the argument are not needed. */
    (void)p_name;
    (void)p_arg;

    if (NULL != p_addr)
    {
        ip_addr_copy(g_broker_addr, *p_addr);
        g_dns = DNS_OK;
    }
    else
    {
        g_dns = DNS_FAILED;
    }
}

/*!
 * @brief lwIP calls this when CONNECT is answered, or the connection ends.
 *
 * It is not called after mqtt_disconnect(), which abort_session() covers.
 */
static void
connection_cb (mqtt_client_t * p_client, void * p_arg,
               mqtt_connection_status_t status)
{
    const uint32_t now_msec = sys_now();

    /* The client and the argument are not needed. */
    (void)p_client;
    (void)p_arg;

    if (MQTT_CONNECT_ACCEPTED == status)
    {
        g_status.b_connected = true;
        g_status.connect_count++;
        g_session_start_msec = now_msec;
        enter_state(MQTT_LINK_STATE_CONNECTED, now_msec);
    }
    else if ((MQTT_LINK_STATE_CONNECTING == g_status.state)
             || (MQTT_LINK_STATE_CONNECTED == g_status.state))
    {
        /* The status is a small enum value, so it fits int32_t. */
        g_status.last_error = (int32_t)status;
        enter_backoff(now_msec);
    }
    else
    {
        /* Not in a state that expects this call, so ignore it. */
    }
}

/*!
 * @brief lwIP calls this when the SUBSCRIBE is answered or times out.
 */
static void
subscribe_done_cb (void * p_arg, err_t result)
{
    /* The argument is not needed. */
    (void)p_arg;

    if (ERR_OK == result)
    {
        g_status.b_subscribed = true;
    }
    else
    {
        /* The poll step closes the session; it is not safe to do that from
         * inside lwIP's receive path.
         */
        g_status.last_error = result;
        gb_sub_failed = true;
    }
}

/*!
 * @brief lwIP calls this when a publish is acknowledged, or times out.
 */
static void
publish_done_cb (void * p_arg, err_t result)
{
    const mqtt_link_inflight_t kind = g_inflight;

    /* The argument is not needed. */
    (void)p_arg;

    g_inflight = INFLIGHT_NONE;

    if (INFLIGHT_BIRTH == kind)
    {
        finish_birth(result);
    }
    else if (INFLIGHT_MESSAGE == kind)
    {
        finish_message(result);
    }
    else
    {
        /* Nothing was in flight, so there is nothing to finish. */
    }
}

/*!
 * @brief lwIP calls this when an incoming message starts.
 *
 * The command is accepted only if its topic is under the prefix and the
 * suffix and the payload both fit.
 */
static void
incoming_publish_cb (void * p_arg, const char * p_topic, u32_t total_len)
{
    const char * p_suffix = NULL;

    /* The argument is not needed. */
    (void)p_arg;

    gb_command_drop = true;
    g_command.payload_len = 0U;

    if (topic_has_prefix(p_topic))
    {
        p_suffix = &p_topic[PREFIX_LEN + 1U];

        if ((strlen(p_suffix) <= MQTT_LINK_TOPIC_MAX)
            && (total_len <= MQTT_LINK_COMMAND_MAX))
        {
            gb_command_drop = false;
        }
    }
}

/*!
 * @brief lwIP calls this for each fragment of an incoming payload.
 */
static void
incoming_data_cb (void * p_arg, const u8_t * p_data, u16_t length,
                  u8_t flags)
{
    /* The argument is not needed. */
    (void)p_arg;

    if (!gb_command_drop)
    {
        append_payload(p_data, length);
    }

    /* The flags byte is widened to unsigned 32 bits so that the mask is not
     * applied to a signed value.
     */
    if (0U != ((uint32_t)flags & DATA_FLAG_LAST_MASK))
    {
        finish_command();
    }
}

#endif /* TM_MQTT */

/*** end of file ***/
