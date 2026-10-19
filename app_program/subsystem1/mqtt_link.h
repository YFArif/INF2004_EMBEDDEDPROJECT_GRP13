/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_link.h
 *
 * @brief MQTT session driver for the NO_SYS lwIP owner task.
 *
 * @par Threading
 * Application tasks never touch lwIP. They hand telemetry to the driver
 * through a kernel message buffer (mqtt_link_publish) and receive
 * commands from a second one (mqtt_link_receive_command). The driver
 * itself runs from mqtt_link_poll(), which the processor-1 radio poll
 * service calls once per pass. That function is the only code in this
 * module that calls lwIP, and it never blocks.
 *
 * @par Build gate
 * NOTE: The whole body of this header (and of mqtt_link.c, mqtt_queue.h
 * and mqtt_queue.c) is gated on TM_MQTT, which build_make/pico_rp2040.mk
 * defines only for MQTT=1 builds (which also require NET=lwip). The
 * default here is 0 so that every other profile, where these files still
 * get compiled -- every source file directly under app_program, and
 * every source file one folder below it, is globbed automatically (see
 * build_make/mtkernel_3/app_program/subdir.mk) -- sees an empty header
 * and an empty translation unit, not a dangling lwIP/kernel dependency.
 *
 * @par Start-up
 * subsystem1_init() (app_program/subsystem1/subsystem1.c) calls
 * mqtt_link_init() exactly once. app_main.c's usermain() runs every
 * subsystemN_init() before any subsystemN_start(), so this still
 * happens before any application task is created; the radio service
 * task already exists and may call mqtt_link_poll() earlier, which
 * does nothing until init succeeds. app_main.c itself never calls
 * mqtt_link_init(). Until init has succeeded, mqtt_link_poll() does
 * nothing and the application-facing functions return
 * MQTT_LINK_ERR_NOT_READY.
 *
 * @par Commands
 * Exactly one task, the application dispatcher, calls
 * mqtt_link_receive_command() and routes each command to the subsystem
 * that owns it. A second caller is not supported: the commands would be
 * split between the two callers.
 *
 * @par Command topic length
 * A command arrives on "<prefix>/cmd/...". The topic delivered to the
 * application is the part after the prefix, including "cmd/", and it must
 * fit MQTT_LINK_TOPIC_MAX characters; a longer one is dropped and counted
 * in command_dropped. A command name after "cmd/" can therefore be at most
 * MQTT_LINK_TOPIC_MAX minus 4 characters. lwIP's incoming header buffer
 * holds only 128 bytes, so a topic longer than roughly 120 characters
 * cannot be held at all: lwIP closes the connection, and the driver
 * reconnects with backoff. If such a message was published retained, the
 * broker redelivers it after every reconnect, and the loop does not end
 * until the retained message is cleared. Configure the broker so that only
 * trusted clients can publish under "<prefix>/cmd/".
 *
 * @par Command delivery
 * Commands are subscribed at QoS 1, which lwIP acknowledges, so the broker
 * sends a command again if the acknowledgement is lost and a command can
 * arrive twice. The dispatcher must tolerate a repeated command. The
 * session is clean, so commands sent while the car is offline are not kept,
 * and a command that finds the command queue full is dropped, counted in
 * command_dropped and not sent again.
 *
 * @par Availability
 * On connect the driver registers a retained last will on the status topic
 * ("<prefix>/status", payload "offline"). After each successful connect it
 * publishes a retained "online" on the same topic. A listener therefore
 * sees "online" while the car is connected and "offline" after the broker
 * notices it has gone. The driver owns this topic; mqtt_link_publish()
 * rejects it.
 *
 * @par Configuration
 * Broker host, port, client ID and topic prefix come from
 * app_program/subsystem1/mqtt_config.h (see
 * app_program/subsystem1/mqtt_config.example.h). The topic prefix is
 * limited to 32 characters, which mqtt_link.c checks at compile time, so
 * that every packet fits lwIP's 256-byte output ring.
 */

#ifndef MQTT_LINK_H
#define MQTT_LINK_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "msg_layout.h"

/* Largest topic suffix, in characters, excluding the terminating NUL.
 * A suffix is the part of a topic after the configured prefix, for
 * example "telemetry/state".
 * NOTE: Provisional; it could shrink to fit the real suffixes
 * (docs/S1_binary_frame.md, section 9.2).
 */
#define MQTT_LINK_TOPIC_MAX         48U

/* Largest payload an application may publish, in bytes: the largest
 * packed message in msg_layout.h (docs/S1_binary_frame.md, section 9.2).
 * The topic prefix cap, MQTT_LINK_TOPIC_MAX and this value together must
 * keep the largest publish packet inside lwIP's 256-byte output ring;
 * mqtt_link.c checks that, and that every message fits, at compile time.
 */
#define MQTT_LINK_PAYLOAD_MAX       MSG_MAX_BYTES

/* Largest command payload delivered to an application, in bytes: one
 * packed COMMAND message (msg_layout.h). A longer incoming command is
 * dropped and counted in command_dropped; mqtt_link.c checks this value
 * against MSG_CMD_BYTES at compile time.
 */
#define MQTT_LINK_COMMAND_MAX       MSG_CMD_BYTES

/* Number of messages the publish queue holds. When the queue is full a new
 * message is not queued: mqtt_link_publish() returns
 * MQTT_LINK_ERR_FULL and the queued messages are kept. The depth also
 * bounds how stale a backlog can be: after a reconnect at most this many
 * messages, the oldest still queued, are sent first.
 */
#ifndef MQTT_LINK_QUEUE_DEPTH
#define MQTT_LINK_QUEUE_DEPTH       8U
#endif

/* How old mqtt_link_status_t.last_poll_msec may be before
 * mqtt_link_get_status() reports b_stale. The poll loop normally calls
 * mqtt_link_poll() every few milliseconds while the link is up, so this
 * is generous margin against ordinary scheduling jitter, while staying
 * far shorter than how long a dead session takes to detect by other
 * means (keepalive, reconnect backoff) -- a consumer relying on b_stale
 * should learn of an outage well before those would otherwise reveal it.
 */
#ifndef MQTT_LINK_STALE_MSEC
#define MQTT_LINK_STALE_MSEC        1000U
#endif

/* Status topic suffix, and the two payloads published on it. The full topic
 * is "<prefix>/status". Owned by the driver; see the Availability note.
 */
#define MQTT_LINK_TOPIC_STATUS      "status"
#define MQTT_LINK_STATUS_ONLINE     "online"
#define MQTT_LINK_STATUS_OFFLINE    "offline"

/* Return codes for the application-facing functions. Zero is success. */
#define MQTT_LINK_OK                0
#define MQTT_LINK_ERR_PARAM         (-1)   /* Bad argument.            */
#define MQTT_LINK_ERR_NOT_READY     (-2)   /* init() has not succeeded. */
#define MQTT_LINK_ERR_FULL          (-3)   /* Publish queue is full.   */
#define MQTT_LINK_ERR_TIMEOUT       (-4)   /* No command arrived.      */
#define MQTT_LINK_ERR_KERNEL        (-5)   /* Kernel call failed.      */

/* Pass as the timeout of mqtt_link_receive_command() to wait forever. */
#define MQTT_LINK_WAIT_FOREVER      UINT32_MAX

struct netif;

/** Connection state of the driver. */
typedef enum
{
    MQTT_LINK_STATE_IDLE = 0,       /* Waiting for link and address.  */
    MQTT_LINK_STATE_RESOLVING,      /* Looking up the broker name.    */
    MQTT_LINK_STATE_CONNECTING,     /* TCP and MQTT handshake.        */
    MQTT_LINK_STATE_CONNECTED,      /* Session is up.                 */
    MQTT_LINK_STATE_BACKOFF         /* Waiting before the next try.   */
} mqtt_link_state_t;

/** Quality of service for a published message. */
typedef enum
{
    MQTT_LINK_QOS_0 = 0,            /* At most once.                  */
    MQTT_LINK_QOS_1 = 1             /* At least once.                 */
} mqtt_link_qos_t;

/** One command received from the broker. */
typedef struct
{
    char        topic[MQTT_LINK_TOPIC_MAX + 1U];    /* Suffix, NUL end. */
    uint16_t    payload_len;
    uint8_t     payload[MQTT_LINK_COMMAND_MAX];
} mqtt_link_command_t;

/** Snapshot of the driver. Read it with mqtt_link_get_status().
 *
 * NOTE: Every field except publish_queue_full is written by the poll
 * service and read through a seqlock. publish_queue_full is counted by the
 * publishing tasks, which may run on either core, so it is a separate word
 * that is incremented atomically and copied alongside the seqlock snapshot.
 *
 * NOTE: b_stale is computed by mqtt_link_get_status() itself, from
 * last_poll_msec and MQTT_LINK_STALE_MSEC, using the same clock
 * mqtt_link_poll() stamped it with (mqtt_queue_now_msec()). Callers never
 * need their own notion of "now" or any time base: true means
 * mqtt_link_poll() has not run recently enough to trust the rest of this
 * snapshot -- typically a WiFi outage, since the poll loop only reaches
 * mqtt_link_poll() while the link is up (see mqtt_link_poll()'s own
 * comment). The internal state machine itself is not stale -- lwIP's own
 * timers keep running and can still correct it -- only what is visible
 * through this snapshot can lag.
 */
typedef struct
{
    mqtt_link_state_t   state;
    int32_t                 last_error;         /* Last error or status.    */
    uint32_t                connect_count;      /* Sessions established.    */
    uint32_t                disconnect_count;   /* Sessions lost or failed. */
    uint32_t                backoff_msec;       /* Current retry delay.     */
    uint32_t                connected_msec;     /* Age of this session.     */
    uint32_t                publish_sent;       /* Completed publishes.     */
    uint32_t                publish_failed;     /* Failed publishes.        */
    uint32_t                publish_queue_full; /* Dropped, queue was full. */
    uint32_t                command_received;   /* Delivered to the app.    */
    uint32_t                command_dropped;    /* Oversized or queue full. */
    uint32_t                last_poll_msec;     /* Clock at the last poll.  */
    uint8_t                 broker_addr[4];     /* Resolved address.        */
    uint16_t                broker_port;
    bool                    b_initialised;
    bool                    b_connected;
    bool                    b_subscribed;
    bool                    b_stale;            /* Not polled recently.     */
} mqtt_link_status_t;

/**
 * @brief Create the kernel objects the driver needs.
 *
 * Call once from subsystem1_init(), before creating any other task. It
 * makes no lwIP calls. A later call after success does nothing and
 * returns success.
 *
 * If it fails, the poll step never starts and the other functions return
 * MQTT_LINK_ERR_NOT_READY. The failure code is also left in last_error, so
 * mqtt_link_get_status() shows why: MQTT_LINK_ERR_PARAM for an unusable
 * configuration, or MQTT_LINK_ERR_KERNEL when a message buffer could not be
 * created, which usually means the kernel has no free message-buffer ID.
 *
 * @return MQTT_LINK_OK, or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_link_init(void);

/**
 * @brief Advance the session by one non-blocking step.
 *
 * Call from the processor-1 radio poll service only, once per pass. It is
 * safe to call before the netif is up, before DNS is configured, or before
 * mqtt_link_init() has succeeded -- every state checks its own real
 * preconditions (link_ready(), the DNS lookup's own result, the queue's
 * ready flag) rather than trusting the caller, and backs off cleanly on
 * any of them. In this port, the caller normally gates on CYW43_LINK_UP,
 * which already implies DHCP succeeded; DNS is resolved lazily, is only
 * needed for a non-numeric broker host, and a missing DNS server is just
 * another backoff-and-retry case, not a precondition to satisfy first.
 * Once ready, it connects, registers the last will, subscribes, drains
 * the publish queue, forwards commands, and reconnects with backoff
 * after a failure.
 *
 * @param[in] p_netif The station netif.
 */
void mqtt_link_poll(struct netif * p_netif);

/**
 * @brief Copy the driver status.
 *
 * Safe from any task on either core. The copy is read under a seqlock and
 * is tried up to eight times.
 *
 * @param[out] p_status Receives the snapshot.
 *
 * @return True if p_status holds a consistent snapshot. False if p_status
 *         is NULL, or if all eight tries met an update in progress, which
 *         can happen if this task preempted the radio service in the middle
 *         of one; the copy is then not consistent and must not be acted on.
 *         publish_queue_full is filled in either way.
 */
bool mqtt_link_get_status(mqtt_link_status_t * p_status);

/**
 * @brief Queue one message for publication.
 *
 * Safe from any task on either core. It never blocks, and it works whether
 * or not the session is up. If the queue already holds
 * MQTT_LINK_QUEUE_DEPTH messages, the new message is dropped, the queued
 * ones are kept, publish_queue_full is incremented and MQTT_LINK_ERR_FULL
 * is returned.
 *
 * It builds the queue record on the caller's stack, so it uses about 200
 * bytes of it.
 *
 * WARNING: measured with -fstack-usage against this port's real build
 * flags: this function's own frame plus mqtt_queue_put_publish()'s is
 * about 224 bytes, before whatever the kernel's own tk_snd_mbf() adds on
 * top. A caller on app.h's default TASK_STACK_SIZE (2048 bytes) must
 * budget for that on top of its own locals and call depth -- app.h's own
 * note that tm_printf alone costs about 1 KB leaves little headroom if
 * both are used in the same call path. Give a task that calls this
 * function its own larger stack via tk_cre_tsk() rather than
 * app_start_task()'s shared default.
 *
 * NOTE: A topic under "cmd/" is refused because the car subscribes to that
 * level and would receive its own message back as a command.
 *
 * @param[in] p_topic     Topic suffix, NUL terminated, at most
 *                        MQTT_LINK_TOPIC_MAX characters. It must not be
 *                        MQTT_LINK_TOPIC_STATUS, start with a slash or
 *                        with "cmd/", or contain # or +.
 * @param[in] p_payload   Payload bytes. May be NULL when payload_len is 0.
 * @param[in] payload_len Payload size, at most MQTT_LINK_PAYLOAD_MAX.
 * @param[in] qos         Quality of service.
 * @param[in] b_retain    True to ask the broker to retain the message.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY or
 *         MQTT_LINK_ERR_FULL. MQTT_LINK_ERR_PARAM covers every refused
 *         topic listed above, and any other bad argument.
 */
int32_t mqtt_link_publish(const char * p_topic, const uint8_t * p_payload,
                          uint16_t payload_len, mqtt_link_qos_t qos,
                          bool b_retain);

/**
 * @brief Take the next command received from the broker.
 *
 * Called by the application dispatcher task only. Commands arrive on the
 * topics under "<prefix>/cmd/" and the delivered topic is the suffix after
 * the prefix.
 *
 * If mqtt_link_init() has not succeeded, this returns
 * MQTT_LINK_ERR_NOT_READY at once, even with MQTT_LINK_WAIT_FOREVER, because
 * there is nothing to wait for. A dispatcher must treat that as "MQTT is
 * unavailable" and sleep or exit. A loop that calls this again without a
 * delay never blocks, so it would starve every task of lower priority.
 *
 * @param[out] p_command   Receives the command.
 * @param[in]  timeout_msec Wait limit in milliseconds. Zero polls without
 *                          waiting. MQTT_LINK_WAIT_FOREVER waits forever.
 *
 * @return MQTT_LINK_OK, MQTT_LINK_ERR_PARAM, MQTT_LINK_ERR_NOT_READY,
 *         MQTT_LINK_ERR_TIMEOUT or MQTT_LINK_ERR_KERNEL.
 */
int32_t mqtt_link_receive_command(mqtt_link_command_t * p_command,
                                  uint32_t timeout_msec);

#endif /* TM_MQTT */

#endif /* MQTT_LINK_H */

/*** end of file ***/
