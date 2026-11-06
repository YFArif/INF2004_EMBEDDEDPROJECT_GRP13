/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file telemetry_task.c
 *
 * @brief S1's telemetry_task, Stage 4a (see telemetry_task.h): takes each
 *        dashboard command from the MQTT driver, posts it to S5 with
 *        subsystem5_command_post(), publishes its ack on <prefix>/ack, and
 *        publishes the heartbeat once a second while the session is up.
 *
 * @par Commands while the session is down
 * The driver's session is clean, so the broker keeps nothing for the car:
 * a command sent while the car is offline is lost and never acked.
 * Commands already received when a session drops are still posted; their
 * acks wait in the publish queue and go out after the reconnect.
 *
 * @par Kernel and driver calls
 * This file makes them all; the logic that needs none is in s1_telem.c,
 * which the host test S1-U14 runs with a stub in place of
 * subsystem5_command_post().
 */

#include "telemetry_task.h"

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "msg_codec.h"
#include "msg_layout.h"
#include "mqtt_link.h"
#include "s1_telem.h"

/* S1_TELEM_STACK_REPORT=1 prints the stack high-water mark once a minute,
 * for the NFR11 check in S1-U06. Off in every normal build.
 */
#ifndef S1_TELEM_STACK_REPORT
#define S1_TELEM_STACK_REPORT       0
#endif

/* S1's opcodes are passed to S5 unchanged, so they must be S5's values;
 * the stack is counted in 8-byte words.
 */
typedef uint8_t telemetry_task_check_t[((MSG_OP_START == S5_CMD_START)
    && (MSG_OP_STOP == S5_CMD_STOP) && (MSG_OP_RESET == S5_CMD_RESET)
    && (0U == (TELEMETRY_TASK_STACK_BYTES % 8U))) ? 1 : -1];

#define STACK_WORDS                 (TELEMETRY_TASK_STACK_BYTES / 8U)
#define STACK_FILL                  (0xDEADBEEFDEADBEEFULL)
#define MSEC_PER_SEC                1000U

/* Telemetry frame at 2 Hz. Only the S4 fields are filled for now; the
 * other subsystems' fields are sent as 0 until they are added.
 */
#define TEL_PERIOD_MSEC             500U
#define TEL_HALF_RANGE_MSEC         0x80000000U

#if S1_TELEM_STACK_REPORT
#define REPORT_PERIOD_MSEC          60000U
#define REPORT_HALF_RANGE_MSEC      0x80000000U
#define REPORT_LINE_MAX             64
#endif

/* The task's own stack, 8-byte aligned as the processor needs. */
static uint64_t         g_stack[STACK_WORDS];

/* Written by telemetry_task only. */
static s1_telem_stats_t g_stats;
static uint8_t          g_hb_seq;
static uint8_t          g_tel_seq;

static void     telemetry_task(INT stacd, void * p_exinf);
static int32_t  post_to_s5(uint8_t opcode, uint8_t cmd_id);
static void     handle_command(const mqtt_link_command_t * p_cmd);
static void     send_heartbeat(void);
static void     send_telemetry(void);
static uint32_t now_msec(void);
static uint32_t uptime_s(void);
#if S1_TELEM_STACK_REPORT
static void     report_stack(void);
#endif

/*!
 * @brief Create and start telemetry_task on core 0, on its own stack.
 *
 * Called from subsystem1_start(), only after mqtt_link_init() succeeded.
 *
 * @return 0, or the negative kernel error from tk_cre_tsk() or
 *         tk_sta_tsk().
 */
int32_t
telemetry_task_start (void)
{
    T_CTSK   ctsk = {0};
    ID       tid = 0;
    ER       result = E_OK;
    uint32_t idx = 0U;

    for (idx = 0U; idx < STACK_WORDS; idx++)
    {
        g_stack[idx] = STACK_FILL;
    }

    /* NOTE: The casts are safe: telemetry_task has the kernel's task entry
     * signature, which FP stands for, and the stack is 2048 bytes.
     */
    ctsk.tskatr = TA_HLNG | TA_RNG3 | TA_USERBUF;
    ctsk.task = (FP)telemetry_task;
    ctsk.itskpri = TELEMETRY_TASK_PRI;
    ctsk.stksz = (SZ)sizeof(g_stack);
    ctsk.bufptr = g_stack;
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;
    ctsk.assprc = TP_PRC1;
#endif

    tid = tk_cre_tsk(&ctsk);

    if (tid < E_OK)
    {
        result = tid;
    }
    else
    {
        result = tk_sta_tsk(tid, 0);
    }

    if (E_OK == result)
    {
        /* The cast is safe: the format string is ASCII. */
        tm_printf((UB *)"S1: telemetry_task started\n");
    }

    /* ER is the kernel's W, a signed 32-bit integer on this port, so the
     * conversion keeps every value.
     */
    return ((int32_t)result);
}

/*!
 * @brief How much of telemetry_task's stack has been used so far.
 *
 * @return Bytes from the top of the stack to the deepest word written.
 */
uint32_t
telemetry_task_stack_used (void)
{
    uint32_t untouched = 0U;

    /* The stack grows down: count the words at the bottom that still hold
     * the fill pattern.
     */
    while ((untouched < STACK_WORDS) && (STACK_FILL == g_stack[untouched]))
    {
        untouched++;
    }

    return ((STACK_WORDS - untouched) * 8U);
}

/*!
 * @brief telemetry_task entry point.
 *
 * @param[in] stacd   Start code from tk_sta_tsk(); unused.
 * @param[in] p_exinf Extended information; unused.
 */
static void
telemetry_task (INT stacd, void * p_exinf)
{
    mqtt_link_command_t cmd = {0U, {0U}};
    uint32_t            next_hb_msec = now_msec();
    uint32_t            next_tel_msec = next_hb_msec;
#if S1_TELEM_STACK_REPORT
    uint32_t            next_report_msec = next_hb_msec;
#endif
    int32_t             rx_result = MQTT_LINK_OK;

    /* The casts mark the kernel's start-up arguments as unused. */
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        rx_result = mqtt_link_receive_command(&cmd,
                                              TELEMETRY_TASK_RX_WAIT_MSEC);

        if (MQTT_LINK_OK == rx_result)
        {
            handle_command(&cmd);
        }
        else if (MQTT_LINK_ERR_TIMEOUT == rx_result)
        {
            /* No command this pass. */
        }
        else if (MQTT_LINK_ERR_NOT_READY == rx_result)
        {
            /* Not expected: subsystem1_start() starts this task only after
             * mqtt_link_init() succeeded. The driver's rule is never to
             * call again without a wait, so the task ends. The cast is
             * safe: the format string is ASCII.
             */
            tm_printf((UB *)"S1: telemetry_task: MQTT link not ready\n");
            tk_ext_tsk();
        }
        else
        {
            /* A kernel error: wait one pass rather than spin. The cast is
             * safe: 50 ms fits the kernel's TMO.
             */
            g_stats.rx_error = s1_telem_sat_add(g_stats.rx_error, 1U);
            (void)tk_dly_tsk((TMO)TELEMETRY_TASK_RX_WAIT_MSEC);
        }

        if (s1_telem_hb_due(now_msec(), &next_hb_msec))
        {
            send_heartbeat();
        }

        if ((now_msec() - next_tel_msec) < TEL_HALF_RANGE_MSEC)
        {
            next_tel_msec += TEL_PERIOD_MSEC;
            send_telemetry();
        }

#if S1_TELEM_STACK_REPORT
        if ((now_msec() - next_report_msec) < REPORT_HALF_RANGE_MSEC)
        {
            next_report_msec += REPORT_PERIOD_MSEC;
            report_stack();
        }
#endif
    }
}

/*!
 * @brief Post one command to S5: the post function given to s1_telem.c.
 *
 * subsystem5_command_post() never blocks: it queues the command with
 * TMO_POL and raises EV_COMMAND for vehicle_ctrl_task.
 *
 * @param[in] opcode MSG_OP_START, MSG_OP_STOP or MSG_OP_RESET (equal to
 *                   S5's S5_CMD_xxx; checked at compile time).
 * @param[in] cmd_id The dashboard's command id.
 *
 * @return 0 (E_OK) when queued; E_TMOUT when S5's queue is full; another
 *         negative kernel error if S5's queue does not exist.
 */
static int32_t
post_to_s5 (uint8_t opcode, uint8_t cmd_id)
{
    /* ER is the kernel's W, a signed 32-bit integer on this port, so the
     * conversion keeps every value.
     */
    return ((int32_t)subsystem5_command_post(opcode, cmd_id));
}

/*!
 * @brief Post one received command and publish its ack.
 *
 * The ack goes out at QoS 1 whether or not the session is up; if the
 * publish queue is full it is dropped and counted.
 *
 * @param[in] p_cmd The command from mqtt_link_receive_command().
 */
static void
handle_command (const mqtt_link_command_t * p_cmd)
{
    s1_telem_pub_t ack = {MQTT_LINK_TOPIC_ACK, MQTT_LINK_QOS_1, false, 0U,
                          {0U}};

    if (s1_telem_on_command(p_cmd, post_to_s5, &g_stats, &ack))
    {
        if (MQTT_LINK_OK != mqtt_link_publish(ack.topic, ack.payload,
                                              ack.len, ack.qos,
                                              ack.b_retain))
        {
            g_stats.ack_publish_failed =
                s1_telem_sat_add(g_stats.ack_publish_failed, 1U);
        }
    }
}

/*!
 * @brief Publish one heartbeat, if the session is up.
 *
 * The counter goes up for every heartbeat handed to the driver, so a
 * heartbeat the full queue dropped shows as a gap on the dashboard.
 */
static void
send_heartbeat (void)
{
    mqtt_link_status_t status;
    s1_telem_hb_in_t   hb_in = {0U, 0U, 0U};
    s1_telem_pub_t     hb_msg = {MQTT_LINK_TOPIC_TELEMETRY, MQTT_LINK_QOS_0,
                                 false, 0U, {0U}};
    bool               b_fresh = mqtt_link_get_status(&status);
    bool               b_send = false;

    if (b_fresh && s1_telem_hb_should_publish(status.state, status.b_stale))
    {
        hb_in.uptime_s = uptime_s();
        hb_in.connect_count = status.connect_count;
        hb_in.command_dropped = status.command_dropped;
        b_send = s1_telem_heartbeat(g_hb_seq, &hb_in, &g_stats, &hb_msg);
    }

    if (!b_send)
    {
        g_stats.hb_skipped = s1_telem_sat_add(g_stats.hb_skipped, 1U);
    }
    else
    {
        /* The counter is modulo 256 by design: the cast keeps the low
         * byte.
         */
        g_hb_seq = (uint8_t)(g_hb_seq + 1U);

        if (MQTT_LINK_OK == mqtt_link_publish(hb_msg.topic, hb_msg.payload,
                                              hb_msg.len, hb_msg.qos,
                                              hb_msg.b_retain))
        {
            g_stats.hb_published = s1_telem_sat_add(g_stats.hb_published, 1U);
        }
        else
        {
            g_stats.hb_publish_failed =
                s1_telem_sat_add(g_stats.hb_publish_failed, 1U);
        }
    }
}

/*!
 * @brief Publish one telemetry frame, if the session is up.
 *
 * Fills the S4 fields from subsystem4_terrain_get(). Every other field
 * is 0 for now. A frame the full queue dropped shows as a seq gap.
 */
static void
send_telemetry (void)
{
    mqtt_link_status_t      status;
    subsystem4_terrain_t    terrain;
    msg_codec_telemetry_t   tel = {0};
    uint8_t                 buf[MSG_TEL_BYTES] = {0U};
    uint32_t                len = 0U;
    bool                    b_fresh = mqtt_link_get_status(&status);

    if (b_fresh && s1_telem_hb_should_publish(status.state, status.b_stale))
    {
        subsystem4_terrain_get(&terrain);

        tel.seq          = g_tel_seq;
        tel.pitch_hdeg   = terrain.pitch_hdeg;
        tel.roll_hdeg    = terrain.roll_hdeg;
        tel.hump_state   = terrain.hump_state;
        tel.peak_mm      = terrain.peak_mm;
        tel.hump_count   = terrain.hump_count;
        tel.motion_class = terrain.motion_class;
        tel.turn_dps     = terrain.turn_rate_dps;
        tel.b_imu_ok     = terrain.b_imu_ok;
        tel.b_cal_done   = terrain.b_cal_done;

        len = msg_codec_put_telemetry(&tel, buf, (uint32_t)sizeof(buf));

        if (MSG_TEL_BYTES == len)
        {
            /* Modulo 256 by design. */
            g_tel_seq = (uint8_t)(g_tel_seq + 1U);
            (void)mqtt_link_publish(MQTT_LINK_TOPIC_TELEMETRY, buf,
                                    (uint16_t)len, MQTT_LINK_QOS_0, false);
        }
    }
}

/*!
 * @brief Read the kernel's millisecond clock.
 *
 * @return Milliseconds since boot, low 32 bits (wraps after 49.7 days;
 *         s1_telem_hb_due() handles the wrap).
 */
static uint32_t
now_msec (void)
{
    SYSTIM otm;

    (void)tk_get_otm(&otm);

    /* otm.lo is the kernel's UW, an unsigned 32-bit integer on this port,
     * so the conversion keeps the value.
     */
    return ((uint32_t)otm.lo);
}

/*!
 * @brief Seconds since boot, for the heartbeat.
 *
 * @return Whole seconds; UINT32_MAX once the millisecond count passes
 *         32 bits (the codec saturates the field at 65535 long before).
 */
static uint32_t
uptime_s (void)
{
    SYSTIM   otm;
    uint32_t seconds = UINT32_MAX;

    (void)tk_get_otm(&otm);

    if (0 == otm.hi)
    {
        /* otm.lo is the kernel's UW, an unsigned 32-bit integer on this
         * port, so the conversion keeps the value.
         */
        seconds = ((uint32_t)otm.lo) / MSEC_PER_SEC;
    }

    return (seconds);
}

#if S1_TELEM_STACK_REPORT
/*!
 * @brief Print the stack high-water mark (S1_TELEM_STACK_REPORT builds).
 */
static void
report_stack (void)
{
    UB line[REPORT_LINE_MAX];

    /* The casts are safe: the format string is ASCII, and both values
     * are at most 2048, which an unsigned int holds.
     */
    (void)tm_sprintf(line, (const UB *)"S1: telemetry_task stack %u of %u B\n",
                     (unsigned int)telemetry_task_stack_used(),
                     (unsigned int)TELEMETRY_TASK_STACK_BYTES);
    tm_printf(line);
}
#endif

#endif /* TM_MQTT */

/*** end of file ***/
