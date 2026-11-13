/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file telemetry_task.c
 *
 * @brief S1's telemetry_task (see telemetry_task.h): takes each dashboard
 *        command from the MQTT driver, posts it to S5 with
 *        subsystem5_command_post(), publishes its ack on <prefix>/ack, and,
 *        while the session is up, publishes the heartbeat once a second,
 *        the telemetry frame at 2 Hz and the live message at 5 Hz.
 *
 * @par Commands while the session is down
 * The driver's session is clean, so the broker keeps nothing for the car:
 * a command sent while the car is offline is lost and never acked.
 * Commands already received when a session drops are still posted; their
 * acks wait in the publish queue and go out after the reconnect.
 *
 * @par Keeping room for the acks
 * The publish queue is first in, first out and the driver sends one message
 * at a time, so a periodic message delays any ack queued behind it. Before
 * each periodic message this task reads how full the queue is
 * (mqtt_link_publish_free()) and compares it with that message's limit in
 * s1_telem.h: S1_TELEM_TEL_QUEUED_MAX, S1_TELEM_LIVE_QUEUED_MAX or
 * S1_TELEM_HB_QUEUED_MAX. A message held back is counted, not retried. Acks
 * are never held back.
 *
 * @par WiFi fault bit
 * On each pass this task reads the radio link and the driver status and
 * applies s1_telem_wifi_update(). S1 sets and clears S5_FAULT_WIFI itself, as
 * a level bit, and prints one console line per change. It raises no EV_FAULT:
 * S5 does not act on this bit (docs/S1_binary_frame.md section 3.1).
 *
 * @par Kernel and driver calls
 * This file makes them all; the logic that needs none is in s1_telem.c,
 * which the host test S1-U14 runs with stubs in place of
 * subsystem5_command_post(), subsystem5_fault_set() and
 * subsystem5_fault_clear().
 */

#include "telemetry_task.h"

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/s3_api.h"
#include "../subsystem3/s3_code39.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "msg_codec.h"
#include "msg_layout.h"
#include "mqtt_link.h"
#include "s1_radio.h"
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

/* The other subsystems' values are copied to the wire with no remapping,
 * so the numbers S1 relies on must be theirs: the fault bits (S5), the
 * barcode commands, junction turns, "no letter" and UNCAL (S3), and the
 * IMPACT class (S4). A renumbering by an owner stops the build here. The
 * casts only make each comparison unsigned: every enum constant here is
 * small and not negative, so it keeps its value.
 */
typedef uint8_t telemetry_task_check_map_t[((MSG_FAULT_STALL == S5_FAULT_STALL)
    && (MSG_FAULT_ENCODER == S5_FAULT_ENCODER)
    && (MSG_FAULT_IMU == S5_FAULT_IMU)
    && (MSG_FAULT_NO_ECHO == S5_FAULT_NO_ECHO)
    && (MSG_FAULT_WIFI == S5_FAULT_WIFI)
    && (MSG_BC_CMD_NONE == (uint32_t)S3_CMD_NONE)
    && (MSG_BC_CMD_LEFT == (uint32_t)S3_CMD_LEFT)
    && (MSG_BC_CMD_RIGHT == (uint32_t)S3_CMD_RIGHT)
    && (MSG_BC_CMD_STRAIGHT == (uint32_t)S3_CMD_STRAIGHT)
    && (MSG_BC_CMD_UTURN == (uint32_t)S3_CMD_UTURN)
    && (MSG_JUNC_STRAIGHT == S3_JUNC_STRAIGHT)
    && (MSG_JUNC_LEFT == S3_JUNC_LEFT)
    && (MSG_JUNC_RIGHT == S3_JUNC_RIGHT)
    && (MSG_JUNC_UTURN == S3_JUNC_UTURN)
    && (MSG_BC_LETTER_NONE == S3_C39_IDX_NONE)
    && (S1_TELEM_LINE_UNCAL == (uint32_t)S3_LINE_UNCAL)
    && (S1_TELEM_CLASS_IMPACT == (uint32_t)S4_CLASS_IMPACT)) ? 1 : -1];

#define STACK_WORDS                 (TELEMETRY_TASK_STACK_BYTES / 8U)
#define STACK_FILL                  (0xDEADBEEFDEADBEEFULL)
#define MSEC_PER_SEC                1000U

/* The telemetry frame (2 Hz) and the live message (5 Hz) use the
 * absolute schedule of s1_telem_due(), like the heartbeat; the periods are
 * S1_TELEM_TEL_PERIOD_MSEC and S1_TELEM_LIVE_PERIOD_MSEC, and
 * S1_TELEM_TEL_OFFSET_MSEC and S1_TELEM_LIVE_OFFSET_MSEC keep the deadlines
 * of all three apart.
 */

/* One console line for a change of the WiFi fault bit. tm_sprintf() does not
 * limit its output, so the buffer is sized for the widest line with room to
 * spare. The widest line, in characters:
 *   "S1: wifi fault " 15 + "clear" 5 + " link=" 6 + "-2147483648" 11
 *   + " mqtt=" 6 + state 1 (0 to 4) + " stale=" 7 + 1 (0 or 1)
 *   + " mask=0x" 8 + 2 (at most 1f) + a newline 1 = 63,
 * plus the NUL tm_sprintf() writes = 64 bytes. 80 leaves 16 bytes spare.
 */
#define WIFI_LINE_BYTES             80U

#if S1_TELEM_STACK_REPORT
#define REPORT_PERIOD_MSEC          60000U
#define REPORT_HALF_RANGE_MSEC      0x80000000U
#define REPORT_LINE_MAX             64
#endif

/* The task's own stack, 8-byte aligned as the processor needs. */
static uint64_t         g_stack[STACK_WORDS];

/* Written by telemetry_task only. */
static s1_telem_stats_t g_stats = {0U};
static uint8_t          g_hb_seq = 0U;
static uint8_t          g_tel_seq = 0U;
static uint8_t          g_live_seq = 0U;
static bool             gb_wifi_fault_set = false;

/* The subsystem values and the two messages being filled. They are here,
 * not on the task's 2048-byte stack, because only telemetry_task uses them.
 */
static s1_telem_src_t        g_src;
static msg_codec_telemetry_t g_tel;
static msg_codec_live_t      g_live;

static void     telemetry_task(INT stacd, void * p_exinf);
static int32_t  post_to_s5(uint8_t opcode, uint8_t cmd_id);
static void     handle_command(const mqtt_link_command_t * p_cmd);
static void     update_wifi_fault(void);
static void     print_wifi_change(int32_t link_status,
                                  const mqtt_link_status_t * p_status);
static void     send_heartbeat(void);
static void     publish_heartbeat(const mqtt_link_status_t * p_status);
static void     collect_sources(s1_telem_src_t * p_src);
static void     send_periodic(bool b_tel_due, bool b_live_due);
static void     publish_frame(void);
static void     publish_live(void);
static void     publish_periodic(const uint8_t * p_buf, uint16_t len,
                                 uint8_t * p_seq);
static uint32_t now_msec(void);
static uint32_t uptime_s(void);
#if S1_TELEM_STACK_REPORT
static uint32_t telemetry_task_stack_used(void);
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
    ID       h_task = 0;
    ER       result = E_OK;
    uint32_t idx = 0U;

    for (idx = 0U; idx < STACK_WORDS; idx++)
    {
        g_stack[idx] = STACK_FILL;
    }

    /* NOTE: The casts are safe: telemetry_task has the kernel's task entry
     * signature, which FP stands for, and the stack is 2048 bytes.
     */
    ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
    ctsk.task    = (FP)telemetry_task;
    ctsk.itskpri = TELEMETRY_TASK_PRI;
    ctsk.stksz   = (SZ)sizeof(g_stack);
    ctsk.bufptr  = g_stack;
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;
    ctsk.assprc = TP_PRC1;
#endif

    h_task = tk_cre_tsk(&ctsk);

    if (h_task < E_OK)
    {
        result = h_task;
    }
    else
    {
        result = tk_sta_tsk(h_task, 0);
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
    uint32_t            next_tel_msec = next_hb_msec
                                        + S1_TELEM_TEL_OFFSET_MSEC;
    uint32_t            next_live_msec = next_hb_msec
                                         + S1_TELEM_LIVE_OFFSET_MSEC;
    bool                b_tel_due = false;
    bool                b_live_due = false;
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
             * safe: 50 ms fits the kernel's TMO. The (void) drops
             * tk_dly_tsk()'s result: a wait that ends early only brings
             * the next pass forward.
             */
            g_stats.rx_error = s1_telem_sat_add(g_stats.rx_error, 1U);
            (void)tk_dly_tsk((TMO)TELEMETRY_TASK_RX_WAIT_MSEC);
        }

        update_wifi_fault();

        if (s1_telem_hb_due(now_msec(), &next_hb_msec))
        {
            send_heartbeat();
        }

        b_tel_due  = s1_telem_due(now_msec(), S1_TELEM_TEL_PERIOD_MSEC,
                                  &next_tel_msec);
        b_live_due = s1_telem_due(now_msec(), S1_TELEM_LIVE_PERIOD_MSEC,
                                  &next_live_msec);

        if (b_tel_due || b_live_due)
        {
            send_periodic(b_tel_due, b_live_due);
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
 * @brief Apply the WiFi fault rule once; print one line if the bit changed.
 *
 * Called once per pass, so a change is seen within about one command wait.
 * The flag changes before the line is built, so the console cannot delay it.
 *
 * NOTE: subsystem5_fault_clear() is in the private block of subsystem5.h. S1
 * calls it for this bit only: Tribunal 1 (Version B) makes WiFi loss a level
 * bit that S1 sets and clears itself.
 */
static void
update_wifi_fault (void)
{
    mqtt_link_status_t     status;
    s1_telem_wifi_status_t wifi_status = {false, false, false,
                                          MQTT_LINK_STATE_IDLE, 0U};
    int32_t                link_status = 0;

    wifi_status.b_link_up     = s1_radio_link_read(&link_status);
    wifi_status.b_status_read = mqtt_link_get_status(&status);

    /* After a failed read the copy may be torn (mqtt_link_get_status()), so
     * nothing is taken from it and the rule holds the level. For the same
     * reason print_wifi_change() is reached only after a good read.
     */
    if (wifi_status.b_status_read)
    {
        wifi_status.b_stale       = status.b_stale;
        wifi_status.state         = status.state;
        wifi_status.connect_count = status.connect_count;
    }

    if (s1_telem_wifi_update(&wifi_status, subsystem5_fault_set,
                             subsystem5_fault_clear, &gb_wifi_fault_set))
    {
        print_wifi_change(link_status, &status);
    }
}

/*!
 * @brief Print one console line for a change of the WiFi fault bit.
 *
 * mask is read back from S5's flag, so the line shows the flag itself, not
 * what this task believes it set.
 *
 * @param[in] link_status The radio link status just read (CYW43_LINK_xxx).
 * @param[in] p_status    The driver status just read.
 */
static void
print_wifi_change (int32_t link_status, const mqtt_link_status_t * p_status)
{
    UB line[WIFI_LINE_BYTES];

    /* The casts are safe: the format string is ASCII; an int is 32 bits on
     * this port, so it holds link_status; the state is a small enum value;
     * the mask is at most 0x1F. The length tm_sprintf() returns is not needed.
     */
    (void)tm_sprintf(line,
                     (const UB *)"S1: wifi fault %s link=%d mqtt=%d stale=%d "
                                 "mask=0x%02x\n",
                     gb_wifi_fault_set ? "set" : "clear", (int)link_status,
                     (int)p_status->state, p_status->b_stale ? 1 : 0,
                     (unsigned int)(subsystem5_fault_get()
                                    & MSG_FAULT_MASK_ALL));
    tm_printf(line);
}

/*!
 * @brief Publish one heartbeat, if the session is up and the publish queue
 *        is short enough.
 */
static void
send_heartbeat (void)
{
    mqtt_link_status_t status;
    const bool         b_fresh = mqtt_link_get_status(&status);

    if ((!b_fresh)
        || (!s1_telem_hb_should_publish(status.state, status.b_stale)))
    {
        g_stats.hb_skipped = s1_telem_sat_add(g_stats.hb_skipped, 1U);
    }
    else if (!s1_telem_queue_allows(mqtt_link_publish_free(),
                                    S1_TELEM_HB_QUEUED_MAX))
    {
        g_stats.periodic_skipped =
            s1_telem_sat_add(g_stats.periodic_skipped, 1U);
    }
    else
    {
        publish_heartbeat(&status);
    }
}

/*!
 * @brief Pack one heartbeat and hand it to the driver.
 *
 * The counter goes up for every heartbeat handed to the driver, so a
 * heartbeat the driver refuses shows as a gap on the dashboard as well as in
 * hb_publish_failed.
 *
 * @param[in] p_status The driver status just read.
 */
static void
publish_heartbeat (const mqtt_link_status_t * p_status)
{
    s1_telem_hb_in_t hb_in = {0U, 0U, 0U};
    s1_telem_pub_t   hb_msg = {MQTT_LINK_TOPIC_TELEMETRY, MQTT_LINK_QOS_0,
                               false, 0U, {0U}};

    hb_in.uptime_s        = uptime_s();
    hb_in.connect_count   = p_status->connect_count;
    hb_in.command_dropped = p_status->command_dropped;

    if (!s1_telem_heartbeat(g_hb_seq, &hb_in, &g_stats, &hb_msg))
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
 * @brief Copy the latest values from S2, S3, S4 and S5 into plain integers.
 *
 * Every call here is a getter the owner provides for telemetry. Those of
 * S2, S3, S4 and S5 take the owner's mutex and wait forever for it, then
 * copy a snapshot. The owners hold their mutexes only for a copy, so this
 * task rarely waits long. If S3 cannot be read (not started), b_s3_ok is
 * false and s1_telem.c sends the documented defaults.
 *
 * WARNING: The wait can also go the other way. If cyw43_utk_task (priority
 * 4, core 0) preempts this task while it holds an owner's mutex, a core-1
 * task that wants that mutex waits until this task runs again: priority
 * inheritance lifts this task only to the waiter's priority, still below 4.
 * The window is a few microseconds, about 7 times a second. See
 * docs/S1_binary_frame.md section 3.4; it is known and not worked around.
 *
 * @param[out] p_src Receives the values.
 */
static void
collect_sources (s1_telem_src_t * p_src)
{
    subsystem2_odom_t        odom = {0U};
    subsystem2_status_t      motion = {S2_STATE_IDLE};
    s3_line_status_t         line = {0};
    subsystem4_terrain_t     terrain = {0};
    subsystem5_obst_status_t obst = {S5_OBST_HOLD};
    s5_profile_t             profile = {0U};

    *p_src = (s1_telem_src_t){0U};

    subsystem2_odom_get(&odom);
    subsystem2_status_get(&motion);
    p_src->b_estop      = subsystem2_estop_is_set();
    /* subsystem2_state_t has 5 values, so the cast keeps the value. */
    p_src->motion_state = (uint8_t)motion.state;
    p_src->target_l_mms = motion.target_l_mms;
    p_src->target_r_mms = motion.target_r_mms;
    p_src->count_l      = odom.count_l;
    p_src->count_r      = odom.count_r;
    p_src->dist_mm      = odom.dist_mm;
    p_src->speed_l_mms  = odom.speed_l_mms;
    p_src->speed_r_mms  = odom.speed_r_mms;
    p_src->heading_cdeg = odom.heading_cdeg;

    if (E_OK == s3_get_line_status(&line))
    {
        /* The S3 fields are bit-fields of 1 to 10 bits; each fits the plain
         * type it is copied to.
         */
        p_src->b_s3_ok      = true;
        p_src->line_state   = (uint8_t)line.state;
        p_src->steer_dv_mms = (int32_t)line.steer_dv;
        p_src->outer_ir     = (uint8_t)line.n_outer;
        p_src->centre_ir    = (uint8_t)line.n_centre;
        p_src->b_overshoot  = (0U != line.overshoot);
        p_src->last_ch      = (uint8_t)line.last_ch;
        p_src->last_cmd     = (uint8_t)line.last_cmd;
        p_src->b_last_rev   = (0U != line.last_rev);
        p_src->junc_turn    = (uint8_t)line.junc_turn;
        p_src->rejects      = (uint8_t)line.rejects;
        p_src->junctions    = (uint8_t)line.junctions;
        p_src->losts        = (uint8_t)line.losts;
        p_src->barcodes     = (uint8_t)line.barcodes;
    }

    subsystem4_terrain_get(&terrain);
    p_src->pitch_hdeg    = terrain.pitch_hdeg;
    p_src->roll_hdeg     = terrain.roll_hdeg;
    p_src->turn_rate_dps = terrain.turn_rate_dps;
    p_src->peak_mm       = terrain.peak_mm;
    p_src->hump_state    = terrain.hump_state;
    p_src->motion_class  = terrain.motion_class;
    p_src->hump_count    = terrain.hump_count;
    p_src->b_imu_ok      = terrain.b_imu_ok;
    p_src->b_cal_done    = terrain.b_cal_done;
    p_src->b_impact      = terrain.b_impact;

    subsystem5_obst_status_get(&obst);
    subsystem5_profile_get(&profile);
    /* subsystem5_ctrl_state_t has 11 values, so the cast keeps the value. */
    p_src->ctrl_state      = (uint8_t)subsystem5_ctrl_state_get();
    p_src->fault_mask      = subsystem5_fault_get();
    p_src->b_range_valid   = obst.b_range_valid;
    p_src->range_mm        = obst.range_mm;
    p_src->servo_deg       = obst.servo_deg;
    p_src->b_profile_valid = profile.b_valid;
    p_src->width_mm        = profile.width_mm;
    p_src->clear_left_mm   = profile.clear_left_mm;
    p_src->clear_right_mm  = profile.clear_right_mm;
}

/*!
 * @brief Publish the telemetry frame and/or the live message, if the
 *        session is up.
 *
 * The values are read once for both.
 *
 * @param[in] b_tel_due  The 2 Hz frame is due.
 * @param[in] b_live_due The 5 Hz live message is due.
 */
static void
send_periodic (bool b_tel_due, bool b_live_due)
{
    mqtt_link_status_t status;
    const bool         b_fresh = mqtt_link_get_status(&status);

    if (b_fresh && (s1_telem_hb_should_publish(status.state, status.b_stale)))
    {
        collect_sources(&g_src);

        if (b_tel_due)
        {
            publish_frame();
        }

        if (b_live_due)
        {
            publish_live();
        }
    }
}

/*!
 * @brief Pack the telemetry frame from g_src and hand it to the driver, or
 *        count it as skipped if the publish queue is too full.
 */
static void
publish_frame (void)
{
    uint8_t buf[MSG_TEL_BYTES] = {0U};

    if (!s1_telem_queue_allows(mqtt_link_publish_free(),
                               S1_TELEM_TEL_QUEUED_MAX))
    {
        g_stats.periodic_skipped =
            s1_telem_sat_add(g_stats.periodic_skipped, 1U);
    }
    else
    {
        s1_telem_fill_frame(&g_src, g_tel_seq, &g_tel);

        if (MSG_TEL_BYTES == msg_codec_put_telemetry(&g_tel, buf,
                                                     MSG_TEL_BYTES))
        {
            publish_periodic(buf, MSG_TEL_BYTES, &g_tel_seq);
        }
    }
}

/*!
 * @brief Pack the live message from g_src and hand it to the driver, or
 *        count it as skipped if the publish queue is too full.
 */
static void
publish_live (void)
{
    uint8_t buf[MSG_LIV_BYTES] = {0U};

    if (!s1_telem_queue_allows(mqtt_link_publish_free(),
                               S1_TELEM_LIVE_QUEUED_MAX))
    {
        g_stats.periodic_skipped =
            s1_telem_sat_add(g_stats.periodic_skipped, 1U);
    }
    else
    {
        s1_telem_fill_live(&g_src, g_live_seq, &g_live);

        if (MSG_LIV_BYTES == msg_codec_put_live(&g_live, buf, MSG_LIV_BYTES))
        {
            publish_periodic(buf, MSG_LIV_BYTES, &g_live_seq);
        }
    }
}

/*!
 * @brief Hand one packed telemetry or live message to the driver.
 *
 * The seq goes up for every message handed to the driver, so one the driver
 * refuses shows as a gap on the dashboard as well as in periodic_failed. A
 * message the queue rule skipped never gets here and uses no seq number.
 *
 * @param[in]     p_buf The packed message.
 * @param[in]     len   Its length.
 * @param[in,out] p_seq The message type's counter.
 */
static void
publish_periodic (const uint8_t * p_buf, uint16_t len, uint8_t * p_seq)
{
    /* The counter is modulo 256 by design: the cast keeps the low byte. */
    *p_seq = (uint8_t)(*p_seq + 1U);

    if (MQTT_LINK_OK != mqtt_link_publish(MQTT_LINK_TOPIC_TELEMETRY, p_buf,
                                          len, MQTT_LINK_QOS_0, false))
    {
        g_stats.periodic_failed =
            s1_telem_sat_add(g_stats.periodic_failed, 1U);
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

    /* The (void) drops tk_get_otm()'s result, which is always E_OK in this
     * kernel (kernel/tkernel/time_calls.c).
     */
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

    /* The (void) drops tk_get_otm()'s result, which is always E_OK in this
     * kernel (kernel/tkernel/time_calls.c).
     */
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
 * @brief How much of telemetry_task's stack has been used so far.
 *
 * @return Bytes from the top of the stack to the deepest word written.
 */
static uint32_t
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
 * @brief Print the stack high-water mark (S1_TELEM_STACK_REPORT builds).
 */
static void
report_stack (void)
{
    UB line[REPORT_LINE_MAX];

    /* The casts are safe: the format string is ASCII, and both values
     * are at most 2048, which an unsigned int holds. The (void) drops the
     * length tm_sprintf() returns, which is not needed.
     */
    (void)tm_sprintf(line, (const UB *)"S1: telemetry_task stack %u of %u B\n",
                     (unsigned int)telemetry_task_stack_used(),
                     (unsigned int)TELEMETRY_TASK_STACK_BYTES);
    tm_printf(line);
}
#endif

#endif /* TM_MQTT */

/*** end of file ***/

