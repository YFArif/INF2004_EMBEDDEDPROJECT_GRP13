/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file codec_timing_main.c
 *
 * @brief S1-U08: on-target timing of the minimum-bit message codec.
 *
 * @par What it measures
 * msg_codec_put_telemetry(), msg_codec_get_telemetry(),
 * msg_codec_put_command() and msg_codec_get_command() on the Pico W, with
 * the RP2040's 1 MHz free-running timer (TIMERAWL). For each operation,
 * on core 1 and then on core 0, at task priority 14:
 * - the first call after an XIP cache flush (cold cache: the deployed
 *   figure, because at 2 Hz other code runs between calls);
 * - 1000 timed calls: min, mean and max in microseconds, each result
 *   decoded or compared outside the timed window (round trip);
 * - 1000 calls back to back in one window: the warm mean, in ns per call;
 * - the CPU share at the real rate: telemetry at 2 Hz as a percentage of
 *   one core; a command as a percentage of the 300 ms NFR15 budget.
 * The timer's own overhead (1000 empty read pairs) is reported apart and
 * not subtracted.
 *
 * @par Pass criterion (core 1 only)
 * Each of the four operations round-trips correctly in all 1000 timed
 * calls, and its max over those calls is at most 1000 us. Core 0 runs
 * below the WiFi service task and is reported only. The 25 to 40 us
 * telemetry estimate is printed as information, not as a gate.
 *
 * @par How it is built
 * testing/subsystem1/run_codec_timing.sh exports the committed tree,
 * copies this file over app_program/app_main.c in that copy only (with
 * this header beside it), and builds NET=lwip MQTT=1, the profile that
 * compiles the codec. Include paths below are therefore relative to
 * app_program/. The repository's template files are never changed.
 *
 * @par Output
 * Lines start with "[S1-U08]" (figures) or "[TEST]" (the gate, in the
 * test_report.h format). The report repeats every 30 s, so a serial
 * capture started late still holds a complete round.
 *
 * @par Test code
 * Never part of the firmware build. usermain() and app_start_task() keep
 * the kernel types their callers require.
 */

#include "codec_timing_main.h"

#include <stdbool.h>
#include <stdint.h>

#include "app.h"
#include "test_report.h"
#include "subsystem1/msg_codec.h"

#if !TM_MQTT
#error "S1-U08 needs NET=lwip MQTT=1 (msg_codec is gated on TM_MQTT)"
#endif

#define ITERATIONS          1000U
#define LIMIT_US            1000U       /* Gate: max per call, core 1.    */
#define ESTIMATE_LOW_US     25U         /* Telemetry estimate, info only. */
#define ESTIMATE_HIGH_US    40U
#define TIMING_PRI          14
#define USB_WAIT_MS         5000
#define REPORT_PERIOD_MS    30000
#define TEL_HZ              2U          /* Telemetry frames per second.   */
#define NFR15_BUDGET_US     300000U     /* STOP round trip budget.        */

/* XIP cache control (RP2040 datasheet, XIP_CTRL block). Writing 1 to
 * FLUSH empties the cache; reading FLUSH waits until that is done.
 */
#define XIP_CTRL_BASE       0x14000000U
#define XIP_FLUSH           (XIP_CTRL_BASE + 0x04U)

/** The four timed operations. */
typedef enum
{
    OP_PUT_TEL = 0,
    OP_GET_TEL,
    OP_PUT_CMD,
    OP_GET_CMD,
    OP_COUNT
} op_t;

/** Results for one operation on one core. */
typedef struct
{
    uint32_t    first_us;       /* First call after an XIP flush.       */
    uint32_t    min_us;
    uint32_t    max_us;
    uint64_t    sum_us;         /* Over the ITERATIONS timed calls.     */
    uint32_t    batch_us;       /* ITERATIONS calls in one window.      */
    uint32_t    correct;        /* Timed calls that round-tripped.      */
} op_stats_t;

static const char * const g_op_name[OP_COUNT] =
{
    "put_telemetry", "get_telemetry", "put_command", "get_command"
};

/* Written by usermain() or core1_task and read by the other task, so
 * volatile (Barr-C 1.8.c). The tasks take turns through tk_wup_tsk() and
 * tk_slp_tsk(), so the two never write at the same time.
 */
static volatile ID       g_tid_core0 = 0;
static volatile ID       g_tid_core1 = 0;
static volatile uint32_t g_round = 0U;

static uint32_t now_us(void);
static void     xip_flush(void);
static void     stats_reset(op_stats_t * p_s);
static void     stats_add(op_stats_t * p_s, uint32_t elapsed_us);
static void     tel_fill(uint32_t idx, msg_codec_telemetry_t * p_tel);
static bool     tel_same(const msg_codec_telemetry_t * p_a,
                         const msg_codec_telemetry_t * p_b);
static void     cmd_fill(uint32_t idx, msg_codec_command_t * p_cmd);
static void     time_put_tel(op_stats_t * p_s);
static void     time_get_tel(op_stats_t * p_s);
static void     time_put_cmd(op_stats_t * p_s);
static void     time_get_cmd(op_stats_t * p_s);
static void     time_overhead(op_stats_t * p_s);
static void     measure_all(op_stats_t * p_stats);
static void     print_op(uint32_t core, op_t op, const op_stats_t * p_s);
static void     print_core(uint32_t core, const op_stats_t * p_stats,
                           const op_stats_t * p_overhead);
static void     gate_core1(const op_stats_t * p_stats);
static bool     check_all_functions(void);
static void     core1_task(INT stacd, void * exinf);
static void     core0_task(INT stacd, void * exinf);

/*!
 * @brief Entry point: wait for the USB console, check every codec
 *        function once, then start the two timing tasks.
 *
 * @return Never returns while the system runs (0 by convention).
 */
EXPORT INT
usermain (void)
{
    tm_printf((UB *)"\n=== S1-U08 msg_codec timing (layout version %u) ===\n",
              MSG_LAYOUT_VERSION);

    /* Give the host time to open the USB serial port. */
    (void)tk_dly_tsk(USB_WAIT_MS);

    tm_printf((UB *)"[S1-U08] all 11 codec functions called once: %s\n",
              check_all_functions() ? "ok" : "FAIL");

    /* core0_task sleeps until core1_task wakes it, so create it first.
     * FP is the kernel's generic task-entry type; the kernel calls the
     * entry as void (INT stacd, void *exinf), which is exactly the
     * signature of both tasks, so the casts lose nothing.
     */
    g_tid_core0 = app_start_task((FP)core0_task, TIMING_PRI, TP_PRC1,
                                 "s1u08 core 0");
    g_tid_core1 = app_start_task((FP)core1_task, TIMING_PRI, TP_PRC2,
                                 "s1u08 core 1");

    (void)tk_slp_tsk(TMO_FEVR);

    return (0);
}

/*!
 * @brief Create and start one task (the same as app_main.c's version,
 *        which this test app replaces; app.h declares it for every
 *        subsystem).
 *
 * @param[in] entry Task function.
 * @param[in] pri   Initial priority.
 * @param[in] prc   TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for any.
 * @param[in] name  Name printed if a kernel call fails.
 *
 * @return The task ID, or a negative error code.
 */
EXPORT ID
app_start_task (FP entry, PRI pri, UW prc, const char * name)
{
    T_CTSK ctsk = { 0 };
    ID     tid;
    ER     er;

    ctsk.task = entry;
    ctsk.itskpri = pri;
    ctsk.stksz = TASK_STACK_SIZE;
    ctsk.tskatr = TA_HLNG | TA_RNG3;
#if TK_SUPPORT_SMP
    if (0U != prc)
    {
        ctsk.tskatr |= TA_ASSPRC;
        ctsk.assprc = prc;
    }
#else
    (void)prc;
#endif

    tid = tk_cre_tsk(&ctsk);
    if (tid >= E_OK)
    {
        er = tk_sta_tsk(tid, 0);
        CHECK(er, name);
    }
    else
    {
        CHECK(tid, name);
    }

    return (tid);
}

/*!
 * @brief Read the 1 MHz free-running timer.
 *
 * @return Microseconds; wraps every 2^32 us, so differences are taken
 *         modulo 2^32.
 */
static uint32_t
now_us (void)
{
    /* UW is a 32-bit unsigned type on this target, so the value is kept. */
    return ((uint32_t)in_w(TIMER_TIMERAWL));
}

/*!
 * @brief Empty the XIP cache, so the next call fetches its code from
 *        flash (a cold call).
 */
static void
xip_flush (void)
{
    out_w(XIP_FLUSH, 1U);
    (void)in_w(XIP_FLUSH);
}

/*!
 * @brief Clear one operation's results.
 *
 * @param[out] p_s Results.
 */
static void
stats_reset (op_stats_t * p_s)
{
    p_s->first_us = 0U;
    p_s->min_us = UINT32_MAX;
    p_s->max_us = 0U;
    p_s->sum_us = 0U;
    p_s->batch_us = 0U;
    p_s->correct = 0U;
}

/*!
 * @brief Add one timed call.
 *
 * @param[in,out] p_s        Results.
 * @param[in]     elapsed_us The call's time.
 */
static void
stats_add (op_stats_t * p_s, uint32_t elapsed_us)
{
    if (elapsed_us < p_s->min_us)
    {
        p_s->min_us = elapsed_us;
    }

    if (elapsed_us > p_s->max_us)
    {
        p_s->max_us = elapsed_us;
    }

    p_s->sum_us += elapsed_us;
}

/*!
 * @brief Telemetry input for timed call idx: the TEL-1 vector of
 *        docs/S1_binary_frame.md 10.2, with seq and the counts changed
 *        every call. Every value is inside its field's range, so a round
 *        trip gives back exactly this struct.
 *
 * @param[in]  idx   Call number, 0 to ITERATIONS - 1.
 * @param[out] p_tel The fields.
 */
static void
tel_fill (uint32_t idx, msg_codec_telemetry_t * p_tel)
{
    /* idx is below 1000, so idx % 256 fits uint8_t. */
    p_tel->seq = (uint8_t)(idx % 256U);
    p_tel->ctrl_state = 3U;
    p_tel->motion_state = 1U;
    p_tel->fault_mask = MSG_FAULT_IMU;
    p_tel->line_state = 0U;
    p_tel->bc_letter = 11U;
    p_tel->bc_cmd = 2U;
    p_tel->b_bc_dir = true;
    p_tel->speed_l_mms = 300;
    p_tel->speed_r_mms = -120;
    p_tel->count_l = 4464U + idx;
    p_tel->count_r = 60000U + idx;
    p_tel->dist_mm = -1234;
    p_tel->pitch_hdeg = -2;
    p_tel->roll_hdeg = 1;
    p_tel->hump_state = 2U;
    p_tel->peak_mm = 37U;
    p_tel->hump_count = 3U;
    p_tel->motion_class = 4U;
    p_tel->turn_dps = -45;
    p_tel->b_imu_ok = true;
    p_tel->b_cal_done = true;
    p_tel->closest_mm = 350U;
    p_tel->angle_idx = 12U;
    p_tel->width_mm = 210U;
    p_tel->clear_l_mm = 400U;
    p_tel->clear_r_mm = 1023U;
}

/*!
 * @brief Compare two sets of telemetry fields.
 *
 * @param[in] p_a First.
 * @param[in] p_b Second.
 *
 * @return True if every field is equal.
 */
static bool
tel_same (const msg_codec_telemetry_t * p_a, const msg_codec_telemetry_t * p_b)
{
    return ((p_a->seq == p_b->seq) && (p_a->ctrl_state == p_b->ctrl_state)
            && (p_a->motion_state == p_b->motion_state)
            && (p_a->fault_mask == p_b->fault_mask)
            && (p_a->line_state == p_b->line_state)
            && (p_a->bc_letter == p_b->bc_letter)
            && (p_a->bc_cmd == p_b->bc_cmd)
            && (p_a->b_bc_dir == p_b->b_bc_dir)
            && (p_a->speed_l_mms == p_b->speed_l_mms)
            && (p_a->speed_r_mms == p_b->speed_r_mms)
            && (p_a->count_l == p_b->count_l)
            && (p_a->count_r == p_b->count_r)
            && (p_a->dist_mm == p_b->dist_mm)
            && (p_a->pitch_hdeg == p_b->pitch_hdeg)
            && (p_a->roll_hdeg == p_b->roll_hdeg)
            && (p_a->hump_state == p_b->hump_state)
            && (p_a->peak_mm == p_b->peak_mm)
            && (p_a->hump_count == p_b->hump_count)
            && (p_a->motion_class == p_b->motion_class)
            && (p_a->turn_dps == p_b->turn_dps)
            && (p_a->b_imu_ok == p_b->b_imu_ok)
            && (p_a->b_cal_done == p_b->b_cal_done)
            && (p_a->closest_mm == p_b->closest_mm)
            && (p_a->angle_idx == p_b->angle_idx)
            && (p_a->width_mm == p_b->width_mm)
            && (p_a->clear_l_mm == p_b->clear_l_mm)
            && (p_a->clear_r_mm == p_b->clear_r_mm));
}

/*!
 * @brief Command input for timed call idx: START, STOP and RESET in turn,
 *        with every cmd_id from 0 to 63.
 *
 * @param[in]  idx   Call number.
 * @param[out] p_cmd The command.
 */
static void
cmd_fill (uint32_t idx, msg_codec_command_t * p_cmd)
{
    /* idx % 3 is 0 to 2 and idx % 64 is 0 to 63, so both fit uint8_t. */
    p_cmd->opcode = (uint8_t)(MSG_OP_START + (idx % 3U));
    p_cmd->cmd_id = (uint8_t)(idx % 64U);
}

/*!
 * @brief Time msg_codec_put_telemetry(); check each frame by decoding it.
 *
 * @param[out] p_s Results.
 */
static void
time_put_tel (op_stats_t * p_s)
{
    msg_codec_telemetry_t in;
    msg_codec_telemetry_t out;
    uint8_t               buf[MSG_TEL_BYTES];
    uint32_t              idx;
    uint32_t              t0;
    uint32_t              len;

    stats_reset(p_s);
    tel_fill(0U, &in);
    xip_flush();
    t0 = now_us();
    (void)msg_codec_put_telemetry(&in, buf, MSG_TEL_BYTES);
    p_s->first_us = now_us() - t0;

    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        tel_fill(idx, &in);
        t0 = now_us();
        len = msg_codec_put_telemetry(&in, buf, MSG_TEL_BYTES);
        stats_add(p_s, now_us() - t0);

        if ((MSG_TEL_BYTES == len)
            && msg_codec_get_telemetry(buf, MSG_TEL_BYTES, &out)
            && tel_same(&in, &out))
        {
            p_s->correct++;
        }
    }

    tel_fill(1U, &in);
    t0 = now_us();
    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        (void)msg_codec_put_telemetry(&in, buf, MSG_TEL_BYTES);
    }
    p_s->batch_us = now_us() - t0;
}

/*!
 * @brief Time msg_codec_get_telemetry(); each frame is encoded outside
 *        the timed window and the decoded fields are compared.
 *
 * @param[out] p_s Results.
 */
static void
time_get_tel (op_stats_t * p_s)
{
    msg_codec_telemetry_t in;
    msg_codec_telemetry_t out;
    uint8_t               buf[MSG_TEL_BYTES];
    uint32_t              idx;
    uint32_t              t0;
    bool                  b_ok;

    stats_reset(p_s);
    tel_fill(0U, &in);
    (void)msg_codec_put_telemetry(&in, buf, MSG_TEL_BYTES);
    xip_flush();
    t0 = now_us();
    (void)msg_codec_get_telemetry(buf, MSG_TEL_BYTES, &out);
    p_s->first_us = now_us() - t0;

    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        tel_fill(idx, &in);
        (void)msg_codec_put_telemetry(&in, buf, MSG_TEL_BYTES);
        t0 = now_us();
        b_ok = msg_codec_get_telemetry(buf, MSG_TEL_BYTES, &out);
        stats_add(p_s, now_us() - t0);

        if (b_ok && tel_same(&in, &out))
        {
            p_s->correct++;
        }
    }

    t0 = now_us();
    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        (void)msg_codec_get_telemetry(buf, MSG_TEL_BYTES, &out);
    }
    p_s->batch_us = now_us() - t0;
}

/*!
 * @brief Time msg_codec_put_command(); check each command by decoding it.
 *
 * @param[out] p_s Results.
 */
static void
time_put_cmd (op_stats_t * p_s)
{
    msg_codec_command_t in;
    msg_codec_command_t out;
    uint8_t             buf[MSG_CMD_BYTES];
    uint32_t            idx;
    uint32_t            t0;
    uint32_t            len;

    stats_reset(p_s);
    cmd_fill(0U, &in);
    xip_flush();
    t0 = now_us();
    (void)msg_codec_put_command(&in, buf, MSG_CMD_BYTES);
    p_s->first_us = now_us() - t0;

    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        cmd_fill(idx, &in);
        t0 = now_us();
        len = msg_codec_put_command(&in, buf, MSG_CMD_BYTES);
        stats_add(p_s, now_us() - t0);

        if ((MSG_CMD_BYTES == len)
            && msg_codec_get_command(buf, MSG_CMD_BYTES, &out)
            && (out.opcode == in.opcode) && (out.cmd_id == in.cmd_id))
        {
            p_s->correct++;
        }
    }

    cmd_fill(1U, &in);
    t0 = now_us();
    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        (void)msg_codec_put_command(&in, buf, MSG_CMD_BYTES);
    }
    p_s->batch_us = now_us() - t0;
}

/*!
 * @brief Time msg_codec_get_command(); each command is encoded outside
 *        the timed window and the decoded fields are compared.
 *
 * @param[out] p_s Results.
 */
static void
time_get_cmd (op_stats_t * p_s)
{
    msg_codec_command_t in;
    msg_codec_command_t out;
    uint8_t             buf[MSG_CMD_BYTES];
    uint32_t            idx;
    uint32_t            t0;
    bool                b_ok;

    stats_reset(p_s);
    cmd_fill(0U, &in);
    (void)msg_codec_put_command(&in, buf, MSG_CMD_BYTES);
    xip_flush();
    t0 = now_us();
    (void)msg_codec_get_command(buf, MSG_CMD_BYTES, &out);
    p_s->first_us = now_us() - t0;

    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        cmd_fill(idx, &in);
        (void)msg_codec_put_command(&in, buf, MSG_CMD_BYTES);
        t0 = now_us();
        b_ok = msg_codec_get_command(buf, MSG_CMD_BYTES, &out);
        stats_add(p_s, now_us() - t0);

        if (b_ok && (out.opcode == in.opcode) && (out.cmd_id == in.cmd_id))
        {
            p_s->correct++;
        }
    }

    t0 = now_us();
    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        (void)msg_codec_get_command(buf, MSG_CMD_BYTES, &out);
    }
    p_s->batch_us = now_us() - t0;
}

/*!
 * @brief Time ITERATIONS empty pairs of timer reads.
 *
 * @param[out] p_s Results (first_us, batch_us and correct are unused).
 */
static void
time_overhead (op_stats_t * p_s)
{
    uint32_t idx;
    uint32_t t0;

    stats_reset(p_s);

    for (idx = 0U; idx < ITERATIONS; idx++)
    {
        t0 = now_us();
        stats_add(p_s, now_us() - t0);
    }
}

/*!
 * @brief Run the four operations on the calling core.
 *
 * @param[out] p_stats OP_COUNT results.
 */
static void
measure_all (op_stats_t * p_stats)
{
    time_put_tel(&p_stats[OP_PUT_TEL]);
    time_get_tel(&p_stats[OP_GET_TEL]);
    time_put_cmd(&p_stats[OP_PUT_CMD]);
    time_get_cmd(&p_stats[OP_GET_CMD]);
}

/*!
 * @brief Print one operation's figures.
 *
 * Means are printed with three decimals from integer arithmetic, because
 * tm_printf has no floating point. The CPU share is in millionths of a
 * percent: telemetry at TEL_HZ uses t * TEL_HZ / 1e6 s of every second,
 * i.e. t * 200 millionths of a percent per us; a command uses t / 300 ms
 * of the NFR15 budget, i.e. t * 1000 / 3 millionths of a percent per us.
 *
 * @param[in] core Core number, 0 or 1.
 * @param[in] op   The operation.
 * @param[in] p_s  Its results.
 */
static void
print_op (uint32_t core, op_t op, const op_stats_t * p_s)
{
    /* sum_us / ITERATIONS is the mean of 32-bit times, so the mean in
     * thousandths of a microsecond fits uint32_t for any mean below about
     * 4.29 s, far above anything measured here.
     */
    const uint32_t mean_milli = (uint32_t)((p_s->sum_us * 1000U) / ITERATIONS);
    const bool     b_tel = (op <= OP_GET_TEL);
    uint32_t       cold_upct;
    uint32_t       warm_upct;

    if (b_tel)
    {
        cold_upct = p_s->first_us * (TEL_HZ * 100U);
        warm_upct = (p_s->batch_us * (TEL_HZ * 100U)) / ITERATIONS;
    }
    else
    {
        cold_upct = (p_s->first_us * 1000U) / 3U;
        warm_upct = p_s->batch_us / 3U;
    }

    tm_printf((UB *)"[S1-U08] core %u %s: first call (cold, deployed) %u us;"
              " batch mean (warm) %u.%03u us\n", core, g_op_name[op],
              p_s->first_us, p_s->batch_us / 1000U, p_s->batch_us % 1000U);
    tm_printf((UB *)"[S1-U08] core %u %s: n=%u min %u mean %u.%03u max %u us;"
              " round trips %u/%u\n", core, g_op_name[op], ITERATIONS,
              p_s->min_us, mean_milli / 1000U, mean_milli % 1000U,
              p_s->max_us, p_s->correct, ITERATIONS);
    tm_printf((UB *)"[S1-U08] core %u %s: CPU share %s deployed %u.%06u,"
              " warm %u.%06u percent\n", core, g_op_name[op],
              b_tel ? "at 2 Hz of one core:" : "of the 300 ms NFR15 budget:",
              cold_upct / 1000000U, cold_upct % 1000000U,
              warm_upct / 1000000U, warm_upct % 1000000U);

    if (b_tel)
    {
        tm_printf((UB *)"[S1-U08] core %u %s: estimate %u to %u us"
                  " (information, not a gate)\n", core, g_op_name[op],
                  ESTIMATE_LOW_US, ESTIMATE_HIGH_US);
    }
}

/*!
 * @brief Print one core's figures.
 *
 * @param[in] core       Core number, 0 or 1.
 * @param[in] p_stats    OP_COUNT results.
 * @param[in] p_overhead Timer overhead.
 */
static void
print_core (uint32_t core, const op_stats_t * p_stats,
            const op_stats_t * p_overhead)
{
    /* As in print_op(): a mean below about 4.29 s fits uint32_t. */
    const uint32_t mean_milli =
        (uint32_t)((p_overhead->sum_us * 1000U) / ITERATIONS);
    uint32_t       op;

    tm_printf((UB *)"[S1-U08] round %u, core %u, priority %d%s\n", g_round,
              core, TIMING_PRI,
              (0U == core) ? " (below the WiFi service; report only)" : "");
    tm_printf((UB *)"[S1-U08] core %u timer overhead: n=%u min %u mean"
              " %u.%03u max %u us (not subtracted)\n", core, ITERATIONS,
              p_overhead->min_us, mean_milli / 1000U, mean_milli % 1000U,
              p_overhead->max_us);

    for (op = 0U; op < (uint32_t)OP_COUNT; op++)
    {
        /* op is below OP_COUNT, so it names a valid operation. */
        print_op(core, (op_t)op, &p_stats[op]);
    }
}

/*!
 * @brief The pass criterion, on core 1's results only.
 *
 * @param[in] p_stats OP_COUNT results from core 1.
 */
static void
gate_core1 (const op_stats_t * p_stats)
{
    TEST_INIT();

    TEST_CHECK("S1-U08", ITERATIONS == p_stats[OP_PUT_TEL].correct,
               "put_telemetry: 1000 calls round-trip (core 1)");
    TEST_CHECK("S1-U08", LIMIT_US >= p_stats[OP_PUT_TEL].max_us,
               "put_telemetry: max <= 1000 us (core 1)");
    TEST_CHECK("S1-U08", ITERATIONS == p_stats[OP_GET_TEL].correct,
               "get_telemetry: 1000 calls round-trip (core 1)");
    TEST_CHECK("S1-U08", LIMIT_US >= p_stats[OP_GET_TEL].max_us,
               "get_telemetry: max <= 1000 us (core 1)");
    TEST_CHECK("S1-U08", ITERATIONS == p_stats[OP_PUT_CMD].correct,
               "put_command: 1000 calls round-trip (core 1)");
    TEST_CHECK("S1-U08", LIMIT_US >= p_stats[OP_PUT_CMD].max_us,
               "put_command: max <= 1000 us (core 1)");
    TEST_CHECK("S1-U08", ITERATIONS == p_stats[OP_GET_CMD].correct,
               "get_command: 1000 calls round-trip (core 1)");
    TEST_CHECK("S1-U08", LIMIT_US >= p_stats[OP_GET_CMD].max_us,
               "get_command: max <= 1000 us (core 1)");

    TEST_SUMMARY("s1u08");
}

/*!
 * @brief Call every public codec function once, so the whole codec is in
 *        the image, and check the results.
 *
 * @return True if every call behaved as expected.
 */
static bool
check_all_functions (void)
{
    msg_codec_telemetry_t tel;
    msg_codec_telemetry_t tel_out;
    msg_codec_heartbeat_t hb = { 42U, 1234U, -67, 3U, 1U, 0U, 2U };
    msg_codec_heartbeat_t hb_out;
    msg_codec_event_t     evt = { { 0x21U, { 0x25U, 0x30U } }, 123456U };
    msg_codec_event_t     evt_out;
    msg_codec_command_t   cmd = { MSG_OP_STOP, 5U };
    msg_codec_command_t   cmd_out;
    msg_codec_ack_t       ack = { MSG_OP_STOP, 5U, MSG_ACK_ACCEPTED };
    msg_codec_ack_t       ack_out;
    uint8_t               buf[MSG_MAX_BYTES];

    tel_fill(0U, &tel);
    tel.pitch_hdeg = msg_codec_cdeg_to_hdeg(-75);

    return ((MSG_TEL_BYTES == msg_codec_put_telemetry(&tel, buf, MSG_MAX_BYTES))
            && msg_codec_get_telemetry(buf, MSG_TEL_BYTES, &tel_out)
            && tel_same(&tel, &tel_out)
            && (MSG_HB_BYTES
                == msg_codec_put_heartbeat(&hb, buf, MSG_MAX_BYTES))
            && msg_codec_get_heartbeat(buf, MSG_HB_BYTES, &hb_out)
            && (hb_out.rssi_dbm == hb.rssi_dbm)
            && (MSG_EVT_BYTES == msg_codec_put_event(&evt, buf, MSG_MAX_BYTES))
            && msg_codec_get_event(buf, MSG_EVT_BYTES, &evt_out)
            && (evt_out.t_ms == evt.t_ms)
            && (MSG_CMD_BYTES
                == msg_codec_put_command(&cmd, buf, MSG_MAX_BYTES))
            && msg_codec_get_command(buf, MSG_CMD_BYTES, &cmd_out)
            && (cmd_out.cmd_id == cmd.cmd_id)
            && (MSG_ACK_BYTES == msg_codec_put_ack(&ack, buf, MSG_MAX_BYTES))
            && msg_codec_get_ack(buf, MSG_ACK_BYTES, &ack_out)
            && (ack_out.result == ack.result));
}

/*!
 * @brief Core 1 task: measure, print, gate, then hand over to core 0 and
 *        wait for the next round.
 *
 * @param[in] stacd Unused.
 * @param[in] exinf Unused.
 */
static void
core1_task (INT stacd, void * exinf)
{
    static op_stats_t stats[OP_COUNT];
    static op_stats_t overhead;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        g_round++;
        time_overhead(&overhead);
        measure_all(stats);
        print_core(1U, stats, &overhead);
        gate_core1(stats);

        (void)tk_wup_tsk(g_tid_core0);
        (void)tk_slp_tsk(TMO_FEVR);
    }
}

/*!
 * @brief Core 0 task: when woken, measure and print (report only), wait
 *        REPORT_PERIOD_MS, then start the next round on core 1.
 *
 * @param[in] stacd Unused.
 * @param[in] exinf Unused.
 */
static void
core0_task (INT stacd, void * exinf)
{
    static op_stats_t stats[OP_COUNT];
    static op_stats_t overhead;

    (void)stacd;
    (void)exinf;

    for (;;)
    {
        (void)tk_slp_tsk(TMO_FEVR);
        time_overhead(&overhead);
        measure_all(stats);
        print_core(0U, stats, &overhead);
        tm_printf((UB *)"[S1-U08] round %u done; next round in %d s\n\n",
                  g_round, REPORT_PERIOD_MS / 1000);

        (void)tk_dly_tsk(REPORT_PERIOD_MS);
        (void)tk_wup_tsk(g_tid_core1);
    }
}

/*** end of file ***/
