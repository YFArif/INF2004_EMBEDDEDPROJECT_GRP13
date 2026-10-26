/** @file subsystem2.c
 *
 * @brief Subsystem 2 -- kernel objects, shared data and task start-up.
 *
 * @par
 * subsystem2_init() creates every S2 kernel object and brakes both
 * motors; subsystem2_start() starts motion_task and its 20 ms cyclic
 * handler. Other code reaches the objects only through the functions
 * here (except the legacy flg_motion, see subsystem2.h).
 *
 * @par Objects
 * - mbf_motion: IF-04 commands to motion_task, 8 deep, never blocks.
 * - flg_motion: IF-05 MF_ESTOP latch, plus the private MF_TICK bit set by
 *   cyc_motion every 20 ms. Several tasks may wait on it (TA_WMUL).
 * - mtx_odom:   guards the odometry and the status block (IF-06).
 * - mtx_steer:  guards the steering correction (IF-02).
 * - cyc_motion: 20 ms cyclic handler that wakes motion_task.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_encoder.h"     /* drv_encoder_now_us(): the 1 MHz timer.    */
#include "drv_motor.h"
#include "motion_task.h"
#include "subsystem2.h"

/* A steering value older than this is ignored (10 x line_task's 10 ms). */
#define STEER_STALE_US          (100000u)

/* subsystem2_gpio_isr_register() waits this long for S2's handler. */
#define GPIO_HOOK_WAIT_MS       (2000u)

/* Private bit of flg_motion: one 20 ms period has passed. */
#define MF_TICK                 (0x80000000u)

/* The kernel stores each message with a 4-byte length header and rounds
 * the message up to 4 bytes (kernel/tkernel/messagebuf.h). */
#define MBF_HEADER_BYTES        (4u)
#define MBF_SLOT_BYTES          (MBF_HEADER_BYTES + \
                                 ((sizeof(subsystem2_cmd_t) + 3u) & ~3u))

/* motion_task's own stack, the same size the kernel would allocate
 * (TASK_STACK_SIZE plus DEFAULT_SYS_STKSZ, 128 on the Cortex-M0+). It is
 * filled with a pattern first, so the deepest use can be read back
 * (NFR11: at most 75 %). uint64_t keeps it 8-byte aligned. */
#define MOTION_STACK_WORDS      (S2_MOTION_STACK_BYTES / 8u)
#define STACK_FILL              (0xDEADBEEFDEADBEEFull)

_Static_assert(6u == sizeof(subsystem2_cmd_t),
               "IF-04 command layout changed: tell S5");
_Static_assert(0u == (S2_MOTION_STACK_BYTES % 8u),
               "stack must be 8-byte sized");
_Static_assert((TASK_STACK_SIZE + 128) == (int) S2_MOTION_STACK_BYTES,
               "keep motion_task's stack equal to the other tasks'");

/* S3's interface (app_program/subsystem3/s3_api.h). Declared weak so this
 * file builds and runs with or without S3 in the image: without it the
 * address is NULL and the steering stays at 0 (BARR-C 4.2c deviation). */
extern BOOL s3_get_steer_sp(H * p_dv_mms, UB * p_seq) __attribute__((weak));
extern void s3_set_odometry(UW (* fn)(void)) __attribute__((weak));

ID                         flg_motion   = 0;    /* Legacy extern.      */
static ID                  g_mbf_motion = 0;
static ID                  g_mtx_odom   = 0;
static ID                  g_mtx_steer  = 0;
static ID                  g_cyc_motion = 0;
static subsystem2_odom_t   g_odom       = {0};
static subsystem2_status_t g_status     = {0};
static int16_t             g_steer_mms  = 0;
static uint32_t            g_steer_us   = 0u;  /* When it was set.       */
static bool                g_steer_ever = false;
static uint8_t             g_s3_seq     = 0u;  /* Last S3 sequence seen. */
static uint32_t            g_s3_us      = 0u;  /* When it last changed.  */
static bool                g_s3_seen    = false;
static uint64_t            g_motion_stack[MOTION_STACK_WORDS];

static ID   motion_task_create(void);
static int16_t s3_steer_read(void);
static ER   cmd_send(subsystem2_cmd_type_t type, int16_t value,
                     uint16_t speed_mms);
static void motion_tick(void * p_exinf);

/*!
 * @brief Create subsystem 2's kernel objects and brake both motors
 *        (called by usermain()).
 *
 * @return E_OK, or the first kernel error.
 */
ER
subsystem2_init (void)
{
    ER     result = E_OK;
    T_CMBF cmbf   = {0};
    T_CFLG cflg   = {0};
    T_CMTX cmtx   = {0};
    T_CCYC ccyc   = {0};

    // Motors first: braked from the earliest possible moment.
    drv_motor_init();

    // NOTE: The casts are safe: a command is 6 bytes and the buffer
    // 8 x 12 bytes, far below SZ's and INT's limits.
    cmbf.mbfatr  = TA_TFIFO;
    cmbf.bufsz   = (SZ) (S2_CMD_QUEUE_LEN * MBF_SLOT_BYTES);
    cmbf.maxmsz  = (INT) sizeof(subsystem2_cmd_t);
    g_mbf_motion = tk_cre_mbf(&cmbf);

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    flg_motion  = tk_cre_flg(&cflg);

    // Priority inheritance bounds the wait when S1 / S3 / S4 / S5 read
    // the odometry while motion_task writes it (Week 6 report section 3).
    cmtx.mtxatr = TA_INHERIT;
    g_mtx_odom  = tk_cre_mtx(&cmtx);
    g_mtx_steer = tk_cre_mtx(&cmtx);

    // Created stopped; started in subsystem2_start() once motion_task
    // exists. NOTE: The cast is safe: motion_tick has the kernel's cyclic
    // handler signature, void (void *), which FP stands for.
    ccyc.cycatr  = TA_HLNG;
    ccyc.cychdr  = (FP) motion_tick;
    ccyc.cyctim  = S2_MOTION_PERIOD_MS;
    ccyc.cycphs  = S2_MOTION_PERIOD_MS;
    g_cyc_motion = tk_cre_cyc(&ccyc);

    if (g_mbf_motion < E_OK)
    {
        result = g_mbf_motion;
    }
    else if (flg_motion < E_OK)
    {
        result = flg_motion;
    }
    else if (g_mtx_odom < E_OK)
    {
        result = g_mtx_odom;
    }
    else if (g_mtx_steer < E_OK)
    {
        result = g_mtx_steer;
    }
    else if (g_cyc_motion < E_OK)
    {
        result = g_cyc_motion;
    }
    else
    {
        result = producer_init();   /* Template example.               */
    }

    return (result);
}

/*!
 * @brief Start subsystem 2's tasks (called by usermain()).
 *
 * @return E_OK, or the first kernel error.
 *
 * NOTE: motion_task is pinned to core 1 with the rest of the control
 * loop, so network bursts on core 0 cannot delay it (Week 6 report 3).
 */
ER
subsystem2_start (void)
{
    ER result = E_OK;
    ID tid    = motion_task_create();

    if (tid < E_OK)
    {
        result = tid;
    }
    else
    {
        result = tk_sta_cyc(g_cyc_motion);
    }

    if (E_OK == result)
    {
        result = producer_start();  /* Template example.               */
    }

    // S3's line task measures distance moved with this (IF-S3d).
    if (NULL != s3_set_odometry)
    {
        s3_set_odometry(subsystem2_odom_total_mm);
    }

    return (result);
}

/*!
 * @brief Queue one command for motion_task (IF-04). Never blocks.
 *
 * @param[in] p_cmd The command; copied, so it may be reused at once.
 *
 * @return E_OK, or E_TMOUT if S2_CMD_QUEUE_LEN commands are waiting.
 */
ER
subsystem2_motion_send (subsystem2_cmd_t const * p_cmd)
{
    // NOTE: The cast is safe: a command is 6 bytes.
    return (tk_snd_mbf(g_mbf_motion, p_cmd, (INT) sizeof(*p_cmd),
                       TMO_POL));
}

/*!
 * @brief Brake now and cancel any MOVE / TURN / FOLLOW (IF-04).
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
ER
subsystem2_stop (void)
{
    return (cmd_send(S2_CMD_STOP, 0, 0u));
}

/*!
 * @brief Drive at a speed, steered by line_task (IF-04). Sent again while
 *        following, it only changes the speed.
 *
 * @param[in] speed_mms Robot speed; 0 = S2_SPEED_CRUISE_MMS.
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
ER
subsystem2_follow (uint16_t speed_mms)
{
    return (cmd_send(S2_CMD_FOLLOW, 0, speed_mms));
}

/*!
 * @brief Drive straight by encoder distance; EV_MOTION_DONE at the end.
 *
 * @param[in] dist_mm   Negative = reverse.
 * @param[in] speed_mms 0 = S2_SPEED_CRUISE_MMS.
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
ER
subsystem2_move (int16_t dist_mm, uint16_t speed_mms)
{
    return (cmd_send(S2_CMD_MOVE, dist_mm, speed_mms));
}

/*!
 * @brief Turn on the spot by encoder arc; EV_MOTION_DONE at the end.
 *
 * @param[in] angle_deg Positive = left; clipped to +/- 720.
 * @param[in] speed_mms Wheel speed; 0 = S2_SPEED_TURN_MMS.
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
ER
subsystem2_turn (int16_t angle_deg, uint16_t speed_mms)
{
    return (cmd_send(S2_CMD_TURN, angle_deg, speed_mms));
}

/*!
 * @brief Leave the E-stop. Call subsystem2_estop_clear() first: a CLEAR
 *        that arrives while MF_ESTOP is still set is discarded.
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
ER
subsystem2_clear (void)
{
    return (cmd_send(S2_CMD_CLEAR, 0, 0u));
}

/*!
 * @brief Deepest use of motion_task's stack so far (NFR11).
 *
 * @return Bytes used, out of S2_MOTION_STACK_BYTES.
 */
uint32_t
subsystem2_stack_used (void)
{
    uint32_t untouched = 0u;

    // The stack grows down: count the words at the bottom that still hold
    // the fill pattern.
    while ((untouched < MOTION_STACK_WORDS) &&
           (STACK_FILL == g_motion_stack[untouched]))
    {
        untouched++;
    }

    return ((MOTION_STACK_WORDS - untouched) * 8u);
}

/*!
 * @brief Set the emergency-stop latch (IF-05). motion_task brakes within
 *        a fraction of a millisecond and stays braked until CLEAR.
 */
void
subsystem2_estop_set (void)
{
    CHECK(tk_set_flg(flg_motion, MF_ESTOP), "flg_motion");
}

/*!
 * @brief Clear the emergency-stop latch. Only vehicle_ctrl_task (S5) and
 *        the S2 tests call this; motion_task stays braked until it also
 *        receives S2_CMD_CLEAR.
 */
void
subsystem2_estop_clear (void)
{
    CHECK(tk_clr_flg(flg_motion, ~MF_ESTOP), "flg_motion");
}

/*!
 * @brief Read the emergency-stop latch.
 *
 * @return true while MF_ESTOP is set.
 */
bool
subsystem2_estop_is_set (void)
{
    T_RFLG rflg = {0};

    CHECK(tk_ref_flg(flg_motion, &rflg), "flg_motion");

    return (0u != (rflg.flgptn & MF_ESTOP));
}

/*!
 * @brief Copy the latest odometry (IF-06).
 *
 * @param[out] p_odom The odometry.
 */
void
subsystem2_odom_get (subsystem2_odom_t * p_odom)
{
    CHECK(tk_loc_mtx(g_mtx_odom, TMO_FEVR), "mtx_odom");
    *p_odom = g_odom;
    CHECK(tk_unl_mtx(g_mtx_odom), "mtx_odom");
}

/*!
 * @brief Copy motion_task's latest status.
 *
 * @param[out] p_status The status.
 */
void
subsystem2_status_get (subsystem2_status_t * p_status)
{
    CHECK(tk_loc_mtx(g_mtx_odom, TMO_FEVR), "mtx_odom");
    *p_status = g_status;
    CHECK(tk_unl_mtx(g_mtx_odom), "mtx_odom");
}

/*!
 * @brief Set the steering correction (IF-02); the latest value wins.
 *
 * @param[in] dv_mms Positive turns left: the left wheel runs at
 *                   speed - dv and the right wheel at speed + dv.
 */
void
subsystem2_steer_set (int16_t dv_mms)
{
    CHECK(tk_loc_mtx(g_mtx_steer, TMO_FEVR), "mtx_steer");
    g_steer_mms  = dv_mms;
    g_steer_us   = drv_encoder_now_us();
    g_steer_ever = true;
    CHECK(tk_unl_mtx(g_mtx_steer), "mtx_steer");
}

/*!
 * @brief Legacy name of subsystem2_odom_get(), still used by S5.
 *
 * @param[out] p_odom The odometry.
 */
void
s2_odom_get (subsystem2_odom_t * p_odom)
{
    subsystem2_odom_get(p_odom);
}

/*!
 * @brief Wait for the next 20 ms period (motion_task only).
 *
 * @param[in] b_estop_wakes true: MF_ESTOP also ends the wait at once.
 *                          Pass false while already braked for it, since
 *                          the latch stays set.
 */
void
subsystem2_cycle_wait (bool b_estop_wakes)
{
    UINT pattern = 0u;
    UINT wait    = MF_TICK;

    if (b_estop_wakes)
    {
        wait |= MF_ESTOP;
    }

    // TWF_BITCLR would also clear MF_ESTOP, which only S5 may clear, so
    // only the tick bit is cleared, explicitly.
    CHECK(tk_wai_flg(flg_motion, wait, TWF_ORW, &pattern, TMO_FEVR),
          "flg_motion");
    CHECK(tk_clr_flg(flg_motion, ~MF_TICK), "flg_motion");
}

/*!
 * @brief Take the next waiting command, if any (motion_task only).
 *
 * @param[out] p_cmd The command.
 *
 * @return true if a command was taken.
 */
bool
subsystem2_cmd_receive (subsystem2_cmd_t * p_cmd)
{
    INT size = tk_rcv_mbf(g_mbf_motion, p_cmd, TMO_POL);

    // NOTE: The cast is safe: a command is 6 bytes.
    return ((INT) sizeof(*p_cmd) == size);
}

/*!
 * @brief Publish odometry and status together (motion_task only).
 *
 * @param[in] p_odom   The odometry.
 * @param[in] p_status The status.
 */
void
subsystem2_odom_put (subsystem2_odom_t const * p_odom,
                     subsystem2_status_t const * p_status)
{
    CHECK(tk_loc_mtx(g_mtx_odom, TMO_FEVR), "mtx_odom");
    g_odom   = *p_odom;
    g_status = *p_status;
    CHECK(tk_unl_mtx(g_mtx_odom), "mtx_odom");
}

/*!
 * @brief Read the steering correction (motion_task only).
 *
 * @return mm/s; positive turns left. 0 if line_task has not set it within
 *         the last STEER_STALE_US, so a stopped line_task never leaves
 *         the robot steering on an old value.
 */
int16_t
subsystem2_steer_get (void)
{
    int16_t dv_mms = 0;

    bool    b_set  = false;

    CHECK(tk_loc_mtx(g_mtx_steer, TMO_FEVR), "mtx_steer");
    if (g_steer_ever &&
        ((drv_encoder_now_us() - g_steer_us) <= STEER_STALE_US))
    {
        dv_mms = g_steer_mms;
        b_set  = true;
    }
    CHECK(tk_unl_mtx(g_mtx_steer), "mtx_steer");

    // A fresh subsystem2_steer_set() (the S2 tests) wins; otherwise S3.
    if (!b_set)
    {
        dv_mms = s3_steer_read();
    }

    return (dv_mms);
}

/*!
 * @brief Read S3's steering value (IF-02) and convert it to S2's sign.
 *
 * @return mm/s, positive = left (S2's convention). 0 if S3 is not in the
 *         image, is not steering (uncalibrated, disabled, searching), or
 *         its sequence number has not changed for STEER_STALE_US.
 *
 * NOTE: S3 reports dv with positive = turn RIGHT; S2's positive is left,
 * so the sign flips. S2_S3_STEER_PERCENT scales it to the per-wheel offset
 * (left = speed + offset, right = speed - offset for a right turn).
 * Called only by motion_task.
 */
static int16_t
s3_steer_read (void)
{
    H       dv_mms = 0;
    UB      seq    = 0;
    int32_t out    = 0;
    uint32_t now   = drv_encoder_now_us();

    if ((NULL != s3_get_steer_sp) && s3_get_steer_sp(&dv_mms, &seq))
    {
        if ((!g_s3_seen) || (seq != g_s3_seq))
        {
            g_s3_seen = true;
            g_s3_seq  = (uint8_t) seq;
            g_s3_us   = now;
        }

        if ((now - g_s3_us) <= STEER_STALE_US)
        {
            out = -(((int32_t) dv_mms * S2_S3_STEER_PERCENT) / 100);
        }
    }

    // NOTE: The cast is safe: S3's value is 10-bit (+/-511) and the
    // percentage is at most a few hundred.
    return ((int16_t) out);
}

/*!
 * @brief Total distance of the robot centre, for S3's line task.
 *
 * @return Millimetres since power-up as a 32-bit value that wraps; S3
 *         uses differences. Negative distances (reversing) wrap upward.
 */
uint32_t
subsystem2_odom_total_mm (void)
{
    subsystem2_odom_t odom = {0};

    subsystem2_odom_get(&odom);

    // NOTE: The cast is safe: modular arithmetic on the distance is the
    // intent (S3 differences it).
    return ((uint32_t) odom.dist_mm);
}

/*!
 * @brief Attach a handler to the shared IO_IRQ_BANK0 interrupt (IF-11).
 *
 * @param[in] p_hook Handler, interrupt context, called with the interrupt
 *                   number after the encoder edges. It must clear the
 *                   pins it owns itself.
 *
 * @return E_OK once S2's handler is installed (so it is now safe to enable
 *         the caller's own pin interrupts), or E_TMOUT after 2 s.
 *
 * NOTE: Task context. Only one handler can own an interrupt number, so a
 * subsystem must NOT call tk_def_int() for IO_IRQ_BANK0 (13) itself: that
 * would silently replace this one and leave the other's edges uncleared,
 * and the interrupt would fire forever.
 */
ER
subsystem2_gpio_isr_register (void (* p_hook)(UINT intno))
{
    uint32_t waited = 0u;

    drv_encoder_hook_set(p_hook);

    while ((!drv_encoder_isr_ready()) && (waited < GPIO_HOOK_WAIT_MS))
    {
        CHECK(tk_dly_tsk(10), "tk_dly_tsk");
        waited += 10u;
    }

    return (drv_encoder_isr_ready() ? E_OK : E_TMOUT);
}

/*!
 * @brief Create and start motion_task on core 1, on its own pre-filled
 *        stack (S2_MOTION_STACK_BYTES).
 *
 * @return The task ID, or the kernel error.
 */
static ID
motion_task_create (void)
{
    T_CTSK   ctsk   = {0};
    ID       tid    = 0;
    ER       result = E_OK;
    uint32_t idx    = 0u;

    for (idx = 0u; idx < MOTION_STACK_WORDS; idx++)
    {
        g_motion_stack[idx] = STACK_FILL;
    }

    // NOTE: The casts are safe: motion_task has the kernel's task entry
    // signature, which FP stands for, and the stack is 2176 bytes.
    ctsk.tskatr  = TA_HLNG | TA_RNG3 | TA_USERBUF;
    ctsk.task    = (FP) motion_task;
    ctsk.itskpri = S2_PRI_MOTION;
    ctsk.stksz   = (SZ) sizeof(g_motion_stack);
    ctsk.bufptr  = g_motion_stack;
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;
    ctsk.assprc  = TP_PRC2;
#endif

    tid = tk_cre_tsk(&ctsk);

    if (tid >= E_OK)
    {
        result = tk_sta_tsk(tid, 0);
        if (E_OK != result)
        {
            tid = result;
        }
    }

    CHECK(tid, "motion_task");

    return (tid);
}

/*!
 * @brief Build and queue one command.
 *
 * @param[in] type      Command type.
 * @param[in] value     mm or degrees.
 * @param[in] speed_mms 0 = default.
 *
 * @return E_OK, or E_TMOUT if the command queue is full.
 */
static ER
cmd_send (subsystem2_cmd_type_t type, int16_t value, uint16_t speed_mms)
{
    // NOTE: The cast is safe: command types are 0 ... 4.
    subsystem2_cmd_t const cmd =
    {
        .type      = (uint8_t) type,
        .value     = value,
        .speed_mms = speed_mms
    };

    return (subsystem2_motion_send(&cmd));
}

/*!
 * @brief cyc_motion handler: start the next 20 ms cycle. Runs in the
 *        timer interrupt, so it only sets a flag bit.
 *
 * @param[in] p_exinf Extended information (unused).
 */
static void
motion_tick (void * p_exinf)
{
    (void) p_exinf;
    (void) tk_set_flg(flg_motion, MF_TICK);
}

/*** end of file ***/
