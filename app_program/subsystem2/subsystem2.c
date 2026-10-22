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

#include "drv_motor.h"
#include "motion_task.h"
#include "subsystem2.h"

/* Private bit of flg_motion: one 20 ms period has passed. */
#define MF_TICK                 (0x80000000u)

/* The kernel stores each message with a 4-byte length header and rounds
 * the message up to 4 bytes (kernel/tkernel/messagebuf.h). */
#define MBF_HEADER_BYTES        (4u)
#define MBF_SLOT_BYTES          (MBF_HEADER_BYTES + \
                                 ((sizeof(subsystem2_cmd_t) + 3u) & ~3u))

ID                         flg_motion   = 0;    /* Legacy extern.      */
static ID                  g_mbf_motion = 0;
static ID                  g_mtx_odom   = 0;
static ID                  g_mtx_steer  = 0;
static ID                  g_cyc_motion = 0;
static subsystem2_odom_t   g_odom       = {0};
static subsystem2_status_t g_status     = {0};
static int16_t             g_steer_mms  = 0;

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
    ID tid    = app_start_task(motion_task, S2_PRI_MOTION, TP_PRC2,
                               "motion_task");

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
    g_steer_mms = dv_mms;
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
 * @return mm/s; positive turns left.
 */
int16_t
subsystem2_steer_get (void)
{
    int16_t dv_mms = 0;

    CHECK(tk_loc_mtx(g_mtx_steer, TMO_FEVR), "mtx_steer");
    dv_mms = g_steer_mms;
    CHECK(tk_unl_mtx(g_mtx_steer), "mtx_steer");

    return (dv_mms);
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
