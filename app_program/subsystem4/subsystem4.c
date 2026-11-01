/** @file subsystem4.c
 *
 * @brief Subsystem 4 -- creates its kernel objects, starts imu_task and
 *        provides the public getters (see subsystem4.h).
 *
 * @par Kernel objects
 * flg_imu (S4_EV_TICK from cyc_imu every 20 ms, S4_EV_CAL on request),
 * cyc_imu, mtx_terrain (TA_INHERIT; guards the packed terrain and the
 * status), plus the template's mbf_summary and flg_events.
 */

#include <stdbool.h>
#include <stdint.h>

#include "s4_terrain.h"
#include "subsystem4.h"

EXPORT ID mbf_summary;      /* Template example, see subsystem4.h.       */
EXPORT ID flg_events;

static ID g_flg_imu     = E_NOEXS;
static ID g_cyc_imu     = E_NOEXS;
static ID g_mtx_terrain = E_NOEXS;

static s4_terrain_packed_t g_terrain;   /* Written by imu_task only.     */
static subsystem4_status_t g_status;    /* Written by imu_task only.     */

static void imu_tick(void * p_exinf);

/*!
 * @brief Create subsystem 4's kernel objects.
 *
 * @return E_OK, or the first kernel error.
 */
ER
subsystem4_init (void)
{
    T_CFLG cflg   = {0};
    T_CMBF cmbf   = {0};
    T_CMTX cmtx   = {0};
    T_CCYC ccyc   = {0};
    ER     result = E_OK;

    cflg.flgatr = TA_TFIFO | TA_WMUL;
    flg_events  = tk_cre_flg(&cflg);
    g_flg_imu   = tk_cre_flg(&cflg);

    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) (4 * S4_SUMMARY_MAX);
    cmbf.maxmsz = (INT) S4_SUMMARY_MAX;
    mbf_summary = tk_cre_mbf(&cmbf);

    // Priority inheritance bounds the wait of S1 and S5 when they read
    // the terrain while imu_task writes it.
    cmtx.mtxatr   = TA_INHERIT;
    g_mtx_terrain = tk_cre_mtx(&cmtx);

    // NOTE: The cast is safe: imu_tick has the cyclic handler signature.
    ccyc.cycatr = TA_HLNG;
    ccyc.cychdr = (FP) imu_tick;
    ccyc.cyctim = S4_IMU_PERIOD_MS;
    ccyc.cycphs = S4_IMU_PERIOD_MS;
    g_cyc_imu   = tk_cre_cyc(&ccyc);

    if (flg_events < E_OK)
    {
        result = flg_events;
    }
    else if (g_flg_imu < E_OK)
    {
        result = g_flg_imu;
    }
    else if (mbf_summary < E_OK)
    {
        result = mbf_summary;
    }
    else if (g_mtx_terrain < E_OK)
    {
        result = g_mtx_terrain;
    }
    else if (g_cyc_imu < E_OK)
    {
        result = g_cyc_imu;
    }
    else
    {
        // Every object exists.
    }

    return (result);
}

/*!
 * @brief Start imu_task (core 0), then its 20 ms cyclic handler.
 *
 * @return E_OK, or the kernel error.
 */
ER
subsystem4_start (void)
{
    ER result = E_OK;
    ID tid    = app_start_task((FP) imu_task, S4_PRI_IMU, TP_PRC1,
                               "imu_task");

    if (tid < E_OK)
    {
        result = tid;
    }
    else
    {
        result = tk_sta_cyc(g_cyc_imu);
    }

    return ((result < E_OK) ? result : E_OK);
}

/*!
 * @brief Copy the terrain data (IF-08). Locks, copies 6 bytes, unlocks,
 *        and only then unpacks, so the lock is held for well under 1 us.
 *
 * @param[out] p_terrain The unpacked terrain data.
 */
void
subsystem4_terrain_get (subsystem4_terrain_t * p_terrain)
{
    s4_terrain_packed_t copy = {0u, 0u};

    CHECK(tk_loc_mtx(g_mtx_terrain, TMO_FEVR), "mtx_terrain");
    copy = g_terrain;
    CHECK(tk_unl_mtx(g_mtx_terrain), "mtx_terrain");

    s4_terrain_unpack(&copy, p_terrain);
}

/*!
 * @brief Tell whether the IMU calibration is done (S5 CALIBRATING).
 *
 * @return true once the offsets are measured.
 */
bool
subsystem4_cal_done (void)
{
    subsystem4_terrain_t terrain;

    subsystem4_terrain_get(&terrain);

    return (terrain.b_cal_done);
}

/*!
 * @brief Ask imu_task to measure the offsets again. The robot must stand
 *        still and level for about 2 s; subsystem4_cal_done() is false
 *        until it has finished.
 */
void
subsystem4_calibrate (void)
{
    CHECK(tk_set_flg(g_flg_imu, S4_EV_CAL), "flg_imu");
}

/*!
 * @brief Copy imu_task's health and timing.
 *
 * @param[out] p_status The status.
 */
void
subsystem4_status_get (subsystem4_status_t * p_status)
{
    CHECK(tk_loc_mtx(g_mtx_terrain, TMO_FEVR), "mtx_terrain");
    *p_status = g_status;
    CHECK(tk_unl_mtx(g_mtx_terrain), "mtx_terrain");
}

/*!
 * @brief imu_task only: wait for the next tick or a request.
 *
 * @return The S4_EV_xxx bits that were set (they are cleared).
 */
uint32_t
subsystem4_wait (void)
{
    UINT flgptn = 0u;
    ER   er     = tk_wai_flg(g_flg_imu, S4_EV_TICK | S4_EV_CAL,
                             TWF_ORW | TWF_BITCLR, &flgptn, TMO_FEVR);

    return ((E_OK == er) ? (uint32_t) flgptn : 0u);
}

/*!
 * @brief imu_task only: pack and store the terrain data.
 *
 * @param[in] p_terrain The new values; each is clamped to its field.
 */
void
subsystem4_terrain_put (subsystem4_terrain_t const * p_terrain)
{
    s4_terrain_packed_t packed = {0u, 0u};

    // Pack outside the lock; inside it only the 6 bytes are copied.
    s4_terrain_pack(p_terrain, &packed);

    CHECK(tk_loc_mtx(g_mtx_terrain, TMO_FEVR), "mtx_terrain");
    g_terrain = packed;
    CHECK(tk_unl_mtx(g_mtx_terrain), "mtx_terrain");
}

/*!
 * @brief imu_task only: store its health and timing.
 *
 * @param[in] p_status The status.
 */
void
subsystem4_status_put (subsystem4_status_t const * p_status)
{
    CHECK(tk_loc_mtx(g_mtx_terrain, TMO_FEVR), "mtx_terrain");
    g_status = *p_status;
    CHECK(tk_unl_mtx(g_mtx_terrain), "mtx_terrain");
}

/*!
 * @brief Cyclic handler, every 20 ms: wake imu_task.
 *
 * @param[in] p_exinf Unused.
 */
static void
imu_tick (void * p_exinf)
{
    (void) p_exinf;
    (void) tk_set_flg(g_flg_imu, S4_EV_TICK);
}

/*** end of file ***/
