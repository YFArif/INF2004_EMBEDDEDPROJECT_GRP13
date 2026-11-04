/** @file subsystem5.c
 *
 * @brief Subsystem 5 -- kernel objects, shared data and task start-up.
 *
 * @par
 * subsystem5_init() creates every S5 kernel object; subsystem5_start()
 * starts the tasks. The object IDs stay in this file: other code reaches
 * them only through the functions here, so the objects cannot be misused
 * (for example, another task waiting on flg_obst).
 *
 * @par Objects
 * - flg_mission: events for vehicle_ctrl_task (IF-03), one waiter.
 * - flg_fault:   latched fault causes, read by S1 for telemetry.
 * - flg_obst:    requests to obstacle_task, one waiter.
 * - mtx_obst:    guards the obstacle profile, the live status and the
 *                controller state (IF-09).
 * - mbf_nav:     barcode commands from S3 (IF-01).
 * - mbf_cmd:     dashboard commands from S1.
 */

#include <stdbool.h>
#include <stdint.h>

#include "obstacle_task.h"
#include "s5_profile.h"
#include "subsystem5.h"
#include "vehicle_ctrl_task.h"
#include "../subsystem3/s3_config.h"

/* Message buffer sizing: each message takes a 4-byte header plus its
 * size rounded up to 4 bytes (kernel messagebuf.h).
 */
#define MBF_HEADER_BYTES    (4u)
#define MBF_ROUND_BYTES     (4u)
#define NAV_QUEUE_LEN       (4u)
#define CMD_QUEUE_LEN       (8u)
#define CMD_BYTES           (2u)        /* {opcode, cmd_id}.             */

static ID                       g_flg_mission = 0;
static ID                       g_flg_fault   = 0;
static ID                       g_flg_obst    = 0;
static ID                       g_mtx_obst    = 0;
static ID                       g_mbf_nav     = 0;
static ID                       g_mbf_cmd     = 0;
static s5_profile_t             g_profile     = {0};
static subsystem5_obst_status_t g_obst_status = {0};
static subsystem5_ctrl_state_t  g_ctrl_state  = S5_CTRL_INIT;

static ID create_flag(void);
static ID create_buffer(uint32_t msg_bytes, uint32_t count);

/*!
 * @brief Create subsystem 5's kernel objects (called by usermain()).
 *
 * @return E_OK, or the first kernel error.
 */
ER
subsystem5_init (void)
{
    ER     result = E_OK;
    T_CMTX cmtx   = {0};

    g_flg_mission = create_flag();
    g_flg_fault   = create_flag();
    g_flg_obst    = create_flag();

    // Priority inheritance bounds the wait when telemetry reads the
    // profile while obstacle_task writes it (Week 6 report 8.3).
    cmtx.mtxatr = TA_INHERIT;
    g_mtx_obst  = tk_cre_mtx(&cmtx);

    g_mbf_nav = create_buffer(sizeof(subsystem5_nav_msg_t), NAV_QUEUE_LEN);
    g_mbf_cmd = create_buffer(CMD_BYTES, CMD_QUEUE_LEN);

    if (g_flg_mission < E_OK)
    {
        result = g_flg_mission;
    }
    else if (g_flg_fault < E_OK)
    {
        result = g_flg_fault;
    }
    else if (g_flg_obst < E_OK)
    {
        result = g_flg_obst;
    }
    else if (g_mtx_obst < E_OK)
    {
        result = g_mtx_obst;
    }
    else if (g_mbf_nav < E_OK)
    {
        result = g_mbf_nav;
    }
    else if (g_mbf_cmd < E_OK)
    {
        result = g_mbf_cmd;
    }
    else
    {
        result = E_OK;
    }

    return (result);
}

/*!
 * @brief Start subsystem 5's tasks (called by usermain()).
 *
 * @return E_OK, or the kernel error from creating a task.
 *
 * NOTE: obstacle_task runs on core 0 with the WiFi task; vehicle_ctrl_task
 * runs on core 1 with the control loop (Week 6 report section 3).
 */
ER
subsystem5_start (void)
{
#if ((S3_EXPERIMENT_MODE != 0) || (S3_DRIVE_TEST != 0))
    tm_printf((UB *) "S5 tasks DISABLED: isolated S3 test owns motors/buttons; "
                     "no ultrasonic protection.\n");
    return E_OK;
#else
    ER result  = E_OK;
    ID tid_obs = app_start_task(obstacle_task, S5_PRI_OBSTACLE, TP_PRC1,
                                "obstacle_task");
    ID tid_veh = app_start_task(vehicle_ctrl_task, S5_PRI_VEHICLE, TP_PRC2,
                                "vehicle_ctrl_task");

    if (tid_obs < E_OK)
    {
        result = tid_obs;
    }
    else if (tid_veh < E_OK)
    {
        result = tid_veh;
    }
    else
    {
        result = E_OK;
    }

    return (result);
#endif
}

/*!
 * @brief Queue a barcode command for vehicle_ctrl_task (IF-01, S3) and
 *        raise EV_BARCODE.
 *
 * @param[in] p_msg The decoded command; copied, so it may be reused.
 *
 * @return E_OK, or E_TMOUT if NAV_QUEUE_LEN commands are already waiting.
 */
ER
subsystem5_nav_post (subsystem5_nav_msg_t const * p_msg)
{
    // NOTE: The cast is safe: the message is a few bytes.
    ER result = tk_snd_mbf(g_mbf_nav, p_msg, (INT) sizeof(*p_msg), TMO_POL);

    if (E_OK == result)
    {
        subsystem5_mission_signal(EV_BARCODE);
    }

    return (result);
}

/*!
 * @brief Take the oldest queued barcode command (vehicle_ctrl_task only).
 *
 * @param[out] p_msg The command.
 *
 * @return true if one was waiting.
 */
bool
subsystem5_nav_take (subsystem5_nav_msg_t * p_msg)
{
    // NOTE: The cast is safe: the message is a few bytes.
    return ((INT) sizeof(*p_msg) == tk_rcv_mbf(g_mbf_nav, p_msg, TMO_POL));
}

/*!
 * @brief Queue a dashboard command for vehicle_ctrl_task (S1) and raise
 *        EV_COMMAND. vehicle_ctrl_task acknowledges it.
 *
 * @param[in] opcode S5_CMD_START, S5_CMD_STOP or S5_CMD_RESET.
 * @param[in] cmd_id The dashboard's command number, echoed in the ack.
 *
 * @return E_OK, or E_TMOUT if CMD_QUEUE_LEN commands are already waiting.
 */
ER
subsystem5_command_post (uint8_t opcode, uint8_t cmd_id)
{
    uint8_t record[CMD_BYTES] = {opcode, cmd_id};

    // NOTE: The cast is safe: the record is 2 bytes.
    ER result = tk_snd_mbf(g_mbf_cmd, record, (INT) CMD_BYTES, TMO_POL);

    if (E_OK == result)
    {
        subsystem5_mission_signal(EV_COMMAND);
    }

    return (result);
}

/*!
 * @brief Take the oldest queued dashboard command (vehicle_ctrl_task
 *        only).
 *
 * @param[out] p_opcode The command.
 * @param[out] p_cmd_id Its number.
 *
 * @return true if one was waiting.
 */
bool
subsystem5_command_take (uint8_t * p_opcode, uint8_t * p_cmd_id)
{
    uint8_t record[CMD_BYTES] = {0u, 0u};

    // NOTE: The cast is safe: the record is 2 bytes.
    bool b_got = ((INT) CMD_BYTES == tk_rcv_mbf(g_mbf_cmd, record, TMO_POL));

    *p_opcode = record[0];
    *p_cmd_id = record[1];

    return (b_got);
}

/*!
 * @brief The controller's current state, e.g. for telemetry (S1 field
 *        ctrl_state).
 *
 * @return The state.
 */
subsystem5_ctrl_state_t
subsystem5_ctrl_state_get (void)
{
    subsystem5_ctrl_state_t state = S5_CTRL_INIT;

    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        state = g_ctrl_state;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }

    return (state);
}

/*!
 * @brief Publish the controller's state (vehicle_ctrl_task only).
 *
 * @param[in] state The new state.
 */
void
subsystem5_ctrl_state_put (subsystem5_ctrl_state_t state)
{
    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        g_ctrl_state = state;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }
}

/*!
 * @brief Milliseconds since power-up, from the kernel clock.
 *
 * @return Time in ms; wraps after 49 days. Differences of two readings
 *         are correct across the wrap when taken as uint32_t.
 */
uint32_t
subsystem5_now_ms (void)
{
    SYSTIM now = {0, 0u};

    CHECK(tk_get_otm(&now), "tk_get_otm");

    return (now.lo);
}

/*!
 * @brief Raise events for vehicle_ctrl_task (IF-03).
 *
 * @param[in] events One or more EV_xxx bits.
 */
void
subsystem5_mission_signal (uint32_t events)
{
    CHECK(tk_set_flg(g_flg_mission, events), "flg_mission");
}

/*!
 * @brief Wait for any of the given mission events and clear them.
 *
 * @param[in]  events     EV_xxx bits to wait for.
 * @param[in]  timeout_ms Longest wait in ms.
 * @param[out] p_events   The events that were set.
 *
 * @return true if an event arrived before the timeout.
 *
 * NOTE: flg_mission allows one waiter: vehicle_ctrl_task, or the S5
 * tests while vehicle_ctrl_task does not exist yet.
 */
bool
subsystem5_mission_wait (uint32_t events, uint32_t timeout_ms,
                         uint32_t * p_events)
{
    UINT pattern = 0u;

    // NOTE: The cast is safe: timeouts in S5 are a few seconds, far
    // below TMO's maximum.
    ER result = tk_wai_flg(g_flg_mission, events, TWF_ORW | TWF_BITCLR,
                           &pattern, (TMO) timeout_ms);

    *p_events = pattern & events;

    return (E_OK == result);
}

/*!
 * @brief Latch one or more fault causes (S5_FAULT_xxx).
 *
 * @param[in] faults Fault bits to set.
 */
void
subsystem5_fault_set (uint32_t faults)
{
    CHECK(tk_set_flg(g_flg_fault, faults), "flg_fault");
}

/*!
 * @brief Clear fault causes; only vehicle_ctrl_task does this.
 *
 * @param[in] faults Fault bits to clear.
 */
void
subsystem5_fault_clear (uint32_t faults)
{
    CHECK(tk_clr_flg(g_flg_fault, ~faults), "flg_fault");
}

/*!
 * @brief Read the latched fault causes, e.g. for telemetry.
 *
 * @return S5_FAULT_xxx bits.
 */
uint32_t
subsystem5_fault_get (void)
{
    T_RFLG   rflg   = {0};
    uint32_t faults = 0u;

    if (E_OK == tk_ref_flg(g_flg_fault, &rflg))
    {
        faults = rflg.flgptn;
    }

    return (faults);
}

/*!
 * @brief Ask obstacle_task to do something (OB_xxx).
 *
 * @param[in] requests One OB_xxx bit.
 */
void
subsystem5_obst_request (uint32_t requests)
{
    CHECK(tk_set_flg(g_flg_obst, requests & OB_ALL_REQUESTS), "flg_obst");
}

/*!
 * @brief Take the pending requests (obstacle_task only).
 *
 * @param[in] b_wait true to block until a request arrives, false to
 *                   return at once.
 *
 * @return The OB_xxx bits taken, 0 if none.
 */
uint32_t
subsystem5_obst_take (bool b_wait)
{
    UINT     pattern  = 0u;
    uint32_t requests = 0u;
    ER       result   = tk_wai_flg(g_flg_obst, OB_ALL_REQUESTS,
                                   TWF_ORW | TWF_BITCLR, &pattern,
                                   b_wait ? TMO_FEVR : TMO_POL);

    if (E_OK == result)
    {
        requests = pattern & OB_ALL_REQUESTS;
    }

    return (requests);
}

/*!
 * @brief Tell whether a request is pending, without taking it.
 *
 * @param[in] requests OB_xxx bits to look for.
 *
 * @return true if any of them is set.
 */
bool
subsystem5_obst_pending (uint32_t requests)
{
    T_RFLG rflg      = {0};
    bool   b_pending = false;

    if (E_OK == tk_ref_flg(g_flg_obst, &rflg))
    {
        b_pending = (0u != (rflg.flgptn & requests));
    }

    return (b_pending);
}

/*!
 * @brief Copy out the latest obstacle profile (IF-09).
 *
 * @param[out] p_profile The profile; b_valid is false before the first
 *                       scan.
 */
void
subsystem5_profile_get (s5_profile_t * p_profile)
{
    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        *p_profile = g_profile;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }
}

/*!
 * @brief Publish a new obstacle profile (obstacle_task only).
 *
 * @param[in] p_profile The profile.
 */
void
subsystem5_profile_put (s5_profile_t const * p_profile)
{
    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        g_profile = *p_profile;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }
}

/*!
 * @brief Copy out the live ultrasonic status.
 *
 * @param[out] p_status Latest state, range and servo angle.
 */
void
subsystem5_obst_status_get (subsystem5_obst_status_t * p_status)
{
    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        *p_status = g_obst_status;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }
}

/*!
 * @brief Publish the live ultrasonic status (obstacle_task only).
 *
 * @param[in] p_status Latest state, range and servo angle.
 */
void
subsystem5_obst_status_put (subsystem5_obst_status_t const * p_status)
{
    if (E_OK == tk_loc_mtx(g_mtx_obst, TMO_FEVR))
    {
        g_obst_status = *p_status;
        CHECK(tk_unl_mtx(g_mtx_obst), "mtx_obst");
    }
}

/*!
 * @brief Sleep the calling task.
 *
 * @param[in] delay_ms Milliseconds; rounded up to the 10 ms kernel tick.
 */
void
subsystem5_delay_ms (uint32_t delay_ms)
{
    if (0u != delay_ms)
    {
        // The cast discards the result: tk_dly_tsk() returns an error
        // only if another task releases the wait, which S5 never does.
        (void) tk_dly_tsk(delay_ms);
    }
}

/*!
 * @brief Create one event flag with one waiter and all bits clear.
 *
 * @return The flag's ID, or a negative kernel error.
 */
static ID
create_flag (void)
{
    T_CFLG cflg = {0};

    cflg.flgatr   = TA_TFIFO | TA_WSGL;
    cflg.iflgptn  = 0u;

    return (tk_cre_flg(&cflg));
}

/*!
 * @brief Create one message buffer for count messages of msg_bytes each.
 *
 * @param[in] msg_bytes Size of one message.
 * @param[in] count     How many may wait at once.
 *
 * @return The buffer's ID, or a negative kernel error.
 */
static ID
create_buffer (uint32_t msg_bytes, uint32_t count)
{
    T_CMBF   cmbf       = {0};
    uint32_t rounded    = ((msg_bytes + MBF_ROUND_BYTES - 1u) / MBF_ROUND_BYTES)
                          * MBF_ROUND_BYTES;
    uint32_t slot_bytes = MBF_HEADER_BYTES + rounded;

    // NOTE: The casts are safe: both sizes are a few dozen bytes.
    cmbf.mbfatr = TA_TFIFO;
    cmbf.bufsz  = (SZ) (count * slot_bytes);
    cmbf.maxmsz = (INT) msg_bytes;

    return (tk_cre_mbf(&cmbf));
}

/*** end of file ***/
