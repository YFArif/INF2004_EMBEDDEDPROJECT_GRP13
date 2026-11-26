/** @file app_main.c
 *
 * @brief START HERE. usermain() below is your program's entry point.
 *
 * @par How micro T-Kernel starts your code
 *   reset -> kernel initialises itself -> creates the "initial task"
 *         -> initial task calls usermain()
 *
 * @par
 * usermain() runs inside that initial task at priority 1 (the highest).
 * On a single core nothing you create runs until usermain() blocks, but
 * on the dual-core build the OTHER core may start a new task at once.
 * So usermain() works in two phases:
 *   1. every subsystem creates its kernel objects  (subsystemN_init)
 *   2. every subsystem starts its tasks             (subsystemN_start)
 * and then puts the initial task to sleep forever.
 * If usermain() RETURNS, the kernel shuts the whole system down.
 *
 * @par The example system: one subsystem per folder, one task each
 *   subsystem1/  LED       blinks GP19, paced by a cyclic handler
 *   subsystem2/  Producer  makes a "sample" every 300 ms     (core 1)
 *   subsystem3/  Consumer  collects samples into statistics  (core 0)
 *   subsystem4/  Monitor   prints reports, pauses/resumes the LED
 *   subsystem5/  (empty template for the fifth student)
 *
 * @par How they talk to each other (the interfaces, see TEAM.md section 5)
 *   IF-01  S2 -> S3  mailbox gh_mbx_samples   (owned by S3)
 *   IF-02  S3 -> S2  semaphore gh_sem_slots + memory pool gh_mpf_samples
 *                    (owned by S2: S3 hands slots and blocks back)
 *   IF-03  S3 -> S4  message buffer gh_mbf_summary + event flag
 *                    gh_flg_events (owned by S4)
 *   IF-04  S4 -> S3  reads g_app_stats under mutex gh_mtx_stats
 *                    (owned by S3)
 *   IF-05  S4 -> S1  suspends/resumes the LED task gh_tid_led (owned by S1)
 * The subsystem that RECEIVES or OWNS the data creates the object and
 * publishes its ID in its header; the other side just uses it.
 *
 * @par micro T-Kernel APIs used, and where to find them
 *   tasks       tk_cre_tsk tk_sta_tsk tk_slp_tsk tk_wup_tsk tk_can_wup
 *               tk_dly_tsk tk_sus_tsk tk_rsm_tsk tk_chg_pri tk_ref_tsk
 *               tk_get_tid tk_get_prc                    app_main.c, S1-S4
 *   semaphore   tk_cre_sem tk_wai_sem tk_sig_sem tk_ref_sem       S2, S3, S4
 *   mem. pool   tk_cre_mpf tk_get_mpf tk_rel_mpf tk_ref_mpf       S2, S3, S4
 *   mailbox     tk_cre_mbx tk_snd_mbx tk_rcv_mbx                  S2, S3
 *   mutex       tk_cre_mtx tk_loc_mtx tk_unl_mtx                  S3, S4
 *   msg buffer  tk_cre_mbf tk_snd_mbf tk_rcv_mbf                  S3, S4
 *   event flag  tk_cre_flg tk_set_flg tk_wai_flg                  S3, S4
 *   cyclic      tk_cre_cyc tk_ref_cyc                             S1
 *   alarm       tk_cre_alm tk_sta_alm tk_stp_alm                  S4
 *   time        tk_get_tim                                        S2, S4
 *   system      tk_ref_ver                                        app_main.c
 *
 * @par
 * Output appears on the USB serial port (115200 8N1 in any terminal).
 */

#include "app.h"

/*!
 * @brief Create and start one task (declared in app.h for every subsystem).
 *
 * @param[in] p_entry The function the task runs.
 * @param[in] pri     The task's initial priority (1 = highest).
 * @param[in] prc     TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any
 *                    core". Ignored by the single-core build.
 * @param[in] p_name  The task's name, printed if a kernel call fails.
 *
 * @return The new task's ID, or the negative kernel error from
 *         tk_cre_tsk().
 */
EXPORT ID
app_start_task (FP p_entry, PRI pri, UW prc, const char * p_name)
{
    T_CTSK ctsk = {0};
    ID     h_task;
    ER     err;

    ctsk.task    = p_entry;             /* function the task runs */
    ctsk.itskpri = pri;                 /* initial priority (1 = highest) */
    ctsk.stksz   = TASK_STACK_SIZE;     /* kernel allocates the stack */
    ctsk.tskatr  = TA_HLNG              /* written in a high-level language */
                   | TA_RNG3;           /* runs at user protection level */
#if TK_SUPPORT_SMP
    if (0 != prc)
    {
        ctsk.tskatr  |= TA_ASSPRC;       /* honour the affinity below */
        ctsk.assprc  = prc;              /* which core(s) it may run on */
    }
#else
    /* The cast marks prc as unused: the single-core build has no
     * affinity.
     */
    (void)prc;
#endif

    /* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
    h_task = tk_cre_tsk(&ctsk);
    if (h_task < E_OK)
    {
        CHECK(h_task, p_name);
        return h_task;
    }

    /* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
    err = tk_sta_tsk(h_task, 0);
    CHECK(err, p_name);
    return h_task;
}

/*!
 * @brief The program's entry point, called by the kernel's initial task.
 *
 * Creates every subsystem's kernel objects, then starts every
 * subsystem's tasks, runs the self-tests in a TEST=1 build, and then
 * sleeps forever.
 *
 * @return 0. It never returns in practice: returning would shut the
 *         system down.
 */
EXPORT INT
usermain (void)
{
    T_RVER ver;

    /* tk_ref_ver: ask the kernel who it is.  The casts below are safe:
     * every format string is ASCII.
     */
    tk_ref_ver(&ver);
    tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
              " (spec 0x%04x, product 0x%04x) ===\n", ver.spver, ver.prver);
#if TK_SUPPORT_SMP
    tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
              tk_get_prc() - 1);    /* tk_get_prc() counts from 1 */
#else
    tm_printf((UB *)"single-core build\n");
#endif

    /* Phase 1: every subsystem creates its kernel objects. */
    CHECK(subsystem1_init(), "subsystem1_init");
    CHECK(subsystem2_init(), "subsystem2_init");
    CHECK(subsystem3_init(), "subsystem3_init");
    CHECK(subsystem4_init(), "subsystem4_init");
    CHECK(subsystem5_init(), "subsystem5_init");

    /* Phase 2: every subsystem starts its tasks. */
    CHECK(subsystem1_start(), "subsystem1_start");
    CHECK(subsystem2_start(), "subsystem2_start");
    CHECK(subsystem3_start(), "subsystem3_start");
    CHECK(subsystem4_start(), "subsystem4_start");
    CHECK(subsystem5_start(), "subsystem5_start");

    /* The cast is safe: the format string is ASCII. */
    tm_printf((UB *)"all subsystems started\n\n");

#if APP_TEST
    /* Test build (make TEST=1): run every subsystem's tests in turn,
     * here in the initial task, while the system is running.  Tests may
     * call tk_dly_tsk() to let the other tasks work for a while.  The
     * casts are safe: the format strings are ASCII.
     */
    tm_printf((UB *)"[TEST] === test run start ===\n");
    subsystem1_test();
    subsystem2_test();
    subsystem3_test();
    subsystem4_test();
    subsystem5_test();
    tm_printf((UB *)"[TEST] === test run end ===\n");
#endif

    /* Sleep forever.  Returning from usermain() would shut down. */
    tk_slp_tsk(TMO_FEVR);
    return 0;
}

/*** end of file ***/

