/** @file task_producer.c
 *
 * @brief TEMPLATE EXAMPLE producer task, kept only for the S3/S4 examples
 *        (see task_producer.h).  Delete it with them.
 *
 * @par
 * APIs shown: tk_cre_sem, tk_cre_mpf, tk_wai_sem, tk_get_mpf,
 *             tk_snd_mbx, tk_get_tim, tk_dly_tsk, tk_get_prc
 */

#include "task_producer.h"
#include "../subsystem3/subsystem3.h"   /* IF-01: gh_mbx_samples */

EXPORT ID gh_sem_slots;     /* IF-02, see task_producer.h */
EXPORT ID gh_mpf_samples;

/*!
 * @brief Create the example's semaphore and memory pool (called from
 *        subsystem2_init()).
 *
 * @return E_OK, or the first kernel error.
 */
EXPORT ER
task_producer_init (void)
{
    T_CSEM csem = {0};
    T_CMPF cmpf = {0};

    csem.sematr  = TA_TFIFO;
    csem.isemcnt = S2_POOL_COUNT;
    csem.maxsem  = S2_POOL_COUNT;
    gh_sem_slots = tk_cre_sem(&csem);
    if (gh_sem_slots < E_OK)
    {
        return gh_sem_slots;
    }

    cmpf.mpfatr    = TA_TFIFO | TA_RNG3;
    cmpf.mpfcnt    = S2_POOL_COUNT;
    cmpf.blfsz     = sizeof(task_producer_sample_t);
    gh_mpf_samples = tk_cre_mpf(&cmpf);
    return ((gh_mpf_samples < E_OK) ? gh_mpf_samples : E_OK);
}

/*!
 * @brief Start the example task on core 1 (called from subsystem2_start()).
 *
 * @return E_OK, or the kernel error from app_start_task().
 */
EXPORT ER
task_producer_start (void)
{
    ID h_tid = app_start_task(task_producer_task, S2_PRI_PRODUCER, TP_PRC2,
                              "producer_task");

    return ((h_tid < E_OK) ? h_tid : E_OK);
}

/*!
 * @brief A fake measurement: a triangle wave between 0 and 100.
 *
 * @param[in] seq The sample sequence number.
 *
 * @return The "measurement", 0..100.
 */
EXPORT INT
task_producer_make_value (UW seq)
{
    /* The cast is safe: seq % 20 is 0..19, which fits INT. */
    INT phase = (INT)(seq % 20);

    return (((phase < 10) ? phase : (20 - phase)) * 10);
}

/*!
 * @brief The producer task: send one sample to subsystem 3 every
 *        S2_PERIOD_MS.
 *
 * @param[in] stacd   Start code; not used.
 * @param[in] p_exinf Extended information; not used.
 */
EXPORT void
task_producer_task (INT stacd, void * p_exinf)
{
    UW                       seq = 0;
    task_producer_sample_t * p_msg;
    SYSTIM                   now;
    ER                       err;

    /* The casts mark the arguments as unused. */
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        /* 1. Take a free slot.  If the consumer is behind and all
         *    slots are in use, this task blocks here until one is
         *    returned (flow control).
         */
        err = tk_wai_sem(gh_sem_slots, 1, TMO_FEVR);
        CHECK(err, "tk_wai_sem");

        /* 2. Get a memory block for the message.  The semaphore
         *    guarantees one is free, so poll (TMO_POL = don't wait).
         *    The cast is the kernel's generic block pointer type.
         */
        err = tk_get_mpf(gh_mpf_samples, (void **)&p_msg, TMO_POL);
        if (err < E_OK)
        {
            CHECK(err, "tk_get_mpf");
            tk_sig_sem(gh_sem_slots, 1);    /* give the slot back */
            tk_dly_tsk(S2_PERIOD_MS);
            continue;
        }

        /* 3. Fill it in.  tk_get_tim gives milliseconds since boot
         *    as a 64-bit hi:lo pair; lo is enough for ~49 days.
         */
        tk_get_tim(&now);
        p_msg->seq     = seq++;
        p_msg->time_ms = now.lo;
        p_msg->value   = task_producer_make_value(p_msg->seq);
#if TK_SUPPORT_SMP
        p_msg->prc     = tk_get_prc();  /* which core am I on? */
#else
        p_msg->prc     = 1;
#endif

        /* 4. Send the POINTER to subsystem 3.  From here on the block
         *    belongs to the receiver: do not touch p_msg again.  The
         *    cast is safe: the sample starts with its T_MSG header.
         */
        err = tk_snd_mbx(gh_mbx_samples, (T_MSG *)p_msg);
        CHECK(err, "tk_snd_mbx");

        /* 5. Wait for the next period.  Other tasks run meanwhile. */
        tk_dly_tsk(S2_PERIOD_MS);
    }
}

/*** end of file ***/

