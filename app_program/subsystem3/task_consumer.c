/** @file task_consumer.c
 *
 * @brief Course template example (subsystem 3 "consumer"), moved here
 *        from subsystem3.c and reformatted.  See task_consumer.h.
 *
 * @par
 * APIs shown: tk_cre_mbx, tk_cre_mtx, tk_rcv_mbx, tk_rel_mpf,
 *             tk_sig_sem, tk_loc_mtx, tk_unl_mtx, tk_snd_mbf, tk_set_flg
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"   /* IF-01/02: sample, pool, slots */
#include "../subsystem4/subsystem4.h"   /* IF-03: gh_mbf_summary,        */
                                        /* gh_flg_events                 */

EXPORT ID                    gh_mbx_samples;    /* IF-01 */
EXPORT ID                    gh_mtx_stats;      /* IF-04 */
EXPORT task_consumer_stats_t g_app_stats;

/*!
 * @brief Create the example's mailbox and mutex (called from
 *        subsystem3_init()).
 *
 * @return E_OK, or the first kernel error.
 */
EXPORT ER
task_consumer_init (void)
{
    T_CMBX cmbx = {0};
    T_CMTX cmtx = {0};

    /* Mailbox: passes a POINTER to a message (zero copy).  The message
     * memory must stay valid until the receiver is done with it, which
     * is why subsystem 2 takes it from its memory pool.
     */
    cmbx.mbxatr    = TA_TFIFO | TA_MFIFO;   /* FIFO waiters, FIFO messages */
    gh_mbx_samples = tk_cre_mbx(&cmbx);
    if (gh_mbx_samples < E_OK)
    {
        return gh_mbx_samples;
    }

    /* Mutex: only one task at a time may touch g_app_stats.  TA_INHERIT
     * (priority inheritance) avoids priority inversion.
     */
    cmtx.mtxatr  = TA_INHERIT;
    gh_mtx_stats = tk_cre_mtx(&cmtx);
    return ((gh_mtx_stats < E_OK) ? gh_mtx_stats : E_OK);
}

/*!
 * @brief The consumer task: collect each sample into the statistics and
 *        report every S3_BATCH_SIZE samples to subsystem 4.
 *
 * @param[in] stacd   Start code; not used.
 * @param[in] p_exinf Extended information; not used.
 */
EXPORT void
task_consumer_task (INT stacd, void * p_exinf)
{
    task_producer_sample_t * p_msg;
    UB                       text[S4_SUMMARY_MAX];  /* tm_sprintf adds a NUL. */
    INT                      len;
    UW                       count;
    ER                       err;

    /* The casts mark the arguments as unused. */
    (void)stacd;
    (void)p_exinf;

    for (;;)
    {
        /* 1. Wait for the next sample (TMO_FEVR = wait forever).  The
         *    cast is the kernel's generic message pointer type; every
         *    sample starts with its T_MSG header.
         */
        err = tk_rcv_mbx(gh_mbx_samples, (T_MSG **)&p_msg, TMO_FEVR);
        if (err < E_OK)
        {
            CHECK(err, "tk_rcv_mbx");
            continue;
        }

        /* 2. Update the shared statistics.  Subsystem 4 reads them
         *    too, so lock the mutex around every access (IF-04).
         */
        tk_loc_mtx(gh_mtx_stats, TMO_FEVR);
        if ((0 == g_app_stats.received) || (p_msg->value < g_app_stats.min))
        {
            g_app_stats.min = p_msg->value;
        }
        if ((0 == g_app_stats.received) || (p_msg->value > g_app_stats.max))
        {
            g_app_stats.max = p_msg->value;
        }
        g_app_stats.sum += p_msg->value;
        count = ++g_app_stats.received;
        tk_unl_mtx(gh_mtx_stats);

        /* 3. Every S3_BATCH_SIZE samples, send a text line to subsystem
         *    4 through its message buffer (the text is copied), then
         *    set FLG_BATCH to wake it up (IF-03).
         */
        if (0 == (count % S3_BATCH_SIZE))
        {
            /* tm_sprintf: like sprintf, returns the length.  The longest
             * line here is ~35 bytes < S4_SUMMARY_MAX.  The cast is safe:
             * the format is an ASCII string literal.
             */
            len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
                             count, p_msg->value, p_msg->prc - 1);

            /* TMO_POL: if the buffer is full, drop the line rather
             * than block the data path.
             */
            err = tk_snd_mbf(gh_mbf_summary, text, len, TMO_POL);
            if (E_OK == err)
            {
                tk_set_flg(gh_flg_events, FLG_BATCH);
            }
        }

        /* 4. Done with the sample (IF-02): return its memory block to
         *    the pool, then the slot so the producer may go on.
         */
        tk_rel_mpf(gh_mpf_samples, p_msg);
        tk_sig_sem(gh_sem_slots, 1);
    }
}

/*** end of file ***/

