/*
 *	task_consumer.c
 *	Course template example (subsystem 3 "consumer"), unchanged apart
 *	from moving its objects here from subsystem3.c.  See task_consumer.h.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx, tk_rcv_mbx, tk_rel_mpf,
 *	            tk_sig_sem, tk_loc_mtx, tk_unl_mtx, tk_snd_mbf, tk_set_flg
 */

#include "subsystem3.h"
#include "../subsystem2/subsystem2.h"	/* IF-01/02: SAMPLE_MSG, pool, slots */
#include "../subsystem4/subsystem4.h"	/* IF-03: mbf_summary, flg_events */

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

EXPORT ER consumer_init(void)
{
	T_CMBX	cmbx = {0};
	T_CMTX	cmtx = {0};

	/* Mailbox: passes a POINTER to a message (zero copy).  The message
	   memory must stay valid until the receiver is done with it, which
	   is why subsystem 2 takes it from its memory pool. */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;	/* FIFO waiters, FIFO messages */
	mbx_samples = tk_cre_mbx(&cmbx);
	if (mbx_samples < E_OK) {
		return mbx_samples;
	}

	/* Mutex: only one task at a time may touch app_stats.  TA_INHERIT
	   (priority inheritance) avoids priority inversion. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	return (mtx_stats < E_OK) ? mtx_stats : E_OK;
}

EXPORT void consumer_task(INT stacd, void *exinf)
{
	SAMPLE_MSG	*msg;
	UB		text[S4_SUMMARY_MAX];	/* tm_sprintf adds a '\0' */
	INT		len;
	UW		count;
	ER		er;

	(void)stacd; (void)exinf;

	while (1) {
		/* 1. Wait for the next sample (TMO_FEVR = wait forever) */
		er = tk_rcv_mbx(mbx_samples, (T_MSG **)&msg, TMO_FEVR);
		if (er < E_OK) {
			CHECK(er, "tk_rcv_mbx");
			continue;
		}

		/* 2. Update the shared statistics.  Subsystem 4 reads them
		      too, so lock the mutex around every access (IF-04). */
		tk_loc_mtx(mtx_stats, TMO_FEVR);
		if (app_stats.received == 0 || msg->value < app_stats.min) {
			app_stats.min = msg->value;
		}
		if (app_stats.received == 0 || msg->value > app_stats.max) {
			app_stats.max = msg->value;
		}
		app_stats.sum += msg->value;
		count = ++app_stats.received;
		tk_unl_mtx(mtx_stats);

		/* 3. Every S3_BATCH_SIZE samples, send a text line to subsystem
		      4 through its message buffer (the text is copied), then
		      set FLG_BATCH to wake it up (IF-03). */
		if (count % S3_BATCH_SIZE == 0) {
			/* tm_sprintf: like sprintf, returns the length.
			   The longest line here is ~35 bytes < S4_SUMMARY_MAX. */
			len = tm_sprintf(text, (const UB*)"n=%u last=%d from core %d",
					 count, msg->value, msg->prc - 1);

			/* TMO_POL: if the buffer is full, drop the line rather
			   than block the data path. */
			er = tk_snd_mbf(mbf_summary, text, len, TMO_POL);
			if (er == E_OK) {
				tk_set_flg(flg_events, FLG_BATCH);
			}
		}

		/* 4. Done with the sample (IF-02): return its memory block to
		      the pool, then the slot so the producer may go on. */
		tk_rel_mpf(mpf_samples, msg);
		tk_sig_sem(sem_slots, 1);
	}
}
