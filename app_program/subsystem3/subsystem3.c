/*
 *	subsystem3.c
 *	Subsystem 3 -- Consumer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mtx
 */

#include "subsystem3.h"

EXPORT ID		mbx_samples;	/* IF-01 */
EXPORT ID		mtx_stats;	/* IF-04 */
EXPORT APP_STATS	app_stats;

EXPORT ER subsystem3_init(void)
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
	   (priority inheritance) avoids priority inversion: a low-priority
	   holder is temporarily raised to the waiter's priority. */
	cmtx.mtxatr = TA_INHERIT;
	mtx_stats = tk_cre_mtx(&cmtx);
	if (mtx_stats < E_OK) {
		return mtx_stats;
	}
	return s3_init();	/* Buddy 3: mbf_nav, mtx_s3, flg_s3 (s3_api.h) */
}

EXPORT ER subsystem3_start(void)
{
	/* Pinned to core 0 (TP_PRC1): opposite core to the producer */
	ID tid = app_start_task(consumer_task, S3_PRI_CONSUMER, TP_PRC1,
				"consumer_task");
	if (tid < E_OK) {
		return tid;
	}
	return s3_bringup_start();	/* Buddy 3: sensor bring-up task */
}
