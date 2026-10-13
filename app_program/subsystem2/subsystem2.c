/*
 *	subsystem2.c
 *	Subsystem 2 -- Producer: creates its objects and starts its task.
 *
 *	APIs shown: tk_cre_sem, tk_cre_mpf
 */

#include "subsystem2.h"
#include "drv_encoder.h"
#include "drv_motor.h"

EXPORT ID sem_slots; /* IF-02, see subsystem2.h */
EXPORT ID mpf_samples;

EXPORT ID mbx_motion; /* IF-04 */
EXPORT ID flg_motion; /* IF-05 */

LOCAL ID mtx_odom;	/* IF-06 */
LOCAL ID mtx_steer; /* IF-02 */
LOCAL ODOM odom;
LOCAL H steer_sp;

EXPORT void s2_odom_put(const ODOM *in)
{
	tk_loc_mtx(mtx_odom, TMO_FEVR);
	odom = *in;
	tk_unl_mtx(mtx_odom);
}

EXPORT void s2_odom_get(ODOM *out)
{
	tk_loc_mtx(mtx_odom, TMO_FEVR);
	*out = odom;
	tk_unl_mtx(mtx_odom);
}

EXPORT void s2_steer_set(H dv_mms)
{
	tk_loc_mtx(mtx_steer, TMO_FEVR);
	steer_sp = dv_mms;
	tk_unl_mtx(mtx_steer);
}

EXPORT H s2_steer_get(void)
{
	H dv;

	tk_loc_mtx(mtx_steer, TMO_FEVR);
	dv = steer_sp;
	tk_unl_mtx(mtx_steer);
	return dv;
}

EXPORT ER subsystem2_init(void)
{
	T_CSEM csem = {0};
	T_CMPF cmpf = {0};
	T_CMBX cmbx = {0};
	T_CFLG cflg = {0};
	T_CMTX cmtx = {0};
	ER er;

	/* Semaphore: a counter of free "slots".  The producer takes one
	   (tk_wai_sem) before each sample; the consumer gives one back
	   (tk_sig_sem).  It never exceeds the number of memory blocks, so
	   the producer can never run out of memory. */
	csem.sematr = TA_TFIFO;		  /* waiting tasks queue in FIFO order */
	csem.isemcnt = S2_POOL_COUNT; /* initial count */
	csem.maxsem = S2_POOL_COUNT;  /* maximum count */
	sem_slots = tk_cre_sem(&csem);
	if (sem_slots < E_OK)
	{
		return sem_slots;
	}

	/* Fixed-size memory pool: S2_POOL_COUNT blocks, each exactly one
	   SAMPLE_MSG.  Faster and more predictable than malloc(). */
	cmpf.mpfatr = TA_TFIFO | TA_RNG3;
	cmpf.mpfcnt = S2_POOL_COUNT;	 /* number of blocks */
	cmpf.blfsz = sizeof(SAMPLE_MSG); /* size of one block */
	mpf_samples = tk_cre_mpf(&cmpf);
	if (mpf_samples < E_OK)
	{
		return mpf_samples;
	}

	/* Motion interfaces: command mailbox, emergency-stop flag, two data mutexes */
	cmbx.mbxatr = TA_TFIFO | TA_MFIFO;
	mbx_motion = tk_cre_mbx(&cmbx);
	if (mbx_motion < E_OK)
	{
		return mbx_motion;
	}

	cflg.flgatr = TA_TFIFO | TA_WMUL;
	flg_motion = tk_cre_flg(&cflg);
	if (flg_motion < E_OK)
	{
		return flg_motion;
	}

	cmtx.mtxatr = TA_INHERIT;
	mtx_odom = tk_cre_mtx(&cmtx);
	if (mtx_odom < E_OK)
	{
		return mtx_odom;
	}
	mtx_steer = tk_cre_mtx(&cmtx);
	if (mtx_steer < E_OK)
	{
		return mtx_steer;
	}

	/* Wheel encoders on GP16/GP17: edge interrupt with microsecond stamps */
	er = s2_enc_init();
	if (er < E_OK)
	{
		return er;
	}

	/* Motors on GP8-GP11: PWM starts with both motors braked */
	return s2_motor_init();
}

EXPORT ER subsystem2_start(void)
{
	/* Pinned to core 1 (TP_PRC2): samples cross to subsystem 3 on core 0 */
	ID tid = app_start_task(producer_task, S2_PRI_PRODUCER, TP_PRC2,
							"producer_task");
#if APP_DRIVE
	if (tid >= E_OK)
	{
		return s2_drive_demo_start();
	}
#endif
	return (tid < E_OK) ? tid : E_OK;
}
