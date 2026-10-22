/*
 *	task_producer.h
 *	TEMPLATE EXAMPLE, kept only because the subsystem 3 and 4 examples
 *	(task_consumer.c, task_monitor.c) still use these objects.  Delete
 *	this file and task_producer.c once S3 and S4 replace their examples.
 *	The code is the course template's and is not BARR-C formatted.
 */

#ifndef TASK_PRODUCER_H
#define TASK_PRODUCER_H

#include "../app.h"

#define S2_PRI_PRODUCER 14 /* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS 300   /* one sample every 300 ms */

/* IF-01 data format: one sample, sent to subsystem 3's mailbox.
   A mailbox message MUST start with a T_MSG header. */
typedef struct
{
	T_MSG hdr;	/* used by the kernel, do not touch */
	UW seq;		/* sample sequence number */
	UW time_ms; /* system time when the sample was taken */
	INT value;	/* the "measurement", 0..100 */
	INT prc;	/* processor (core) that produced it: 1 or 2 */
} SAMPLE_MSG;

#define S2_POOL_COUNT 4 /* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
   mpf_samples (tk_rel_mpf) and then a slot back to sem_slots
   (tk_sig_sem, count 1).  Nothing else may use these two objects. */
IMPORT ID sem_slots;
IMPORT ID mpf_samples;

IMPORT ER producer_init(void);
IMPORT ER producer_start(void);
IMPORT void producer_task(INT stacd, void *exinf);
IMPORT INT s2_make_value(UW seq);

#endif /* TASK_PRODUCER_H */
