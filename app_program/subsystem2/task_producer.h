/** @file task_producer.h
 *
 * @brief Template example producer task (interfaces IF-01 and IF-02).
 *
 * @par
 * TEMPLATE EXAMPLE, kept only because the subsystem 3 and 4 examples
 * (task_consumer.c, task_monitor.c) still use these objects.  Delete
 * this file and task_producer.c once S3 and S4 replace their examples.
 */

#ifndef TASK_PRODUCER_H
#define TASK_PRODUCER_H

#include "../app.h"

#define S2_PRI_PRODUCER 14  /* priority band 14-15 (TEAM.md) */
#define S2_PERIOD_MS    300 /* one sample every 300 ms */

/* IF-01 data format: one sample, sent to subsystem 3's mailbox.
 * A mailbox message MUST start with a T_MSG header.
 */
typedef struct
{
    T_MSG hdr;      /* used by the kernel, do not touch */
    UW    seq;      /* sample sequence number */
    UW    time_ms;  /* system time when the sample was taken */
    INT   value;    /* the "measurement", 0..100 */
    INT   prc;      /* processor (core) that produced it: 1 or 2 */
} task_producer_sample_t;

#define S2_POOL_COUNT   4   /* at most 4 samples in flight */

/* IF-02: the receiver of a sample gives its memory block back to
 * gh_mpf_samples (tk_rel_mpf) and then a slot back to gh_sem_slots
 * (tk_sig_sem, count 1).  Nothing else may use these two objects.
 */
IMPORT ID gh_sem_slots;
IMPORT ID gh_mpf_samples;

IMPORT ER   task_producer_init(void);
IMPORT ER   task_producer_start(void);
IMPORT void task_producer_task(INT stacd, void * p_exinf);
IMPORT INT  task_producer_make_value(UW seq);

#endif /* TASK_PRODUCER_H */

/*** end of file ***/

