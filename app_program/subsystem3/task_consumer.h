/** @file task_consumer.h
 *
 * @brief Course template example (Subsystem 3 "consumer"): kept only
 *        because subsystem 2 (task_producer.c) and subsystem 4
 *        (task_monitor.c) still use these objects. Not part of Buddy 3's
 *        line / barcode work; delete together with the S2 / S4 examples.
 */

#ifndef TASK_CONSUMER_H
#define TASK_CONSUMER_H

#include "../app.h"

#define S3_PRI_CONSUMER     (16)    /* Priority band 16-17 (TEAM.md).    */
#define S3_BATCH_SIZE       (5)     /* Report to subsystem 4 every N.    */

/* IF-01 (template): send SAMPLE_MSG (subsystem2.h) here with tk_snd_mbx. */
extern ID mbx_samples;

/* IF-04 (template): statistics. Read them ONLY while holding mtx_stats:
 *   tk_loc_mtx(mtx_stats, TMO_FEVR); copy = app_stats; tk_unl_mtx(mtx_stats);
 */
typedef struct
{
    UW  received;   /* Samples received so far. */
    INT min;
    INT max;
    W   sum;
} APP_STATS;

extern ID        mtx_stats;
extern APP_STATS app_stats;

ER   consumer_init(void);
void consumer_task(INT stacd, void * exinf);

#endif /* TASK_CONSUMER_H */

/*** end of file ***/
