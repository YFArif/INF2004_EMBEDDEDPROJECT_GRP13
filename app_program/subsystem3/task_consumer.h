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

/* IF-01 (template): send a task_producer_sample_t (task_producer.h) here
 * with tk_snd_mbx.
 */
extern ID gh_mbx_samples;

/* IF-04 (template): statistics. Read them ONLY while holding gh_mtx_stats:
 *   tk_loc_mtx(gh_mtx_stats, TMO_FEVR); copy = g_app_stats;
 *   tk_unl_mtx(gh_mtx_stats);
 */
typedef struct
{
    UW  received;   /* Samples received so far. */
    INT min;
    INT max;
    W   sum;
} task_consumer_stats_t;

extern ID                    gh_mtx_stats;
extern task_consumer_stats_t g_app_stats;

ER   task_consumer_init(void);
void task_consumer_task(INT stacd, void * p_exinf);

#endif /* TASK_CONSUMER_H */

/*** end of file ***/

