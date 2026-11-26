/** @file subsystem3.c
 *
 * @brief Subsystem 3 -- creates its kernel objects and starts its tasks.
 *        See subsystem3.h for the file map.
 */

#include "../app.h"

#include "subsystem3.h"

/*!
 * @brief Create subsystem 3's kernel objects (called before any task).
 *
 * @return E_OK or the first kernel error.
 */
ER
subsystem3_init (void)
{
    ER err = task_consumer_init();  /* Template example (S2 / S4 use it). */

    if (err < E_OK)
    {
        return err;
    }
    return s3_init();   /* gh_mbf_nav, gh_mtx_s3, gh_flg_s3, gh_flg_line. */
}

/*!
 * @brief Start subsystem 3's tasks.
 *
 * @return E_OK or the first kernel error.
 */
ER
subsystem3_start (void)
{
    ID h_tid = app_start_task((FP) task_consumer_task, S3_PRI_CONSUMER, TP_PRC1,
                              "consumer_task");   /* Template example.     */

    if (h_tid < E_OK)
    {
        return h_tid;
    }
    return s3_start();          /* task_barcode, then task_line.         */
}

/*** end of file ***/

