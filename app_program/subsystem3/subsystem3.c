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
    ER er = consumer_init();    /* Template example (S2 / S4 use it).    */

    if (er < E_OK)
    {
        return er;
    }
    return s3_init();           /* mbf_nav, mtx_s3, flg_s3, flg_line.    */
}

/*!
 * @brief Start subsystem 3's tasks.
 *
 * @return E_OK or the first kernel error.
 */
ER
subsystem3_start (void)
{
    ID tid = app_start_task((FP) consumer_task, S3_PRI_CONSUMER, TP_PRC1,
                            "consumer_task");   /* Template example.     */

    if (tid < E_OK)
    {
        return tid;
    }
    return s3_start();          /* task_barcode, then task_line.         */
}

/*** end of file ***/
