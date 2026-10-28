/** @file task_barcode.h
 *
 * @brief Subsystem 3 -- task_barcode: sets up S3's hardware on core 1,
 *        starts task_line, then decodes every barcode pass and publishes
 *        a navigation command (IF-01).
 */

#ifndef TASK_BARCODE_H
#define TASK_BARCODE_H

#include "../app.h"         /* ER for the kernel error code.            */

ER task_barcode_start(void);

#endif /* TASK_BARCODE_H */

/*** end of file ***/
