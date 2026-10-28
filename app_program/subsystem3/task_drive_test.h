/** @file task_drive_test.h
 *
 * @brief Subsystem 3 -- TEST ONLY: line following on Buddy 2's motors
 *        before the Vehicle Controller exists. Built in only when
 *        S3_DRIVE_TEST is 1 (s3_config.h); otherwise
 *        task_drive_test_start() does nothing.
 */

#ifndef TASK_DRIVE_TEST_H
#define TASK_DRIVE_TEST_H

#include "../app.h"         /* ER for the kernel error code.            */

ER task_drive_test_start(void);

#endif /* TASK_DRIVE_TEST_H */

/*** end of file ***/
