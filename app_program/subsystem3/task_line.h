/** @file task_line.h
 *
 * @brief Subsystem 3 -- task_line: every 10 ms reads the Left IR and the
 *        Centre sensor, runs the combiner (s3_line.c) and publishes the
 *        steering, the status and the events.
 */

#ifndef TASK_LINE_H
#define TASK_LINE_H

#include "../app.h"         /* ER for the kernel error code.            */

ER task_line_start(void);

#endif /* TASK_LINE_H */

/*** end of file ***/
