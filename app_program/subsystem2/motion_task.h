/** @file motion_task.h
 *
 * @brief Subsystem 2 -- motion_task entry point.
 */

#ifndef MOTION_TASK_H
#define MOTION_TASK_H

#include <tk/tkernel.h>     /* INT for the kernel's task entry type.    */

void motion_task(INT stacd, void * p_exinf);

#endif /* MOTION_TASK_H */

/*** end of file ***/
