/** @file s5_ranger.h
 *
 * @brief One complete, paced HC-SR04 measurement with fault detection.
 *
 * @par
 * s5_ranger_measure() enforces the datasheet's 60 ms minimum between
 * pings (NFR7) by itself, so any caller (monitoring, scanning, side
 * watch) can simply measure in a loop. It sleeps while it waits, so call
 * it only from obstacle_task.
 */

#ifndef S5_RANGER_H
#define S5_RANGER_H

#include <stdbool.h>
#include <stdint.h>

/** What one measurement found. */
typedef enum
{
    S5_RANGER_OK = 0,       /* Echo received; dist_mm is valid.          */
    S5_RANGER_CLEAR,        /* No echo within range: nothing ahead.      */
    S5_RANGER_BUSY,         /* Previous echo still high; no ping sent.   */
    S5_RANGER_INVALID,      /* The sensor did not answer this ping.      */
    S5_RANGER_FAULT         /* The sensor has failed (latched).          */
} s5_ranger_status_t;

/** One measurement. */
typedef struct
{
    s5_ranger_status_t  status;
    uint16_t            dist_mm;    /* Valid only when status is OK.     */
    uint32_t            echo_us;    /* Raw echo width, for logging.      */
} s5_ranger_reading_t;

void s5_ranger_init(void);
void s5_ranger_measure(s5_ranger_reading_t * p_reading);
bool s5_ranger_is_within(s5_ranger_reading_t const * p_reading,
                         uint32_t limit_mm);
bool s5_ranger_is_conclusive(s5_ranger_reading_t const * p_reading);
void s5_ranger_reset_fault(void);

#endif /* S5_RANGER_H */

/*** end of file ***/
