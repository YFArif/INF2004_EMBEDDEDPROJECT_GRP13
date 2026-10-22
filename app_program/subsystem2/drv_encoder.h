/** @file drv_encoder.h
 *
 * @brief Subsystem 2 -- wheel encoder tick counter (GP16 left, GP17 right).
 *
 * @par
 * Each encoder gives one square-wave phase (no direction). A tick is one
 * debounced rising edge: the pin must really be at the new level when the
 * interrupt reads it, and at least DRV_ENCODER_MIN_GAP_US must have
 * passed since the last accepted change. The counters run from power-up
 * and are never cleared, so readers work with differences.
 *
 * @par
 * Owned by subsystem 2. Only motion_task sets it up; any S2 code may read.
 */

#ifndef DRV_ENCODER_H
#define DRV_ENCODER_H

#include <stdint.h>

#include <tk/tkernel.h>     /* ER for the kernel error code.            */

#define DRV_ENCODER_LEFT_PIN    (16u)   /* Grove 4 pin 1.                */
#define DRV_ENCODER_RIGHT_PIN   (17u)   /* Grove 4 pin 2.                */

/* Real edges are at least ~240 us apart at 0.65 m/s (S2-U06 duty 800). */
#define DRV_ENCODER_MIN_GAP_US  (100u)

/** The two wheels. */
typedef enum
{
    DRV_ENCODER_LEFT = 0,
    DRV_ENCODER_RIGHT,
    DRV_ENCODER_COUNT
} drv_encoder_t;

/** Counters of one wheel since power-up. */
typedef struct
{
    uint32_t ticks;         /* Debounced rising edges.                   */
    uint32_t tick_time_us;  /* 1 MHz timer at the last counted tick.     */
    uint32_t raw_rise;      /* Every rising-edge interrupt event.        */
    uint32_t raw_fall;      /* Every falling-edge interrupt event.       */
    uint32_t rejected;      /* Edge events rejected as glitches.         */
    uint32_t min_gap_us;    /* Shortest gap between two raw edge events. */
} drv_encoder_count_t;

ER       drv_encoder_init(void);
void     drv_encoder_get(drv_encoder_t wheel, drv_encoder_count_t * p_count);
uint32_t drv_encoder_now_us(void);

#endif /* DRV_ENCODER_H */

/*** end of file ***/
