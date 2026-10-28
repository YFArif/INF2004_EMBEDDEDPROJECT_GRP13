/** @file drv_barcode.h
 *
 * @brief Subsystem 3 -- barcode sensor driver: digital output (DOUT) on
 *        GP7 and analogue output (AOUT) on GP28 (ADC channel 2).
 *
 * @par Two independent paths
 * DIGITAL: the module's comparator output. A GPIO edge interrupt stores
 * the time between edges in a ring of 16-bit gaps (16 us units). Edges
 * closer than DRV_BARCODE_GLITCH_US are chatter: counted, not stored.
 * ANALOGUE: the sampler interrupt (drv_adc.c) stores every AOUT sample
 * as 8 bits in a 2 s ring. A move of DRV_BARCODE_ACT_STEP away from the
 * level at the last activity, or any DOUT edge, opens an "activity
 * window".
 * A pass is finished when both paths have been quiet for
 * DRV_BARCODE_QUIET_US. task_barcode then takes the pass and decodes it.
 *
 * @par Sharing (Change Requirement 1)
 * Each ISR-to-task item has one writer: the gap ring write index and the
 * gaps are written only by the GPIO interrupt, the read index only by the
 * task, so no lock is needed. The window indices are read with
 * interrupts disabled; both interrupts run on core 1, as does the task.
 *
 * @par Shared interrupt
 * IO_IRQ_BANK0 belongs to subsystem 2 (wheel encoders). S3's handler is
 * attached to it with subsystem2_gpio_isr_register(): only one
 * tk_def_int() per interrupt number is allowed.
 */

#ifndef DRV_BARCODE_H
#define DRV_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#include "../app.h"         /* ER for the kernel error code.            */

#define DRV_BARCODE_DOUT_PIN    (7u)    /* Rewired 28 Oct (was GP6).    */
#define DRV_BARCODE_ADC_CH      (2u)    /* AOUT on GP28 (was GP26).     */

#define DRV_BARCODE_GLITCH_US   (2000u)     /* Measured chatter: 0.5 ... */
                                            /* 0.85 ms.                  */
#define DRV_BARCODE_QUIET_US    (300000u)   /* Whole code at 0.30 m/s =  */
                                            /* 470 ms.                   */
#define DRV_BARCODE_ACT_STEP    (12)        /* 8-bit units (= 192 of     */
                                            /* 4096): well above noise.  */
#define DRV_BARCODE_DIG_RING    (64u)       /* Power of 2; a code needs  */
                                            /* 30 edges.                 */
#define DRV_BARCODE_RING        (4096u)     /* 4096 x 0.5 ms = 2 s.      */

/** Opened by: bit 0 = AOUT moved, bit 1 = DOUT edge. */
#define DRV_BARCODE_SRC_AOUT    (1u)
#define DRV_BARCODE_SRC_DOUT    (2u)

/** One finished pass, as handed to task_barcode. The buffers belong to
 *  the driver and stay valid until the next drv_barcode_take_pass(). */
typedef struct
{
    uint32_t const * p_dur;     /* Digital element durations, us.        */
    uint32_t         n_edges;   /* Edges stored; durations = n_edges - 1. */
    uint32_t         glitches;  /* Chatter edges dropped in this pass.   */
    uint32_t         lost;      /* Edges dropped because the ring was    */
                                /* full: digital path not trusted.       */
    uint8_t const *  p_win;     /* Analogue window, 8-bit samples.       */
    uint32_t         n_win;     /* Samples in the window.                */
    uint32_t         src;       /* DRV_BARCODE_SRC_xxx bits.             */
    bool             b_clipped; /* Pass too slow: its start was lost.    */
} drv_barcode_pass_t;

ER   drv_barcode_init(void);
void drv_barcode_sample_isr(uint32_t idx);
bool drv_barcode_pass_done(void);
void drv_barcode_take_pass(drv_barcode_pass_t * p_pass);

#endif /* DRV_BARCODE_H */

/*** end of file ***/
