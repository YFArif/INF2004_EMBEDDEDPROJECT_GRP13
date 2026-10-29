/** @file drv_button.h
 *
 * @brief Robo Pico user buttons, polled: Button 1 (GP20) starts a run,
 *        Button 2 (GP21) asks for calibration. A press pulls the pin low.
 *
 * @par
 * Only vehicle_ctrl_task calls drv_button_update(). Polling every
 * 50 ms is plenty for a human press and needs no GPIO interrupt, so the
 * buttons stay out of the shared IO_IRQ_BANK0 handler (IF-11).
 *
 * NOTE: In make TEST=1 images the S2 and S5 test steps also read GP20
 * (read-only) to wait for "next step"; vehicle_ctrl_task then ignores the
 * buttons, so a test step press never starts a run.
 */

#ifndef DRV_BUTTON_H
#define DRV_BUTTON_H

#include <stdbool.h>
#include <stdint.h>

#define DRV_BUTTON_START_PIN    (20u)   /* Button 1.                     */
#define DRV_BUTTON_CAL_PIN      (21u)   /* Button 2.                     */

/** Which button. */
typedef enum
{
    DRV_BUTTON_START = 0,
    DRV_BUTTON_CAL,
    DRV_BUTTON_COUNT
} drv_button_id_t;

/** What a button did since the last update. */
typedef enum
{
    DRV_BUTTON_NONE = 0,        /* Nothing new.                          */
    DRV_BUTTON_PRESSED,         /* Short press, reported on release.     */
    DRV_BUTTON_HELD             /* Held for 2 s, reported once.          */
} drv_button_event_t;

void               drv_button_init(void);
bool               drv_button_is_down(drv_button_id_t button);
drv_button_event_t drv_button_update(drv_button_id_t button,
                                     uint32_t elapsed_ms);

#endif /* DRV_BUTTON_H */

/*** end of file ***/
