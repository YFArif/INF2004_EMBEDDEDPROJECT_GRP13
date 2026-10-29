/** @file drv_button.c
 *
 * @brief Robo Pico user buttons, polled, with debounce and long press.
 *
 * @par
 * A press must last DEBOUNCE_MS to count. Releasing it before HOLD_MS is
 * a short press (PRESSED); holding it HOLD_MS is reported once as HELD,
 * and the release that follows reports nothing.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_button.h"
#include "drv_s5_hw.h"

#define DEBOUNCE_MS         (50u)
#define HOLD_MS             (2000u)     /* Week 6 report 5: "held 2 s".  */

/** Per-button tracking. */
typedef struct
{
    uint32_t    down_ms;        /* How long it has been held so far.     */
    bool        b_held_sent;    /* HELD already reported for this press. */
} button_track_t;

static const uint32_t g_pins[DRV_BUTTON_COUNT] =
{
    DRV_BUTTON_START_PIN, DRV_BUTTON_CAL_PIN
};

static button_track_t g_track[DRV_BUTTON_COUNT] = {{0u, false}, {0u, false}};

/*!
 * @brief Set both button pins as inputs with pull-ups.
 */
void
drv_button_init (void)
{
    for (uint32_t idx = 0u; idx < DRV_BUTTON_COUNT; idx++)
    {
        drv_s5_hw_pin_input_pullup(g_pins[idx]);
        g_track[idx].down_ms     = 0u;
        g_track[idx].b_held_sent = false;
    }
}

/*!
 * @brief Read a button right now (no debounce).
 *
 * @param[in] button Which button.
 *
 * @return true while it is pressed.
 */
bool
drv_button_is_down (drv_button_id_t button)
{
    // A press pulls the pin to ground.
    return (!drv_s5_hw_pin_read(g_pins[button]));
}

/*!
 * @brief Track a button; call it regularly.
 *
 * @param[in] button     Which button.
 * @param[in] elapsed_ms Time since the previous call for this button.
 *
 * @return PRESSED, HELD or NONE.
 */
drv_button_event_t
drv_button_update (drv_button_id_t button, uint32_t elapsed_ms)
{
    drv_button_event_t event   = DRV_BUTTON_NONE;
    button_track_t *   p_track = &g_track[button];

    if (drv_button_is_down(button))
    {
        p_track->down_ms += elapsed_ms;

        if ((p_track->down_ms >= HOLD_MS) && (!p_track->b_held_sent))
        {
            p_track->b_held_sent = true;
            event                = DRV_BUTTON_HELD;
        }
    }
    else
    {
        // Released: a debounced press shorter than a hold is a click.
        if ((p_track->down_ms >= DEBOUNCE_MS) && (!p_track->b_held_sent))
        {
            event = DRV_BUTTON_PRESSED;
        }

        p_track->down_ms     = 0u;
        p_track->b_held_sent = false;
    }

    return (event);
}

/*** end of file ***/
