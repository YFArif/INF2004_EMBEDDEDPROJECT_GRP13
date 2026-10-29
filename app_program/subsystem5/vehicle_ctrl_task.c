/** @file vehicle_ctrl_task.c
 *
 * @brief vehicle_ctrl_task: wakes on any mission event, or every TICK_MS
 *        so that buttons are read and time limits are checked.
 *
 * @par Each wake-up
 * 1. Dashboard commands (mbf_cmd): carried out and acknowledged.
 * 2. EV_START, EV_STOP and EV_RESET raised directly by another task:
 *    carried out like commands, without an acknowledgement.
 * 3. Buttons (not in test images): Button 1 press = START, or STOP if a
 *    run is in progress; Button 1 held 2 s = RESET; Button 2 press =
 *    calibrate.
 * 4. The mission state machine, with the events of this wake-up.
 */

#include <stdbool.h>
#include <stdint.h>

#include "drv_button.h"
#include "s5_links.h"
#include "s5_mission.h"
#include "subsystem5.h"
#include "vehicle_ctrl_task.h"

#define TICK_MS             (50u)

static void handle_commands(uint32_t now_ms);
static void handle_event_commands(uint32_t events, uint32_t now_ms);
static void handle_buttons(uint32_t elapsed_ms, uint32_t now_ms);

/*!
 * @brief vehicle_ctrl_task entry point (created by subsystem5_start()).
 *
 * @param[in] stacd   Start code from tk_sta_tsk(); unused.
 * @param[in] p_exinf Extended information; unused.
 */
void
vehicle_ctrl_task (INT stacd, void * p_exinf)
{
    // The casts mark the kernel's start-up arguments as unused.
    (void) stacd;
    (void) p_exinf;

    uint32_t last_ms = subsystem5_now_ms();

    drv_button_init();
    s5_mission_init(last_ms);

    for (;;)
    {
        uint32_t events = 0u;

        // A time-out only means "no event this tick"; events stays 0.
        (void) subsystem5_mission_wait(EV_ALL, TICK_MS, &events);

        uint32_t now_ms = subsystem5_now_ms();

        handle_commands(now_ms);
        handle_event_commands(events, now_ms);
        handle_buttons(now_ms - last_ms, now_ms);
        s5_mission_step(events, now_ms);
        last_ms = now_ms;
    }
}

/*!
 * @brief Carry out and acknowledge every queued dashboard command.
 *
 * @param[in] now_ms Current time.
 */
static void
handle_commands (uint32_t now_ms)
{
    uint8_t opcode = 0u;
    uint8_t cmd_id = 0u;

    while (subsystem5_command_take(&opcode, &cmd_id))
    {
        s5_links_ack(opcode, cmd_id, s5_mission_command(opcode, now_ms));
    }
}

/*!
 * @brief Carry out EV_START, EV_STOP and EV_RESET set by another task.
 *
 * @param[in] events Mission events of this wake-up.
 * @param[in] now_ms Current time.
 */
static void
handle_event_commands (uint32_t events, uint32_t now_ms)
{
    // Results are not acknowledged: these did not come from the dashboard.
    if (0u != (events & EV_STOP))
    {
        (void) s5_mission_command(S5_CMD_STOP, now_ms);
    }

    if (0u != (events & EV_RESET))
    {
        (void) s5_mission_command(S5_CMD_RESET, now_ms);
    }

    if (0u != (events & EV_START))
    {
        (void) s5_mission_command(S5_CMD_START, now_ms);
    }
}

/*!
 * @brief Turn button presses into commands.
 *
 * @param[in] elapsed_ms Time since the previous call.
 * @param[in] now_ms     Current time.
 *
 * NOTE: Not in test images: there Button 1 is the "next step" button of
 * the S2 and S5 tests, and must not start a run.
 */
static void
handle_buttons (uint32_t elapsed_ms, uint32_t now_ms)
{
#if APP_TEST
    // The casts mark both values as unused in test images.
    (void) elapsed_ms;
    (void) now_ms;
#else
    drv_button_event_t start = drv_button_update(DRV_BUTTON_START,
                                                 elapsed_ms);
    drv_button_event_t cal   = drv_button_update(DRV_BUTTON_CAL, elapsed_ms);

    // The casts discard the result: a button has no acknowledgement.
    // Button 1 toggles: it also stops a run, so the robot can always be
    // stopped by hand.
    if (DRV_BUTTON_PRESSED == start)
    {
        (void) s5_mission_command(s5_mission_is_running() ? S5_CMD_STOP
                                                          : S5_CMD_START,
                                  now_ms);
    }
    else if (DRV_BUTTON_HELD == start)
    {
        (void) s5_mission_command(S5_CMD_RESET, now_ms);
    }
    else
    {
        // No Button 1 action.
    }

    if (DRV_BUTTON_PRESSED == cal)
    {
        s5_mission_calibrate(now_ms);
    }
#endif
}

/*** end of file ***/
