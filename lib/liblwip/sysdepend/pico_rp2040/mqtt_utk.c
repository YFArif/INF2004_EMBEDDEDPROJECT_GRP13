/** @file mqtt_utk.c
 *
 * @brief Hook from the radio service task into the S1 MQTT driver.
 *
 * @par
 * cyw43_utk_task() (lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c) calls
 * mqtt_utk_start() on every pass of its 2 ms poll loop while the link is
 * CYW43_LINK_UP, after sys_check_timeouts(). Each call advances the S1
 * driver (app_program/subsystem1/mqtt_link.c) by one non-blocking step;
 * the driver owns the broker session, topics, last will, queues and
 * reconnects. Everything runs on this one task (processor 1, core 0), as
 * TEAM.md section 3 requires.
 *
 * @par
 * Replaces the template's demo client; the broker settings now come from
 * app_program/subsystem1/mqtt_config.h, not config/wifi_secrets.h.
 *
 * @par Radio power save (NFR15 stage B, TEST_LOG S1-U21)
 * On its first call, the first pass with the link up, mqtt_utk_start()
 * turns the radio's power save off (CYW43_NONE_PM), reads the setting back
 * and prints one console line. Until then the driver keeps its default,
 * CYW43_PERFORMANCE_PM (PM2), set once in cyw43_wifi_set_up(). Once per
 * boot: nothing in cyw43-driver v1.1.1 resets the setting after boot, and
 * S1-U20 reads it back after a rejoin.
 *
 * NOTE: Built only in MQTT=1 builds (MQTT_PORT_OBJS in
 * build_make/pico_rp2040.mk), as the demo was.
 */

#include "mqtt_utk.h"

#include <stdbool.h>
#include <stdint.h>

#include "cyw43.h"
#include "lwip/netif.h"

/* tm/tmonitor.h pulls in only tk/typedef.h, not tk/tkernel.h, so it can
 * share this file with the cyw43 and lwIP headers, as in cyw43_utk.c.
 */
#include <tm/tmonitor.h>

/* app_program/ is not on the include path of lib/ objects (INCPATH in
 * build_make/makefile), so the driver's header is named relative to this
 * file. The compiler then checks the call below against the real
 * prototype.
 */
#include "../../../../app_program/subsystem1/mqtt_link.h"

/* The power-save mode is bits 0 to 3 of a cyw43 power-management value
 * (cyw43_pm_value() in cyw43.h).
 */
#define PM_MODE_MASK                0x0FU

/* One console line for the power-save setting. tm_sprintf() does not limit
 * its output. The widest line, in characters:
 *   "S1: wifi pm off result=" 23 + "-2147483648" 11 + " read=" 6
 *   + "-2147483648" 11 + " mode=" 6 + 2 (at most 15) + " pm=0x" 6 + 8
 *   + "\n" 1 = 74,
 * plus the NUL tm_sprintf() writes = 75 bytes. 96 leaves 21 bytes spare.
 */
#define PM_LINE_BYTES               96U

static bool gb_pm_off_done = false;

static void pm_off_once(void);

/*!
 * @brief Advance the S1 MQTT driver by one step.
 *
 * On the first call it also turns the radio's power save off, once per
 * boot (pm_off_once()).
 *
 * NOTE: The name is the template's. cyw43_utk.c calls it on every poll
 * pass while the link is up, not once.
 */
void
mqtt_utk_start (void)
{
    if (!gb_pm_off_done)
    {
        pm_off_once();
    }

    mqtt_link_poll(netif_default);
}

/*!
 * @brief Turn the radio's power save off and report it, once per boot.
 *
 * NOTE: Runs on cyw43_utk_task, the only task that may call the cyw43
 * driver (TEAM.md section 3); cyw43_wifi_pm() and cyw43_wifi_get_pm() take
 * the driver's lock themselves. One attempt per boot: a failure is shown on
 * the console, not retried on every 2 ms pass.
 */
static void
pm_off_once (void)
{
    UB       line[PM_LINE_BYTES];
    uint32_t pm_read = 0U;
    int32_t  set_result = 0;
    int32_t  get_result = 0;

    /* The casts are safe: int is 32 bits on this port. */
    set_result = (int32_t)cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM);
    get_result = (int32_t)cyw43_wifi_get_pm(&cyw43_state, &pm_read);
    gb_pm_off_done = true;

    /* The casts are safe: the format string is ASCII, an int holds an
     * int32_t on this port, and the mode is at most 15. The length that
     * tm_sprintf() returns is not needed.
     */
    (void)tm_sprintf(line,
                     (const UB *)"S1: wifi pm off result=%d read=%d mode=%u "
                                 "pm=0x%08x\n",
                     (int)set_result, (int)get_result,
                     (unsigned int)(pm_read & PM_MODE_MASK),
                     (unsigned int)pm_read);
    tm_printf(line);
}

/*** end of file ***/
