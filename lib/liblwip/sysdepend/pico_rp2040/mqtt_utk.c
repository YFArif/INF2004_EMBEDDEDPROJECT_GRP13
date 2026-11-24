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
 * @par Radio power save
 * On its first call each boot, mqtt_utk_start() turns the radio's power
 * save off (CYW43_NONE_PM) and prints one console line. The driver's
 * default, CYW43_PERFORMANCE_PM, lets the radio doze between beacons, so
 * the phone hotspot holds every packet for the car until it wakes; that
 * delays command acks (NFR15) and fits the uplink silences in S1-U21.
 * cyw43_wifi_set_up() sets the default only when the radio is first
 * switched on, so a WiFi rejoin keeps the setting.
 *
 * NOTE: Built only in MQTT=1 builds (MQTT_PORT_OBJS in
 * build_make/pico_rp2040.mk), as the demo was.
 */

#include "mqtt_utk.h"

#include <stdbool.h>

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

static bool gb_pm_off_done = false;

/*!
 * @brief Advance the S1 MQTT driver by one step.
 *
 * On the first call each boot it also turns the radio's power save off.
 * One attempt only: a failure is shown on the console, not retried on
 * every 2 ms pass.
 *
 * NOTE: The name is the template's. cyw43_utk.c calls it on every poll
 * pass while the link is up, not once. It runs on cyw43_utk_task, the
 * only task that may call the cyw43 driver (TEAM.md section 3).
 */
void
mqtt_utk_start (void)
{
    if (!gb_pm_off_done)
    {
        gb_pm_off_done = true;
        /* cyw43_wifi_pm() returns an int error code, 0 on success. */
        tm_printf((UB *)"S1: wifi pm off result=%d\n",
                  cyw43_wifi_pm(&cyw43_state, CYW43_NONE_PM));
    }

    mqtt_link_poll(netif_default);
}

/*** end of file ***/

