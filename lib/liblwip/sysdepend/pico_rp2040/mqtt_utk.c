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
 * NOTE: Built only in MQTT=1 builds (MQTT_PORT_OBJS in
 * build_make/pico_rp2040.mk), as the demo was.
 */

#include "mqtt_utk.h"

#include "lwip/netif.h"

/* app_program/ is not on the include path of lib/ objects (INCPATH in
 * build_make/makefile), so the driver's header is named relative to this
 * file. The compiler then checks the call below against the real
 * prototype.
 */
#include "../../../../app_program/subsystem1/mqtt_link.h"

/*!
 * @brief Advance the S1 MQTT driver by one step.
 *
 * NOTE: The name is the template's. cyw43_utk.c calls it on every poll
 * pass while the link is up, not once.
 */
void
mqtt_utk_start (void)
{
    mqtt_link_poll(netif_default);
}

/*** end of file ***/
