/*
 *	mqtt_utk.h
 *	MQTT client wiring for micro T-Kernel (RP2040, NO_SYS=1)
 */

#ifndef MQTT_UTK_H
#define MQTT_UTK_H

/*
 * Called by cyw43_utk_task() (see cyw43_utk.c) on every pass of its 2 ms
 * poll loop while the link is CYW43_LINK_UP. Advances the S1 MQTT driver
 * (app_program/subsystem1/mqtt_link.h) by one non-blocking step, with
 * mqtt_link_poll(netif_default). The broker settings come from
 * app_program/subsystem1/mqtt_config.h, not config/wifi_secrets.h.
 */
void mqtt_utk_start(void);

#endif /* MQTT_UTK_H */
