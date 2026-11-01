/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file mqtt_config.example.h
 *
 * @brief Local broker settings for the MQTT link (TM_MQTT=1 builds).
 *
 * @par
 * Copy this file to mqtt_config.h in this same folder, fill in the values
 * below, then change the 0 on the MQTT_LINK_CFG_SET line to 1. The copy is
 * ignored by Git through app_program/subsystem1/.gitignore -- broker
 * addresses and credentials differ between networks and must never be
 * committed.
 *
 * @par
 * It is a value to flip rather than a line to delete, so that an unedited
 * copy fails loudly at compile time instead of building fine and failing to
 * connect at runtime.
 *
 * @par
 * mqtt_link.c checks the length limits below at compile time. A value that
 * is too long stops the build.
 */

#ifndef MQTT_CONFIG_H
#define MQTT_CONFIG_H

#define MQTT_LINK_CFG_SET               0

#if !MQTT_LINK_CFG_SET
#error "mqtt_config.h: set the broker details and MQTT_LINK_CFG_SET to 1"
#endif

/* Broker host name or dotted IPv4 address. A dotted address is used as it is.
 * A name is resolved through DNS. The build requires NET=lwip either way;
 * name resolution additionally requires LWIP_DNS, which this port's
 * lwipopts.h always enables (see lib/liblwip/sysdepend/pico_rp2040/).
 */
#define MQTT_LINK_CFG_BROKER_HOST       "mqtt.example.com"

/* Broker TCP port, 1 to 65535. 1883 is the unencrypted MQTT default. */
#define MQTT_LINK_CFG_BROKER_PORT       1883U

/* Client ID, at most 32 characters. It must be unique on the broker: a second
 * client connecting with the same ID disconnects the first.
 */
#define MQTT_LINK_CFG_CLIENT_ID         "robocar-buddy1"

/* Topic prefix, at most 32 characters, with no leading or trailing slash and
 * no wildcard characters. Every topic the car publishes starts with it, and
 * it subscribes to "<prefix>/cmd/#".
 */
#define MQTT_LINK_CFG_TOPIC_PREFIX      "robocar/car1"

/* Optional broker login, each at most 32 characters. Use "" for none. */
#define MQTT_LINK_CFG_USERNAME          ""
#define MQTT_LINK_CFG_PASSWORD          ""

#endif /* MQTT_CONFIG_H */

/*** end of file ***/
