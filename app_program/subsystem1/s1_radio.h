/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_radio.h
 *
 * @brief Reads the radio link state for S1 code that also uses the kernel,
 *        and the signal strength for the MQTT driver.
 *
 * @par Why it is separate
 * CYW43_LINK_UP is defined in cyw43.h, which must not be included next to the
 * kernel headers, because the two disagree about size_t. s1_radio.c includes
 * cyw43.h and no kernel header; its callers include only this file, which
 * uses plain stdint and stdbool types. It is the split mqtt_link.c and
 * mqtt_queue.c already use.
 *
 * @par Build gate
 * NOTE: The whole body is gated on TM_MQTT (default 0), like the rest of the
 * MQTT link, so builds without MQTT are unchanged.
 */

#ifndef S1_RADIO_H
#define S1_RADIO_H

#ifndef TM_MQTT
#define TM_MQTT 0
#endif

#if TM_MQTT

#include <stdbool.h>
#include <stdint.h>

bool s1_radio_link_read(int32_t * p_link_status);

/* WARNING: s1_radio_rssi_read() asks the radio chip (an ioctl, a few ms), so
 * only the radio service task, cyw43_utk_task, may call it (TEAM.md section
 * 3). mqtt_link_poll() calls it there; no application task may.
 */
bool s1_radio_rssi_read(int32_t * p_rssi_dbm);

#endif /* TM_MQTT */

#endif /* S1_RADIO_H */

/*** end of file ***/

