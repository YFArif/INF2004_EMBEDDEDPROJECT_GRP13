/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_radio.h
 *
 * @brief Reads the radio link state for S1 code that also uses the kernel.
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

#endif /* TM_MQTT */

#endif /* S1_RADIO_H */

/*** end of file ***/

