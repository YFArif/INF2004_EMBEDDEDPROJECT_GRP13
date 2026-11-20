/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_radio.c
 *
 * @brief Reads the radio link state (see s1_radio.h).
 *
 * @par
 * s1_radio_link_read() makes one call, cyw43_utk_get_status(), the one
 * read TEAM.md section 3 allows a task to make. s1_radio_rssi_read() calls
 * the radio driver, so only the radio service task may call it (it runs
 * there from mqtt_link_poll(); assumption A15).
 *
 * NOTE: Use CYW43_LINK_UP, not a number. Until branch s1-fr17 the comment in
 * cyw43_utk.h called 2 "joined + got an IP", but in cyw43.h 2 is
 * CYW43_LINK_NOIP and CYW43_LINK_UP is 3.
 */

#include "s1_radio.h"

#if TM_MQTT

#include <stddef.h>

#include "cyw43.h"
#include "cyw43_utk.h"

/*!
 * @brief Read the radio link status.
 *
 * Safe from any task. Only link_status is used from the copy, and it is one
 * aligned word, so it cannot be half written.
 *
 * @param[out] p_link_status Receives the status (CYW43_LINK_xxx in cyw43.h).
 *
 * @return True if the link is CYW43_LINK_UP; false if it is not, or if
 *         p_link_status is NULL.
 */
bool
s1_radio_link_read (int32_t * p_link_status)
{
    T_CYW43_UTK_STATUS radio;
    bool               b_link_up = false;

    if (NULL != p_link_status)
    {
        cyw43_utk_get_status(&radio);

        *p_link_status = radio.link_status;
        b_link_up      = (CYW43_LINK_UP == *p_link_status);
    }

    return (b_link_up);
}

/*!
 * @brief Read the signal strength of the joined access point.
 *
 * WARNING: Radio service task only (see s1_radio.h).
 *
 * @param[out] p_rssi_dbm Receives the RSSI in dBm (negative on a real
 *                        link). Unchanged when false is returned.
 *
 * @return True if the chip answered; false if p_rssi_dbm is NULL, the
 *         station is not active, or the request failed.
 */
bool
s1_radio_rssi_read (int32_t * p_rssi_dbm)
{
    int32_t rssi = 0;
    bool    b_read = false;

    if ((NULL != p_rssi_dbm)
        && (0 == cyw43_wifi_get_rssi(&cyw43_state, &rssi)))
    {
        *p_rssi_dbm = rssi;
        b_read      = true;
    }

    return (b_read);
}

#endif /* TM_MQTT */

/*** end of file ***/

