/*
 * SPDX-License-Identifier: Apache-2.0
 */

/** @file s1_radio.c
 *
 * @brief Reads the radio link state (see s1_radio.h).
 *
 * @par
 * The only call is cyw43_utk_get_status(), the one read TEAM.md section 3
 * allows a task to make. Nothing here calls the radio driver or lwIP.
 *
 * NOTE: Use CYW43_LINK_UP, not a number. The comment in cyw43_utk.h calls 2
 * "joined + got an IP", but in cyw43.h 2 is CYW43_LINK_NOIP and CYW43_LINK_UP
 * is 3.
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

#endif /* TM_MQTT */

/*** end of file ***/

