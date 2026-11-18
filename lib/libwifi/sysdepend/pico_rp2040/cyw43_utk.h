#ifndef CYW43_UTK_H
#define CYW43_UTK_H

#include <stdint.h>

typedef struct {
    volatile int32_t init_result;
    volatile uint32_t init_complete;
    volatile uint32_t ready;
    volatile uint32_t scan_active;
    volatile uint32_t scan_complete;
    volatile uint32_t scan_count;
    volatile int32_t strongest_rssi;
    volatile uint32_t poll_count;
    volatile uint32_t owner_prc;
    uint8_t mac[6];
    /* Only meaningful on a NET=lwip build -- see cyw43_utk_task(). link_status
     * mirrors cyw43_tcpip_link_status()'s CYW43_LINK_* values (cyw43.h):
     * negative = the join failed, 0 = DOWN, 1 = JOIN (a join is active: just
     * requested, or the link was lost and the driver still waits), 2 = NOIP
     * (associated, no address yet), 3 = UP (associated, with an address). */
    volatile int32_t link_status;
    volatile uint32_t ip_addr;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);

#endif
