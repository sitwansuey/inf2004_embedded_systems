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
     * negative = down/join failed, 1 = joined, 2 = joined + got an IP. */
    volatile int32_t link_status;
    volatile uint32_t ip_addr;
} T_CYW43_UTK_STATUS;

int32_t cyw43_utk_start(void);
void cyw43_utk_get_status(T_CYW43_UTK_STATUS *status);
void cyw43_utk_task(int32_t stacd, void *exinf);

/* Re-join after a mid-run AP loss (cyw43_utk.c). A dropped association reads
 * DOWN (0: EV_DISASSOC zeroed the driver's join state) or JOIN (1: link-down
 * event, join still marked active), or NOIP (2) if DHCP stalls -- never the
 * negative statuses the 5 s join retry handles, so without this the radio
 * stays off the network until reboot. Once the link has been UP, no UP for
 * WIFI_LOST_REJOIN_US leaves the AP and joins again WIFI_LEAVE_SETTLE_US
 * later, on a later poll, so the leave's own EV_DISASSOC cannot zero the new
 * join's state. Before the first UP the same applies after the longer
 * WIFI_FIRST_JOIN_US, counted from the boot join (the task calls
 * wifi_rejoin_joined() for it): an AP that drops the robot during its first
 * join no longer leaves it off the network until reboot. Every join restarts
 * the wait: attempts stay >= 5 s apart, for ever. Pure (no driver calls) so
 * the host tests can drive it. */
#define WIFI_LOST_REJOIN_US  (10 * 1000 * 1000ull)
#define WIFI_FIRST_JOIN_US   (20 * 1000 * 1000ull)
#define WIFI_LEAVE_SETTLE_US (1000 * 1000ull)
enum { WIFI_REJOIN_WAIT, WIFI_REJOIN_LEAVE, WIFI_REJOIN_JOIN };
typedef struct { uint64_t alive_us, join_us; uint8_t was_up; } T_WIFI_REJOIN;

/* up: the link status is UP; failed: it is negative (the 5 s retry's case). */
static inline int wifi_rejoin_step(T_WIFI_REJOIN *w, int up, int failed,
                                   uint64_t now_us)
{
    if(up) {
        w->was_up = 1; w->alive_us = now_us; w->join_us = 0;
        return WIFI_REJOIN_WAIT;
    }
    if(w->join_us != 0) {
        if(now_us < w->join_us) return WIFI_REJOIN_WAIT;
        w->join_us = 0; w->alive_us = now_us;
        return WIFI_REJOIN_JOIN;
    }
    if(failed || now_us - w->alive_us <
       (w->was_up? WIFI_LOST_REJOIN_US: WIFI_FIRST_JOIN_US)) {
        return WIFI_REJOIN_WAIT;
    }
    w->join_us = now_us + WIFI_LEAVE_SETTLE_US;
    return WIFI_REJOIN_LEAVE;
}

/* A join was requested (at boot, or by the 5 s retry): restart the wait. */
static inline void wifi_rejoin_joined(T_WIFI_REJOIN *w, uint64_t now_us)
{
    w->alive_us = now_us; w->join_us = 0;
}

#endif
