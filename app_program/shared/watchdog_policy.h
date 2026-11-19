/* Watchdog supervisor decision (pure C, host-tested in
 * testing/integration/watchdog_test.c). app_watchdog.c calls
 * wdt_policy_step() every 100 ms from a kernel cyclic handler with the
 * heartbeat counters (include/tm/robot_health.h) and does what it returns.
 *
 * Arming: never before grace_ms, never before boot_done (the network task
 * finished cyw43 init and its first join), and only when every required
 * heartbeat has advanced at least once and none is older than window_ms.
 * A loop that hangs during boot therefore leaves the watchdog off (the
 * console says which loop) instead of causing a reboot loop.
 * Armed: FEED while every required heartbeat advanced within window_ms;
 * the first one older than that latches STALE (record it, reboot) and
 * every later step returns HOLD (never fed again).
 *
 * Naming: a loop that hangs on a core starves the lower-priority loops on
 * that core, and they go stale at about the same time. So STALE names the
 * highest-priority stale loop on its core (core 0: network P4 > VC P5 >
 * motor P14; core 1: telemetry P13 > IMU P18), and stale_mask lists every
 * loop older than window_ms / 2 (each beats every 10-110 ms when healthy).
 * With stale loops on both cores, the one stuck longest is named. A hang
 * in an unmonitored higher-priority task (USB P3) or an interrupt storm
 * looks like a network hang: every core-0 loop is in the mask. */
#ifndef CSC02_WATCHDOG_POLICY_H
#define CSC02_WATCHDOG_POLICY_H
#include <stdbool.h>
#include <stdint.h>
#include "../../include/tm/robot_health.h"

typedef struct {
    uint32_t window_ms;     /* a heartbeat older than this is stale */
    uint32_t grace_ms;      /* no arming before this uptime */
    uint32_t required_mask; /* bit i: heartbeat i is monitored */
} wdt_policy_config_t;

typedef struct {
    uint32_t count[HEALTH_HB_COUNT];      /* last value seen */
    uint32_t changed_ms[HEALTH_HB_COUNT]; /* when it last advanced */
    uint32_t seen_mask;                   /* advanced at least once */
    uint32_t stale_age_ms;                /* STALE: its age then */
    uint8_t armed, tripped, stale_id;
    uint8_t stale_mask;                   /* STALE: every loop older than window_ms / 2 */
} wdt_policy_t;

typedef enum {
    WDT_WAIT,  /* not armed yet: leave the watchdog off */
    WDT_ARM,   /* start the watchdog now (and feed) */
    WDT_FEED,  /* feed it */
    WDT_STALE, /* stale_id stopped: write the record, reboot, never feed */
    WDT_HOLD   /* already stale: never feed again */
} wdt_action_t;

void wdt_policy_init(wdt_policy_t *p);
wdt_action_t wdt_policy_step(wdt_policy_t *p, const wdt_policy_config_t *c,
                             const uint32_t count[HEALTH_HB_COUNT],
                             uint32_t now_ms, bool boot_done);
/* Required heartbeats never seen, or older than window_ms, at now_ms. */
uint32_t wdt_policy_missing(const wdt_policy_t *p, const wdt_policy_config_t *c,
                            uint32_t now_ms);
/* "VC", "motor", "network", "telemetry", "IMU"; "?" out of range. */
const char *wdt_heartbeat_name(uint32_t id);
/* "panic", "hard assert", "lwIP assert", "hard fault", "heartbeat stale",
 * "watchdog armed"; "?" out of range. */
const char *wdt_reason_name(uint32_t reason);
#endif
