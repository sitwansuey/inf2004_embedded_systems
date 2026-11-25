#include "watchdog_policy.h"

/* Monitored loops from the highest task priority down, and their core
 * (app_program task priorities; keep in step with them). */
static const uint8_t by_priority[HEALTH_HB_COUNT] = {
    HEALTH_HB_NET, HEALTH_HB_VC, HEALTH_HB_MOTOR, HEALTH_HB_TELEMETRY,
    HEALTH_HB_IMU};
static const uint8_t core_of[HEALTH_HB_COUNT] = {[HEALTH_HB_VC]        = 0,
                                                 [HEALTH_HB_MOTOR]     = 0,
                                                 [HEALTH_HB_NET]       = 0,
                                                 [HEALTH_HB_TELEMETRY] = 1,
                                                 [HEALTH_HB_IMU]       = 1};

void
wdt_policy_init (wdt_policy_t * p)
{
    *p = (wdt_policy_t){0};
}

uint32_t
wdt_policy_missing (const wdt_policy_t * p, const wdt_policy_config_t * c,
                    uint32_t now_ms)
{
    uint32_t missing = c->required_mask & ~p->seen_mask;
    for (uint32_t i = 0; i < HEALTH_HB_COUNT; ++i)
    {
        uint32_t bit = 1U << i;
        if ((c->required_mask & bit) && (p->seen_mask & bit) &&
            now_ms - p->changed_ms[i] > c->window_ms)
        {
            missing |= bit;
        }
    }
    return missing;
}

wdt_action_t
wdt_policy_step (wdt_policy_t * p, const wdt_policy_config_t * c,
                 const uint32_t count[HEALTH_HB_COUNT], uint32_t now_ms,
                 bool boot_done)
{
    for (uint32_t i = 0; i < HEALTH_HB_COUNT; ++i)
    {
        if (count[i] != p->count[i])
        {
            p->count[i]      = count[i];
            p->changed_ms[i] = now_ms;
            p->seen_mask |= 1U << i;
        }
    }
    if (p->tripped)
    {
        return WDT_HOLD;
    }
    if (!p->armed)
    {
        if (now_ms < c->grace_ms || !boot_done ||
            wdt_policy_missing(p, c, now_ms))
        {
            return WDT_WAIT;
        }
        p->armed = 1;
        return WDT_ARM;
    }
    bool     trip = false;
    uint32_t mask = 0;
    for (uint32_t i = 0; i < HEALTH_HB_COUNT; ++i)
    {
        uint32_t age = now_ms - p->changed_ms[i];
        if (!(c->required_mask & (1U << i)))
        {
            continue;
        }
        if (age > c->window_ms)
        {
            trip = true;
        }
        if (age > c->window_ms / 2U)
        {
            mask |= 1U << i;
        }
    }
    if (!trip)
    {
        return WDT_FEED;
    }
    /* Per core, the highest-priority stale loop; across cores, the one
     * stuck longest (ties: core 0). */
    uint32_t named = HEALTH_HB_COUNT, named_age = 0;
    for (uint32_t core = 0; core < 2U; ++core)
    {
        for (uint32_t k = 0; k < HEALTH_HB_COUNT; ++k)
        {
            uint32_t i = by_priority[k];
            if (core_of[i] != core || !(mask & (1U << i)))
            {
                continue;
            }
            uint32_t age = now_ms - p->changed_ms[i];
            if (named == HEALTH_HB_COUNT || age > named_age)
            {
                named     = i;
                named_age = age;
            }
            break;
        }
    }
    p->tripped      = 1;
    p->stale_id     = (uint8_t) named;
    p->stale_age_ms = named_age;
    p->stale_mask   = (uint8_t) mask;
    return WDT_STALE;
}

const char *
wdt_heartbeat_name (uint32_t id)
{
    static const char * const names[HEALTH_HB_COUNT] = {
        "VC", "motor", "network", "telemetry", "IMU"};
    return id < HEALTH_HB_COUNT ? names[id] : "?";
}

const char *
wdt_reason_name (uint32_t reason)
{
    static const char * const names[HEALTH_REASON_COUNT] = {
        "none",       "panic",           "hard assert",   "lwIP assert",
        "hard fault", "heartbeat stale", "watchdog armed"};
    return reason < HEALTH_REASON_COUNT ? names[reason] : "?";
}

/*** end of file ***/
