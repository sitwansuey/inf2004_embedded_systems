/* Watchdog supervisor decision (app_program/shared/watchdog_policy.c) and
 * the crash record codec (include/tm/robot_health.h). The register layer
 * (lib/libtm/sysdepend/pico_rp2040/robot_health.c) is not built here. */
#include "../../app_program/shared/watchdog_policy.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const wdt_policy_config_t cfg = {1000U, 20000U, (1U << HEALTH_HB_COUNT) - 1U};
static uint32_t count[HEALTH_HB_COUNT];

static void beat_all(void)
{
    for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) ++count[i];
}

/* Steps every 100 ms from `from` to `to` (inclusive), all loops beating
 * except `skip` (HEALTH_HB_COUNT: none); returns the last action. */
static wdt_action_t run(wdt_policy_t *p, const wdt_policy_config_t *c, uint32_t from,
                        uint32_t to, unsigned skip, bool boot_done)
{
    wdt_action_t a = WDT_WAIT;
    for (uint32_t t = from; t <= to; t += 100U) {
        for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) if (i != skip) ++count[i];
        a = wdt_policy_step(p, c, count, t, boot_done);
    }
    return a;
}

static void policy(void)
{
    wdt_policy_t p;

    /* Grace: never armed before 20 s, however healthy. */
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &cfg, 100U, 19900U, HEALTH_HB_COUNT, true) == WDT_WAIT && !p.armed);
    /* Boot not finished (network before its first join): still off. */
    assert(run(&p, &cfg, 20000U, 25000U, HEALTH_HB_COUNT, false) == WDT_WAIT && !p.armed);
    /* All fresh and booted: armed once, then fed every step. */
    beat_all();
    assert(wdt_policy_step(&p, &cfg, count, 25100U, true) == WDT_ARM && p.armed);
    assert(run(&p, &cfg, 25200U, 60000U, HEALTH_HB_COUNT, true) == WDT_FEED);
    /* A loop beating every 900 ms (inside the 1 s window) never trips. */
    for (uint32_t t = 60100U; t <= 70000U; t += 100U) {
        for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i)
            if (i != HEALTH_HB_NET || t % 900U == 0U) ++count[i];
        assert(wdt_policy_step(&p, &cfg, count, t, true) == WDT_FEED);
    }

    /* One stale loop: STALE once, naming it, then HOLD (never fed again). */
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &cfg, 100U, 20000U, HEALTH_HB_COUNT, true) == WDT_ARM);
    uint32_t last = 20000U;
    wdt_action_t a = WDT_FEED;
    for (uint32_t t = 20100U; a == WDT_FEED; t += 100U) {
        for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) if (i != HEALTH_HB_VC) ++count[i];
        a = wdt_policy_step(&p, &cfg, count, t, true);
        last = t;
    }
    assert(a == WDT_STALE && p.stale_id == HEALTH_HB_VC && last == 21100U && p.stale_age_ms == 1100U &&
           p.stale_mask == 1U << HEALTH_HB_VC);
    beat_all(); /* even if it recovers, the decision stands */
    assert(wdt_policy_step(&p, &cfg, count, last + 100U, true) == WDT_HOLD);
    assert(run(&p, &cfg, last + 200U, last + 5000U, HEALTH_HB_COUNT, true) == WDT_HOLD);

    /* Two loops stale when a late step looks (the handler was held off):
     * the one stuck longest is named, not the lower id. */
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &cfg, 100U, 20000U, HEALTH_HB_COUNT, true) == WDT_ARM);
    assert(run(&p, &cfg, 20100U, 20300U, HEALTH_HB_IMU, true) == WDT_FEED);
    for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i)
        if (i != HEALTH_HB_IMU && i != HEALTH_HB_MOTOR) ++count[i];
    assert(wdt_policy_step(&p, &cfg, count, 22000U, true) == WDT_STALE &&
           p.stale_id == HEALTH_HB_IMU && p.stale_age_ms == 2000U &&
           p.stale_mask == ((1U << HEALTH_HB_IMU) | (1U << HEALTH_HB_MOTOR)));

    /* A hang starves the lower-priority loops on its core, which go stale
     * with it: the highest-priority stale loop on that core is named, and
     * the mask lists them all. The network task (P4) beat last and stopped
     * together with VC (P5) and motor (P14): the network is named. */
    static const struct { unsigned stops; unsigned named; } hang[] = {
        {(1U << HEALTH_HB_NET) | (1U << HEALTH_HB_VC) | (1U << HEALTH_HB_MOTOR), HEALTH_HB_NET},
        {(1U << HEALTH_HB_VC) | (1U << HEALTH_HB_MOTOR), HEALTH_HB_VC},
        {(1U << HEALTH_HB_TELEMETRY) | (1U << HEALTH_HB_IMU), HEALTH_HB_TELEMETRY},
        {1U << HEALTH_HB_MOTOR, HEALTH_HB_MOTOR},
    };
    for (unsigned h = 0; h < sizeof(hang) / sizeof(hang[0]); ++h) {
        for (unsigned late = 0; late < 2U; ++late) {
            wdt_policy_init(&p);memset(count, 0, sizeof(count));
            assert(run(&p, &cfg, 100U, 20000U, HEALTH_HB_COUNT, true) == WDT_ARM);
            /* late: the named loop beats one step after the others stop */
            a = WDT_FEED;
            for (uint32_t t = 20100U; a == WDT_FEED; t += 100U) {
                for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i)
                    if (!(hang[h].stops & (1U << i)) || (late && t == 20100U && i == hang[h].named)) ++count[i];
                a = wdt_policy_step(&p, &cfg, count, t, true);
            }
            assert(a == WDT_STALE && p.stale_id == hang[h].named && p.stale_mask == hang[h].stops);
        }
    }

    /* A loop that never ran (or hung during boot) keeps the watchdog off
     * instead of rebooting in a loop, and is reported as missing. */
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &cfg, 100U, 40000U, HEALTH_HB_TELEMETRY, true) == WDT_WAIT);
    assert(wdt_policy_missing(&p, &cfg, 40000U) == 1U << HEALTH_HB_TELEMETRY);
    /* It starts late: armed then. */
    ++count[HEALTH_HB_TELEMETRY];beat_all();
    assert(wdt_policy_step(&p, &cfg, count, 40100U, true) == WDT_ARM);
    /* Stalled before arming: not armed while it is older than the window. */
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &cfg, 100U, 5000U, HEALTH_HB_COUNT, true) == WDT_WAIT);
    assert(run(&p, &cfg, 5100U, 30000U, HEALTH_HB_MOTOR, true) == WDT_WAIT && !p.armed);
    assert(wdt_policy_missing(&p, &cfg, 30000U) == 1U << HEALTH_HB_MOTOR);

    /* No radio in the build: the network heartbeat is not required. */
    wdt_policy_config_t no_net = cfg;
    no_net.required_mask &= ~(1U << HEALTH_HB_NET);
    wdt_policy_init(&p);memset(count, 0, sizeof(count));
    assert(run(&p, &no_net, 100U, 20000U, HEALTH_HB_NET, true) == WDT_ARM);
    assert(run(&p, &no_net, 20100U, 30000U, HEALTH_HB_NET, true) == WDT_FEED);

    /* Counter wrap is just another change. */
    wdt_policy_init(&p);
    for (unsigned i = 0; i < HEALTH_HB_COUNT; ++i) count[i] = UINT32_MAX - 3U;
    assert(run(&p, &cfg, 100U, 20000U, HEALTH_HB_COUNT, true) == WDT_ARM);
    assert(run(&p, &cfg, 20100U, 21000U, HEALTH_HB_COUNT, true) == WDT_FEED);

    assert(!strcmp(wdt_heartbeat_name(HEALTH_HB_VC), "VC") && !strcmp(wdt_heartbeat_name(HEALTH_HB_NET), "network") &&
           !strcmp(wdt_heartbeat_name(HEALTH_HB_COUNT), "?"));
    puts("Watchdog supervisor: no arming before the 20 s grace, the network's first join and every loop running; all loops fresh feed; "
         "one loop older than 1 s trips once and feeding stops for good; the highest-priority stale loop on its core is named "
         "(a network hang is not blamed on the starved VC) with every stale loop in the mask; a loop dead at boot keeps it off PASS");
}

static void record(void)
{
    uint32_t word[4];
    health_record_t in = {HEALTH_REASON_PANIC, 0x1000ABCDU, 0x10020000U, 332305U, 0U}, out;

    health_record_encode(&in, word);
    assert(health_record_decode(word, &out) && !memcmp(&in, &out, sizeof(in)));
    for (uint32_t r = HEALTH_REASON_PANIC; r < HEALTH_REASON_COUNT; ++r) {
        in.reason = r;in.value = r * 7U;in.value2 = ~r;in.uptime_ms = HEALTH_UPTIME_MAX - r;
        in.crash_count = r;
        health_record_encode(&in, word);
        assert(health_record_decode(word, &out) && !memcmp(&in, &out, sizeof(in)));
    }
    /* Uptime holds at 24 bits (about 4.6 h), the count at 255; neither
     * spills into the other. */
    in.reason = HEALTH_REASON_HARD_FAULT;in.uptime_ms = UINT32_MAX;in.crash_count = 1000U;
    health_record_encode(&in, word);
    assert(health_record_decode(word, &out) && out.uptime_ms == HEALTH_UPTIME_MAX && out.crash_count == 255U);
    in.uptime_ms = 0x01000005U;in.crash_count = 2U;health_record_encode(&in, word);
    assert(health_record_decode(word, &out) && out.uptime_ms == HEALTH_UPTIME_MAX && out.crash_count == 2U);
    /* Heartbeat value: named loop and the stale mask. */
    assert(HEALTH_HB_VALUE_ID(HEALTH_HB_VALUE(HEALTH_HB_NET, 0x07U)) == HEALTH_HB_NET &&
           HEALTH_HB_VALUE_MASK(HEALTH_HB_VALUE(HEALTH_HB_NET, 0x07U)) == 0x07U);
    /* Power-on (zeroed or random scratch), a damaged magic or check byte,
     * reason 0 or unknown: no record. */
    memset(word, 0, sizeof(word));assert(!health_record_decode(word, &out));
    in.reason = HEALTH_REASON_HARD_FAULT;health_record_encode(&in, word);
    word[0] ^= 0x00010000U;assert(!health_record_decode(word, &out));
    health_record_encode(&in, word);word[0] ^= 0x00000100U;assert(!health_record_decode(word, &out));
    word[0] = (HEALTH_RECORD_MAGIC << 16) | 0xFF00U;assert(!health_record_decode(word, &out));
    word[0] = (HEALTH_RECORD_MAGIC << 16) | ((~(uint32_t)HEALTH_REASON_COUNT & 0xFFU) << 8) | HEALTH_REASON_COUNT;
    assert(!health_record_decode(word, &out));
    word[0] = 0xFFFFFFFFU;assert(!health_record_decode(word, &out));
    assert(!strcmp(wdt_reason_name(HEALTH_REASON_PANIC), "panic") &&
           !strcmp(wdt_reason_name(HEALTH_REASON_HEARTBEAT), "heartbeat stale") &&
           !strcmp(wdt_reason_name(HEALTH_REASON_COUNT), "?"));
    puts("Crash record: every reason round-trips (value, value2, uptime, crash count); zeroed, damaged or unknown records are rejected PASS");
}

/* Early crashes in a row: counted across reboots, the board holds after
 * HEALTH_HOLD_AFTER of them instead of rebooting in a loop. */
static void early_crashes(void)
{
    uint32_t word[4], prior = 0;
    health_record_t r = {HEALTH_REASON_PANIC, 0x10001234U, 0U, 900U, 0U};

    /* Boots that each panic 0.9 s in: 1, 2, 3 -> hold on the next boot. */
    for (uint32_t boot = 1; boot <= HEALTH_HOLD_AFTER; ++boot) {
        r.crash_count = health_next_crash_count(r.reason, r.uptime_ms, prior);
        health_record_encode(&r, word);
        prior = health_record_prior_crashes(word);
        assert(prior == boot && (prior >= HEALTH_HOLD_AFTER) == (boot == HEALTH_HOLD_AFTER));
    }
    /* Every crash reason counts; a late crash, a heartbeat stop or the
     * armed record starts again at 0. */
    for (uint32_t reason = HEALTH_REASON_PANIC; reason <= HEALTH_REASON_HARD_FAULT; ++reason)
        assert(health_next_crash_count(reason, HEALTH_EARLY_CRASH_MS - 1U, 1U) == 2U);
    assert(health_next_crash_count(HEALTH_REASON_PANIC, HEALTH_EARLY_CRASH_MS, 2U) == 0U);
    assert(health_next_crash_count(HEALTH_REASON_HEARTBEAT, 100U, 2U) == 0U);
    assert(health_next_crash_count(HEALTH_REASON_ARMED, 100U, 2U) == 0U);
    assert(health_next_crash_count(HEALTH_REASON_HARD_FAULT, 0U, 255U) == 255U);
    /* Only a crash record carries a count forward. */
    r.reason = HEALTH_REASON_HEARTBEAT;r.crash_count = 5U;health_record_encode(&r, word);
    assert(health_record_prior_crashes(word) == 0U);
    memset(word, 0, sizeof(word));assert(health_record_prior_crashes(word) == 0U);
    assert(!strcmp(wdt_reason_name(HEALTH_REASON_ARMED), "watchdog armed"));
    puts("Early crashes: counted across reboots, reset by a late crash or a heartbeat stop; the third in a row holds the board PASS");
}

int main(void)
{
    policy();
    record();
    early_crashes();
    return 0;
}
