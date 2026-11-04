/* Actual acquisition task with deterministic kernel wakeups and sensor frames.
 * This checks pacing/quality logic, not real IRQ latency or bus hardware. */
#include "shim/subsystem4/subsystem4.h"
#include "shim/subsystem4/drv_pico_i2c.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

static uint64_t clock_ms, next_conversion_ms;
static unsigned int wakes, limit, sleeps, drains, polls, inits, bus_inits, exits;
static unsigned int invalid_publications;
static bool delayed_wake, clock_failure, drain_failure;
static s4_sensor_cache_t published, last_good;

uint64_t s4_kernel_now_us(void *context)
{
    (void)context;
    return clock_failure && clock_ms == 500U ? UINT64_MAX : clock_ms * 1000U;
}

void s4_pico_i2c_init(lsm303_bus_t *bus)
{
    ++bus_inits;
    *bus = (lsm303_bus_t){0};
}

ER tk_slp_tsk(INT timeout)
{
    REQUIRE(timeout == TMO_FEVR);
    ++sleeps;
    if (wakes == limit) return E_SYS; /* finite test and fail-closed exit */
    ++wakes;
    clock_ms += delayed_wake && wakes == 20U ? 40U : 10U;
    return E_OK;
}

INT tk_can_wup(ID task)
{
    REQUIRE(task == TSK_SELF && drains < wakes);
    ++drains;
    if (drain_failure && wakes == 5U) return E_SYS;
    return delayed_wake && wakes == 20U ? 3 : 0;
}

void tk_ext_tsk(void) { ++exits; }

lsm303_result_t lsm303dlhc_init(lsm303_device_t *device, const lsm303_bus_t *bus)
{
    REQUIRE(wakes > 0 && drains == wakes); /* no polling before a tick */
    REQUIRE(bus->now_us == s4_kernel_now_us);
    ++inits;
    device->bus = *bus;
    device->initialized = true;
    next_conversion_ms = (clock_ms / 20U + 1U) * 20U;
    return LSM303_OK;
}

lsm303_result_t lsm303dlhc_poll(lsm303_device_t *device, lsm303_frame_t *frame)
{
    REQUIRE(device->initialized && drains == wakes);
    REQUIRE(clock_ms > 0);
    ++polls;
    *frame = (lsm303_frame_t){0};
    /* Synthetic 50 Hz conversion source. A late read retains its overrun. */
    if (clock_ms >= next_conversion_ms) {
        frame->accel.fresh = true;
        frame->accel.valid = true;
        frame->accel.status = LSM303_DATA_OK;
        frame->accel.raw[2] = 1000;
        frame->accel.milli_units[2] = 1000;
        frame->accel.read_time_us = clock_ms * 1000U;
        frame->accel.overrun = clock_ms >= next_conversion_ms + 20U;
        frame->mag = frame->accel;
        next_conversion_ms = (clock_ms / 20U + 1U) * 20U;
    }
    return LSM303_OK;
}

ER s4_status_publish(const s4_sensor_cache_t *record)
{
    s4_sensor_status_t view;
    published = *record;
    s4_data_snapshot(record, clock_ms * 1000U, &view);
    if (!view.configured) ++invalid_publications;
    if (view.accel_usable) last_good = *record;
    return E_OK;
}

static void run(bool slow, bool bad_clock, bool bad_drain, unsigned int ticks)
{
    clock_ms = 0; next_conversion_ms = 0;
    wakes = 0; sleeps = 0; drains = 0; polls = 0; inits = 0;
    bus_inits = 0; exits = 0; invalid_publications = 0; limit = ticks;
    delayed_wake = slow; clock_failure = bad_clock; drain_failure = bad_drain;
    published = (s4_sensor_cache_t){0}; last_good = published;
    s4_sensor_task(0, NULL);
    REQUIRE(bus_inits == 1 && exits == 1);
    s4_sensor_status_t final;
    s4_data_snapshot(&published, clock_ms * 1000U, &final);
    REQUIRE(!final.configured && !final.accel_usable && !final.mag_usable);
}

int main(void)
{
    run(false, false, false, 100);
    REQUIRE(wakes == 100 && drains == 100 && sleeps == 101 && polls == 100);
    REQUIRE(inits == 1 && s4_cache_count(&last_good, S4_COUNT_ACCEL) == 50);
    REQUIRE(s4_cache_count(&last_good, S4_COUNT_OVERRUN) == 0);

    run(true, false, false, 100);
    REQUIRE(wakes == 100 && polls == 100 && drains == 100);
    REQUIRE(s4_cache_count(&last_good, S4_COUNT_OVERRUN) > 0);

    run(false, true, false, 200);
    REQUIRE(wakes == 200 && drains == 200 && inits == 2);
    REQUIRE(invalid_publications > 1);
    s4_sensor_status_t recovered;
    s4_data_snapshot(&last_good, clock_ms * 1000U, &recovered);
    REQUIRE(recovered.configured && recovered.accel_usable);

    run(false, false, true, 100);
    REQUIRE(wakes == 5 && drains == 5 && polls == 4);
    puts("S4-U34 PASS actual task: periodic service, 50Hz frames, queued-tick coalescing, retained delayed-read overruns, clock recovery and failed-wait invalidation");
    return EXIT_SUCCESS;
}
