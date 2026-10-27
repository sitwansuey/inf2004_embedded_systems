#include "../../encoder_input.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const odom_config_t config = {100, 200, 100}; /* synthetic only */
static const int32_t flat[3] = {0, 0, 1000};
static const int32_t up[3] = {-500, 0, 866};
static const int32_t down[3] = {500, 0, 866};
static uint8_t payload[EDR_ENCODER_SIZE];
static s4_encoder_input_t input;
static terrain_t terrain;
static uint32_t now, counts;

static void encode(uint32_t ms, uint32_t count)
{
    edr_encoder_t e = {count, count, ms, EDR_ENCODER_VALID};
    assert(edr_encoder_encode(&e, payload, sizeof(payload)));
}
static void tick(const int32_t a[3], uint32_t ticks)
{
    now += 20U; counts += ticks; encode(now, counts);
    s4_encoder_terrain_step(&input, &terrain, &config, now, payload,
        sizeof(payload), true, (uint64_t)now * 1000U, a, true, true, 0, 0);
}
static void reset(void)
{
    input = (s4_encoder_input_t){0}; terrain = (terrain_t){0};
    now = 0; counts = 0;
}
static void baseline(void)
{
    for (unsigned n = 0; n < 7U; ++n) tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && input.odometry.valid);
}
static void hump_scaled(uint32_t ticks, uint16_t height)
{
    for (unsigned n = 0; n < 5U; ++n) tick(up, ticks);
    tick(flat, 0);
    for (unsigned n = 0; n < 5U; ++n) tick(down, ticks);
    for (unsigned n = 0; n < 5U; ++n) tick(flat, 0);
    assert(terrain.height_valid && terrain.height_mm == height);
}
static void hump(void) { hump_scaled(10,50); }
static void apply(size_t size, bool available, uint64_t imu_us)
{
    s4_encoder_terrain_step(&input, &terrain, &config, now, payload, size,
        available, imu_us, flat, true, true, 0, 0);
}
static void invalid_records(void)
{
    reset(); baseline(); hump();
    for (size_t size = 0; size < 17U; ++size) {
        if (size == EDR_ENCODER_SIZE) continue;
        apply(size, true, (uint64_t)now * 1000U);
        assert(input.fault == S4_INPUT_FORMAT && !terrain.height_valid);
        assert(terrain.maximum_mm == 50U);
    }
    encode(now, counts); payload[12] |= 0x80U;
    apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_FORMAT);
    encode(now, counts); payload[12] = 0;
    apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_FORMAT); /* noncanonical invalid counts */
    memset(payload, 0, sizeof(payload));
    apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_UNAVAILABLE);
    memset(payload, 0xff, sizeof(payload));
    apply(sizeof(payload), false, 0);
    assert(input.fault == S4_INPUT_UNAVAILABLE && !input.odometry.valid);
    assert(terrain.maximum_mm == 50U);
    puts("S4-U23 PASS exact lengths, reserved bits, invalid records, failed API and retained maximum");
}
static void clocks_and_repeats(void)
{
    reset(); baseline();
    int32_t distance = input.odometry.distance_um;
    for (unsigned n = 0; n < 5; ++n) apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.odometry.distance_um == distance && input.odometry.valid);
    encode(now, counts + 1U); apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_HISTORY && !input.odometry.initialized);
    baseline(); encode(now - 61U, counts);
    apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_STALE);
    encode(now + 1U, counts); apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_STALE);
    baseline(); tick(flat, 0);
    apply(sizeof(payload), true, (uint64_t)(now - 31U) * 1000U);
    assert(input.fault == S4_INPUT_SKEW && !terrain.height_valid);
    assert(input.odometry.valid); /* terrain quality is independent */
    now += 80U; encode(now, counts);
    apply(sizeof(payload), true, (uint64_t)now * 1000U);
    assert(input.fault == S4_INPUT_HISTORY && !input.odometry.initialized);
    baseline(); now = 1; encode(now, 0); apply(sizeof(payload), true, 1000);
    assert(input.fault == S4_INPUT_HISTORY);
    puts("S4-U24 PASS duplicates, zero intervals, stale/future input, skew, gaps and reboot history");
}
static void wraps(void)
{
    reset(); now = UINT32_MAX - 200U; counts = UINT32_MAX - 5U;
    baseline(); tick(flat, 10);
    assert(input.odometry.valid && input.odometry.distance_um == 20000);
    for (unsigned n = 0; n < 6; ++n) tick(flat, 1);
    assert(now < 200U && input.odometry.valid);
    tick(flat, UINT32_C(0x80000000));
    assert(!input.odometry.valid);
    baseline(); tick(flat, 1000000U);
    assert(!input.odometry.valid);
    reset(); counts = (uint32_t)INT32_MAX - 5U; baseline(); tick(flat, 10);
    assert(input.odometry.valid && input.odometry.delta_um == 20000);
    puts("S4-U25 PASS unsigned/signed count boundaries, timer wrap, half-range and discontinuity");
}
static void traversals(void)
{
    reset(); baseline(); hump(); assert(terrain.maximum_mm == 50U);
    /* Loss partway through ascent invalidates the entire traversal. */
    tick(up, 10); apply(sizeof(payload), false, 0);
    for (unsigned n = 0; n < 5; ++n) tick(up, 10);
    tick(flat, 0);
    for (unsigned n = 0; n < 5; ++n) tick(down, 10);
    for (unsigned n = 0; n < 5; ++n) tick(flat, 0);
    assert(!terrain.height_valid && terrain.maximum_mm == 50U);
    baseline(); hump();
    tick(up, 10); tick(up, UINT32_MAX); /* reversal */
    assert(!terrain.height_valid && terrain.maximum_mm == 50U);
    baseline(); tick(up, 10);
    /* A reverse interval between polls of one IMU sample also cancels height. */
    now+=10U; --counts; encode(now,counts);
    apply(sizeof(payload),true,(uint64_t)(now-10U)*1000U);
    assert(!terrain.height_valid && terrain.tilt_valid && input.odometry.valid);
    baseline(); tick(up, 10);
    const int32_t shock[3] = {1900, 0, 1000}; tick(shock, 0);
    assert(terrain.shock && !terrain.height_valid && terrain.maximum_mm == 50U);
    /* Pause and recovery retain maximum; explicit run reset is zero-init. */
    apply(sizeof(payload), false, 0); baseline();
    assert(terrain.maximum_mm == 50U);
    terrain_step(&terrain, now + 20U, flat, true, 0, true, false, 0, 0);
    assert(!terrain.tilt_valid && terrain.maximum_mm == 50U);
    reset(); assert(!terrain.maximum_mm);
    puts("S4-U26 PASS completed humps, partial recovery, reversal, shock, pause, calibration and run reset");
}
int main(void)
{
    invalid_records(); clocks_and_repeats(); wraps(); traversals();
    reset();baseline();hump();hump_scaled(5,25);
    assert(terrain.maximum_mm==50U);
    hump_scaled(15,75);assert(terrain.maximum_mm==75U);
    apply(sizeof(payload),false,0);assert(terrain.maximum_mm==75U);
    puts("S4-U28 PASS smaller/larger completed humps update maximum only upward");
    return 0;
}
