#include "../../encoder_input.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static const odom_config_t config = {100, 200, 100}; /* synthetic only */
static const int32_t flat[3] = {0, 0, 1000};
/* Robot frame (imu_mount.h): +1 g up at rest, nose-up puts +g on front. */
static const int32_t up[3] = {500, 0, 866};
static const int32_t down[3] = {-500, 0, 866};
static uint8_t payload[ENCODER_SNAPSHOT_SIZE];
static s4_encoder_input_t input;
static terrain_t terrain;
static uint32_t now, counts;
static uint32_t left_bias; /* left wheel = counts + left_bias (a pivot) */

static void encode(uint32_t ms, uint32_t count)
{
    encoder_snapshot_t e = {count + left_bias, count, ms, ENCODER_SNAPSHOT_VALID};
    assert(encoder_snapshot_encode(&e, payload, sizeof(payload)));
}
/* One 20 ms controller tick: the IMU sample is imu_lag ms old and, as
 * vehicle.c passes it, usable (av) or not. */
static void step(const int32_t a[3], uint32_t ticks, uint32_t imu_lag, bool av)
{
    now += 20U; counts += ticks; encode(now, counts);
    s4_encoder_terrain_step(&input, &terrain, &config, now, payload,
        sizeof(payload), true, (uint64_t)(now - imu_lag) * 1000U, a, av, true, 0, 0);
}
static void tick(const int32_t a[3], uint32_t ticks) { step(a, ticks, 0, true); }
static void reset(void)
{
    input = (s4_encoder_input_t){0}; terrain = (terrain_t){0};
    now = 0; counts = 0; left_bias = 0;
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
        if (size == ENCODER_SNAPSHOT_SIZE) continue;
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
    puts("S4-U14 PASS exact lengths, reserved bits, invalid records, failed API and retained maximum");
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
    puts("S4-U15 PASS duplicates, zero intervals, stale/future input, skew, gaps and reboot history");
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
    assert(input.odometry.valid && odom_delta_um(&input.odometry) == 20000);
    puts("S4-U16 PASS unsigned/signed count boundaries, timer wrap, half-range and discontinuity");
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
    tick(up, 10); tick(up, (uint32_t)-5); /* robot-level reversal */
    assert(terrain.state == TERRAIN_INVALID);
    assert(!terrain.height_valid && terrain.maximum_mm == 50U);
    baseline(); tick(up, 10);
    /* A reverse interval between polls of one IMU sample also cancels height. */
    now+=10U; counts-=5U; encode(now,counts);
    apply(sizeof(payload),true,(uint64_t)(now-10U)*1000U);
    assert(terrain.state == TERRAIN_INVALID);
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
    puts("S4-U17 PASS completed humps, partial recovery, reversal, shock, pause, calibration and run reset");
}
/* vehicle.c passes a current sample clipped at +/-2 g (S4 accel_clipped) to
 * terrain as an impact: any clipped robot-frame axis (2047, or -2047/-2048
 * after imu_mount.h) is a shock, mid-hump too, never tilt or height. On
 * FLAT the state shows SHOCK for that sample; mid-hump the hump is kept. */
static void clipped_impacts(void)
{
    static const int32_t limits[3] = {2047, -2047, -2048};
    for (unsigned axis = 0; axis < 3U; ++axis)
        for (unsigned n = 0; n < 3U; ++n) {
            int32_t a[3] = {0, 0, 1000};
            a[axis] = limits[n];
            reset(); baseline(); tick(a, 0);
            assert(terrain.shock && terrain.state == TERRAIN_SHOCK && input.fault == S4_INPUT_OK);
            assert(!terrain.tilt_valid && !terrain.height_valid && input.odometry.valid);
            tick(flat, 0);
            assert(!terrain.shock && terrain.tilt_valid && terrain.state == TERRAIN_FLAT);
            tick(up, 10);
            tick(a, 10);
            assert(terrain.shock && terrain.state == TERRAIN_ASCENT && input.fault == S4_INPUT_OK);
            assert(!terrain.tilt_valid && !terrain.height_valid && input.odometry.valid);
            tick(flat, 0);
            assert(!terrain.shock && terrain.tilt_valid);
        }
    puts("S4-U24 PASS a clipped (+/-2 g) sample is a shock on every axis, never tilt or height");
}
/* I1: a descent entered from FLAT is DESCENT (the 20% slope cap), with no
 * height; it never abandons for being below the start, and it is no hump. */
static void plain_descents(void)
{
    reset(); baseline(); hump();
    tick(down, 10);
    assert(terrain.state == TERRAIN_DESCENT && terrain.plain && !terrain.height_valid);
    for (unsigned n = 0; n < 9U; ++n) {
        tick(down, 10); /* 100 mm below the start by now */
        assert(terrain.state == TERRAIN_DESCENT && terrain.tilt_valid);
    }
    for (unsigned n = 0; n < 4U; ++n) { tick(flat, 0); assert(terrain.state == TERRAIN_DESCENT); }
    tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && !terrain.plain);
    assert(terrain.humps == 1U && terrain.hump_mm == 50U && terrain.maximum_mm == 50U);
    assert(terrain_height_um(&terrain) == 0 && terrain.peak_um == 0U);
    /* A hump whose far side goes well below its start: still DESCENT. */
    tick(up, 10); tick(up, 10); tick(flat, 0);
    for (unsigned n = 0; n < 6U; ++n) { tick(down, 10); assert(terrain.state == TERRAIN_DESCENT); }
    assert(terrain.humps == 2U && terrain.hump_mm == 0U && terrain.peak_um == 0U);
    for (unsigned n = 0; n < 5U; ++n) tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && terrain.humps == 2U && terrain.maximum_mm == 50U);
    /* A gentle down-slope below 5 deg stays FLAT. */
    const int32_t gentle[3] = {-70, 0, 998};
    for (unsigned n = 0; n < 5U; ++n) tick(gentle, 10);
    assert(terrain.state == TERRAIN_FLAT);
    puts("S4-U26 PASS a plain descent is DESCENT without a height, a deep far side stays DESCENT");
}
/* I2: one shock, clipped, skewed or missing sample mid-hump keeps the
 * traversal and its height (its distance at the last good pitch); a run
 * of more than 60 ms (3 ticks) abandons it. */
static void bridged_samples(void)
{
    const int32_t shock[3] = {1900, 0, 1000};
    const int32_t clip[3] = {2047, 0, 1000};
    reset(); baseline();
    tick(up, 10); tick(up, 10);
    tick(shock, 10);
    assert(terrain.shock && terrain.state == TERRAIN_ASCENT && !terrain.tilt_valid);
    assert(terrain_pitch_cdeg(&terrain) == 3000 && terrain_height_um(&terrain) == 30000);
    tick(up, 10);
    assert(!terrain.shock && terrain.tilt_valid && terrain_height_um(&terrain) == 40000);
    tick(clip, 10);
    assert(terrain.shock && terrain.state == TERRAIN_ASCENT && terrain_height_um(&terrain) == 50000);
    tick(flat, 0); assert(terrain.state == TERRAIN_CREST);
    tick(down, 10); assert(terrain.state == TERRAIN_DESCENT);
    step(down, 10, 31, true); /* IMU/encoder skew */
    assert(input.fault == S4_INPUT_SKEW && input.odometry.valid);
    assert(terrain.state == TERRAIN_DESCENT && terrain_height_um(&terrain) == 30000);
    tick(down, 10);
    for (unsigned n = 0; n < 3U; ++n) { /* no usable IMU sample, 3 in a row */
        step(down, 10, 0, false);
        assert(terrain.state == TERRAIN_DESCENT && !terrain.shock && !terrain.tilt_valid);
    }
    assert(terrain_height_um(&terrain) == -10000);
    for (unsigned n = 0; n < 5U; ++n) tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && terrain.height_valid && terrain.height_mm == 50U);
    assert(terrain.humps == 1U && terrain.hump_mm == 50U);
    /* More than 60 ms: abandoned, and recovery needs a flat baseline. */
    tick(up, 10);
    for (unsigned n = 0; n < 4U; ++n) step(up, 10, 0, false);
    assert(terrain.state == TERRAIN_INVALID);
    for (unsigned n = 0; n < 3U; ++n) tick(shock, 10);
    tick(up, 10); assert(terrain.state == TERRAIN_INVALID);
    reset(); baseline(); tick(up, 10);
    for (unsigned n = 0; n < 4U; ++n) tick(shock, 10);
    assert(terrain.shock && terrain.state == TERRAIN_SHOCK && !terrain.initialized);
    /* Lost odometry is never bridged. */
    reset(); baseline(); tick(up, 10); tick(shock, 10);
    apply(sizeof(payload), false, 0);
    assert(terrain.state == TERRAIN_INVALID);
    puts("S4-U27 PASS one shock/clipped/skewed/missing sample keeps a hump; more than 60 ms abandons it");
}
/* I3: the completed height and a completion count are latched at DESCENT
 * to FLAT; a later shock or an invalid hump cannot change a reported one. */
static void hump_latch(void)
{
    const int32_t shock[3] = {1900, 0, 1000};
    reset(); baseline();
    assert(terrain.humps == 0U && terrain.hump_mm == 0U);
    hump();
    assert(terrain.humps == 1U && terrain.hump_mm == 50U);
    tick(shock, 0);
    assert(!terrain.height_valid && terrain.humps == 1U && terrain.hump_mm == 50U);
    tick(flat, 0); assert(terrain.state == TERRAIN_FLAT);
    /* Does not return to its base (30 mm up): an invalid end, peak cleared. */
    for (unsigned n = 0; n < 5U; ++n) tick(up, 10);
    tick(flat, 0); tick(down, 10); tick(down, 10);
    for (unsigned n = 0; n < 5U; ++n) tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && !terrain.height_valid);
    assert(terrain.humps == 2U && terrain.hump_mm == 0U && terrain.peak_um == 0U);
    assert(terrain.height_mm == 50U && terrain.maximum_mm == 50U);
    hump_scaled(5, 25);
    assert(terrain.humps == 3U && terrain.hump_mm == 25U && terrain.maximum_mm == 50U);
    puts("S4-U28 PASS hump end latched with a count; invalid end reports 0 and clears the peak");
}
/* I9: the shock flag describes the current sample only. */
static void shock_clears(void)
{
    const int32_t shock[3] = {1900, 0, 1000};
    reset(); baseline(); tick(shock, 0);
    assert(terrain.shock);
    step(flat, 0, 0, false);
    assert(!terrain.shock && !terrain.tilt_valid);
    for (unsigned n = 0; n < 5U; ++n) step(flat, 0, 0, false);
    assert(!terrain.shock && terrain.state == TERRAIN_INVALID);
    terrain_step(&terrain, now, shock, true, 0, true, true, 0, 0);
    assert(terrain.shock);
    terrain_step(&terrain, now, flat, false, 0, true, true, 0, 0);
    assert(!terrain.shock);
    puts("S4-U29 PASS the shock flag clears on the next sample, usable or not");
}
/* I11: a pivot's stopped wheel turning back a count does not end a hump;
 * a robot-level reversal does. */
static void pivot_backlash(void)
{
    reset(); baseline();
    tick(up, 10); tick(up, 10);
    left_bias -= 11U; tick(up, 10); /* left -1, right +10 */
    assert(input.odometry.valid && terrain.state == TERRAIN_ASCENT);
    left_bias -= 1U; tick(up, 0); /* left -1, right stopped */
    assert(terrain.state == TERRAIN_ASCENT);
    tick(up, 10); tick(up, 10);
    tick(flat, 0);
    for (unsigned n = 0; n < 5U; ++n) tick(down, 10);
    for (unsigned n = 0; n < 5U; ++n) tick(flat, 0);
    assert(terrain.state == TERRAIN_FLAT && terrain.height_valid);
    assert(terrain.height_mm == 44U && terrain.humps == 1U);
    tick(up, 10); left_bias -= 5U; tick(up, 0); /* one wheel back 5 counts */
    assert(terrain.state == TERRAIN_INVALID);
    puts("S4-U30 PASS one wheel back a count during a pivot keeps the hump; a reversal ends it");
}
int main(void)
{
    invalid_records(); clocks_and_repeats(); wraps(); traversals(); clipped_impacts();
    plain_descents(); bridged_samples(); hump_latch(); shock_clears(); pivot_backlash();
    reset();baseline();hump();hump_scaled(5,25);
    assert(terrain.maximum_mm==50U);
    hump_scaled(15,75);assert(terrain.maximum_mm==75U);
    apply(sizeof(payload),false,0);assert(terrain.maximum_mm==75U);
    puts("S4-U19 PASS smaller/larger completed humps update maximum only upward");
    return 0;
}
