/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_data.h"

#include <stdio.h>
#include <stdlib.h>

#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c); \
    exit(EXIT_FAILURE); } } while (0)

static lsm303_frame_t good_frame(uint64_t time_us)
{
    lsm303_frame_t frame = { 0 };

    frame.accel.fresh = true;
    frame.accel.valid = true;
    frame.accel.status = LSM303_DATA_OK;
    frame.accel.read_time_us = time_us;
    frame.accel.milli_units[2] = 1000;
    frame.mag = frame.accel;
    frame.mag.milli_units[2] = 400;
    return frame;
}

static void test_empty_and_init(void)
{
    s4_sensor_status_t state;
    s4_sensor_status_t copy;

    s4_data_reset(&state);
    s4_data_snapshot(&state, 10000, &copy);
    REQUIRE(!copy.configured && !copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.accel_age_ms == S4_NO_SAMPLE_AGE);
    REQUIRE(copy.mag_age_ms == S4_NO_SAMPLE_AGE);
    REQUIRE(copy.init_result == LSM303_NOT_INITIALIZED);
    s4_data_init_result(&state, LSM303_IO_ERROR);
    REQUIRE(!state.configured && state.init_attempts == 1);
    s4_data_init_result(&state, LSM303_OK);
    REQUIRE(state.configured && state.init_attempts == 2);
    puts("[TEST] S4-U15 PASS  absent/uninitialized state cannot publish usable data");
}

static void test_freshness(void)
{
    s4_sensor_status_t state;
    s4_sensor_status_t copy;
    lsm303_frame_t frame = good_frame(10000);
    lsm303_frame_t empty = { 0 };

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 20000, &copy);
    REQUIRE(copy.accel_usable && copy.mag_usable && copy.accel_age_ms == 10);
    s4_data_update(&state, &empty, LSM303_OK);
    s4_data_snapshot(&state, 110000, &copy);
    REQUIRE(copy.accel_usable && copy.accel_age_ms == 100);
    REQUIRE(copy.accel_samples == 1 && copy.latest.accel.read_time_us == 10000);
    REQUIRE(copy.latest.accel.status == LSM303_NO_DATA);
    s4_data_snapshot(&state, 110001, &copy);
    REQUIRE(!copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.latest.accel.milli_units[2] == 1000);
    puts("[TEST] S4-U16 PASS  no-data polls preserve timestamps; stale data expires");
}

static void test_fault_and_recovery(void)
{
    s4_sensor_status_t state;
    s4_sensor_status_t copy;
    lsm303_frame_t frame = good_frame(10000);
    lsm303_frame_t fault = { 0 };

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_update(&state, &frame, LSM303_OK);
    fault.mag.status = LSM303_DATA_IO_ERROR;
    s4_data_update(&state, &fault, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 30000, &copy);
    REQUIRE(copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.mag_io_errors == 1 && copy.accel_io_errors == 0);
    frame = good_frame(40000);
    frame.accel.overrun = true;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 45000, &copy);
    REQUIRE(copy.accel_usable && copy.mag_usable && copy.accel_overruns == 1);
    s4_data_init_result(&state, LSM303_CONFIG_ERROR);
    s4_data_snapshot(&state, 50000, &copy);
    REQUIRE(!copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.accel_samples == 2 && copy.init_attempts == 2);
    REQUIRE(copy.accel_age_ms == S4_NO_SAMPLE_AGE);
    puts("[TEST] S4-U17 PASS  per-channel faults, recovery and reinitialization");
}

static void test_clipping_and_time_boundaries(void)
{
    s4_sensor_status_t state;
    s4_sensor_status_t copy;
    const uint64_t time_us = (UINT64_C(1) << 32) + 40000;
    lsm303_frame_t frame = good_frame(time_us);

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    frame.accel.valid = false;
    frame.accel.status = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, time_us + 2000, &copy);
    REQUIRE(!copy.accel_usable && copy.mag_usable && copy.mag_age_ms == 2);
    s4_data_snapshot(&state, time_us - 1, &copy);
    REQUIRE(!copy.mag_usable && copy.mag_age_ms == S4_NO_SAMPLE_AGE);
    s4_data_snapshot(&state, time_us + UINT64_C(5000000000000), &copy);
    REQUIRE(copy.mag_age_ms == S4_NO_SAMPLE_AGE - 1u && !copy.mag_usable);
    puts("[TEST] S4-U18 PASS  clipping, future timestamps, 64-bit time and age saturation");
}

int main(void)
{
    test_empty_and_init();
    test_freshness();
    test_fault_and_recovery();
    test_clipping_and_time_boundaries();
    return EXIT_SUCCESS;
}
