/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_data.h"

#if defined(NDEBUG)
#error Host tests require assertions
#endif
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

#define REQUIRE(c)                                                             \
    do                                                                         \
    {                                                                          \
        if (!(c))                                                              \
        {                                                                      \
            fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #c);            \
            assert(0 && "host test requirement failed");                       \
        }                                                                      \
    } while (0)

static lsm303_frame_t
good_frame (uint64_t time_us)
{
    lsm303_frame_t frame = {0};

    frame.accel.fresh          = true;
    frame.accel.valid          = true;
    frame.accel.status         = LSM303_DATA_OK;
    frame.accel.read_time_us   = time_us;
    frame.accel.raw[2]         = 1000;
    frame.accel.milli_units[2] = 1000;
    frame.mag                  = frame.accel;
    frame.mag.raw[2]           = 392;
    frame.mag.milli_units[2]   = 400;
    return frame;
}

static void
test_empty_and_init (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;

    s4_data_reset(&state);
    s4_data_snapshot(&state, 10000, &copy);
    REQUIRE(!copy.configured && !copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.accel_age_ms == S4_NO_SAMPLE_AGE);
    REQUIRE(copy.mag_age_ms == S4_NO_SAMPLE_AGE);
    REQUIRE(copy.init_result == LSM303_NOT_INITIALIZED);
    s4_data_init_result(&state, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 10000, &copy);
    REQUIRE(!copy.configured && copy.init_attempts == 1);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_snapshot(&state, 10000, &copy);
    REQUIRE(copy.configured && copy.init_attempts == 2);
    puts("[TEST] S4-U06 PASS  absent/uninitialized state cannot publish usable "
         "data");
}

static void
test_freshness (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;
    lsm303_frame_t     frame = good_frame(10000);
    lsm303_frame_t     empty = {0};

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
    puts("[TEST] S4-U07 PASS  no-data polls preserve timestamps; stale data "
         "expires");
}

static void
test_fault_and_recovery (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;
    lsm303_frame_t     frame = good_frame(10000);
    lsm303_frame_t     fault = {0};

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_update(&state, &frame, LSM303_OK);
    fault.mag.status = LSM303_DATA_IO_ERROR;
    s4_data_update(&state, &fault, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 30000, &copy);
    REQUIRE(copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.mag_io_errors == 1 && copy.accel_io_errors == 0);
    frame               = good_frame(40000);
    frame.accel.overrun = true;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 45000, &copy);
    REQUIRE(copy.accel_usable && copy.mag_usable && copy.accel_overruns == 1);
    s4_data_init_result(&state, LSM303_CONFIG_ERROR);
    s4_data_snapshot(&state, 50000, &copy);
    REQUIRE(!copy.accel_usable && !copy.mag_usable);
    REQUIRE(copy.accel_samples == 2 && copy.init_attempts == 2);
    REQUIRE(copy.accel_age_ms == S4_NO_SAMPLE_AGE);
    puts("[TEST] S4-U08 PASS  per-channel faults, recovery and "
         "reinitialization");
}

static void
test_clipping_and_time_boundaries (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;
    const uint64_t     time_us = ((UINT64_C(1) << 32) + 40) * 1000U;
    lsm303_frame_t     frame   = good_frame(time_us);

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    frame.accel.valid  = false;
    frame.accel.status = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, time_us + 2000, &copy);
    REQUIRE(!copy.accel_usable && copy.mag_usable && copy.mag_age_ms == 2);
    s4_data_snapshot(&state, time_us - 1, &copy);
    REQUIRE(!copy.mag_usable && copy.mag_age_ms == S4_NO_SAMPLE_AGE);
    s4_data_snapshot(&state, time_us + UINT64_C(5000000000000), &copy);
    REQUIRE(copy.mag_age_ms == S4_NO_SAMPLE_AGE - 1u && !copy.mag_usable);
    s4_data_snapshot(&state, UINT64_MAX, &copy);
    REQUIRE(!copy.mag_usable && copy.mag_age_ms == S4_NO_SAMPLE_AGE);
    puts("[TEST] S4-U09 PASS  clipping, future timestamps, 64-bit time and age "
         "saturation");
}

/* A clipped accel sample (driver: an axis at -2048/2047, valid false) is
 * never usable, but while current it is reported clipped (an impact for the
 * VC), also after a no-data poll has overwritten the cached status. */
static void
test_clipped_impact (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;
    lsm303_frame_t     frame = good_frame(10000);
    lsm303_frame_t     empty = {0};
    lsm303_frame_t     fault = {0};

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 20000, &copy);
    REQUIRE(copy.accel_usable && !copy.accel_clipped);
    frame              = good_frame(30000);
    frame.accel.raw[0] = 2047;
    frame.accel.valid  = false;
    frame.accel.status = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 40000, &copy);
    REQUIRE(!copy.accel_usable && copy.accel_clipped &&
            copy.latest.accel.milli_units[0] == 2047);
    s4_data_update(&state, &empty, LSM303_OK);
    s4_data_snapshot(&state, 130000, &copy);
    REQUIRE(copy.latest.accel.status == LSM303_NO_DATA && copy.accel_clipped);
    s4_data_snapshot(&state, 130001, &copy);
    REQUIRE(!copy.accel_usable && !copy.accel_clipped); /* older than 100 ms */
    frame              = good_frame(140000);
    frame.accel.raw[2] = -2048;
    frame.accel.valid  = false;
    frame.accel.status = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 150000, &copy);
    REQUIRE(copy.accel_clipped);
    s4_data_snapshot(&state, 139999, &copy);
    REQUIRE(!copy.accel_clipped); /* sample from the future */
    s4_data_invalidate(&state);
    s4_data_snapshot(&state, 150000, &copy);
    REQUIRE(!copy.configured && !copy.accel_clipped);
    /* An I/O error after a valid sample is a fault, not a clip. */
    s4_data_init_result(&state, LSM303_OK);
    frame = good_frame(160000);
    s4_data_update(&state, &frame, LSM303_OK);
    fault.accel.status = LSM303_DATA_IO_ERROR;
    s4_data_update(&state, &fault, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 170000, &copy);
    REQUIRE(!copy.accel_usable && !copy.accel_clipped);
    puts("[TEST] S4-U25 PASS  a current clipped accel sample is reported "
         "clipped, never usable");
}

/* I4/I5: impacts and accel faults are counted on every sample or poll, so
 * one that lasts a single 10 ms poll is still seen by a 20 ms reader that
 * compares counts. The magnetometer never counts as an IMU fault. */
static void
test_event_counters (void)
{
    s4_sensor_cache_t  state;
    s4_sensor_status_t copy;
    lsm303_frame_t     frame = good_frame(10000);
    lsm303_frame_t     fault = {0};

    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 15000, &copy);
    REQUIRE(copy.accel_impacts == 0 && copy.accel_faults == 0);
    /* A 1.9 g sample, then a normal one before anyone reads. */
    frame                      = good_frame(30000);
    frame.accel.raw[0]         = 1900;
    frame.accel.milli_units[0] = 1900;
    s4_data_update(&state, &frame, LSM303_OK);
    frame = good_frame(50000);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 55000, &copy);
    REQUIRE(copy.accel_usable && copy.accel_impacts == 1 &&
            copy.accel_faults == 0);
    /* 1.79 g is not an impact. */
    frame              = good_frame(70000);
    frame.accel.raw[0] = 1490;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 75000, &copy);
    REQUIRE(copy.accel_impacts == 1);
    /* A clipped sample is an impact and a fault. */
    frame              = good_frame(90000);
    frame.accel.raw[2] = -2048;
    frame.accel.valid  = false;
    frame.accel.status = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    frame = good_frame(110000);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 115000, &copy);
    REQUIRE(copy.accel_usable && copy.accel_impacts == 2 &&
            copy.accel_faults == 1);
    /* One accel I/O error followed by a fresh sample: still counted. */
    fault.accel.status = LSM303_DATA_IO_ERROR;
    s4_data_update(&state, &fault, LSM303_IO_ERROR);
    frame = good_frame(130000);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 135000, &copy);
    REQUIRE(copy.accel_usable && copy.accel_faults == 2);
    /* An overrun sample is a fault; a magnetometer error is not. */
    frame               = good_frame(150000);
    frame.accel.overrun = true;
    s4_data_update(&state, &frame, LSM303_OK);
    fault            = (lsm303_frame_t){0};
    fault.mag.status = LSM303_DATA_IO_ERROR;
    s4_data_update(&state, &fault, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 155000, &copy);
    REQUIRE(copy.accel_faults == 3 && copy.mag_io_errors == 1);
    /* A failed init and a reset of the device count; a good init does not. */
    s4_data_init_result(&state, LSM303_IO_ERROR);
    s4_data_snapshot(&state, 160000, &copy);
    REQUIRE(copy.accel_faults == 4);
    s4_data_init_result(&state, LSM303_OK);
    s4_data_invalidate(&state);
    s4_data_snapshot(&state, 160000, &copy);
    REQUIRE(copy.accel_faults == 5 && copy.accel_impacts == 2);
    puts("[TEST] S4-U31 PASS  impacts and accel faults are counted per sample, "
         "mag faults never");
}

int
main (void)
{
    test_empty_and_init();
    test_freshness();
    test_fault_and_recovery();
    test_clipped_impact();
    test_event_counters();
    test_clipping_and_time_boundaries();
    s4_sensor_cache_t  state;
    s4_sensor_status_t view;
    s4_data_reset(&state);
    s4_data_init_result(&state, LSM303_OK);
    for (unsigned n = 0; n < S4_COUNT_TOTAL; ++n)
    {
        s4_cache_set_count(&state, n, UINT32_MAX - n);
    }
    for (unsigned n = 0; n < S4_COUNT_TOTAL; ++n)
    {
        REQUIRE(s4_cache_count(&state, n) == UINT32_MAX - n);
    }
    s4_cache_increment(&state, S4_COUNT_INIT);
    REQUIRE(s4_cache_count(&state, S4_COUNT_INIT) == 0);
    const uint64_t ms    = (UINT64_MAX - 1U) / 1000U;
    lsm303_frame_t frame = good_frame(ms * 1000U);
    frame.accel.raw[0]   = -2048;
    frame.accel.raw[1]   = 2047;
    frame.mag.raw[0]     = INT16_MIN;
    frame.mag.raw[1]     = INT16_MAX;
    frame.mag.valid      = false;
    frame.mag.status     = LSM303_DATA_CLIPPED;
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, ms * 1000U, &view);
    REQUIRE(view.latest.accel.raw[0] == -2048 &&
            view.latest.accel.raw[1] == 2047);
    REQUIRE(view.latest.mag.raw[0] == INT16_MIN &&
            view.latest.mag.raw[1] == INT16_MAX);
    REQUIRE(view.latest.accel.read_time_us == ms * 1000U && !view.mag_usable);
    frame = good_frame(12345);
    s4_data_update(&state, &frame, LSM303_OK);
    s4_data_snapshot(&state, 13000, &view);
    REQUIRE(!view.accel_usable && !view.mag_usable);
    REQUIRE((state.bytes[S4_CACHE_BYTES - 1U] & 0xf0U) == 0);
    printf("S4-U20 compact cache: %u bits/%u bytes, raw/time boundaries, exact "
           "units, counter isolation/wrap PASS\n",
           (unsigned) S4_CACHE_BITS, (unsigned) S4_CACHE_BYTES);
    return EXIT_SUCCESS;
}

/*** end of file ***/
