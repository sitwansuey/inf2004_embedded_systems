/* SPDX-License-Identifier: Apache-2.0 */
#ifndef S4_SENSOR_DATA_H
#define S4_SENSOR_DATA_H

#include "drv_lsm303dlhc.h"

#define S4_STALE_AFTER_US UINT64_C(100000)
#define S4_NO_SAMPLE_AGE UINT32_MAX

/* Latest values, not a lossless stream. Other subsystems must use the
 * *_usable flags and ages, not simply the most recent numbers. Timestamps
 * use tk_get_otm() milliseconds scaled to us, matching the team's timebase;
 * this does not provide microsecond timestamp resolution. In this cached
 * frame, fresh means a complete sample has been acquired at least once;
 * use sample counters/timestamps to detect updates, and *_usable for use. */
typedef struct
{
    lsm303_frame_t latest;
    uint8_t init_result : 3;
    uint8_t poll_result : 3;
    uint8_t configured : 1;
    uint8_t accel_usable : 1;
    uint8_t mag_usable : 1;
    uint32_t accel_age_ms;
    uint32_t mag_age_ms;
    uint32_t init_attempts;
    uint32_t task_heartbeat;
    uint32_t accel_samples;
    uint32_t mag_samples;
    uint32_t accel_io_errors;
    uint32_t mag_io_errors;
    uint32_t accel_overruns;
    uint32_t publish_errors;
    uint32_t legacy_lines_received;
    uint64_t observed_time_us;
} s4_sensor_status_t;

/* Persistent storage: 500 information bits, rounded to 63 bytes. Derived
 * scaled values and ages exist only in transient calculation/API views. */
enum {
    S4_COUNT_INIT, S4_COUNT_HEARTBEAT, S4_COUNT_ACCEL, S4_COUNT_MAG,
    S4_COUNT_ACCEL_ERROR, S4_COUNT_MAG_ERROR, S4_COUNT_OVERRUN,
    S4_COUNT_PUBLISH_ERROR, S4_COUNT_LEGACY, S4_COUNT_TOTAL
};
#define S4_CACHE_BITS (212U + 32U * S4_COUNT_TOTAL)
#define S4_CACHE_BYTES ((S4_CACHE_BITS + 7U) / 8U)
typedef struct { uint8_t bytes[S4_CACHE_BYTES]; } s4_sensor_cache_t;
static inline uint64_t s4_operating_ms_to_us(uint64_t ms) {
    return ms>(UINT64_MAX-1U)/1000U?UINT64_MAX:ms*1000U;
}
/* Pure data handling, with no RTOS/hardware calls. The sensor task owns its
 * working copy. subsystem4.c protects the published copy with a mutex. */
void s4_data_reset(s4_sensor_cache_t *p_state);
void s4_data_init_result(s4_sensor_cache_t *p_state, lsm303_result_t result);
void s4_data_invalidate(s4_sensor_cache_t *p_state);
uint32_t s4_cache_count(const s4_sensor_cache_t *p_state, unsigned counter);
void s4_cache_set_count(s4_sensor_cache_t *p_state, unsigned counter, uint32_t value);
void s4_cache_increment(s4_sensor_cache_t *p_state, unsigned counter);
void s4_data_update(s4_sensor_cache_t *p_state, const lsm303_frame_t *p_frame,
                    lsm303_result_t result);
void s4_data_snapshot(const s4_sensor_cache_t *p_state, uint64_t now_us,
                      s4_sensor_status_t *p_output);

#endif
