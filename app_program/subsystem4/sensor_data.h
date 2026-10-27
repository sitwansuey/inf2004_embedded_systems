/* SPDX-License-Identifier: Apache-2.0 */
#ifndef S4_SENSOR_DATA_H
#define S4_SENSOR_DATA_H

#include "drv_lsm303dlhc.h"

#define S4_STALE_AFTER_US UINT64_C(100000)
#define S4_NO_SAMPLE_AGE UINT32_MAX

/* Latest values, not a lossless stream. Other subsystems must use the
 * *_usable flags and ages, not simply the most recent numbers. Timestamps
 * use tk_get_tim() milliseconds scaled to us, matching the team's timebase;
 * this does not provide microsecond timestamp resolution. In this cached
 * frame, fresh means a complete sample has been acquired at least once;
 * use sample counters/timestamps to detect updates, and *_usable for use. */
typedef struct
{
    lsm303_frame_t latest;
    lsm303_result_t init_result;
    lsm303_result_t poll_result;
    bool configured;
    bool accel_usable;
    bool mag_usable;
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

/* Pure data handling, with no RTOS/hardware calls. The sensor task owns its
 * working copy. subsystem4.c protects the published copy with a mutex. */
void s4_data_reset(s4_sensor_status_t *p_state);
void s4_data_init_result(s4_sensor_status_t *p_state, lsm303_result_t result);
void s4_data_update(s4_sensor_status_t *p_state, const lsm303_frame_t *p_frame,
                    lsm303_result_t result);
void s4_data_snapshot(const s4_sensor_status_t *p_state, uint64_t now_us,
                      s4_sensor_status_t *p_output);

#endif
