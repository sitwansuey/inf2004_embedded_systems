/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_data.h"

void s4_data_reset(s4_sensor_status_t *p_state)
{
    *p_state = (s4_sensor_status_t){ 0 };
    p_state->init_result = LSM303_NOT_INITIALIZED;
    p_state->poll_result = LSM303_NOT_INITIALIZED;
    p_state->accel_age_ms = S4_NO_SAMPLE_AGE;
    p_state->mag_age_ms = S4_NO_SAMPLE_AGE;
}

void s4_data_init_result(s4_sensor_status_t *p_state, lsm303_result_t result)
{
    p_state->latest = (lsm303_frame_t){ 0 };
    p_state->init_result = result;
    p_state->poll_result = LSM303_NOT_INITIALIZED;
    p_state->configured = (result == LSM303_OK);
    p_state->accel_usable = false;
    p_state->mag_usable = false;
    p_state->accel_age_ms = S4_NO_SAMPLE_AGE;
    p_state->mag_age_ms = S4_NO_SAMPLE_AGE;
    p_state->init_attempts++;
}

static void update_sample(lsm303_sample_t *p_cached,
                           const lsm303_sample_t *p_new)
{
    if (p_new->fresh)
    {
        *p_cached = *p_new;
    }
    else if (p_new->status == LSM303_DATA_IO_ERROR)
    {
        p_cached->valid = false;
    }
    /* Most recent poll status, original complete sample/timestamp. */
    p_cached->status = p_new->status;
}

void s4_data_update(s4_sensor_status_t *p_state, const lsm303_frame_t *p_frame,
                    lsm303_result_t result)
{
    p_state->poll_result = result;
    p_state->accel_samples += p_frame->accel.fresh ? 1u : 0u;
    p_state->mag_samples += p_frame->mag.fresh ? 1u : 0u;
    p_state->accel_io_errors +=
        (p_frame->accel.status == LSM303_DATA_IO_ERROR) ? 1u : 0u;
    p_state->mag_io_errors +=
        (p_frame->mag.status == LSM303_DATA_IO_ERROR) ? 1u : 0u;
    p_state->accel_overruns += p_frame->accel.overrun ? 1u : 0u;
    update_sample(&p_state->latest.accel, &p_frame->accel);
    update_sample(&p_state->latest.mag, &p_frame->mag);
}

static uint32_t sample_age(const lsm303_sample_t *p_sample, uint64_t now_us)
{
    uint64_t age_ms;

    if (!p_sample->fresh || (now_us < p_sample->read_time_us))
    {
        return S4_NO_SAMPLE_AGE;
    }
    age_ms = (now_us - p_sample->read_time_us) / 1000u;
    return (age_ms >= S4_NO_SAMPLE_AGE) ?
           S4_NO_SAMPLE_AGE - 1u : (uint32_t)age_ms;
}

static bool sample_usable(const lsm303_sample_t *p_sample, uint64_t now_us)
{
    return p_sample->fresh && p_sample->valid &&
           (now_us >= p_sample->read_time_us) &&
           ((now_us - p_sample->read_time_us) <= S4_STALE_AFTER_US);
}

void s4_data_snapshot(const s4_sensor_status_t *p_state, uint64_t now_us,
                      s4_sensor_status_t *p_output)
{
    *p_output = *p_state;
    p_output->observed_time_us = now_us;
    p_output->accel_age_ms = sample_age(&p_state->latest.accel, now_us);
    p_output->mag_age_ms = sample_age(&p_state->latest.mag, now_us);
    p_output->accel_usable = p_state->configured &&
                            sample_usable(&p_state->latest.accel, now_us);
    p_output->mag_usable = p_state->configured &&
                          sample_usable(&p_state->latest.mag, now_us);
}
