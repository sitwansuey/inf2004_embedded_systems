/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_cycle.h"

#include <stddef.h>

void
s4_cycle_reset (s4_sensor_cycle_t * p_cycle, const lsm303_bus_t * p_bus,
                void (*recover)(void))
{
    *p_cycle         = (s4_sensor_cycle_t){0};
    p_cycle->bus     = *p_bus;
    p_cycle->recover = recover;
}

void
s4_cycle_run (s4_sensor_cycle_t * p_cycle, s4_sensor_cache_t * p_state,
              uint64_t now_ms)
{
    lsm303_frame_t  frame;
    lsm303_result_t result;
    bool            failed;

    if (!p_cycle->device.initialized)
    {
        if (now_ms < p_cycle->retry_ms)
        {
            return;
        }
        /* A slave left mid-byte can hold SDA low; clear the bus first. */
        if (p_cycle->recover != NULL)
        {
            p_cycle->recover();
        }
        result = lsm303dlhc_init(&p_cycle->device, &p_cycle->bus);
        s4_data_init_result(p_state, result);
        p_cycle->failed_polls = 0;
        p_cycle->idle_polls   = 0;
        /* Back off only after a failed attempt. */
        p_cycle->retry_ms = (result == LSM303_OK) ? 0U : now_ms + S4_RETRY_MS;
        p_cycle->mag_retry_ms = now_ms + S4_RETRY_MS;
    }
    else if (!p_cycle->device.mag_ready && (now_ms >= p_cycle->mag_retry_ms))
    {
        /* The accelerometer keeps running whatever this returns. */
        (void) lsm303dlhc_init_mag(&p_cycle->device);
        p_cycle->mag_retry_ms = now_ms + S4_RETRY_MS;
    }
    if (!p_cycle->device.initialized)
    {
        return;
    }
    result = lsm303dlhc_poll(&p_cycle->device, &frame);
    s4_data_update(p_state, &frame, result);
    if (frame.accel.fresh)
    {
        p_cycle->idle_polls = 0;
    }
    else if (p_cycle->idle_polls < S4_IDLE_POLLS)
    {
        p_cycle->idle_polls++;
        if (p_cycle->idle_polls == S4_IDLE_POLLS)
        {
            s4_cache_increment(p_state, S4_COUNT_FAULT); /* accel stale */
        }
    }
    /* Accel I/O errors, or an accel that answers without new data. A
     * magnetometer error never counts. */
    failed = (frame.accel.status == LSM303_DATA_IO_ERROR) ||
             (p_cycle->idle_polls >= S4_IDLE_POLLS);
    p_cycle->failed_polls =
        failed ? (uint8_t) (p_cycle->failed_polls + 1U) : 0U;
    if (p_cycle->failed_polls >= S4_FAILED_POLLS)
    {
        p_cycle->device.initialized = false;
        s4_data_invalidate(p_state);
        p_cycle->retry_ms = 0; /* re-initialise on the next cycle */
    }
}

/*** end of file ***/
