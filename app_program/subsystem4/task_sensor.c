/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"
#include "drv_pico_i2c.h"

void s4_sensor_task(INT start_code, void *p_exinf)
{
    lsm303_bus_t bus;
    lsm303_device_t device = { 0 };
    s4_sensor_cache_t state;
    struct { uint64_t retry_ms:55; uint64_t failed_polls:4; } recovery={0};

    (void)start_code;
    (void)p_exinf;
    s4_data_reset(&state);
    s4_pico_i2c_init(&bus);
    /* Share the kernel/encoder timebase for public sample timestamps.
     * SDK transaction deadlines still use the hardware timer. */
    bus.now_us = s4_kernel_now_us;

    for (;;)
    {
        uint64_t now = s4_kernel_now_us(NULL);
        if (now == UINT64_MAX) {
            s4_data_invalidate(&state);
            device.initialized = false;
            (void)s4_status_publish(&state);
            (void)tk_dly_tsk(S4_POLL_MS);
            continue;
        }

        if (!device.initialized && (now/1000U >= recovery.retry_ms))
        {
            lsm303_result_t result = lsm303dlhc_init(&device, &bus);

            s4_data_init_result(&state, result);
            recovery.retry_ms = now/1000U + S4_RETRY_US/1000U;
            recovery.failed_polls = 0;
        }
        if (device.initialized)
        {
            lsm303_frame_t frame;
            lsm303_result_t result = lsm303dlhc_poll(&device, &frame);

            s4_data_update(&state, &frame, result);
            recovery.failed_polls = (result == LSM303_IO_ERROR) ? recovery.failed_polls + 1u : 0u;
            if (recovery.failed_polls >= 10u)
            {
                device.initialized = false;
                s4_data_invalidate(&state);
                recovery.retry_ms = now/1000U + S4_RETRY_US/1000U;
            }
        }
        s4_cache_increment(&state,S4_COUNT_HEARTBEAT);
        if (s4_status_publish(&state) < E_OK)
        {
            s4_cache_increment(&state,S4_COUNT_PUBLISH_ERROR);
        }
        (void)tk_dly_tsk(S4_POLL_MS);
    }
}
