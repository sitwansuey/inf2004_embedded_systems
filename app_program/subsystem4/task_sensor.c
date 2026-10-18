/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"
#include "drv_pico_i2c.h"

void s4_sensor_task(INT start_code, void *p_exinf)
{
    lsm303_bus_t bus;
    lsm303_device_t device = { 0 };
    s4_sensor_status_t state;
    uint64_t next_retry = 0;
    uint32_t failed_polls = 0;

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

        if (!device.initialized && (now >= next_retry))
        {
            lsm303_result_t result = lsm303dlhc_init(&device, &bus);

            s4_data_init_result(&state, result);
            next_retry = s4_kernel_now_us(NULL) + S4_RETRY_US;
            failed_polls = 0;
        }
        if (device.initialized)
        {
            lsm303_frame_t frame;
            lsm303_result_t result = lsm303dlhc_poll(&device, &frame);

            s4_data_update(&state, &frame, result);
            failed_polls = (result == LSM303_IO_ERROR) ? failed_polls + 1u : 0u;
            if (failed_polls >= 10u)
            {
                device.initialized = false;
                state.configured = false;
                state.init_result = LSM303_IO_ERROR;
                next_retry = s4_kernel_now_us(NULL) + S4_RETRY_US;
            }
        }
        state.task_heartbeat++;
        state.observed_time_us = s4_kernel_now_us(NULL);
        if (s4_status_publish(&state) < E_OK)
        {
            state.publish_errors++;
        }
        (void)tk_dly_tsk(S4_POLL_MS);
    }
}
