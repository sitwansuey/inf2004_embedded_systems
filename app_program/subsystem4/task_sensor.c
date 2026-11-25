/* SPDX-License-Identifier: Apache-2.0 */
#include "subsystem4.h"
#include "drv_pico_i2c.h"
#include "sensor_cycle.h"
#if defined(_PICO_RP2040_)
/* Watchdog heartbeat (app_watchdog.c): one per loop. Host builds: none. */
#include "../../include/tm/robot_health.h"
#define S4_HEARTBEAT() HEALTH_BEAT(HEALTH_HB_IMU)
#else
#define S4_HEARTBEAT() ((void) 0)
#endif

static void
wait_for_sensor_release (void)
{
    /* Drop releases missed during I2C work. A cyclic wake-up sets the
     * cadence without adding a full kernel tick to a relative delay. */
    if (tk_can_wup(TSK_SELF) >= E_OK)
    {
        (void) tk_slp_tsk(TMO_FEVR);
    }
}

void
s4_sensor_task (INT start_code, void * p_exinf)
{
    lsm303_bus_t      bus;
    s4_sensor_cycle_t cycle;
    s4_sensor_cache_t state;

    (void) start_code;
    (void) p_exinf;
    s4_data_reset(&state);
    s4_pico_i2c_init(&bus);
    /* Share the kernel/encoder timebase for public sample timestamps.
     * SDK transaction deadlines still use the hardware timer. */
    bus.now_us = s4_kernel_now_us;
    /* The bus is cleared before every init, the first one included. */
    s4_cycle_reset(&cycle, &bus, s4_pico_i2c_recover);

    for (;;)
    {
        S4_HEARTBEAT();
        uint64_t now = s4_kernel_now_us(NULL);
        if (now == UINT64_MAX)
        {
            s4_data_invalidate(&state);
            cycle.device.initialized = false;
            (void) s4_status_publish(&state);
            wait_for_sensor_release();
            continue;
        }

        /* Init when due, else poll; accel-only recovery (sensor_cycle.c). */
        s4_cycle_run(&cycle, &state, now / 1000U);
        s4_cache_increment(&state, S4_COUNT_HEARTBEAT);
        if (s4_status_publish(&state) < E_OK)
        {
            s4_cache_increment(&state, S4_COUNT_PUBLISH_ERROR);
        }
        wait_for_sensor_release();
    }
}

/*** end of file ***/
