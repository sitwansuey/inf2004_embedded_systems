/* SPDX-License-Identifier: Apache-2.0 */
#ifndef S4_SENSOR_CYCLE_H
#define S4_SENSOR_CYCLE_H

#include "sensor_data.h"

/* One 10 ms acquisition cycle of s4_sensor_task, with no RTOS or hardware
 * calls of its own, so the recovery rules can be tested on the host.
 * Only the accelerometer decides recovery: the magnetometer is telemetry. */

/* Failed accel polls in a row before the device is re-initialised. */
#define S4_FAILED_POLLS 10U
/* Polls (10 ms) without a fresh accel sample (one every 20 ms) after which
 * each further poll counts as failed: the device answers but sends no data,
 * e.g. a supply dip restored its power-down defaults. About when the sample
 * goes stale (S4_STALE_AFTER_US). */
#define S4_IDLE_POLLS 10U
/* Wait after a failed init attempt; the first attempt is never delayed. */
#define S4_RETRY_MS 1000U

typedef struct
{
    lsm303_device_t device;
    lsm303_bus_t bus;
    void (*recover)(void); /* frees a stuck bus before each init; may be NULL */
    uint64_t retry_ms;     /* next init attempt while the device is down */
    uint64_t mag_retry_ms; /* next magnetometer set-up while only it is down */
    uint8_t failed_polls;
    uint8_t idle_polls;
} s4_sensor_cycle_t;

void s4_cycle_reset(s4_sensor_cycle_t *p_cycle, const lsm303_bus_t *p_bus,
                    void (*recover)(void));
/* Init when due, else poll; updates the task's working copy p_state. */
void s4_cycle_run(s4_sensor_cycle_t *p_cycle, s4_sensor_cache_t *p_state,
                  uint64_t now_ms);

#endif
