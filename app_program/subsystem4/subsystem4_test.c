/* SPDX-License-Identifier: Apache-2.0 */
/*
 *    subsystem4_test.c
 *    On-target tests for subsystem 4.  Built only with: make TEST=1
 *    Build with S4_ONLY=1 TEST=1 for sensor bench testing.  The integrated
 *    test image (make TEST=1 NET=lwip MQTT=1) runs the same checks.
 *
 *    Passive: the checks only read the published IMU snapshot (5 ms lock
 *    wait) and the task table.  They never call the I2C driver: only the
 *    S4 sensor task may.  The identity check (S4-U54) uses the result of
 *    that task's own initialization, which reads the LSM303DLHC identity
 *    registers ('H', '4', '3') and verifies the configuration.
 *
 *    Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *    console output to testing/subsystem4/logs/ and record the result
 *    in testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U51 to S4-U55 (new
 *    IDs: S4-U01 to S4-U50 have all been used before, see the TEST_LOG's
 *    ID-change notes).
 */
#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST
/* Bounded wait for the first initialization: the S4 tasks only run once
 * the initial task (priority 1) blocks. */
#define S4_TEST_READY_MS 1000U
#define S4_TEST_POLL_MS 50U
#define S4_TEST_RUN_MS 500U

void
subsystem4_test (void)
{
    T_RTSK             sensor_task;
    T_RTSK             report_task;
    s4_sensor_status_t before;
    s4_sensor_status_t after;
    ER                 result;
    UINT               waited = 0U;

    TEST_INIT();
    TEST_CHECK("S4-U51", s4_sensor_get_status(NULL) == E_PAR,
               "sensor snapshot rejects NULL");
    TEST_CHECK("S4-U52",
               tk_ref_tsk(tid_s4_sensor, &sensor_task) == E_OK &&
                   tk_ref_tsk(tid_s4_monitor, &report_task) == E_OK &&
                   sensor_task.tskstat != TTS_DMT &&
                   report_task.tskstat != TTS_DMT &&
                   sensor_task.tskbpri == S4_PRI_SENSOR &&
                   report_task.tskbpri == S4_PRI_MONITOR,
               "both S4 tasks exist and use the assigned priority band");

    result = s4_sensor_get_status(&before);
    (void) tk_dly_tsk(S4_TEST_RUN_MS);
    TEST_CHECK("S4-U53",
               result == E_OK && s4_sensor_get_status(&after) == E_OK &&
                   after.task_heartbeat != before.task_heartbeat,
               "acquisition task keeps running");

    /* S4-U54: the IMU accelerometer accepted its configuration (bounded
     * wait for the first attempt). lsm303dlhc_init() does not check the
     * magnetometer result; S4-U55 covers the magnetometer data. */
    result = s4_sensor_get_status(&before);
    while ((result == E_OK) && !before.configured &&
           (waited < S4_TEST_READY_MS))
    {
        (void) tk_dly_tsk(S4_TEST_POLL_MS);
        waited += S4_TEST_POLL_MS;
        result = s4_sensor_get_status(&before);
    }
    TEST_CHECK("S4-U54",
               result == E_OK && before.configured &&
                   before.init_result == LSM303_OK,
               "IMU accelerometer configuration accepted");

    if ((result == E_OK) && before.configured)
    {
        (void) tk_dly_tsk(S4_TEST_RUN_MS);
        TEST_CHECK("S4-U55",
                   s4_sensor_get_status(&after) == E_OK &&
                       after.accel_samples != before.accel_samples &&
                       after.mag_samples != before.mag_samples &&
                       after.accel_usable && after.mag_usable,
                   "connected IMU delivers fresh, unclipped channel data");
    }
    else
    {
        tm_printf((UB *) "[TEST] S4-U55 SKIP  sensor not configured "
                         "(see S4-U54)\n");
    }

    TEST_SUMMARY("subsystem4");
}
#endif

/*** end of file ***/
