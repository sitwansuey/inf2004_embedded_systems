/* SPDX-License-Identifier: Apache-2.0 */
/*
 *	subsystem4_test.c
 *	On-target tests for subsystem 4.  Built only with: make TEST=1
 *	Build with S4_ONLY=1 TEST=1 for sensor bench testing.  The integrated
 *	test image (make TEST=1 NET=lwip MQTT=1) runs the same checks.
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem4/logs/ and record the result
 *	in testing/subsystem4/TEST_LOG.md.  Test IDs: S4-U03 to S4-U06.
 */
#include "subsystem4.h"
#include "../test_report.h"

#if APP_TEST
void subsystem4_test(void)
{
    T_RTSK sensor_task;
    T_RTSK report_task;
    s4_sensor_status_t before;
    s4_sensor_status_t after;
    ER result;

    TEST_INIT();
    TEST_CHECK("S4-U03", s4_sensor_get_status(NULL) == E_PAR,
               "sensor snapshot rejects NULL");
    TEST_CHECK("S4-U04",
               tk_ref_tsk(tid_s4_sensor, &sensor_task) == E_OK &&
               tk_ref_tsk(tid_s4_monitor, &report_task) == E_OK &&
               sensor_task.tskstat != TTS_DMT &&
               report_task.tskstat != TTS_DMT &&
               sensor_task.tskpri == S4_PRI_SENSOR &&
               report_task.tskpri == S4_PRI_MONITOR,
               "both S4 tasks exist and use the assigned priority band");

    result = s4_sensor_get_status(&before);
    (void)tk_dly_tsk(500);
    TEST_CHECK("S4-U05",
               result == E_OK && s4_sensor_get_status(&after) == E_OK &&
               after.task_heartbeat > before.task_heartbeat,
               "acquisition task continues, including with an absent sensor");

    result = s4_sensor_get_status(&before);
    if ((result == E_OK) && before.configured)
    {
        (void)tk_dly_tsk(500);
        TEST_CHECK("S4-U06",
                   s4_sensor_get_status(&after) == E_OK &&
                   after.accel_samples > before.accel_samples &&
                   after.mag_samples > before.mag_samples &&
                   after.accel_usable && after.mag_usable,
                   "connected sensor delivers fresh, unclipped channel data");
    }
    else
    {
        tm_printf((UB *)"[TEST] S4-U06 SKIP  sensor not configured; "
                         "physical communication remains unverified\n");
    }

    TEST_SUMMARY("subsystem4");
}
#endif
