/*
 *	subsystem3_test.c
 *	On-target tests for subsystem 3.  Built only with: make TEST=1
 *	The integrated test image (make TEST=1 NET=lwip MQTT=1) runs them.
 *	On-device algorithm smoke checks, not sensor/hardware validation.
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem3/logs/ and record the result
 *	in testing/subsystem3/TEST_LOG.md.  Test ID: S3-U01.
 */
#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
EXPORT void subsystem3_test(void)
{
    uint16_t value = 0U;
    TEST_INIT();
    TEST_CHECK("S3-U01", s3_normalize(4095U, 0U, 4095U, 4095U, &value) &&
               (value == 1000U), "pure reflectance endpoint");
    TEST_SUMMARY("subsystem3");
}
#endif
