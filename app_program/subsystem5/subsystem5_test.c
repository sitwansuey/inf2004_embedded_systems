/*
 *	subsystem5_test.c
 *	On-target tests for subsystem 5.  Built only with: make TEST=1
 *	The integrated test image (make TEST=1 NET=lwip MQTT=1) runs them.
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem5/logs/ and record the result
 *	in testing/subsystem5/TEST_LOG.md.  Test IDs: S5-U01 to S5-U03.
 */
#include "subsystem5.h"
#include "../test_report.h"
#if APP_TEST
void subsystem5_test(void) {
    TEST_INIT();
    s5_status_t p={.state=S5_MONITOR,.valid=TRUE,.distance_mm=201,.time_ms=1};
    TEST_CHECK("S5-U01",s5_assess_path(&p,TRUE,1).state==S5_PATH_FORWARD,"fresh clear path");
    p.distance_mm=200;
    TEST_CHECK("S5-U02",s5_assess_path(&p,TRUE,1).state==S5_PATH_STOP,"inclusive stop threshold");
    TEST_CHECK("S5-U03",s5_assess_path(&p,TRUE,502).state==S5_PATH_UNAVAILABLE,"stale stop");
    TEST_SUMMARY("subsystem5");
}
#endif
