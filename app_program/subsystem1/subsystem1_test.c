/*
 *	subsystem1_test.c
 *	On-target tests for subsystem 1.  Built only with: make TEST=1
 *	The integrated test image (make TEST=1 NET=lwip MQTT=1) runs them
 *	from usermain() once every subsystem has started.
 *
 *	Passive: the checks only read kernel object state and encode/decode
 *	local buffers.  Nothing is sent over MQTT and no pin is written.
 *
 *	Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *	console output to testing/subsystem1/logs/ and record the result
 *	in testing/subsystem1/TEST_LOG.md.  Test IDs: S1-U21 to S1-U23
 *	(new IDs: S1-U01 to S1-U20 have all been used before, see the
 *	TEST_LOG's ID-change notes).
 */

#include "subsystem1.h"
#include "telemetry_codec.h"
#include "../test_report.h"

#if APP_TEST

/* Static so the 45-field vectors do not sit on the initial task's stack. */
static int64_t s1_test_in[TEL_FIELD_COUNT];
static int64_t s1_test_out[TEL_FIELD_COUNT];
static uint8_t s1_test_packet[TELEMETRY_BYTES];

/* A snapshot with version 1, most fields 0 and a few fields at the ends
 * of their ranges (telemetry_codec.c field table), so the signed offset
 * encoding and the 32-bit fields are exercised.  Encode, decode, compare
 * every field, then check that an out-of-range field is rejected. */
static BOOL s1_test_roundtrip(void)
{
	UINT	i;

	for (i = 0; i < TEL_FIELD_COUNT; i++) {
		s1_test_in[i] = 0;
		s1_test_out[i] = -1;
	}
	s1_test_in[TEL_VERSION] = 1;
	s1_test_in[TEL_SEQUENCE] = 65535;
	s1_test_in[TEL_LEFT_COUNTS] = -2147483647LL - 1LL;
	s1_test_in[TEL_RIGHT_COUNTS] = 2147483647LL;
	s1_test_in[TEL_LINE_ERROR] = -1000;
	s1_test_in[TEL_ROLL_CDEG] = 18000;
	s1_test_in[TEL_IMU_FAULT] = 1;

	if (!telemetry_encode(s1_test_packet, s1_test_in) ||
	    !telemetry_decode(s1_test_packet, s1_test_out)) {
		return FALSE;
	}
	for (i = 0; i < TEL_FIELD_COUNT; i++) {
		if (s1_test_out[i] != s1_test_in[i]) {
			return FALSE;
		}
	}
	/* An out-of-range field must be rejected (version is fixed at 1). */
	s1_test_in[TEL_VERSION] = 2;
	return !telemetry_encode(s1_test_packet, s1_test_in);
}

EXPORT void subsystem1_test(void)
{
	T_RCYC	rcyc;
	T_RTSK	rtsk;
	ER	er;

	TEST_INIT();

	/* S1-U21: the LED cyclic handler exists and is running */
	er = tk_ref_cyc(cyc_led, &rcyc);
	TEST_CHECK("S1-U21", er == E_OK && (rcyc.cycstat & TCYC_STA),
		   "LED cyclic handler is running");

	/* S1-U22: between ticks the LED task is asleep, not busy-looping.
	   Sample it 10 times over ~1 s; it should almost always be waiting. */
	{
		INT	i, waiting = 0;
		for (i = 0; i < 10; i++) {
			tk_dly_tsk(97);	/* not a multiple of the tick */
			if (tk_ref_tsk(tid_led, &rtsk) == E_OK &&
			    (rtsk.tskstat & TTS_WAI)) {
				waiting++;
			}
		}
		TEST_CHECK("S1-U22", waiting >= 9, "LED task sleeps between ticks");
	}

	/* S1-U23: the 55-byte telemetry snapshot codec round-trips range
	   ends on the target compiler and rejects an out-of-range field */
	TEST_CHECK("S1-U23", s1_test_roundtrip(),
		   "telemetry snapshot codec round-trips and rejects bad version");

	TEST_SUMMARY("subsystem1");
}

#endif /* APP_TEST */
