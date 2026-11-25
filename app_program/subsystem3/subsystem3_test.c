/*
 *    subsystem3_test.c
 *    On-target tests for subsystem 3.  Built only with: make TEST=1
 *    The integrated test image (make TEST=1 NET=lwip MQTT=1) runs them.
 *    S3-U01 is an on-device algorithm smoke check; S3-U66 and S3-U67 read
 *    the live status the IR sampling task publishes.  All are passive:
 *    they read state only and never take the controller binding.
 *
 *    Each TEST_CHECK prints a PASS/FAIL line on the console.  Save the
 *    console output to testing/subsystem3/logs/ and record the result
 *    in testing/subsystem3/TEST_LOG.md.  Test IDs: S3-U01, S3-U66, S3-U67
 *    (S3-U66/U67 are new IDs: S3-U02 to S3-U65 have all been used).
 */
#include "subsystem3.h"
#include "../test_report.h"

#if APP_TEST
/* Bounded waits: the sampling task (priority 16, 10 ms period) only runs
 * once the initial task (priority 1) blocks. */
#define S3_TEST_READY_MS 500U
#define S3_TEST_POLL_MS 20U
#define S3_TEST_ADC_MAX 4095U /* RP2040 ADC is 12-bit */

/* s3_status_get never waits for the lock (E_TMOUT while the sampler
 * publishes), so retry a few times. */
static ER
s3_test_status (s3_status_t * p_status)
{
    ER er = E_TMOUT;
    for (UINT i = 0U; (i < 5U) && (er == E_TMOUT); ++i)
    {
        er = s3_status_get(p_status);
        if (er == E_TMOUT)
        {
            tk_dly_tsk(1);
        }
    }
    return er;
}

EXPORT void
subsystem3_test (void)
{
    uint16_t    value = 0U;
    s3_status_t first = {0};
    s3_status_t later = {0};
    ER          er;
    UINT        waited   = 0U;
    BOOL        in_range = TRUE;

    TEST_INIT();
    TEST_CHECK("S3-U01",
               s3_normalize(4095U, 0U, 4095U, 4095U, &value) &&
                   (value == 1000U),
               "pure reflectance endpoint");

    /* S3-U66: the line sensors are configured and the sampling task keeps
     * publishing fresh scans without an acquisition error */
    er = s3_test_status(&first);
    while ((er == E_OK) && (!first.configured || first.sample_time_ms == 0U) &&
           (waited < S3_TEST_READY_MS))
    {
        tk_dly_tsk(S3_TEST_POLL_MS);
        waited += S3_TEST_POLL_MS;
        er = s3_test_status(&first);
    }
    if (er == E_OK)
    {
        tk_dly_tsk(5U * S3_PERIOD_MS);
        er = s3_test_status(&later);
    }
    TEST_CHECK("S3-U66",
               (er == E_OK) && first.configured && later.configured &&
                   (later.acquisition_error == E_OK) &&
                   (later.sample_time_ms != first.sample_time_ms),
               "IR sampler configured and publishing fresh scans");

    /* S3-U67: every channel of the latest scan is within the 12-bit ADC
     * range (the digital GP28 barcode channel reads 0 or 1) */
    for (UINT i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        if (later.sample.raw[i] > S3_TEST_ADC_MAX)
        {
            in_range = FALSE;
        }
    }
    TEST_CHECK("S3-U67",
               (er == E_OK) && (later.acquisition_error == E_OK) &&
                   (later.sample.valid_mask != 0U) && in_range,
               "IR raw readings within the ADC range");
    TEST_SUMMARY("subsystem3");
}
#endif

/*** end of file ***/
