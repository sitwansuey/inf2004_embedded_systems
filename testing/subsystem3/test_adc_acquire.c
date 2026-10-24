/* Execute the production sampler + copied-message ADC worker. Scheduling,
 * clocks and device values are deterministic fakes; no hardware is emulated. */
#include "adapter_fixture.h"

static void setup(bool raw, UW lease_ms)
{
    s3_adapter_config_t cfg = {0};
    reset();
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.ir = (s3_ir_config_t){S3_IR_ADC, {26U, 27U, 28U}};
    CHECK((raw ? s3_configure_raw(&cfg.ir) : s3_configure(&cfg)) == E_OK);
    CHECK(subsystem3_start() == E_OK);
    CHECK(s3_controller_bind() == E_OK);
    if (!raw) CHECK(s3_controller_control(S3_MODE_TRACK, false, lease_ms) == E_OK);
    caller = 42;
    adc_auto_worker = true;
}

static void test_current_scan_with_jitter(void)
{
    const uint32_t wake_gaps[] = {20U, 30U, 40U, 20U};
    const uint32_t starts[] = {0U, UINT32_MAX - 39U};
    unsigned int trial;
    unsigned int scan;
    for (trial = 0U; trial < 2U; ++trial)
    {
        setup(true, 0U);
        fake_now = starts[trial];
        adc_read_ms = 1U;
        for (scan = 0U; scan < 16U; ++scan)
        {
            s3_status_t snapshot;
            adc_values[0] = 100U + scan;
            adc_values[1] = 200U + scan;
            adc_values[2] = 300U + scan;
            adc_wait_delay_ms = (scan % 2U) * 10U;
            CHECK(s3_service_once() == E_OK);
            CHECK(s3_status_get(&snapshot) == E_OK);
            CHECK(snapshot.acquisition_error == E_OK && snapshot.sample.valid_mask == 7U);
            CHECK(snapshot.sample.raw[0] == 100U + scan);
            CHECK(snapshot.sample.raw[1] == 200U + scan && snapshot.sample.raw[2] == 300U + scan);
            CHECK(fake_now - snapshot.sample_time_ms == 3U);
            CHECK(!ir.adc_pending && !have_request && !have_reply);
            CHECK(!snapshot.configured && !snapshot.output.steering_allowed);
            CHECK(queue_count == 0U && !core.have_sample);
            /* If testing an old prefetching sampler, let its worker run
             * during the sleep too. The negative control must not starve
             * the worker just because that sampler never waits for it. */
            if (have_request)
            {
                caller = 43;
                (void)s3_adc_worker_once(TMO_POL);
                caller = 42;
            }
            fake_now += wake_gaps[scan % 4U];
        }
        CHECK(adc_reads == 48U && adc_opens == 1U && adc_reply_waits == 16U);
    }
}

static void stop_during_reply_wait(void)
{
    ID previous = caller;
    caller = 7;
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(!status.output.steering_allowed);
    caller = previous;
}

static void test_stop_wins_wait(void)
{
    setup(false, 1000U);
    CHECK(s3_service_once() == E_OK && status.output.steering_allowed);
    fake_now = 20U;
    adc_wait_hook = stop_during_reply_wait;
    CHECK(s3_service_once() == E_TMOUT);
    CHECK(!status.output.steering_allowed && queue_count == 0U);
    CHECK(adc_reads == 6U && !ir.adc_pending);
}

static void test_lease_expires_in_wait(void)
{
    setup(false, 25U);
    CHECK(s3_service_once() == E_OK && status.output.steering_allowed);
    fake_now = 20U;
    adc_wait_delay_ms = 10U;
    CHECK(s3_service_once() == E_OK && fake_now == 30U);
    CHECK(status.acquisition_error == E_OK && status.sample.valid_mask == 7U);
    CHECK(!status.output.steering_allowed && status.output.error == S3_ERROR_STALE);
}

static void test_wait_failures(void)
{
    unsigned int failure;
    for (failure = 0U; failure < 3U; ++failure)
    {
        ER expected = failure == 0U ? E_TMOUT : (failure == 1U ? E_SYS : E_IO);
        setup(false, 1000U);
        if (failure == 0U) adc_wait_delay_ms = 30U;
        if (failure == 1U) adc_fail_clock_after_wait = true;
        if (failure == 2U) malformed_reply = 1;
        CHECK(s3_service_once() == expected);
        CHECK(ir.adc_failed && !status.output.steering_allowed);
        CHECK(status.acquisition_error == expected && status.sample.valid_mask == 0U);
        CHECK(adc_reply_waits == 1U);
    }
    CHECK(s3_adc_receive_wait(NULL) == E_PAR);
    fail_time = false;
    caller = 7;
    CHECK(s3_adc_receive_wait(&adc_reply) == E_OACV);
}

static void test_bounded_absent_reply(void)
{
    setup(false, 1000U);
    adc_auto_worker = false;
    adc_wait_delay_ms = 20U; /* Model the rounded kernel timeout, no reply. */
    CHECK(s3_service_once() == E_TMOUT && fake_now == 20U);
    CHECK(adc_reply_waits == 1U && adc_reads == 0U && s3_ir_waiting(&ir));
    CHECK(!status.output.steering_allowed && status.sample.valid_mask == 0U);
    fake_now = 40U;
    CHECK(s3_service_once() == E_TMOUT && ir.adc_failed);
    CHECK(adc_reply_waits == 1U && adc_reads == 0U);
}

static void test_calibrated_paused_jitter(void)
{
    const uint32_t gaps[] = {20U, 40U, 30U, 1000U};
    const uint32_t starts[] = {0U, UINT32_MAX - 19U};
    for (unsigned int trial = 0U; trial < 2U; ++trial)
    {
        setup(false, 1000U);
        caller = 7;
        CHECK(s3_controller_control(S3_MODE_PAUSED, false, 0U) == E_OK);
        caller = 42;
        fake_now = starts[trial];
        for (unsigned int scan = 0U; scan < 12U; ++scan)
        {
            CHECK(s3_service_once() == E_OK);
            CHECK(status.configured && status.acquisition_error == E_OK);
            CHECK(status.sample.valid_mask == 7U && status.output.error == S3_ERROR_NONE);
            CHECK(!status.output.steering_allowed && queue_count == 0U);
            CHECK(core.mode == S3_MODE_PAUSED && core.latched_error == S3_ERROR_NONE);
            fake_now += gaps[scan % 4U];
        }
    }
}

int main(void)
{
    RUN("S3-U37 current scan/jitter/rollover/raw authority", test_current_scan_with_jitter);
    RUN("S3-U38 STOP during reply wait", test_stop_wins_wait);
    RUN("S3-U39 lease expires during reply wait", test_lease_expires_in_wait);
    RUN("S3-U40 clock/delivery/transport failure", test_wait_failures);
    RUN("S3-U41 absent reply bounded", test_bounded_absent_reply);
    RUN("S3-U45 configured paused jitter/rollover", test_calibrated_paused_jitter);
    (void)printf("Buddy 3 ADC acquisition: %u groups, %u checks, %u failures\n", groups, checks, failures);
    return failures == 0U ? 0 : 1;
}
