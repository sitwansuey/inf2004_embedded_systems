/* Tests actual sampler/ADC worker code; fake time and device, no hardware. */
#include "adapter_fixture.h"

static s3_ir_t sensor;
static s3_sample_t result;

static void sensor_setup(void)
{
    const s3_ir_config_t cfg = {S3_IR_ADC, {28U, 26U, 27U}};
    reset();
    sensor = (s3_ir_t){0};
    caller = 42;
    CHECK(s3_adc_available());
    CHECK(s3_ir_open(&sensor, &cfg) == E_OK);
    CHECK(adc_opens == 0U && adc_reads == 0U && gpio_configs == 0U);
}

static ER worker_tick(void)
{
    ER er;
    caller = 43;
    er = s3_adc_worker_once(TMO_POL);
    caller = 42;
    return er;
}

static void begin_request(uint32_t now_ms)
{
    fake_now = now_ms;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_TMOUT);
    CHECK(result.valid_mask == 0U && s3_ir_waiting(&sensor));
    CHECK(have_request && !have_reply);
}

static void test_pipeline(void)
{
    sensor_setup();
    begin_request(100U);
    CHECK(adc_request.channel[0] == 2U && adc_request.channel[1] == 0U &&
          adc_request.channel[2] == 1U);
    CHECK(worker_tick() == E_OK && adc_reads == 3U && adc_opens == 1U);
    fake_now = 110U;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_OK);
    CHECK(result.time_ms == 100U && result.valid_mask == S3_SENSOR_MASK);
    CHECK(result.raw[0] == 0U && result.raw[1] == 0U && result.raw[2] == 4095U);
    CHECK(have_request && sensor.adc_sequence == 2U);
    CHECK(worker_tick() == E_OK && adc_opens == 1U);
    fake_now = 120U;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_OK && result.time_ms == 110U);
    sensor_setup();
    sensor.adc_sequence = UINT32_MAX;
    begin_request(UINT32_MAX - 5U);
    CHECK(adc_request.sequence == 1U);
    CHECK(worker_tick() == E_OK);
    fake_now = 4U;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_OK);
    CHECK(result.time_ms == UINT32_MAX - 5U);
}

static void test_timeout(void)
{
    unsigned int previous_reads;
    /* L3: a worker held off past its deadline before converting drops that
     * scan only. The client keeps waiting (no latch) and resubmits. */
    sensor_setup(); begin_request(0U);
    fake_now = 29U;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_TMOUT && s3_ir_waiting(&sensor));
    fake_now = 30U;
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_TMOUT && s3_ir_waiting(&sensor));
    CHECK(!sensor.adc_failed && result.valid_mask == 0U && adc_reads == 0U);
    CHECK(worker_tick() == E_TMOUT && adc_opens == 0U && !worker_failed);
    CHECK(adc_reply.error == E_TMOUT && adc_reply.retry && adc_reply.sample.valid_mask == 0U);
    CHECK(s3_ir_poll(&sensor, 31U, &result) == E_TMOUT && s3_ir_waiting(&sensor));
    CHECK(!sensor.adc_failed && result.valid_mask == 0U);
    CHECK(have_request && adc_request.issued_ms == 31U && adc_request.sequence == 2U);
    fake_now = 31U;
    CHECK(worker_tick() == E_OK && adc_reads == 3U && adc_opens == 1U);
    CHECK(s3_ir_poll(&sensor, 41U, &result) == E_OK && result.time_ms == 31U);
    CHECK(result.valid_mask == S3_SENSOR_MASK);
    /* A driver read that itself overran the deadline latches for the boot. */
    sensor_setup(); begin_request(0U);
    adc_read_ms = 1000U;
    CHECK(worker_tick() == E_TMOUT && fake_now == 1000U);
    CHECK(adc_reads == 1U && adc_reply.sample.valid_mask == 0U);
    CHECK(worker_failed && !adc_reply.retry);
    CHECK(s3_ir_poll(&sensor, fake_now, &result) == E_TMOUT && sensor.adc_failed);
    previous_reads = adc_reads;
    CHECK(worker_tick() == E_OBJ && adc_reads == previous_reads);
    /* Rollover: the same drop and resubmit across the 32-bit wrap. */
    sensor_setup(); begin_request(UINT32_MAX - 5U);
    CHECK(s3_ir_poll(&sensor, 24U, &result) == E_TMOUT && !sensor.adc_failed);
    fake_now = 24U;
    CHECK(worker_tick() == E_TMOUT && !worker_failed && adc_reads == 0U);
    CHECK(s3_ir_poll(&sensor, 24U, &result) == E_TMOUT && adc_request.issued_ms == 24U);
    CHECK(worker_tick() == E_OK);
    CHECK(s3_ir_poll(&sensor, 30U, &result) == E_OK && result.time_ms == 24U);
}

static void test_unavailable(void)
{
    sensor_setup(); begin_request(10U);
    adc_open_result = E_NOSPT;
    CHECK(worker_tick() == E_NOSPT && adc_reads == 0U && worker_failed);
    CHECK(s3_ir_poll(&sensor, 11U, &result) == E_NOSPT && sensor.adc_failed);
    CHECK(result.valid_mask == 0U);
    CHECK(s3_ir_poll(&sensor, 12U, &result) == E_NOSPT && adc_opens == 1U);
    sensor_setup(); begin_request(0U);
    adc_read_result = E_IO;
    CHECK(worker_tick() == E_IO && adc_reads == 1U);
    CHECK(s3_ir_poll(&sensor, 1U, &result) == E_IO && result.valid_mask == 0U);
}

static void test_invalid_readings(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 8U; ++scenario)
    {
        sensor_setup(); begin_request(100U);
        if (scenario == 0U) { adc_values[2] = 4096U; }
        if (scenario == 1U) { adc_read_size = 0; }
        if (scenario == 2U) { adc_read_size = 2; }
        if (scenario < 3U)
        {
            CHECK(worker_tick() == E_IO);
        }
        else
        {
            CHECK(worker_tick() == E_OK);
            if (scenario == 3U) { adc_reply.sample.raw[0] = 65535U; }
            if (scenario == 4U) { adc_reply.sample.valid_mask = 3U; }
            if (scenario == 5U) { adc_reply.sample.time_ms = 111U; }
            if (scenario == 6U) { adc_reply.sample.time_ms = 99U; }
            if (scenario == 7U) { adc_reply.sequence += 1U; }
        }
        if (scenario == 7U)
        {
            /* L3: a reply for another scan is stale: discarded, still waiting. */
            CHECK(s3_ir_poll(&sensor, 110U, &result) == E_TMOUT);
            CHECK(!sensor.adc_failed && s3_ir_waiting(&sensor));
            CHECK(result.valid_mask == 0U && !have_request && !have_reply);
            continue;
        }
        CHECK(s3_ir_poll(&sensor, 110U, &result) == E_IO);
        CHECK(sensor.adc_failed && result.valid_mask == 0U && !have_request);
    }
}

static void test_lifecycle_and_ownership(void)
{
    unsigned int failure;
    for (failure = 1U; failure <= 3U; ++failure)
    {
        reset(); caller = 42; fail_create = failure;
        CHECK(s3_adc_start() == E_SYS && deletes == failure - 1U);
        CHECK(h_requests == 0 && h_replies == 0 && h_worker == 0);
        fail_create = 0U;
        CHECK(s3_adc_start() == E_OK);
    }
    reset(); caller = 42; fail_start = true;
    CHECK(s3_adc_start() == E_SYS && deletes == 3U);
    CHECK(h_requests == 0 && h_replies == 0 && h_worker == 0 && h_client == 0);
    fail_start = false;
    CHECK(s3_adc_start() == E_OK);
    CHECK(s3_adc_start() == E_OBJ);
    CHECK(s3_adc_submit(NULL) == E_PAR && s3_adc_receive(NULL) == E_PAR);
    caller = 7;
    CHECK(s3_adc_submit(&(s3_adc_request_t){0}) == E_OACV);
    CHECK(s3_adc_receive(&adc_reply) == E_OACV);
    CHECK(s3_adc_worker_once(TMO_POL) == E_OACV);
    CHECK(adc_reads == 0U && adc_opens == 0U);
}

static void test_transport_and_request_validation(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 4U; ++scenario)
    {
        sensor_setup(); begin_request(0U);
        if (scenario == 0U) { adc_request.channel[0] = 3U; }
        if (scenario == 1U) { adc_request.channel[0] = adc_request.channel[1]; }
        if (scenario == 2U) { adc_request.sequence = 0U; }
        if (scenario == 3U) { malformed_request = 1; }
        CHECK(worker_tick() < E_OK && adc_reads == 0U && adc_opens == 0U);
        CHECK(worker_failed && adc_reply.sample.valid_mask == 0U);
    }
    sensor_setup(); begin_request(0U);
    CHECK(worker_tick() == E_OK);
    malformed_reply = 1;
    CHECK(s3_ir_poll(&sensor, 10U, &result) == E_IO && sensor.adc_failed);
    sensor_setup();
    have_request = true;
    CHECK(s3_ir_poll(&sensor, 0U, &result) == E_TMOUT && sensor.adc_failed);
    CHECK(!s3_ir_waiting(&sensor) && adc_opens == 0U);
    sensor_setup(); begin_request(0U);
    have_reply = true;
    CHECK(worker_tick() == E_TMOUT && worker_failed);
    sensor_setup(); begin_request(0U);
    fail_time = true;
    CHECK(worker_tick() == E_SYS && adc_opens == 0U);
}

static void integration_setup(void)
{
    s3_adapter_config_t cfg = {0};
    unsigned int i;
    reset();
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.ir.mode = S3_IR_ADC;
    for (i = 0U; i < 3U; ++i) { cfg.ir.pin[i] = (uint8_t)(26U + i); }
    CHECK(s3_configure(&cfg) == E_OK);
    CHECK(subsystem3_start() == E_OK);
    CHECK(s3_controller_bind() == E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    caller = 42; fake_now = 10U;
    CHECK(s3_service_once() == E_TMOUT && !status.output.steering_allowed);
    CHECK(adc_reads == 0U && s3_ir_waiting(&ir));
}

static void test_integration_success_and_hold(void)
{
    s3_status_t snapshot;
    s3_event_record_t record;
    integration_setup();
    CHECK(worker_tick() == E_OK);
    fake_now = 20U;
    CHECK(s3_service_once() == E_OK && status.output.steering_allowed);
    caller = 7;
    CHECK(s3_status_get(&snapshot) == E_OK && snapshot.sample_time_ms == 10U);
    CHECK(snapshot.output.line.error == 0 && snapshot.acquisition_error == E_OK);
    CHECK(snapshot.sample.valid_mask == S3_SENSOR_MASK && snapshot.sample.raw[1] == 4095U);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    adc_auto_worker = true;
    caller = 42;
    fake_now = 30U;
    CHECK(s3_service_once() == E_OK && !status.output.steering_allowed);
    /* L3: the worker is held off 30 ms (busy core): that scan is dropped and
     * resubmitted, nothing latches. */
    adc_wait_delay_ms = 30U;
    fake_now = 60U;
    CHECK(s3_service_once() == E_TMOUT && !ir.adc_failed && s3_ir_waiting(&ir));
    CHECK(adc_reads == 6U && status.acquisition_error == E_TMOUT);
    CHECK(!worker_failed && have_request && adc_request.issued_ms == 90U);
    /* A failed driver read latches for the boot; TRACK cannot clear it. */
    adc_wait_delay_ms = 0U; adc_read_result = E_IO;
    fake_now = 100U;
    CHECK(s3_service_once() == E_IO && ir.adc_failed && adc_reads == 7U);
    caller = 7;
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    caller = 42; fake_now = 110U;
    CHECK(s3_service_once() == E_IO && !status.output.steering_allowed);
    CHECK(status.output.error == S3_ERROR_SENSOR && adc_reads == 7U);
    caller = 7;
    CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_FAULT);
}

static void timeout_while_reading(void)
{
    unsigned int reads = adc_reads;
    caller = 42;
    /* The sampler never cancels or latches a worker inside a read: it
     * reports waiting, with no reads, no steering and no event. */
    CHECK(s3_service_once() == E_TMOUT);
    CHECK(adc_reads == reads && !ir.adc_failed && s3_ir_waiting(&ir));
    CHECK(!status.output.steering_allowed && queue_count == 0U);
    caller = 43;
}

static void test_worker_stall_isolation(void)
{
    s3_event_record_t record;
    integration_setup();
    adc_read_ms = 100U; /* within the 1000 ms lease */
    adc_read_hook = timeout_while_reading;
    /* The read itself overran its deadline: permanent for this boot. */
    CHECK(worker_tick() == E_TMOUT && worker_failed && !adc_reply.retry);
    CHECK(adc_reads == 1U && status.acquisition_error == E_TMOUT);
    caller = 42; fake_now += 10U;
    CHECK(s3_service_once() == E_TMOUT && ir.adc_failed && !s3_ir_waiting(&ir));
    CHECK(status.output.error == S3_ERROR_SENSOR && !status.output.steering_allowed);
    caller = 7;
    CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_FAULT);
    caller = 42; fake_now += 10U;
    CHECK(s3_service_once() == E_TMOUT && queue_count == 0U);
    CHECK(adc_reads == 1U && !status.output.steering_allowed);
}

static void test_integration_device_errors(void)
{
    unsigned int scenario;
    for (scenario = 0U; scenario < 3U; ++scenario)
    {
        s3_event_record_t record;
        ER expected = (scenario == 0U) ? E_NOSPT : E_IO;
        integration_setup();
        if (scenario == 0U) { adc_open_result = E_NOSPT; }
        if (scenario == 1U) { adc_values[0] = 4096U; }
        if (scenario == 2U) { adc_read_size = 0; }
        CHECK(worker_tick() == expected);
        fake_now = 20U;
        CHECK(s3_service_once() == expected);
        CHECK(status.acquisition_error == expected && status.output.error == S3_ERROR_SENSOR);
        CHECK(!status.output.steering_allowed && status.sample.valid_mask == 0U);
        CHECK(status.sample.raw[0] == 0U && status.sample.raw[1] == 0U && status.sample.raw[2] == 0U);
        caller = 7;
        CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_FAULT);
        caller = 42; fake_now = 30U;
        CHECK(s3_service_once() == expected && queue_count == 0U);
    }
}

int main(void)
{
    RUN("S3-U20 ADC pipeline/timestamp/rollover", test_pipeline);
    RUN("S3-U21 ADC deadline and late completion", test_timeout);
    RUN("S3-U22 ADC unavailable/read error", test_unavailable);
    RUN("S3-U23 ADC invalid/partial readings", test_invalid_readings);
    RUN("S3-U24 ADC lifecycle/ownership", test_lifecycle_and_ownership);
    RUN("S3-U25 ADC transport/request validation", test_transport_and_request_validation);
    RUN("S3-U26 ADC controller integration", test_integration_success_and_hold);
    RUN("S3-U27 blocked worker isolation/fault", test_worker_stall_isolation);
    RUN("S3-U28 acquisition errors/raw snapshot", test_integration_device_errors);
    (void)printf("Buddy 3 ADC: %u groups, %u checks, %u failures\n", groups, checks, failures);
    return (failures == 0U) ? 0 : 1;
}
