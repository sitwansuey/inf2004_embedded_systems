#include "adapter_fixture.h"

static void setup(void)
{
    s3_adapter_config_t cfg = {0};
    unsigned int i;
    reset();
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.algorithms.line.raw_max = 1U;
    cfg.algorithms.line.line_mask = 3U;
    cfg.algorithms.barcode_sensor = 2U;
    cfg.algorithms.map_a = S3_NAV_LEFT; /* Explicitly a test mapping only. */
    cfg.ir.mode = S3_IR_DIGITAL;
    for (i = 0U; i < 3U; ++i)
    {
        cfg.algorithms.line.black[i] = 1U;
        cfg.ir.pin[i] = (uint8_t)(i + 2U); /* Synthetic host pins. */
    }
    CHECK(s3_configure(&cfg) == E_OK);
    CHECK(subsystem3_start() == E_OK);
    CHECK(s3_controller_bind() == E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    gpio[2] = 1U;
}

static ER tick(bool black)
{
    ER result;
    fake_now += 10U;
    caller = 7;
    if (auto_renew && control.mode == S3_MODE_TRACK &&
        ((fake_now - control.issued_ms) >= 100U))
    {
        CHECK(s3_controller_renew(1000U) == E_OK);
    }
    caller = 42; gpio[4] = black ? 1U : 0U;
    result = s3_service_once();
    caller = 7;
    return result;
}

static void marker(void)
{
    const unsigned int widths[] = {
        1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1
    };
    unsigned int i;
    unsigned int t;
    for (i = 0U; i < 22U; ++i) { (void)tick(false); }
    (void)tick(true);
    for (i = 0U; i < 29U; ++i)
    {
        for (t = 10U; t <= widths[i] * 40U; t += 10U)
        {
            bool black = (i % 2U) == 0U;
            (void)tick(t == widths[i] * 40U ? !black : black);
        }
    }
    for (t = 0U; t < 500U; t += 10U) { (void)tick(false); }
}

static void test_disabled_and_lifecycle(void)
{
    s3_status_t snapshot;
    unsigned int failure;
    reset();
    CHECK(subsystem3_start() == E_OBJ);
    for (failure = 1U; failure <= 3U; ++failure)
    {
        reset(); fail_create = failure;
        CHECK(subsystem3_init() == E_SYS);
        CHECK(deletes == failure - 1U);
        fail_create = 0U;
        CHECK(subsystem3_init() == E_OK);
    }
    reset(); CHECK(subsystem3_init() == E_OK);
    CHECK(subsystem3_init() == E_OBJ);
    fail_start = true; CHECK(subsystem3_start() == E_SYS);
    CHECK(!started && deletes == 1U);
    fail_start = false; CHECK(subsystem3_start() == E_OK);
    CHECK(subsystem3_start() == E_OBJ);
    caller = 42;
    CHECK(s3_service_once() == E_NOSPT);
    CHECK(gpio_reads == 0U && gpio_configs == 0U);
    CHECK(s3_status_get(&snapshot) == E_OK);
    CHECK(!snapshot.configured && !snapshot.output.steering_allowed);
    CHECK(snapshot.output.error == S3_ERROR_CONFIG);
}

static void test_controller_leases_and_safety(void)
{
    s3_status_t snapshot;
    unsigned int i;
    setup();
    CHECK(tick(false) == E_OK);
    CHECK(s3_status_get(&snapshot) == E_OK && snapshot.output.steering_allowed);
    caller = 8;
    CHECK(s3_controller_bind() == E_OACV);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OACV);
    CHECK(s3_event_receive(&(s3_event_record_t){0}) == E_OACV);
    caller = 7;
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 0U) == E_PAR);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1001U) == E_PAR);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    (void)tick(false);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 100U) == E_OK);
    auto_renew = false;
    for (i = 0U; i < 10U; ++i) { (void)tick(false); }
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    CHECK(s3_controller_renew(1000U) == E_OBJ);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    (void)tick(false);
    CHECK(s3_status_get(&snapshot) == E_OK && snapshot.output.steering_allowed);
    fake_now += 30U;
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    fail_time = true;
    CHECK(s3_status_get(&snapshot) == E_SYS && !snapshot.output.steering_allowed);
    caller = 42;
    CHECK(s3_service_once() == E_SYS);
}

static void test_controller_enum_boundaries(void)
{
    const int invalid_modes[] = {-1, S3_MODE_OVERRIDE + 1, 99};
    unsigned char before_control[sizeof(control)];
    unsigned char before_status[sizeof(status)];
    size_t i;

    setup();
    CHECK(tick(false) == E_OK);
    CHECK(status.output.steering_allowed);
    (void)memcpy(before_control, &control, sizeof(control));
    (void)memcpy(before_status, &status, sizeof(status));
    for (i = 0U; i < sizeof(invalid_modes) / sizeof(invalid_modes[0]); ++i)
    {
        CHECK(s3_controller_control((s3_mode_t)invalid_modes[i], false, 0U) == E_PAR);
        CHECK(s3_controller_control((s3_mode_t)invalid_modes[i], false, 1000U) == E_PAR);
        CHECK(memcmp(before_control, &control, sizeof(control)) == 0);
        CHECK(memcmp(before_status, &status, sizeof(status)) == 0);
    }
    fail_time = true;
    CHECK(s3_controller_control((s3_mode_t)-1, false, 0U) == E_PAR);
    fail_time = false;
    CHECK(s3_controller_control(S3_MODE_PAUSED, false, 0U) == E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
}

static void test_override_publication_race(void)
{
    s3_status_t snapshot;
    setup(); (void)tick(false);
    race_override = true;
    CHECK(tick(false) == E_TMOUT);
    CHECK(s3_status_get(&snapshot) == E_OK && !snapshot.output.steering_allowed);
    CHECK(queue_count == 0U);
    (void)tick(false);
    CHECK(!status.output.steering_allowed);
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    (void)tick(false);
    CHECK(status.output.steering_allowed);
    busy_mutex = true;
    CHECK(s3_status_get(&snapshot) == E_TMOUT && !snapshot.output.steering_allowed);
    CHECK(tick(false) == E_TMOUT);
}

static void test_event_handoff_and_staleness(void)
{
    s3_event_record_t record;
    setup(); marker();
    CHECK(queue_count == 1U && !status.output.steering_allowed);
    CHECK(s3_event_receive(&record) == E_OK);
    CHECK(record.event.kind == S3_EVENT_NAVIGATION && record.event.symbol == 'A');
    CHECK(record.event.command == S3_NAV_LEFT);
    CHECK(s3_event_receive(&record) == E_TMOUT && record.event.kind == S3_EVENT_NONE);
    CHECK(s3_controller_renew(1000U) == E_OK);
    CHECK((core.mode == S3_MODE_TRACK) && core.awaiting_controller); /* Renewal cannot resume. */
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    marker(); CHECK(queue_count == 0U); /* Same physical marker, no rearm. */
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 1000U) == E_OK);
    marker(); CHECK(queue_count == 1U);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_event_receive(&record) == E_TMOUT); /* Generation invalidation. */
    setup(); marker(); fake_now += 100U;
    CHECK(s3_event_receive(&record) == E_TMOUT); /* Event too old. */
    setup(); marker();
    gpio[2] = 2U; (void)tick(false);
    CHECK(s3_event_receive(&record) == E_OK); /* Fault may be received. */
    CHECK(record.event.kind == S3_EVENT_FAULT); /* Old turn was discarded. */
}

static void test_transport_overflow(void)
{
    s3_event_record_t record;
    setup(); queue_count = 4U;
    (void)memset(queue, 0, sizeof(queue));
    marker();
    CHECK(status.output.error == S3_ERROR_TRANSPORT);
    CHECK(!status.output.steering_allowed && core.awaiting_controller);
    CHECK(s3_event_receive(&record) == E_TMOUT);
}

static void test_sensor_configuration(void)
{
    s3_ir_config_t cfg = {S3_IR_DIGITAL, {26U, 27U, 28U}};
    s3_ir_t sensor = {0};
    s3_sample_t sample;
    reset();
    CHECK(s3_ir_config_valid(&cfg));
    cfg.mode = (s3_ir_mode_t)-1;
    CHECK(!s3_ir_config_valid(&cfg));
    cfg.mode = (s3_ir_mode_t)99;
    CHECK(!s3_ir_config_valid(&cfg));
    cfg.mode = S3_IR_DIGITAL;
    cfg.pin[0] = 29U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 23U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 19U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 27U; CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0] = 26U;
    gpio_config_error = E_IO;
    CHECK(s3_ir_open(&sensor, &cfg) == E_IO);
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_NOSPT);
    gpio_config_error = E_OK;
    CHECK(s3_ir_open(&sensor, &cfg) == E_OK);
    CHECK(s3_ir_open(&sensor, &cfg) == E_OBJ);
    gpio[26] = 2U;
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_IO && sample.valid_mask == 0U);
}

static void test_adc_boundary(void)
{
    s3_ir_config_t cfg = {S3_IR_ADC, {26U, 27U, 28U}};
    s3_ir_t sensor = {0};
    s3_sample_t sample;
    s3_adapter_config_t adapter = {0};
    reset();
    CHECK(s3_ir_open(&sensor, &cfg) == E_NOSPT);
    CHECK(gpio_configs == 0U);
    CHECK(s3_ir_poll(&sensor, 0U, &sample) == E_NOSPT);
    CHECK(sample.valid_mask == 0U);
    CHECK(subsystem3_init() == E_OK);
    s3_config_defaults(&adapter.algorithms);
    adapter.ir = cfg;
    CHECK(s3_configure(&adapter) == E_NOSPT);
    CHECK(configuration.ir.mode == S3_IR_DISABLED);
}

int main(void)
{
    RUN("S3-U15 lifecycle/disabled", test_disabled_and_lifecycle);
    RUN("S3-U16 controller leases", test_controller_leases_and_safety);
    RUN("S3-U32 controller enum bounds", test_controller_enum_boundaries);
    RUN("S3-U17 safety publication race", test_override_publication_race);
    RUN("S3-U18 event handoff/staleness", test_event_handoff_and_staleness);
    RUN("S3-U19 event queue full", test_transport_overflow);
    RUN("S3-U20 sensor configuration", test_sensor_configuration);
    RUN("S3-U21 ADC boundary", test_adc_boundary);
    (void)printf("Buddy 3 adapter: %u groups, %u checks, %u failures\n",
                 groups, checks, failures);
    return (failures == 0U) ? 0 : 1;
}
