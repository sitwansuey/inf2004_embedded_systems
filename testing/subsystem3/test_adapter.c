#include "adapter_fixture.h"

/* Robot-like barcode profile for the L2 checks: re-arm after a failed
 * capture, end-of-marker quiet time, fault-cut reads count as failed. */
static bool l2_profile;

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
    if (l2_profile)
    {
        cfg.algorithms.barcode_retry = true;
        cfg.algorithms.barcode.end_quiet_ms = 300U;
        cfg.algorithms.barcode_fault_fails = true;
    }
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
    for (failure = 1U; failure <= 2U; ++failure) /* state mutex, event buffer */
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
    /* L1: the turn was decoded from a good sample before the sensor fault,
     * so it is still delivered (the fault that follows stops the vehicle). */
    CHECK(s3_event_receive(&record) == E_OK);
    CHECK(record.event.kind == S3_EVENT_NAVIGATION && record.event.symbol == 'A');
    CHECK(s3_event_receive(&record) == E_OK); /* Fault may be received. */
    CHECK(record.event.kind == S3_EVENT_FAULT);
}

/* Marker ticks before the decoding sample, measured by a plain run. */
static unsigned int decode_lock_base;
static void marker_find_decode(void)
{
    const unsigned int widths[] = {
        1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1
    };
    unsigned int i;
    unsigned int t;
    decode_lock_base = 0U;
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
    for (t = 0U; t < 500U; t += 10U)
    {
        unsigned int before = sample_lock_calls;
        (void)tick(false);
        if ((queue_count == 1U) && (decode_lock_base == 0U)) decode_lock_base = before;
    }
}

/* L1/L6: a decoded turn reaches the controller although the S3 status shows
 * a later silent latch, is exactly max_sample_gap_ms old, or the decode
 * itself was published late; S3 shows that it awaits the controller. */
static void test_turn_survives_latch(void)
{
    s3_event_record_t record;
    s3_status_t snapshot;

    /* (a) The next sample is late: S3 latches STALE silently. */
    setup(); marker();
    CHECK(queue_count == 1U);
    delay_lock_call = sample_lock_calls + 2U; state_lock_delay_ms = 30U;
    (void)tick(false);
    CHECK(status.output.error != S3_ERROR_NONE && status.acquisition_error == E_TMOUT);
    (void)tick(false); /* the next, fresh sample still shows the latch */
    CHECK(status.output.error != S3_ERROR_NONE && status.acquisition_error == E_OK);
    CHECK(queue_count == 1U && core.awaiting_controller);
    CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_NAVIGATION);
    CHECK(record.event.symbol == 'A' && record.event.command == S3_NAV_LEFT);
    /* The controller acknowledges (holding the marker): the latch clears. */
    CHECK(s3_controller_control(S3_MODE_TRACK, false, 1000U) == E_OK);
    (void)tick(false);
    CHECK(status.output.error == S3_ERROR_NONE && status.output.steering_allowed);

    /* (b) Drained when the status is exactly 30 ms (the gap limit) old. S3
     * itself has no fault: it still waits for the controller. */
    setup(); marker();
    fake_now += 30U;
    CHECK(s3_status_get(&snapshot) == E_OK);
    CHECK(snapshot.output.awaiting_controller && snapshot.output.error == S3_ERROR_NONE);
    CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_NAVIGATION);

    /* (c) The decoding sample itself is published 30 ms late. */
    setup(); marker_find_decode();
    CHECK(decode_lock_base != 0U);
    setup();
    delay_lock_call = decode_lock_base + 3U; state_lock_delay_ms = 30U;
    marker();
    CHECK(core.barcode_ok_count == 1U && core.awaiting_controller);
    CHECK(status.output.error == S3_ERROR_STALE);
    CHECK(queue_count == 1U);
    CHECK(s3_event_receive(&record) == E_OK && record.event.kind == S3_EVENT_NAVIGATION);
    CHECK(record.event.symbol == 'A');
    /* A safety directive still wins: OVERRIDE invalidates the turn. */
    setup(); marker();
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_OK);
    CHECK(s3_event_receive(&record) == E_TMOUT);
}

/* L6: other requests use the sampler's own gap limit (a gap above
 * max_sample_gap_ms faults), so a 30 ms old status is still current. */
static void test_request_gap_alignment(void)
{
    s3_event_record_t record;
    unsigned int trial;
    unsigned int i;
    for (trial = 0U; trial < 2U; ++trial)
    {
        setup(); (void)tick(false);
        gpio[2] = 0U; /* line gone: LOST, then a recovery request */
        for (i = 0U; (i < 30U) && (queue_count == 0U); ++i) { (void)tick(false); }
        CHECK(queue_count == 1U && status.output.error == S3_ERROR_NONE);
        fake_now += (trial == 0U) ? 30U : 31U;
        if (trial == 0U)
        {
            CHECK(s3_event_receive(&record) == E_OK);
            CHECK(record.event.kind == S3_EVENT_RECOVERY_REQUEST);
        }
        else
        {
            CHECK(s3_event_receive(&record) == E_TMOUT);
        }
    }
}

/* L7: mode and lease requests wait briefly for a busy status lock instead of
 * failing at once; past the bound, E_TMOUT changes nothing. */
static void test_controller_busy_lock(void)
{
    uint32_t generation;
    uint32_t issued;
    setup(); (void)tick(false);
    fake_now += 50U;
    busy_for_poll = true; /* a telemetry copy holds the lock briefly */
    CHECK(s3_controller_renew(1000U) == E_OK && last_lock_timeout == S3_STATE_LOCK_WAIT_MS);
    CHECK(control.issued_ms == fake_now);
    generation = control.generation;
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 100U) == E_OK);
    CHECK(control.generation == generation + 1U && last_lock_timeout == S3_STATE_LOCK_WAIT_MS);
    busy_for_poll = false;
    busy_mutex = true; /* still busy after the bounded wait */
    fake_now += 50U;
    generation = control.generation; issued = control.issued_ms;
    CHECK(s3_controller_renew(100U) == E_TMOUT);
    CHECK(s3_controller_control(S3_MODE_OVERRIDE, false, 0U) == E_TMOUT);
    CHECK(control.generation == generation && control.issued_ms == issued);
    CHECK(control.mode == S3_MODE_TRACK && control.lease_ms == 100U);
    busy_mutex = false;
    CHECK(s3_controller_renew(100U) == E_OK); /* retried next period */
    fake_now += 100U;
    CHECK(s3_controller_renew(100U) == E_OBJ); /* only expiry is final */
}

/* L2: a capture that ends by itself with 3-8 elements is counted as partial
 * (status + one USB line), never as a failed read; a read cut short by a
 * sampler hiccup is a failed read, so the controller can re-read it. */
static void test_barcode_partial_and_abort(void)
{
    s3_status_t snapshot;
    unsigned int i;
    unsigned int t;
    l2_profile = true; setup(); l2_profile = false;
    for (i = 0U; i < 25U; ++i) { (void)tick(false); }
    for (i = 0U; i < 5U; ++i)
    {
        for (t = 0U; t < 40U; t += 10U) { (void)tick((i % 2U) == 0U); }
    }
    for (i = 0U; i < 40U; ++i) { (void)tick(false); }
    CHECK(s3_status_get(&snapshot) == E_OK);
    CHECK(snapshot.output.barcode_partial_count == 1U && snapshot.output.barcode_fail_count == 0U);
    CHECK(snapshot.output.barcode_abort_count == 0U && snapshot.output.barcode_ok_count == 0U);
    CHECK(snapshot.output.barcode_result == S3_BARCODE_RESULT_NONE);
    CHECK(snapshot.output.error == S3_ERROR_NONE && queue_count == 0U);
    CHECK(log_lines == 1U && strstr(log_last, "partial") != NULL);
    CHECK(strstr(log_last, " 5 elements") != NULL);
    /* Next marker: 4 elements in, the 5th sample is late (silent STALE). */
    for (i = 0U; i < 25U; ++i) { (void)tick(false); }
    for (i = 0U; i < 4U; ++i)
    {
        for (t = 0U; t < 40U; t += 10U) { (void)tick((i % 2U) == 0U); }
    }
    (void)tick(true);
    CHECK(status.output.barcode_result == S3_BARCODE_RESULT_READING);
    CHECK(status.output.barcode_elements == 4U);
    delay_lock_call = sample_lock_calls + 2U; state_lock_delay_ms = 30U;
    (void)tick(true);
    CHECK(s3_status_get(&snapshot) == E_OK);
    CHECK(snapshot.output.barcode_fail_count == 1U && snapshot.output.barcode_abort_count == 1U);
    CHECK(snapshot.output.barcode_result == S3_BARCODE_RESULT_FAILED);
    CHECK(snapshot.output.barcode_elements == 4U && snapshot.output.barcode_bars == 3U);
    CHECK(snapshot.output.barcode_partial_count == 1U && snapshot.output.error != S3_ERROR_NONE);
    CHECK(queue_count == 0U); /* silent latch: no FAULT event */
    CHECK(log_lines == 2U && strstr(log_last, "cut short") != NULL);
    CHECK(strstr(log_last, " 4 elements") != NULL);
    /* As the vehicle does after the next fresh sample shows the latch, it
     * re-arms (TRACK, marker clear): the re-read decodes. */
    (void)tick(false);
    CHECK(status.output.barcode_fail_count == 1U && queue_count == 0U);
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 1000U) == E_OK);
    marker();
    CHECK(status.output.barcode_ok_count == 1U && status.output.barcode_fail_count == 1U);
    CHECK(queue_count == 1U && queue[0].event.kind == S3_EVENT_NAVIGATION);
    CHECK(queue[0].event.symbol == 'A' && queue[0].generation == control.generation);
    CHECK(log_lines == 2U); /* a full read logs nothing */
    /* Default profile: the same hiccup is not counted (no re-read). */
    setup();
    for (i = 0U; i < 25U; ++i) { (void)tick(false); }
    for (i = 0U; i < 4U; ++i)
    {
        for (t = 0U; t < 40U; t += 10U) { (void)tick((i % 2U) == 0U); }
    }
    delay_lock_call = sample_lock_calls + 2U; state_lock_delay_ms = 30U;
    (void)tick(true);
    CHECK(status.output.barcode_fail_count == 0U && status.output.barcode_abort_count == 0U);
    CHECK(log_lines == 0U);
}

static void test_controller_set_map(void)
{
    s3_event_record_t record;
    unsigned int ok;
    setup();
    CHECK(s3_controller_set_map(27U, 1U) == E_PAR && s3_controller_set_map(1U, 5U) == E_PAR);
    CHECK(s3_controller_set_map(0U, 1U) == E_PAR);
    caller = 9; CHECK(s3_controller_set_map(1U, 2U) == E_OACV); caller = 7;
    busy_mutex = true; CHECK(s3_controller_set_map(1U, 2U) == E_TMOUT); busy_mutex = false;
    CHECK(control.map_generation == 0U);
    CHECK(s3_controller_set_map(1U, S3_NAV_RIGHT) == E_OK);
    CHECK(s3_controller_set_map(2U, S3_NAV_UTURN) == E_OK); /* both kept until applied */
    CHECK(core.map_override[0] == S3_MAP_DEFAULT); /* the S3 task applies it */
    marker();
    CHECK(core.map_override[0] == S3_NAV_RIGHT && core.map_override[1] == S3_NAV_UTURN);
    CHECK(s3_event_receive(&record) == E_OK && record.event.symbol == 'A');
    CHECK(record.event.command == S3_NAV_RIGHT);
    CHECK(s3_controller_set_map(1U, S3_NAV_NONE) == E_OK); /* read, no turn */
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 1000U) == E_OK);
    ok = status.output.barcode_ok_count;
    marker();
    CHECK(queue_count == 0U && status.output.barcode_ok_count == ok + 1U);
    CHECK(status.output.last_symbol == 'A');
    CHECK(s3_controller_set_map(0U, 0U) == E_OK); /* codebook again */
    CHECK(s3_controller_control(S3_MODE_TRACK, true, 1000U) == E_OK);
    marker();
    CHECK(core.map_override[0] == S3_MAP_DEFAULT && core.map_override[1] == S3_MAP_DEFAULT);
    CHECK(s3_event_receive(&record) == E_OK && record.event.command == S3_NAV_LEFT);
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
    RUN("S3-U51 controller barcode map", test_controller_set_map);
    RUN("S3-U52 decoded turn survives a later latch (L1/L6)", test_turn_survives_latch);
    RUN("S3-U53 request gap limit aligned (L6)", test_request_gap_alignment);
    RUN("S3-U54 controller tolerates a busy lock (L7)", test_controller_busy_lock);
    RUN("S3-U55 partial and fault-cut barcode reads (L2)", test_barcode_partial_and_abort);
    RUN("S3-U20 sensor configuration", test_sensor_configuration);
    RUN("S3-U21 ADC boundary", test_adc_boundary);
    (void)printf("Buddy 3 adapter: %u groups, %u checks, %u failures\n",
                 groups, checks, failures);
    return (failures == 0U) ? 0 : 1;
}
