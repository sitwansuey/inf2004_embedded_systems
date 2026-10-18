#include "buddy3_core.h"

#include <limits.h>
#include <stdio.h>
#include <string.h>

static unsigned int checks;
static unsigned int failures;
static unsigned int groups;
#define CHECK(condition) do { ++checks; if (!(condition)) { \
    ++failures; (void)printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #condition); \
} } while (0)
#define RUN(id, function) do { unsigned int before = failures; ++groups; \
    function(); (void)printf("%s %s\n", id, failures == before ? "PASS" : "FAIL"); \
} while (0)

/* Independently transcribed from PDF forms 7/15: black/white runs. */
static const uint32_t sample_a[29] = {
    1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1
};
static const uint32_t sample_z[29] = {
    1,3,1,1,3,1,3,1,1,1,1,3,3,1,3,1,1,1,1,1,1,3,1,1,3,1,3,1,1
};

static s3_config_t config(void)
{
    s3_config_t result;
    unsigned int i;
    s3_config_defaults(&result);
    result.line.raw_max = 1000U;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        result.line.black[i] = 1000U;
    }
    return result;
}

static void scale(const uint32_t *p_source, uint32_t *p_destination, uint32_t unit)
{
    size_t i;
    for (i = 0U; i < 29U; ++i)
    {
        p_destination[i] = p_source[i] * unit;
    }
}

static void test_normalization(void)
{
    uint16_t value = 999U;
    CHECK(s3_normalize(100U, 100U, 900U, 1023U, &value) && value == 0U);
    CHECK(s3_normalize(900U, 100U, 900U, 1023U, &value) && value == 1000U);
    CHECK(s3_normalize(500U, 100U, 900U, 1023U, &value) && value == 500U);
    CHECK(s3_normalize(0U, 100U, 900U, 1023U, &value) && value == 0U);
    CHECK(s3_normalize(1023U, 100U, 900U, 1023U, &value) && value == 1000U);
    CHECK(s3_normalize(100U, 900U, 100U, 1023U, &value) && value == 1000U);
    CHECK(s3_normalize(900U, 900U, 100U, 1023U, &value) && value == 0U);
    CHECK(!s3_normalize(500U, 500U, 500U, 1023U, &value) && value == 0U);
    CHECK(!s3_normalize(1024U, 100U, 900U, 1023U, &value));
    CHECK(!s3_normalize(0U, 0U, 1U, 0U, &value));
    CHECK(!s3_normalize(0U, 0U, 1U, 1U, NULL));
    CHECK(s3_normalize(0U, 1U, 0U, 1U, &value) && value == 1000U);
    CHECK(s3_normalize(UINT16_MAX, 0U, UINT16_MAX, UINT16_MAX, &value));
    CHECK(value == 1000U);
}

static void test_calibration(void)
{
    s3_calibration_t calibration;
    s3_config_t cfg = config();
    s3_line_config_t before = cfg.line;
    uint16_t white[] = {100U, 900U, 100U};
    uint16_t black[] = {900U, 100U, 900U};
    unsigned int i;

    s3_calibration_init(&calibration, 1000U);
    CHECK(!s3_calibration_finish(&calibration, 100U, &cfg.line));
    CHECK(memcmp(&cfg.line, &before, sizeof(before)) == 0);
    for (i = 0U; i < 4U; ++i)
    {
        CHECK(s3_calibration_add(&calibration, false, white));
        CHECK(s3_calibration_add(&calibration, true, black));
    }
    CHECK(s3_calibration_finish(&calibration, 100U, &cfg.line));
    CHECK(cfg.line.white[1] == 900U && cfg.line.black[1] == 100U);
    CHECK(!s3_calibration_finish(&calibration, 900U, &cfg.line));
    CHECK(!s3_calibration_finish(&calibration, 0U, &cfg.line));
    white[0] = 1001U;
    CHECK(!s3_calibration_add(&calibration, false, white));
    CHECK(calibration.count[0] == 4U);
    white[0] = 100U;
    for (i = 4U; i < 64U; ++i)
    {
        CHECK(s3_calibration_add(&calibration, false, white));
    }
    CHECK(!s3_calibration_add(&calibration, false, white));
    s3_calibration_init(&calibration, 1000U);
    for (i = 0U; i < 4U; ++i)
    {
        white[0] = (i == 0U) ? 500U : 100U;
        CHECK(s3_calibration_add(&calibration, false, white));
        CHECK(s3_calibration_add(&calibration, true, black));
    }
    CHECK(!s3_calibration_finish(&calibration, 100U, &cfg.line));
    s3_calibration_init(&calibration, 0U);
    CHECK(!s3_calibration_add(&calibration, true, black));
}

static void test_line_positions(void)
{
    s3_config_t cfg = config();
    s3_line_t line = {0};
    s3_line_result_t result;
    uint16_t raw[3] = {0U, 0U, 0U};
    unsigned int i;
    for (i = 0U; i < 3U; ++i)
    {
        (void)memset(raw, 0, sizeof(raw));
        raw[i] = 1000U;
        s3_line_reset(&line);
        s3_line_update(&line, &cfg.line, raw, 7U, 0U, &result);
        CHECK(result.state == S3_LINE_TRACKING);
        CHECK(result.error == ((int)i - 1) * 1000);
    }
    raw[0] = 1000U;
    raw[1] = 500U;
    raw[2] = 0U;
    s3_line_reset(&line);
    s3_line_update(&line, &cfg.line, raw, 7U, 0U, &result);
    CHECK(result.error == -1000); /* Centre below initial high threshold. */
    raw[1] = 1000U;
    s3_line_update(&line, &cfg.line, raw, 7U, 10U, &result);
    CHECK(result.error == -500 && result.state == S3_LINE_TRACKING);
    raw[2] = 1000U;
    s3_line_update(&line, &cfg.line, raw, 7U, 20U, &result);
    CHECK(result.state == S3_LINE_JUNCTION);
    raw[1] = 0U;
    s3_line_update(&line, &cfg.line, raw, 7U, 30U, &result);
    CHECK(result.state == S3_LINE_JUNCTION);
    raw[0] = 1001U;
    s3_line_update(&line, &cfg.line, raw, 7U, 40U, &result);
    CHECK(result.state == S3_LINE_INVALID);
    raw[0] = 1000U;
    s3_line_update(&line, &cfg.line, raw, 6U, 50U, &result);
    CHECK(result.state == S3_LINE_INVALID);
    cfg.line.line_mask = 1U;
    s3_line_update(&line, &cfg.line, raw, 1U, 60U, &result);
    CHECK(result.state == S3_LINE_TRACKING); /* Cannot infer a junction. */
}

static void test_hysteresis_loss_rollover(void)
{
    s3_config_t cfg = config();
    s3_line_t line = {0};
    s3_line_result_t result;
    uint16_t raw[] = {0U, 600U, 0U};
    s3_line_update(&line, &cfg.line, raw, 7U, 0U, &result);
    CHECK(result.state == S3_LINE_TRACKING);
    raw[1] = 500U;
    s3_line_update(&line, &cfg.line, raw, 7U, 10U, &result);
    CHECK(result.state == S3_LINE_TRACKING);
    raw[1] = 400U;
    s3_line_update(&line, &cfg.line, raw, 7U, 20U, &result);
    CHECK(result.state == S3_LINE_SEARCHING);
    raw[1] = 500U;
    s3_line_update(&line, &cfg.line, raw, 7U, 30U, &result);
    CHECK(result.state == S3_LINE_SEARCHING);
    s3_line_update(&line, &cfg.line, raw, 7U, 119U, &result);
    CHECK(result.state == S3_LINE_SEARCHING);
    s3_line_update(&line, &cfg.line, raw, 7U, 120U, &result);
    CHECK(result.state == S3_LINE_LOST && !result.recovery_expired);
    s3_line_update(&line, &cfg.line, raw, 7U, 620U, &result);
    CHECK(result.recovery_expired);
    raw[1] = 1000U;
    s3_line_update(&line, &cfg.line, raw, 7U, 630U, &result);
    CHECK(result.state == S3_LINE_TRACKING && !result.recovery_expired);
    s3_line_reset(&line);
    raw[1] = 0U;
    s3_line_update(&line, &cfg.line, raw, 7U, UINT32_MAX - 49U, &result);
    s3_line_update(&line, &cfg.line, raw, 7U, 50U, &result);
    CHECK(result.state == S3_LINE_LOST);
}

static void test_config_boundaries(void)
{
    s3_config_t cfg = config();
    s3_core_t core;
    CHECK(s3_config_valid(&cfg));
    cfg.line.line_mask = 0U;
    CHECK(!s3_core_init(&core, &cfg));
    cfg = config(); cfg.barcode_sensor = 2U;
    CHECK(!s3_config_valid(&cfg)); /* One sensor cannot serve both tracks. */
    cfg.line.line_mask = 3U;
    CHECK(s3_config_valid(&cfg));
    cfg.map_a = (s3_navigation_t)99;
    CHECK(!s3_config_valid(&cfg));
    cfg = config(); cfg.line.recovery_ms = UINT32_MAX;
    CHECK(!s3_config_valid(&cfg));
    cfg = config(); cfg.line.threshold_low = cfg.line.threshold_high;
    CHECK(!s3_config_valid(&cfg));
    cfg = config(); cfg.line.position[0] = 0;
    CHECK(!s3_config_valid(&cfg));
    cfg = config(); cfg.barcode.min_run_ms = 0U;
    CHECK(!s3_config_valid(&cfg));
    cfg = config(); cfg.max_sample_gap_ms = 0U;
    CHECK(!s3_config_valid(&cfg));
    CHECK(!s3_core_init(NULL, &cfg));
}

static void test_barcode_scale_direction_speed(void)
{
    uint32_t runs[29];
    uint32_t reverse[29];
    s3_symbol_t symbol;
    unsigned int scale_index;
    size_t i;
    const uint32_t units[] = {40U, 80U, 1000U, 100000000U};
    for (scale_index = 0U; scale_index < 4U; ++scale_index)
    {
        scale(sample_a, runs, units[scale_index]);
        CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_OK);
        CHECK(symbol.symbol == 'A' && !symbol.reverse);
        for (i = 0U; i < 29U; ++i) { reverse[i] = runs[28U - i]; }
        CHECK(s3_barcode_decode(reverse, 29U, &symbol) == S3_BARCODE_OK);
        CHECK(symbol.symbol == 'A' && symbol.reverse);
        scale(sample_z, runs, units[scale_index]);
        CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_OK);
        CHECK(symbol.symbol == 'Z');
    }
    scale(sample_a, runs, 100U);
    for (i = 0U; i < 9U; ++i) { runs[i + 10U] = sample_a[i + 10U] * 120U; }
    for (i = 0U; i < 9U; ++i) { runs[i + 20U] = sample_a[i + 20U] * 140U; }
    runs[9] = 110U; runs[19] = 130U;
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_OK);
    CHECK(symbol.symbol == 'A');
    /* Deterministic bounded edge jitter, not fabricated hardware noise. */
    for (i = 0U; i < 29U; ++i) { runs[i] += (uint32_t)(i % 3U) * 3U; }
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_OK);
}

static void test_barcode_corruption(void)
{
    uint32_t runs[29];
    s3_symbol_t symbol;
    size_t i;
    CHECK(s3_barcode_decode(NULL, 29U, &symbol) != S3_BARCODE_OK);
    CHECK(symbol.symbol == 0);
    scale(sample_a, runs, 40U);
    CHECK(s3_barcode_decode(runs, 28U, &symbol) == S3_BARCODE_INCOMPLETE);
    runs[0] = 0U;
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_BAD_PATTERN);
    scale(sample_a, runs, 40U); runs[9] = 120U;
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_BAD_PATTERN);
    scale(sample_a, runs, 40U); runs[0] = 120U;
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_BAD_PATTERN);
    scale(sample_a, runs, 40U);
    for (i = 10U; i < 19U; ++i) { runs[i] = 40U; }
    runs[10] = 120U; runs[11] = 120U; runs[12] = 120U;
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_UNKNOWN);
    CHECK(symbol.symbol == 0);
    for (i = 0U; i < 29U; ++i) { runs[i] = UINT32_MAX; }
    CHECK(s3_barcode_decode(runs, 29U, &symbol) == S3_BARCODE_BAD_PATTERN);
    CHECK(s3_barcode_decode(runs, UINT32_MAX, &symbol) == S3_BARCODE_INCOMPLETE);
}

static s3_barcode_status_t capture(s3_barcode_t *p_capture,
                                    const s3_barcode_config_t *p_config,
                                    const uint32_t *p_runs, uint32_t *p_time,
                                    s3_symbol_t *p_symbol)
{
    size_t i;
    s3_barcode_status_t status;
    *p_time += 10U;
    status = s3_barcode_feed(p_capture, p_config, true, *p_time, p_symbol);
    CHECK(status == S3_BARCODE_NONE);
    for (i = 0U; i < 29U; ++i)
    {
        *p_time += p_runs[i];
        status = s3_barcode_feed(p_capture, p_config, (i % 2U) != 0U,
                                 *p_time, p_symbol);
        CHECK(status == S3_BARCODE_NONE);
    }
    *p_time += 1000U;
    return s3_barcode_feed(p_capture, p_config, false, *p_time, p_symbol);
}

static void test_capture_duplicate_rearm(void)
{
    s3_barcode_t barcode;
    s3_config_t cfg = config();
    s3_symbol_t symbol;
    uint32_t runs[29];
    uint32_t now = UINT32_MAX - 1000U;
    scale(sample_a, runs, 40U);
    s3_barcode_init(&barcode);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, false, now, &symbol) ==
          S3_BARCODE_NONE);
    now += 200U;
    (void)s3_barcode_feed(&barcode, &cfg.barcode, false, now, &symbol);
    CHECK(capture(&barcode, &cfg.barcode, runs, &now, &symbol) == S3_BARCODE_OK);
    CHECK(symbol.symbol == 'A' && barcode.state == S3_CAPTURE_HOLD);
    CHECK(capture(&barcode, &cfg.barcode, runs, &now, &symbol) == S3_BARCODE_NONE);
    CHECK(symbol.symbol == 0);
    s3_barcode_rearm(&barcode); now += 10U;
    (void)s3_barcode_feed(&barcode, &cfg.barcode, false, now, &symbol);
    CHECK(barcode.state == S3_CAPTURE_READY);
    CHECK(capture(&barcode, &cfg.barcode, runs, &now, &symbol) == S3_BARCODE_OK);
    CHECK(symbol.symbol == 'A');
    /* Slower / larger Z print: capture, not just standalone decoding. */
    scale(sample_z, runs, 80U);
    s3_barcode_init(&barcode); now += 10U;
    (void)s3_barcode_feed(&barcode, &cfg.barcode, false, now, &symbol);
    now += 200U;
    (void)s3_barcode_feed(&barcode, &cfg.barcode, false, now, &symbol);
    CHECK(capture(&barcode, &cfg.barcode, runs, &now, &symbol) == S3_BARCODE_OK);
    CHECK(symbol.symbol == 'Z');
}

static void arm(s3_barcode_t *p_barcode, const s3_barcode_config_t *p_cfg)
{
    s3_symbol_t symbol;
    s3_barcode_init(p_barcode);
    (void)s3_barcode_feed(p_barcode, p_cfg, false, 0U, &symbol);
    (void)s3_barcode_feed(p_barcode, p_cfg, false, 200U, &symbol);
    (void)s3_barcode_feed(p_barcode, p_cfg, true, 210U, &symbol);
}

static void test_capture_failures(void)
{
    s3_barcode_t barcode;
    s3_config_t cfg = config();
    s3_symbol_t symbol;
    s3_barcode_status_t result = S3_BARCODE_NONE;
    unsigned int i;
    arm(&barcode, &cfg.barcode);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, false, 215U, &symbol) ==
          S3_BARCODE_NOISE);
    arm(&barcode, &cfg.barcode);
    for (i = 0U; i < 30U; ++i)
    {
        result = s3_barcode_feed(&barcode, &cfg.barcode, (i % 2U) != 0U,
                                 250U + i * 40U, &symbol);
    }
    CHECK(result == S3_BARCODE_OVERFLOW && barcode.count == 29U);
    arm(&barcode, &cfg.barcode);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, true, 10210U, &symbol) ==
          S3_BARCODE_TIMEOUT);
    arm(&barcode, &cfg.barcode);
    (void)s3_barcode_feed(&barcode, &cfg.barcode, false, 250U, &symbol);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, false, 1250U, &symbol) ==
          S3_BARCODE_INCOMPLETE);
    arm(&barcode, &cfg.barcode);
    s3_barcode_abort(&barcode);
    CHECK(barcode.state == S3_CAPTURE_HOLD);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, false, 300U, &symbol) ==
          S3_BARCODE_NONE);
    CHECK(s3_barcode_feed(&barcode, &cfg.barcode, false, 300U, &symbol) ==
          S3_BARCODE_NOISE);
}

static uint32_t core_time;
static unsigned int navigation_events;
static unsigned int fault_events;
static unsigned int decoded_symbols;
static s3_output_t latest;

static void core_feed(s3_core_t *p_core, bool black)
{
    s3_sample_t sample = {{1000U, 0U, 0U}, 7U, 0U};
    core_time += 10U;
    sample.time_ms = core_time;
    sample.raw[2] = black ? 1000U : 0U;
    s3_core_step(p_core, &sample, &latest);
    if (latest.event.kind == S3_EVENT_NAVIGATION)
    {
        ++navigation_events;
        CHECK(!latest.steering_allowed && latest.awaiting_controller);
        CHECK(latest.event.command == S3_NAV_LEFT); /* Test-only mapping. */
    }
    if (latest.event.kind == S3_EVENT_FAULT) { ++fault_events; }
    if (latest.barcode_status == S3_BARCODE_OK) { ++decoded_symbols; }
}

static void core_fixture(s3_core_t *p_core, uint32_t runs[29])
{
    size_t i;
    uint32_t t;
    core_feed(p_core, true);
    for (i = 0U; i < 29U; ++i)
    {
        for (t = 10U; t <= runs[i]; t += 10U)
        {
            bool black = (i % 2U) == 0U;
            core_feed(p_core, (t == runs[i]) ? !black : black);
        }
    }
    for (t = 0U; t < 1000U; t += 10U) { core_feed(p_core, false); }
}

static void setup_core(s3_core_t *p_core, bool mapped)
{
    s3_config_t cfg = config();
    unsigned int i;
    cfg.line.line_mask = 3U;
    cfg.barcode_sensor = 2U;
    cfg.map_a = mapped ? S3_NAV_LEFT : S3_NAV_NONE;
    CHECK(s3_core_init(p_core, &cfg));
    CHECK(s3_core_control(p_core, S3_MODE_TRACK, false));
    core_time = 0U; navigation_events = 0U; fault_events = 0U; decoded_symbols = 0U;
    for (i = 0U; i < 22U; ++i) { core_feed(p_core, false); }
    CHECK(latest.steering_allowed);
}

static void test_navigation_handoff_override(void)
{
    s3_core_t core;
    uint32_t runs[29];
    unsigned int i;
    scale(sample_a, runs, 40U);
    setup_core(&core, true);
    core_fixture(&core, runs);
    CHECK(navigation_events == 1U && decoded_symbols == 1U);
    CHECK(core.awaiting_controller && !latest.steering_allowed);
    for (i = 0U; i < 70U; ++i)
    {
        s3_sample_t absent = {{0U, 0U, 0U}, 7U, 0U};
        core_time += 10U; absent.time_ms = core_time;
        s3_core_step(&core, &absent, &latest);
        CHECK(latest.event.kind == S3_EVENT_NONE);
    }
    core_fixture(&core, runs);
    CHECK(navigation_events == 1U);
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    core_fixture(&core, runs);
    CHECK(navigation_events == 1U && latest.steering_allowed);
    CHECK(s3_core_control(&core, S3_MODE_TRACK, true));
    for (i = 0U; i < 22U; ++i) { core_feed(&core, false); }
    core_fixture(&core, runs);
    CHECK(navigation_events == 2U);
    CHECK(s3_core_control(&core, S3_MODE_OVERRIDE, false));
    core_fixture(&core, runs);
    CHECK(navigation_events == 2U && !latest.steering_allowed);
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    core_feed(&core, false);
    CHECK(latest.steering_allowed);
    setup_core(&core, true);
    core_feed(&core, true); /* Pause mid capture: suffix cannot create a turn. */
    CHECK(s3_core_control(&core, S3_MODE_PAUSED, false));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    core_fixture(&core, runs);
    CHECK(navigation_events == 0U);
}

static void test_unmapped_corrupt_and_fault(void)
{
    s3_core_t core;
    uint32_t runs[29];
    s3_sample_t sample = {{1000U, 0U, 0U}, 7U, 0U};
    scale(sample_a, runs, 40U);
    setup_core(&core, false);
    core_fixture(&core, runs);
    CHECK(decoded_symbols == 1U && navigation_events == 0U);
    setup_core(&core, true);
    runs[9] = 120U;
    core_fixture(&core, runs);
    CHECK(navigation_events == 0U && fault_events == 1U);
    CHECK(latest.error == S3_ERROR_BARCODE && !latest.steering_allowed);
    sample.time_ms = core_time + 10U;
    sample.raw[2] = 1001U;
    s3_core_step(&core, &sample, &latest);
    CHECK(!latest.steering_allowed && latest.error == S3_ERROR_SENSOR);
    sample.time_ms += 10U; sample.raw[2] = 0U; sample.valid_mask = 3U;
    s3_core_step(&core, &sample, &latest);
    CHECK(latest.error == S3_ERROR_SENSOR && latest.event.kind == S3_EVENT_NONE);
}

static void test_core_loss_and_time(void)
{
    s3_core_t core;
    s3_config_t cfg = config();
    s3_sample_t sample = {{0U, 0U, 0U}, 7U, 0U};
    s3_output_t output;
    unsigned int recoveries = 0U;
    unsigned int stops = 0U;
    uint32_t t;
    CHECK(s3_core_init(&core, &cfg));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    for (t = 0U; t <= 1000U; t += 10U)
    {
        sample.time_ms = t;
        s3_core_step(&core, &sample, &output);
        if (output.event.kind == S3_EVENT_RECOVERY_REQUEST) { ++recoveries; }
        if (output.event.kind == S3_EVENT_STOP_REQUEST) { ++stops; }
        CHECK(!output.steering_allowed);
    }
    CHECK(recoveries == 1U && stops == 1U);
    sample.raw[1] = 1000U; sample.time_ms += 10U;
    s3_core_step(&core, &sample, &output);
    CHECK(!output.steering_allowed && output.line.state == S3_LINE_TRACKING);
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    sample.time_ms += 10U;
    s3_core_step(&core, &sample, &output);
    CHECK(output.steering_allowed);
    CHECK(s3_core_init(&core, &cfg));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    sample.time_ms = UINT32_MAX - 5U;
    s3_core_step(&core, &sample, &output);
    sample.time_ms = 4U;
    s3_core_step(&core, &sample, &output);
    CHECK(output.steering_allowed);
    s3_core_step(&core, &sample, &output);
    CHECK(output.error == S3_ERROR_STALE && !output.steering_allowed);
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    sample.time_ms = 100U;
    s3_core_step(&core, &sample, &output);
    CHECK(output.error == S3_ERROR_STALE);
    sample.time_ms = 90U;
    s3_core_step(&core, &sample, &output);
    CHECK(output.error == S3_ERROR_STALE);
    CHECK(!s3_core_control(&core, (s3_mode_t)99, false));
    s3_core_step(NULL, &sample, &output);
    CHECK(output.error == S3_ERROR_CONFIG);
}

static void test_enum_boundaries(void)
{
    const int invalid_navigation[] = {-1, S3_NAV_UTURN + 1, 99};
    const int invalid_modes[] = {-1, S3_MODE_OVERRIDE + 1, 99};
    s3_config_t cfg = config();
    s3_core_t core;
    unsigned char before[sizeof(core)];
    size_t i;

    for (i = 0U; i < sizeof(invalid_navigation) / sizeof(invalid_navigation[0]); ++i)
    {
        cfg = config();
        cfg.map_a = (s3_navigation_t)invalid_navigation[i];
        CHECK(!s3_config_valid(&cfg));
        CHECK(!s3_core_init(&core, &cfg));
        cfg = config();
        cfg.map_z = (s3_navigation_t)invalid_navigation[i];
        CHECK(!s3_config_valid(&cfg));
        CHECK(!s3_core_init(&core, &cfg));
    }
    cfg = config();
    cfg.map_a = S3_NAV_NONE;
    cfg.map_z = S3_NAV_UTURN;
    CHECK(s3_core_init(&core, &cfg));
    CHECK(s3_core_control(&core, S3_MODE_TRACK, false));
    s3_core_fault(&core, S3_ERROR_SENSOR);
    (void)memcpy(before, &core, sizeof(core));
    for (i = 0U; i < sizeof(invalid_modes) / sizeof(invalid_modes[0]); ++i)
    {
        CHECK(!s3_core_control(&core, (s3_mode_t)invalid_modes[i], true));
        CHECK(memcmp(before, &core, sizeof(core)) == 0);
    }
    CHECK(s3_core_control(&core, S3_MODE_PAUSED, false));
    CHECK(s3_core_control(&core, S3_MODE_OVERRIDE, false));
}

int main(int argc, char **argv)
{
    RUN("S3-U02 normalization", test_normalization);
    RUN("S3-U04 calibration", test_calibration);
    RUN("S3-U05 line positions", test_line_positions);
    RUN("S3-U06 hysteresis/loss/rollover", test_hysteresis_loss_rollover);
    RUN("S3-U07 configuration bounds", test_config_boundaries);
    RUN("S3-U08 barcode scales/speed/direction", test_barcode_scale_direction_speed);
    RUN("S3-U09 barcode corruption", test_barcode_corruption);
    RUN("S3-U10 capture/rearm/duplicates", test_capture_duplicate_rearm);
    RUN("S3-U11 capture failures/capacity", test_capture_failures);
    RUN("S3-U12 navigation/override", test_navigation_handoff_override);
    RUN("S3-U13 unmapped/corrupted/fault", test_unmapped_corrupt_and_fault);
    RUN("S3-U14 recovery and time", test_core_loss_and_time);
    RUN("S3-U31 enum bounds", test_enum_boundaries);
    if ((argc == 2) && (strcmp(argv[1], "--force-failure") == 0))
    {
        CHECK(2 + 2 == 5);
    }
    (void)printf("Buddy 3 algorithms: %u groups, %u checks, %u failures\n",
                 groups, checks, failures);
    return (failures == 0U) ? 0 : 1;
}
