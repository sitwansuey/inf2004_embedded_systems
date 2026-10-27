/* IR line following and barcode reading. The two line sensors straddle the
 * line, so both read white while the car is centred. Steering matches the
 * line-trace bench on branch buddy4/integration-cr1-20261027: steer toward a
 * sensor that sees black, and drive straight when both are white or both are
 * black (a cross). The GP28 sensor feeds Buddy 3's A/Z barcode decoder; reads
 * are logged only, because the team has not mapped A/Z to navigation yet. */
#include "vehicle_line.h"
#include "app_state.h"
#include "subsystem3/subsystem3.h"
/* Measured endpoints must be visible before the calibration helper. */
#include "subsystem3/buddy3_calibration.h"
#include "subsystem3/buddy3_robot_ir.h"

/* ADC pins in Buddy 3 sensor order: RIGHT line, LEFT line, barcode. */
static const uint8_t ir_pins[S3_SENSOR_COUNT] = {26U, 27U, 28U};
/* s3_line_update() black_mask bits follow the same sensor order. */
#define LINE_RIGHT 1U
#define LINE_LEFT 2U
#define LINE_BOTH (LINE_LEFT | LINE_RIGHT)

static s3_config_t ir_config;
static s3_line_t line;
static s3_barcode_t barcode;
static BOOL barcode_black;
static BOOL fed;            /* fed_ms holds the newest processed scan */
static uint32_t fed_ms;
static BOOL have_line;      /* black_mask holds a usable reading */
static uint8_t black_mask;
static UW good_ms;          /* last time a fresh, valid scan was seen */
static uint16_t raw_left, raw_right, raw_barcode;
static BOOL started;
static UW start_ms;
static UW log_ms;
static uint8_t logged = 0xFFU;
#if VEHICLE_LINE_MAX_MS
static BOOL limit_logged;
#endif

/* Names for s3_barcode_status_t, in enum order. */
static const char *const barcode_results[] = {
    "none", "ok", "incomplete", "noise", "overflow", "timeout",
    "bad pattern", "unknown symbol"
};

EXPORT ER vehicle_line_init(void)
{
    s3_adapter_config_t config = {0};
    s3_config_defaults(&config.algorithms);
    robot_ir_apply_layout(&config.algorithms);
    if (!robot_ir_apply_adc_calibration(&config.algorithms))
        return E_PAR;
    config.ir.mode = S3_IR_ADC;
    for (UINT i = 0; i < S3_SENSOR_COUNT; ++i)
        config.ir.pin[i] = ir_pins[i];
    ir_config = config.algorithms;
    s3_line_reset(&line);
    s3_barcode_init(&barcode);
    return s3_configure(&config);
}

/* Feed one new scan to the decoder and log each read or rejected capture.
 * The decoder holds after every result or abort until it is rearmed; it then
 * waits for blank surface before the next capture. */
static void barcode_update(const s3_sample_t *sample)
{
    UINT sensor = ir_config.barcode_sensor;
    uint16_t darkness;
    s3_symbol_t symbol;
    if (sensor >= S3_SENSOR_COUNT || (sample->valid_mask & (1U << sensor)) == 0U ||
        !s3_normalize(sample->raw[sensor], ir_config.line.white[sensor],
                      ir_config.line.black[sensor], ir_config.line.raw_max,
                      &darkness)) {
        s3_barcode_abort(&barcode);
        s3_barcode_rearm(&barcode);
        return;
    }
    raw_barcode = sample->raw[sensor];
    if (darkness >= ir_config.line.threshold_high) barcode_black = TRUE;
    else if (darkness <= ir_config.line.threshold_low) barcode_black = FALSE;
    s3_barcode_status_t result = s3_barcode_feed(&barcode, &ir_config.barcode,
                                                barcode_black, sample->time_ms,
                                                &symbol);
    if (result == S3_BARCODE_NONE) return;
    if (result == S3_BARCODE_OK)
        tm_printf((UB *)"[BARCODE] read %c%s\n", symbol.symbol,
                  symbol.reverse ? " (scanned in reverse)" : "");
    else if ((UINT)result < sizeof(barcode_results) / sizeof(barcode_results[0]))
        tm_printf((UB *)"[BARCODE] capture rejected: %s\n", barcode_results[result]);
    s3_barcode_rearm(&barcode);
}

/* Track the newest scan; FALSE once no usable scan arrived for the hold time. */
static BOOL line_update(UW now_ms, s3_status_t *ir)
{
    if (s3_status_get(ir) >= E_OK && ir->configured &&
        ir->acquisition_error == E_OK &&
        (ir->sample.valid_mask & LINE_BOTH) == LINE_BOTH &&
        now_ms - ir->sample_time_ms < S3_STATUS_MAX_AGE_MS) {
        if (!fed || ir->sample_time_ms != fed_ms) {
            s3_line_result_t result;
            fed = TRUE;
            fed_ms = ir->sample_time_ms;
            barcode_update(&ir->sample);
            s3_line_update(&line, &ir_config.line, ir->sample.raw,
                           ir->sample.valid_mask, ir->sample_time_ms, &result);
            if (result.state != S3_LINE_INVALID) {
                have_line = TRUE;
                black_mask = result.black_mask & LINE_BOTH;
                good_ms = now_ms;
                raw_left = ir->sample.raw[1];
                raw_right = ir->sample.raw[0];
            }
        } else if (have_line) {
            good_ms = now_ms; /* same scan, still fresh */
        }
    }
    if (have_line && now_ms - good_ms >= VEHICLE_LINE_HOLD_MS) {
        have_line = FALSE;
        s3_line_reset(&line);
    }
    return have_line;
}

static void line_log(UW now_ms, const s3_status_t *ir)
{
    uint8_t state = have_line ? black_mask : 0xFEU;
    if (state == logged && now_ms - log_ms < 1000U) return;
    logged = state;
    log_ms = now_ms;
    if (have_line)
        tm_printf((UB *)"[LINE] left=%s right=%s raw left=%u right=%u barcode=%u\n",
                  (black_mask & LINE_LEFT) ? "black" : "white",
                  (black_mask & LINE_RIGHT) ? "black" : "white",
                  raw_left, raw_right, raw_barcode);
    else
        tm_printf((UB *)"[LINE] no usable IR scan (configured=%u error=%d); stopped\n",
                  (UINT)ir->configured, (INT)ir->acquisition_error);
}

EXPORT void vehicle_line_step(s2_move_command_t *command, UINT *power)
{
    s3_status_t ir = {0};
    SYSTIM now;
    *command = S2_MOVE_STOP;
    *power = 0;
    /* Buddy 3 stamps scans with tk_get_otm(), so use the same clock. */
    if (tk_get_otm(&now) < E_OK) return;
    if (!started) {
        started = TRUE;
        start_ms = now.lo;
    }
    BOOL usable = line_update(now.lo, &ir);
    line_log(now.lo, &ir);
    UW elapsed = now.lo - start_ms;
    if (!usable || elapsed < VEHICLE_START_DELAY_MS) return;
#if VEHICLE_LINE_MAX_MS
    if (elapsed - VEHICLE_START_DELAY_MS >= VEHICLE_LINE_MAX_MS) {
        if (!limit_logged)
            tm_printf((UB *)"[LINE] %u ms run limit reached; stopped. Reset to run again.\n",
                      VEHICLE_LINE_MAX_MS);
        limit_logged = TRUE;
        return;
    }
#endif
    *power = VEHICLE_FORWARD_POWER;
    if (black_mask == LINE_LEFT)
        *command = S2_MOVE_PIVOT_LEFT;   /* drifted right of the line */
    else if (black_mask == LINE_RIGHT)
        *command = S2_MOVE_PIVOT_RIGHT;  /* drifted left of the line */
    else
        *command = S2_MOVE_FORWARD;
}
