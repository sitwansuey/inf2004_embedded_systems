#include "buddy3_core.h"

#include <stddef.h>
#include <string.h>

void s3_config_defaults(s3_config_t *p_config)
{
    uint8_t i;
    if (p_config == NULL)
    {
        return;
    }
    (void)memset(p_config, 0, sizeof(*p_config));
    p_config->line.raw_max = 4095U;
    p_config->line.line_mask = S3_SENSOR_MASK;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        p_config->line.black[i] = 4095U;
        p_config->line.position[i] = (int16_t)(((int32_t)i - 1) * 1000);
    }
    p_config->line.threshold_low = 400U;
    p_config->line.threshold_high = 600U;
    p_config->line.lost_ms = 100U;
    p_config->line.recovery_ms = 500U;
    p_config->barcode_sensor = S3_NO_BARCODE;
    p_config->barcode.quiet_ms = 200U;
    p_config->barcode.min_run_ms = 40U;
    p_config->barcode.timeout_ms = 10000U;
    p_config->max_sample_gap_ms = 30U;
}

static bool navigation_valid(s3_navigation_t command)
{
    return (unsigned int)command <= (unsigned int)S3_NAV_UTURN;
}

bool s3_config_valid(const s3_config_t *p_config)
{
    if ((p_config == NULL) || !s3_line_config_valid(&p_config->line) ||
        !s3_barcode_config_valid(&p_config->barcode) ||
        (p_config->barcode_sensor > S3_NO_BARCODE) ||
        (p_config->max_sample_gap_ms == 0U) ||
        (p_config->max_sample_gap_ms > INT32_MAX) ||
        !navigation_valid(p_config->map_a) || !navigation_valid(p_config->map_z) ||
        !navigation_valid(p_config->map_b) || !navigation_valid(p_config->map_c) ||
        !navigation_valid(p_config->map_d) || !navigation_valid(p_config->map_f))
    {
        return false;
    }
    return (p_config->barcode_sensor == S3_NO_BARCODE) ||
           ((p_config->line.line_mask & (1U << p_config->barcode_sensor)) == 0U);
}

bool s3_core_init(s3_core_t *p_core, const s3_config_t *p_config)
{
    if (p_core == NULL)
    {
        return false;
    }
    (void)memset(p_core, 0, sizeof(*p_core));
    (void)memset(p_core->map_override, S3_MAP_DEFAULT, sizeof(p_core->map_override));
    if (!s3_config_valid(p_config))
    {
        p_core->latched_error = S3_ERROR_CONFIG;
        return false;
    }
    p_core->config = *p_config;
    p_core->mode = S3_MODE_PAUSED;
    p_core->initialized = true;
    s3_barcode_init(&p_core->barcode);
    return true;
}

bool s3_core_control(s3_core_t *p_core, s3_mode_t mode, bool marker_clear)
{
    if ((p_core == NULL) || !p_core->initialized ||
        ((unsigned int)mode > (unsigned int)S3_MODE_OVERRIDE))
    {
        return false;
    }
    /* Abort before rearming. A pause in the middle of a barcode must never
     * allow its suffix to be interpreted as a fresh navigation marker. */
    s3_barcode_abort(&p_core->barcode);
    p_core->mode = mode;
    p_core->barcode_black = false;
    if (mode == S3_MODE_TRACK)
    {
        p_core->awaiting_controller = false;
        p_core->latched_error = S3_ERROR_NONE;
        p_core->recovery_sent = false;
        p_core->stop_sent = false;
        s3_line_reset(&p_core->line);
    }
    if (marker_clear)
    {
        s3_barcode_rearm(&p_core->barcode);
    }
    return true;
}

void s3_core_fault(s3_core_t *p_core, s3_error_t error)
{
    if (p_core != NULL)
    {
        p_core->latched_error = (unsigned)error <= S3_ERROR_TRANSPORT ? error : S3_ERROR_CONFIG;
        p_core->awaiting_controller = true;
        s3_barcode_abort(&p_core->barcode);
    }
}

void s3_core_barcode_end(s3_core_t *p_core)
{
    if (p_core != NULL)
    {
        s3_barcode_request_end(&p_core->barcode);
    }
}

bool s3_core_set_map(s3_core_t *p_core, unsigned letter, unsigned nav)
{
    if ((p_core == NULL) || (letter > 26U) || (nav > (unsigned)S3_NAV_UTURN) ||
        ((letter == 0U) && (nav != 0U)))
    {
        return false;
    }
    if (letter == 0U)
    {
        (void)memset(p_core->map_override, S3_MAP_DEFAULT, sizeof(p_core->map_override));
    }
    else
    {
        p_core->map_override[letter - 1U] = (uint8_t)nav;
    }
    return true;
}

void s3_core_barcode_summary(const s3_core_t *p_core, s3_output_t *p_output)
{
    if ((p_core == NULL) || (p_output == NULL))
    {
        return;
    }
    p_output->last_symbol = p_core->last_symbol;
    p_output->barcode_ok_count = p_core->barcode_ok_count;
    p_output->barcode_fail_count = p_core->barcode_fail_count;
    if (p_core->barcode.state == S3_CAPTURE_RUNNING)
    {
        uint32_t bars = p_core->barcode.bars;
        uint32_t elements = p_core->barcode.elements;
        p_output->barcode_bars = (uint8_t)(bars > S3_BARCODE_BARS_MAX ? S3_BARCODE_BARS_MAX : bars);
        p_output->barcode_elements = (uint8_t)(elements > 255U ? 255U : elements);
        p_output->barcode_result = S3_BARCODE_RESULT_READING;
    }
    else
    {
        p_output->barcode_bars = p_core->last_bars;
        p_output->barcode_elements = p_core->last_elements;
        p_output->barcode_result = p_core->last_result;
    }
}

/* A finished capture becomes the dashboard result unless it was too short
 * to be a marker (a line crossing); bars/elements are the last capture's. */
static void barcode_finished(s3_core_t *p_core, bool decoded)
{
    uint32_t elements = p_core->barcode.elements != 0U ?
                        p_core->barcode.elements : p_core->barcode.count;
    uint32_t bars = p_core->barcode.elements != 0U ?
                    p_core->barcode.bars : (p_core->barcode.count + 1U) / 2U;
    if (!decoded && (elements < S3_BARCODE_FAIL_MIN_RUNS))
    {
        return;
    }
    p_core->last_bars = (uint8_t)(bars > S3_BARCODE_BARS_MAX ? S3_BARCODE_BARS_MAX : bars);
    p_core->last_elements = (uint8_t)(elements > 255U ? 255U : elements);
    p_core->last_result = decoded ? S3_BARCODE_RESULT_DECODED : S3_BARCODE_RESULT_FAILED;
}

static void event(s3_core_t *p_core, s3_output_t *p_output,
                    s3_event_kind_t kind, uint32_t now_ms)
{
    p_output->event.kind = (uint8_t)((unsigned)kind & 7U);
    p_output->event.time_ms = now_ms;
    p_output->event.sequence = ++p_core->sequence;
}

static void fault(s3_core_t *p_core, s3_output_t *p_output,
                    s3_error_t error, uint32_t now_ms)
{
    if ((p_core->latched_error == S3_ERROR_NONE) &&
        (p_core->mode == S3_MODE_TRACK))
    {
        event(p_core, p_output, S3_EVENT_FAULT, now_ms);
    }
    s3_core_fault(p_core, error);
    p_output->error = (uint8_t)((unsigned)error & 7U);
    p_output->awaiting_controller = true;
}

void s3_core_step(s3_core_t *p_core, const s3_sample_t *p_sample,
                   s3_output_t *p_output)
{
    uint32_t elapsed;
    uint8_t used;
    uint8_t barcode_sensor;
    uint16_t darkness;

    if (p_output == NULL)
    {
        return;
    }
    (void)memset(p_output, 0, sizeof(*p_output));
    if ((p_core == NULL) || !p_core->initialized || (p_sample == NULL))
    {
        p_output->error = S3_ERROR_CONFIG;
        return;
    }
    s3_core_barcode_summary(p_core, p_output);
    elapsed = p_sample->time_ms - p_core->last_sample;
    p_core->last_sample = p_sample->time_ms;
    if (p_core->have_sample &&
        ((elapsed == 0U) || (elapsed > p_core->config.max_sample_gap_ms)))
    {
        if ((p_core->mode == S3_MODE_TRACK) || (elapsed == 0U) ||
            (elapsed > INT32_MAX))
        {
            fault(p_core, p_output, S3_ERROR_STALE, p_sample->time_ms);
            return;
        }
        /* PAUSED/OVERRIDE make no timing-based navigation decisions. A
         * forward gap starts a new observation instead of permanently
         * latching a startup scheduling gap as an active-tracking fault.
         * Existing latched faults are deliberately NOT cleared here. */
        s3_line_reset(&p_core->line);
        s3_barcode_abort(&p_core->barcode);
        p_core->barcode_black = false;
    }
    p_core->have_sample = true;
    used = p_core->config.line.line_mask;
    barcode_sensor = p_core->config.barcode_sensor;
    if (barcode_sensor != S3_NO_BARCODE)
    {
        used |= (uint8_t)(1U << barcode_sensor);
    }
    if ((p_sample->valid_mask & used) != used)
    {
        fault(p_core, p_output, S3_ERROR_SENSOR, p_sample->time_ms);
        return;
    }
    s3_line_update(&p_core->line, &p_core->config.line, p_sample->raw,
                   p_sample->valid_mask, p_sample->time_ms, &p_output->line);
    if (p_output->line.state == S3_LINE_INVALID)
    {
        fault(p_core, p_output, S3_ERROR_SENSOR, p_sample->time_ms);
        return;
    }
    if (barcode_sensor != S3_NO_BARCODE)
    {
        if (!s3_normalize(p_sample->raw[barcode_sensor],
                          p_core->config.line.white[barcode_sensor],
                          p_core->config.line.black[barcode_sensor],
                          p_core->config.line.raw_max, &darkness))
        {
            fault(p_core, p_output, S3_ERROR_SENSOR, p_sample->time_ms);
            return;
        }
        if (darkness >= p_core->config.line.threshold_high)
        {
            p_core->barcode_black = true;
        }
        else if (darkness <= p_core->config.line.threshold_low)
        {
            p_core->barcode_black = false;
        }
        p_output->barcode_black = p_core->barcode_black;
    }
    if (p_core->mode != S3_MODE_TRACK)
    {
        s3_barcode_abort(&p_core->barcode);
        p_output->error = (uint8_t)((unsigned)p_core->latched_error & 7U);
        p_output->awaiting_controller = p_core->awaiting_controller;
        return;
    }
    if (!p_core->config.line_straddle &&
        (p_output->line.state == S3_LINE_LOST) &&
        (p_core->latched_error == S3_ERROR_NONE) &&
        (!p_core->awaiting_controller || p_core->recovery_sent))
    {
        s3_barcode_abort(&p_core->barcode);
        if (p_output->line.recovery_expired && !p_core->stop_sent)
        {
            event(p_core, p_output, S3_EVENT_STOP_REQUEST, p_sample->time_ms);
            p_core->stop_sent = true;
            p_core->awaiting_controller = true;
        }
        else if (!p_core->recovery_sent && !p_core->stop_sent)
        {
            event(p_core, p_output, S3_EVENT_RECOVERY_REQUEST, p_sample->time_ms);
            p_core->recovery_sent = true;
            p_core->awaiting_controller = true;
        }
    }
    /* Retain an in-progress barcode through the existing traversable junction.
     * This changes capture gating only; junction steering remains unchanged. */
    bool capture_at_junction=p_core->config.barcode_retry &&
        p_core->config.line.line_mask==3U && p_output->line.state==S3_LINE_JUNCTION &&
        p_output->line.black_mask==3U && p_core->barcode.state==S3_CAPTURE_RUNNING;
    if (((p_output->line.state != S3_LINE_TRACKING) && !capture_at_junction &&
         !p_core->config.line_straddle) ||
        p_core->awaiting_controller || (p_core->latched_error != S3_ERROR_NONE))
    {
        if(p_core->config.barcode_retry && p_core->config.line.line_mask==3U &&
           p_output->line.state==S3_LINE_JUNCTION && p_output->line.black_mask==3U &&
           !p_core->awaiting_controller && p_core->latched_error==S3_ERROR_NONE &&
           p_core->barcode.state==S3_CAPTURE_HOLD) {
            /* Preserve the rearm request across the junction, but require a
             * fresh white quiet interval after it, as in the reference. */
            p_core->barcode.have_sample=false;
        } else s3_barcode_abort(&p_core->barcode);
    }
    else if (barcode_sensor != S3_NO_BARCODE)
    {
        p_core->barcode.position = p_sample->position;
        p_core->barcode.position_valid = p_sample->position_valid;
        p_output->barcode_status =(uint8_t)((unsigned)(p_core->config.barcode_retry ? s3_barcode_feed_resync : s3_barcode_feed)(&p_core->barcode,
                                                  &p_core->config.barcode,
                                                  p_core->barcode_black,
                                                  p_sample->time_ms,
                                                  &p_output->decoded) & 7U);
        if (p_output->barcode_status == S3_BARCODE_OK)
        {
            s3_navigation_t command = S3_NAV_NONE;
            p_output->barcode_ok_count = ++p_core->barcode_ok_count;
            barcode_finished(p_core, true);
            uint8_t wanted = S3_MAP_DEFAULT;
            if (p_output->decoded.symbol >= 'A' && p_output->decoded.symbol <= 'Z')
            {
                p_core->last_symbol = p_output->decoded.symbol;
                p_output->last_symbol = p_core->last_symbol;
                wanted = p_core->map_override[(unsigned)(p_output->decoded.symbol - 'A')];
            }
            /* An operator override wins over the configured codebook. */
            if (wanted != S3_MAP_DEFAULT) command = (s3_navigation_t)wanted;
            else switch (p_output->decoded.symbol) {
                case 'A': command=p_core->config.map_a; break;
                case 'B': command=p_core->config.map_b; break;
                case 'C': command=p_core->config.map_c; break;
                case 'D': command=p_core->config.map_d; break;
                case 'F': command=p_core->config.map_f; break;
                case 'Z': command=p_core->config.map_z; break;
                default: break;
            }
            if (command != S3_NAV_NONE)
            {
                event(p_core, p_output, S3_EVENT_NAVIGATION, p_sample->time_ms);
                p_output->event.command = (uint8_t)((unsigned)command & 7U);
                p_output->event.symbol = p_output->decoded.symbol;
                p_core->awaiting_controller = true;
            }
            else
            {
                p_output->error = S3_ERROR_UNMAPPED;
                if(p_core->config.barcode_retry) {
                    p_core->barcode.last_edge=p_sample->time_ms;
                    s3_barcode_rearm(&p_core->barcode);
                }
            }
        }
        else if (p_output->barcode_status != S3_BARCODE_NONE)
        {
            uint32_t elements = p_core->barcode.elements != 0U ?
                                p_core->barcode.elements : p_core->barcode.count;
            if (elements >= S3_BARCODE_FAIL_MIN_RUNS)
            {
                p_output->barcode_fail_count = ++p_core->barcode_fail_count;
            }
            barcode_finished(p_core, false);
            if(p_core->config.barcode_retry) {
                /* Reject a pattern, not the line sensor. Wait for a fresh
                 * white quiet interval before considering another marker. */
                p_core->barcode.last_edge=p_sample->time_ms;
                s3_barcode_rearm(&p_core->barcode);
            } else fault(p_core, p_output, S3_ERROR_BARCODE, p_sample->time_ms);
        }
    }
    /* Both front sensors on black is a traversable junction. Keep the
     * junction status for telemetry while permitting controller steering. */
    p_output->steering_allowed = ((p_output->line.state == S3_LINE_TRACKING) ||
                                 (p_core->config.line.line_mask == 3U &&
                                  p_output->line.state == S3_LINE_JUNCTION &&
                                  p_output->line.black_mask == 3U)) &&
                                 !p_core->awaiting_controller &&
                                 (p_core->latched_error == S3_ERROR_NONE);
    p_output->awaiting_controller = p_core->awaiting_controller;
    if (p_core->latched_error != S3_ERROR_NONE)
    {
        p_output->error = (uint8_t)((unsigned)p_core->latched_error & 7U);
    }
    s3_core_barcode_summary(p_core, p_output);
}
