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
    return (command >= S3_NAV_NONE) && (command <= S3_NAV_UTURN);
}

bool s3_config_valid(const s3_config_t *p_config)
{
    if ((p_config == NULL) || !s3_line_config_valid(&p_config->line) ||
        !s3_barcode_config_valid(&p_config->barcode) ||
        (p_config->barcode_sensor > S3_NO_BARCODE) ||
        (p_config->max_sample_gap_ms == 0U) ||
        (p_config->max_sample_gap_ms > INT32_MAX) ||
        !navigation_valid(p_config->map_a) || !navigation_valid(p_config->map_z))
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
    if ((p_core == NULL) || !p_core->initialized || (mode < S3_MODE_PAUSED) ||
        (mode > S3_MODE_OVERRIDE))
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
        p_core->latched_error = error;
        p_core->awaiting_controller = true;
        s3_barcode_abort(&p_core->barcode);
    }
}

static void event(s3_core_t *p_core, s3_output_t *p_output,
                    s3_event_kind_t kind, uint32_t now_ms)
{
    p_output->event.kind = kind;
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
    p_output->error = error;
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
    elapsed = p_sample->time_ms - p_core->last_sample;
    p_core->last_sample = p_sample->time_ms;
    if (p_core->have_sample &&
        ((elapsed == 0U) || (elapsed > p_core->config.max_sample_gap_ms)))
    {
        fault(p_core, p_output, S3_ERROR_STALE, p_sample->time_ms);
        return;
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
    }
    if (p_core->mode != S3_MODE_TRACK)
    {
        s3_barcode_abort(&p_core->barcode);
        p_output->error = p_core->latched_error;
        p_output->awaiting_controller = p_core->awaiting_controller;
        return;
    }
    if ((p_output->line.state == S3_LINE_LOST) &&
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
    if ((p_output->line.state != S3_LINE_TRACKING) ||
        p_core->awaiting_controller || (p_core->latched_error != S3_ERROR_NONE))
    {
        s3_barcode_abort(&p_core->barcode);
    }
    else if (barcode_sensor != S3_NO_BARCODE)
    {
        p_output->barcode_status = s3_barcode_feed(&p_core->barcode,
                                                  &p_core->config.barcode,
                                                  p_core->barcode_black,
                                                  p_sample->time_ms,
                                                  &p_output->decoded);
        if (p_output->barcode_status == S3_BARCODE_OK)
        {
            s3_navigation_t command = (p_output->decoded.symbol == 'A') ?
                                      p_core->config.map_a : p_core->config.map_z;
            if (command != S3_NAV_NONE)
            {
                event(p_core, p_output, S3_EVENT_NAVIGATION, p_sample->time_ms);
                p_output->event.command = command;
                p_output->event.symbol = p_output->decoded.symbol;
                p_core->awaiting_controller = true;
            }
            else
            {
                p_output->error = S3_ERROR_UNMAPPED;
            }
        }
        else if (p_output->barcode_status != S3_BARCODE_NONE)
        {
            fault(p_core, p_output, S3_ERROR_BARCODE, p_sample->time_ms);
        }
    }
    p_output->steering_allowed = (p_output->line.state == S3_LINE_TRACKING) &&
                                 !p_core->awaiting_controller &&
                                 (p_core->latched_error == S3_ERROR_NONE);
    p_output->awaiting_controller = p_core->awaiting_controller;
    if (p_core->latched_error != S3_ERROR_NONE)
    {
        p_output->error = p_core->latched_error;
    }
}
