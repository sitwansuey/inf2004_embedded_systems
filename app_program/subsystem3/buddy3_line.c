#include "buddy3_line.h"

#include <stddef.h>

bool s3_normalize(uint16_t raw, uint16_t white, uint16_t black,
                  uint16_t raw_max, uint16_t *p_darkness)
{
    int32_t span = (int32_t)black - (int32_t)white;
    int32_t value;

    if (p_darkness == NULL)
    {
        return false;
    }
    *p_darkness = 0U;
    if ((span == 0) || (raw_max == 0U) || (raw > raw_max) ||
        (white > raw_max) || (black > raw_max))
    {
        return false;
    }
    value = (((int32_t)raw - (int32_t)white) * 1000) / span;
    if (value < 0)
    {
        value = 0;
    }
    if (value > 1000)
    {
        value = 1000;
    }
    *p_darkness = (uint16_t)value;
    return true;
}

#include <string.h>

bool s3_line_config_valid(const s3_line_config_t *p_config)
{
    uint8_t i;
    uint8_t j;

    if ((p_config == NULL) || (p_config->raw_max == 0U) ||
        (p_config->line_mask == 0U) || (p_config->line_mask > S3_SENSOR_MASK) ||
        (p_config->threshold_low >= p_config->threshold_high) ||
        (p_config->threshold_high > 1000U) || (p_config->lost_ms == 0U) ||
        (p_config->recovery_ms == 0U) ||
        (((uint64_t)p_config->lost_ms + p_config->recovery_ms) > INT32_MAX))
    {
        return false;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        if ((p_config->white[i] > p_config->raw_max) ||
            (p_config->black[i] > p_config->raw_max) ||
            (p_config->white[i] == p_config->black[i]) ||
            (p_config->position[i] < -1000) || (p_config->position[i] > 1000))
        {
            return false;
        }
        for (j = 0U; j < i; ++j)
        {
            if (((p_config->line_mask & (1U << i)) != 0U) &&
                ((p_config->line_mask & (1U << j)) != 0U) &&
                (p_config->position[i] == p_config->position[j]))
            {
                return false;
            }
        }
    }
    return true;
}

void s3_line_reset(s3_line_t *p_line)
{
    if (p_line != NULL)
    {
        (void)memset(p_line, 0, sizeof(*p_line));
    }
}

void s3_line_update(s3_line_t *p_line, const s3_line_config_t *p_config,
                    const uint16_t raw[S3_SENSOR_COUNT], uint8_t valid_mask,
                    uint32_t now_ms, s3_line_result_t *p_result)
{
    uint8_t i;
    uint8_t active;
    uint32_t weight = 0U;
    int32_t weighted_position = 0;

    if (p_result == NULL)
    {
        return;
    }
    (void)memset(p_result, 0, sizeof(*p_result));
    p_result->state = S3_LINE_INVALID;
    if ((p_line == NULL) || (raw == NULL) || !s3_line_config_valid(p_config))
    {
        return;
    }
    if ((valid_mask & p_config->line_mask) != p_config->line_mask)
    {
        s3_line_reset(p_line);
        return;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        uint8_t bit = (uint8_t)(1U << i);
        if ((p_config->line_mask & bit) == 0U)
        {
            continue;
        }
        if (!s3_normalize(raw[i], p_config->white[i], p_config->black[i],
                          p_config->raw_max, &p_result->darkness[i]))
        {
            s3_line_reset(p_line);
            return;
        }
        if (p_result->darkness[i] >= p_config->threshold_high)
        {
            p_line->black_mask |= bit;
        }
        else if (p_result->darkness[i] <= p_config->threshold_low)
        {
            p_line->black_mask &= (uint8_t)~bit;
        }
        if ((p_line->black_mask & bit) != 0U)
        {
            weight += p_result->darkness[i];
            weighted_position += (int32_t)p_result->darkness[i] *
                                 p_config->position[i];
        }
    }
    active = p_line->black_mask & p_config->line_mask;
    p_result->black_mask = active;
    if (active == 0U)
    {
        uint32_t elapsed;
        if (!p_line->missing)
        {
            p_line->missing = true;
            p_line->missing_since = now_ms;
        }
        elapsed = now_ms - p_line->missing_since;
        p_result->state = (elapsed >= p_config->lost_ms) ?
                          S3_LINE_LOST : S3_LINE_SEARCHING;
        p_result->recovery_expired = elapsed >=
                                    (p_config->lost_ms + p_config->recovery_ms);
        return;
    }
    p_line->missing = false;
    /* A single line sensor cannot distinguish a junction. All-dark with
     * multiple sensors, or split outer detections, is an indication only. */
    if (((active == p_config->line_mask) && ((active & (active - 1U)) != 0U)) ||
        ((p_config->line_mask == 7U) && (active == 5U)))
    {
        p_result->state = S3_LINE_JUNCTION;
        return;
    }
    if (weight == 0U)
    {
        return;
    }
    p_result->error = (int16_t)(weighted_position / (int32_t)weight);
    p_result->state = S3_LINE_TRACKING;
}

void s3_calibration_init(s3_calibration_t *p_calibration, uint16_t raw_max)
{
    uint8_t surface;
    uint8_t i;

    if (p_calibration == NULL)
    {
        return;
    }
    (void)memset(p_calibration, 0, sizeof(*p_calibration));
    p_calibration->raw_max = raw_max;
    for (surface = 0U; surface < 2U; ++surface)
    {
        for (i = 0U; i < S3_SENSOR_COUNT; ++i)
        {
            p_calibration->minimum[surface][i] = UINT16_MAX;
        }
    }
}

bool s3_calibration_add(s3_calibration_t *p_calibration, bool black,
                        const uint16_t raw[S3_SENSOR_COUNT])
{
    uint8_t i;
    uint8_t surface = black ? 1U : 0U;

    if ((p_calibration == NULL) || (raw == NULL) ||
        (p_calibration->raw_max == 0U) ||
        (p_calibration->count[surface] >= S3_CALIBRATION_MAX_SAMPLES))
    {
        return false;
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        if (raw[i] > p_calibration->raw_max)
        {
            return false;
        }
    }
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        p_calibration->sum[surface][i] += raw[i];
        if (raw[i] < p_calibration->minimum[surface][i])
        {
            p_calibration->minimum[surface][i] = raw[i];
        }
        if (raw[i] > p_calibration->maximum[surface][i])
        {
            p_calibration->maximum[surface][i] = raw[i];
        }
    }
    ++p_calibration->count[surface];
    return true;
}

bool s3_calibration_finish(const s3_calibration_t *p_calibration,
                           uint16_t min_contrast, s3_line_config_t *p_config)
{
    s3_line_config_t candidate;
    uint8_t i;
    uint8_t surface;

    if ((p_calibration == NULL) || (p_config == NULL) || (min_contrast == 0U) ||
        (p_calibration->raw_max == 0U) ||
        (p_calibration->count[0] < S3_CALIBRATION_MIN_SAMPLES) ||
        (p_calibration->count[1] < S3_CALIBRATION_MIN_SAMPLES) ||
        (p_calibration->count[0] > S3_CALIBRATION_MAX_SAMPLES) ||
        (p_calibration->count[1] > S3_CALIBRATION_MAX_SAMPLES))
    {
        return false;
    }
    candidate = *p_config;
    candidate.raw_max = p_calibration->raw_max;
    for (i = 0U; i < S3_SENSOR_COUNT; ++i)
    {
        uint16_t white = (uint16_t)(p_calibration->sum[0][i] /
                                   p_calibration->count[0]);
        uint16_t black = (uint16_t)(p_calibration->sum[1][i] /
                                   p_calibration->count[1]);
        uint16_t contrast = (white > black) ? (white - black) : (black - white);
        if ((contrast < min_contrast) || (white > candidate.raw_max) ||
            (black > candidate.raw_max))
        {
            return false;
        }
        for (surface = 0U; surface < 2U; ++surface)
        {
            if (((uint32_t)p_calibration->maximum[surface][i] -
                 p_calibration->minimum[surface][i]) > (contrast / 4U))
            {
                return false;
            }
        }
        candidate.white[i] = white;
        candidate.black[i] = black;
    }
    if (!s3_line_config_valid(&candidate))
    {
        return false;
    }
    *p_config = candidate;
    return true;
}
