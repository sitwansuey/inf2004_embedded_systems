/* Buddy 3 reflectance and line algorithms. No hardware or RTOS dependencies. */
#ifndef BUDDY3_LINE_H
#define BUDDY3_LINE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_SENSOR_COUNT 3U
#define S3_SENSOR_MASK 7U
#define S3_CALIBRATION_MIN_SAMPLES 4U
#define S3_CALIBRATION_MAX_SAMPLES 64U

typedef struct
{
    uint16_t raw_max;
    uint16_t white[S3_SENSOR_COUNT];
    uint16_t black[S3_SENSOR_COUNT];
    int16_t position[S3_SENSOR_COUNT]; /* Logical left < centre < right. */
    uint8_t line_mask; /* Only physically line-facing sensors. */
    uint16_t threshold_low;
    uint16_t threshold_high;
    uint32_t lost_ms;
    uint32_t recovery_ms;
} s3_line_config_t;

typedef enum
{
    S3_LINE_INVALID,
    S3_LINE_TRACKING,
    S3_LINE_SEARCHING,
    S3_LINE_LOST,
    S3_LINE_JUNCTION
} s3_line_state_t;

typedef struct
{
    s3_line_state_t state;
    uint16_t darkness[S3_SENSOR_COUNT];
    int16_t error; /* Same units as position; negative means line to the left. */
    uint8_t black_mask;
    bool recovery_expired;
} s3_line_result_t;

typedef struct
{
    uint8_t black_mask;
    bool missing;
    uint32_t missing_since;
} s3_line_t;

typedef struct
{
    uint16_t raw_max;
    uint16_t count[2]; /* Explicitly labelled white / black observations. */
    uint32_t sum[2][S3_SENSOR_COUNT];
    uint16_t minimum[2][S3_SENSOR_COUNT];
    uint16_t maximum[2][S3_SENSOR_COUNT];
} s3_calibration_t;

/* Darkness: white=0, black=1000; either electrical polarity is supported. */
bool s3_normalize(uint16_t raw, uint16_t white, uint16_t black,
                  uint16_t raw_max, uint16_t *p_darkness);
bool s3_line_config_valid(const s3_line_config_t *p_config);
void s3_line_reset(s3_line_t *p_line);
void s3_line_update(s3_line_t *p_line, const s3_line_config_t *p_config,
                    const uint16_t raw[S3_SENSOR_COUNT], uint8_t valid_mask,
                    uint32_t now_ms, s3_line_result_t *p_result);
void s3_calibration_init(s3_calibration_t *p_calibration, uint16_t raw_max);
bool s3_calibration_add(s3_calibration_t *p_calibration, bool black,
                        const uint16_t raw[S3_SENSOR_COUNT]);
/* Rejects low contrast, too few observations and unstable surface readings.
 * Does not change p_config on failure. min_contrast is in raw sensor units. */
bool s3_calibration_finish(const s3_calibration_t *p_calibration,
                           uint16_t min_contrast, s3_line_config_t *p_config);

#endif
