#ifndef BUDDY3_CORE_H
#define BUDDY3_CORE_H

#include "buddy3_line.h"
#include "buddy3_barcode.h"

#define S3_NO_BARCODE S3_SENSOR_COUNT

typedef enum
{
    S3_NAV_NONE,
    S3_NAV_LEFT,
    S3_NAV_RIGHT,
    S3_NAV_STRAIGHT,
    S3_NAV_UTURN
} s3_navigation_t;

typedef enum
{
    S3_MODE_PAUSED,
    S3_MODE_TRACK,
    S3_MODE_OVERRIDE
} s3_mode_t;

typedef enum
{
    S3_ERROR_NONE,
    S3_ERROR_CONFIG,
    S3_ERROR_SENSOR,
    S3_ERROR_STALE,
    S3_ERROR_BARCODE,
    S3_ERROR_UNMAPPED,
    S3_ERROR_TRANSPORT
} s3_error_t;

typedef enum
{
    S3_EVENT_NONE,
    S3_EVENT_NAVIGATION,
    S3_EVENT_RECOVERY_REQUEST,
    S3_EVENT_STOP_REQUEST,
    S3_EVENT_FAULT
} s3_event_kind_t;

typedef struct
{
    s3_event_kind_t kind;
    s3_navigation_t command;
    char symbol;
    uint32_t sequence;
    uint32_t time_ms;
} s3_event_t;

typedef struct
{
    s3_line_config_t line;
    s3_barcode_config_t barcode;
    uint8_t barcode_sensor; /* Must not also be in line.line_mask. */
    uint32_t max_sample_gap_ms;
    s3_navigation_t map_a;
    s3_navigation_t map_z;
} s3_config_t;

typedef struct
{
    uint16_t raw[S3_SENSOR_COUNT];
    uint8_t valid_mask;
    uint32_t time_ms;
} s3_sample_t;

typedef struct
{
    s3_line_result_t line;
    s3_barcode_status_t barcode_status;
    s3_symbol_t decoded;
    s3_event_t event;
    s3_error_t error;
    bool steering_allowed;
    bool awaiting_controller;
} s3_output_t;

typedef struct
{
    s3_config_t config;
    s3_line_t line;
    s3_barcode_t barcode;
    s3_mode_t mode;
    s3_error_t latched_error;
    uint32_t last_sample;
    uint32_t sequence;
    bool initialized;
    bool have_sample;
    bool barcode_black;
    bool awaiting_controller;
    bool recovery_sent;
    bool stop_sent;
} s3_core_t;

/* Defaults are testable algorithm settings, not hardware claims. No mapping. */
void s3_config_defaults(s3_config_t *p_config);
bool s3_config_valid(const s3_config_t *p_config);
bool s3_core_init(s3_core_t *p_core, const s3_config_t *p_config);
/* Explicit TRACK is resume/acknowledgement, clears a fault/recovery hold.
 * Call only for a new controller directive, not every sampling cycle. */
bool s3_core_control(s3_core_t *p_core, s3_mode_t mode, bool marker_clear);
void s3_core_fault(s3_core_t *p_core, s3_error_t error);
void s3_core_step(s3_core_t *p_core, const s3_sample_t *p_sample,
                   s3_output_t *p_output);

#endif
