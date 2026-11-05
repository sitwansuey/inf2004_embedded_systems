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
    S3_EVENT_FAULT,
    S3_EVENT_BARCODE /* Decoded A-Z observation; route belongs to controller. */
} s3_event_kind_t;

typedef struct
{
    uint8_t kind : 3;
    uint8_t command : 3;
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
    bool barcode_retry; /* Invalid candidates rearm on fresh white; no drive fault. */
} s3_config_t;

typedef struct
{
    uint32_t rejected_count;
    uint32_t rejected_ms;
    uint32_t rejected_runs;
    uint32_t max_run_ms;
    uint8_t status; /* Last s3_barcode_status_t; retained for slow telemetry. */
    bool junction_abort;
    /* Retain the last complete result through retries, pause and slow Wi-Fi. */
    uint32_t result_id,result_ms,started_ms,result_runs,result_max_run_ms,max_gap_ms,decoded_count;
    uint32_t runs[S3_BARCODE_RUN_COUNT];
    char symbol;
    uint8_t result_status;
    uint8_t result_line_mask;
    bool reverse,result_junction_abort;
} s3_barcode_diagnostics_t;

typedef struct
{
    uint16_t raw[S3_SENSOR_COUNT];
    uint8_t valid_mask;
    uint32_t time_ms;
} s3_sample_t;

typedef struct
{
    s3_line_result_t line;
    s3_event_t event;
    s3_symbol_t decoded;
    uint8_t barcode_status : 3;
    uint8_t error : 3;
    uint8_t steering_allowed : 1;
    uint8_t awaiting_controller : 1;
    uint8_t barcode_scanning : 1;
    uint8_t barcode_white : 1;
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
    uint32_t barcode_max_gap_ms;
    s3_barcode_diagnostics_t barcode_diagnostics;
} s3_core_t;

/* Defaults are testable algorithm settings, not hardware claims. No mapping. */
void s3_config_defaults(s3_config_t *p_config);
bool s3_config_valid(const s3_config_t *p_config);
bool s3_core_init(s3_core_t *p_core, const s3_config_t *p_config);
/* Explicit TRACK is resume/acknowledgement, clears a fault/recovery hold.
 * Call only for a new controller directive, not every sampling cycle. */
bool s3_core_control(s3_core_t *p_core, s3_mode_t mode, bool marker_clear);
void s3_core_fault(s3_core_t *p_core, s3_error_t error);
/* Forward observation gaps while paused/overridden reset temporal history.
 * TRACK gaps, duplicate/backwards time and existing faults remain fail-safe. */
void s3_core_step(s3_core_t *p_core, const s3_sample_t *p_sample,
                   s3_output_t *p_output);

#endif
