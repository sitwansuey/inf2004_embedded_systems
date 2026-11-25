#ifndef BUDDY3_CORE_H
#define BUDDY3_CORE_H

#include "buddy3_line.h"
#include "buddy3_barcode.h"

#define S3_NO_BARCODE S3_SENSOR_COUNT
/* A rejected capture shorter than one framed character (9 elements) is a
 * line crossing or speck, not a barcode that failed to decode. */
#define S3_BARCODE_FAIL_MIN_RUNS 9U
/* From this many elements the vehicle treats a capture as a marker being read
 * (follower off). A capture that ends by itself with this many but fewer than
 * S3_BARCODE_FAIL_MIN_RUNS is counted as partial, not as a failed read; one
 * cut short by a sampler fault is a failed read (see barcode_fault_fails). */
#define S3_BARCODE_MARKER_MIN_RUNS 3U
/* Dashboard summary of the barcode reader. */
#define S3_BARCODE_RESULT_NONE 0U
#define S3_BARCODE_RESULT_READING 1U
#define S3_BARCODE_RESULT_DECODED 2U
#define S3_BARCODE_RESULT_FAILED 3U
#define S3_BARCODE_BARS_MAX 31U
/* map_override entry: use the configured codebook for this letter. */
#define S3_MAP_DEFAULT 0xFFU

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
    uint8_t  kind : 3;
    uint8_t  command : 3;
    char     symbol;
    uint32_t sequence;
    uint32_t time_ms;
} s3_event_t;

typedef struct
{
    s3_line_config_t    line;
    s3_barcode_config_t barcode;
    bool barcode_retry; /* Reference bounded resync; does not change navigation
                           mapping. */
    uint8_t         barcode_sensor; /* Must not also be in line.line_mask. */
    uint32_t        max_sample_gap_ms;
    s3_navigation_t map_a;
    s3_navigation_t map_z;
    s3_navigation_t map_b, map_c, map_d, map_f;
    /* Line sensors straddle the line: both WHITE is centred, so all-white is
     * not LOST and barcode capture is not gated on single-sensor TRACKING. */
    bool line_straddle;
    /* A TRACK capture of >= S3_BARCODE_MARKER_MIN_RUNS elements cut short by
     * s3_core_fault (a sampler hiccup) counts as a failed read: fail counter,
     * FAILED result, so the controller re-reads the marker. Off by default. */
    bool barcode_fault_fails;
} s3_config_t;

typedef struct
{
    uint16_t raw[S3_SENSOR_COUNT];
    uint8_t  valid_mask;
    uint32_t time_ms;
    /* Optional cumulative wheel travel when the scan was taken (half encoder
     * counts, forward and backward both add). When valid, barcode bars are
     * measured by distance, so stops and speed changes cannot distort them. */
    uint32_t position;
    bool     position_valid;
} s3_sample_t;

typedef struct
{
    s3_line_result_t line;
    s3_event_t       event;
    s3_symbol_t      decoded;
    uint8_t          barcode_status : 3;
    uint8_t          error : 3;
    uint8_t          steering_allowed : 1;
    uint8_t          awaiting_controller : 1;
    /* This sample's barcode sensor (GP28) classified black, in every mode;
     * 0 when the sample is invalid or there is no barcode sensor. */
    uint8_t barcode_black : 1;
    char    last_symbol; /* Latched most recent decoded A..Z, 0 if none yet. */
    /* Wrapping counters so a 10 ms poller cannot miss a one-sample result. */
    uint8_t barcode_ok_count;
    uint8_t
        barcode_fail_count; /* only captures of >= S3_BARCODE_FAIL_MIN_RUNS */
    uint8_t barcode_bars; /* live while reading, else last marker; saturates at
                             31 */
    /* Black+white elements completed, same lifetime as barcode_bars;
     * a full *X* marker is 29. Saturates at 255. */
    uint8_t barcode_elements;
    uint8_t barcode_result; /* S3_BARCODE_RESULT_* */
    /* Wrapping: captures that ended by themselves with
     * S3_BARCODE_MARKER_MIN_RUNS..S3_BARCODE_FAIL_MIN_RUNS-1 elements. Not a
     * read and not in barcode_fail_count; for read-rate measurement only. */
    uint8_t barcode_partial_count;
    /* Wrapping: reads cut short by a sampler fault (barcode_fault_fails);
     * each one is also in barcode_fail_count. */
    uint8_t barcode_abort_count;
} s3_output_t;

typedef struct
{
    s3_config_t  config;
    s3_line_t    line;
    s3_barcode_t barcode;
    s3_mode_t    mode;
    s3_error_t   latched_error;
    uint32_t     last_sample;
    uint32_t     sequence;
    bool         initialized;
    bool         have_sample;
    bool         barcode_black;
    bool         awaiting_controller;
    bool         recovery_sent;
    bool         stop_sent;
    char         last_symbol;
    uint8_t      barcode_ok_count;
    uint8_t      barcode_fail_count;
    uint8_t      last_bars;
    uint8_t      last_elements;
    uint8_t      last_result;
    uint8_t      barcode_partial_count;
    uint8_t      barcode_abort_count;
    uint8_t      partial_elements; /* elements of the last partial capture */
    /* Operator letter -> s3_navigation_t, A..Z; S3_MAP_DEFAULT = codebook. */
    uint8_t map_override[26];
} s3_core_t;

/* Defaults are testable algorithm settings, not hardware claims. No mapping. */
void s3_config_defaults (s3_config_t * p_config);
bool s3_config_valid (const s3_config_t * p_config);
bool s3_core_init (s3_core_t * p_core, const s3_config_t * p_config);
/* Explicit TRACK is resume/acknowledgement, clears a fault/recovery hold.
 * Call only for a new controller directive, not every sampling cycle. */
bool s3_core_control (s3_core_t * p_core, s3_mode_t mode, bool marker_clear);
/* Latch an error and abort any capture. With config.barcode_fault_fails, a
 * TRACK capture of >= S3_BARCODE_MARKER_MIN_RUNS elements it cuts short is
 * counted as a failed read (barcode_fail_count, barcode_abort_count). */
void s3_core_fault (s3_core_t * p_core, s3_error_t error);
/* Finish a running barcode capture at the next edge-free sample (the
 * controller measured the end of the marker by distance). No-op otherwise. */
void s3_core_barcode_end (s3_core_t * p_core);
/* Runtime codebook override for letter 1..26 (A..Z): nav 0..4 is the
 * s3_navigation_t used on its next decode (0 = read, no turn). Letter 0 with
 * nav 0 restores the configured codebook for every letter. False if invalid. */
bool s3_core_set_map (s3_core_t * p_core, unsigned letter, unsigned nav);
/* Copy latched barcode diagnostics (letter, counters, bars, result). */
void s3_core_barcode_summary (const s3_core_t * p_core, s3_output_t * p_output);
/* Forward observation gaps while paused/overridden reset temporal history.
 * TRACK gaps, duplicate/backwards time and existing faults remain fail-safe. */
void s3_core_step (s3_core_t * p_core, const s3_sample_t * p_sample,
                   s3_output_t * p_output);

#endif

/*** end of file ***/
