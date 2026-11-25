/* Code 39 A-Z symbols, independent of the runtime navigation mapping. */
#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_BARCODE_RUN_COUNT 29U
#define S3_BARCODE_MAX_RESYNC_PAIRS 8U

typedef struct
{
    uint32_t quiet_ms;   /* Required blank surface before/after a capture. */
    uint32_t min_run_ms; /* Optional pulse-duration floor; zero disables it. */
    uint32_t timeout_ms;
    /* Nonzero: a capture is finished only after this long with no edge at
     * all. Every edge until then is counted (overflow/noise are reported at
     * the end instead of aborting mid-marker). Zero keeps the legacy exits. */
    uint32_t end_quiet_ms;
    /* end_quiet_ms mode with positions: a run is noise only when it is both
     * shorter than min_run_ms and shorter than this distance (position
     * units); zero disables the noise check for position captures. */
    uint32_t min_run_position;
} s3_barcode_config_t;

typedef enum
{
    S3_BARCODE_NONE,
    S3_BARCODE_OK,
    S3_BARCODE_INCOMPLETE,
    S3_BARCODE_NOISE,
    S3_BARCODE_OVERFLOW,
    S3_BARCODE_TIMEOUT,
    S3_BARCODE_BAD_PATTERN,
    S3_BARCODE_UNKNOWN
} s3_barcode_status_t;

typedef enum
{
    S3_CAPTURE_WAIT_QUIET,
    S3_CAPTURE_READY,
    S3_CAPTURE_RUNNING,
    S3_CAPTURE_HOLD
} s3_capture_state_t;

typedef struct
{
    char symbol; /* A..Z on success; zero on any failure. */
    bool reverse;
} s3_symbol_t;

typedef struct
{
    s3_capture_state_t state;
    uint32_t runs[S3_BARCODE_RUN_COUNT]; /* Black first, alternating. */
    uint32_t last_sample;
    uint32_t last_edge;
    uint32_t started;
    uint32_t max_run;
    uint32_t count;
    uint32_t resync_pairs; /* Discarded leading black/white pairs; bounded. */
    bool     have_sample;
    bool     black;
    bool     rearm_requested;
    bool completed_window_checked; /* One early decode attempt per saved window.
                                    */
    /* end_quiet_ms mode only: totals for the whole marker, first full window
     * before any resync slide, and the first problem seen while capturing. */
    uint32_t bars;     /* black elements entered, including the opening bar */
    uint32_t elements; /* completed black+white elements */
    uint32_t first_runs[S3_BARCODE_RUN_COUNT];
    bool     have_first;
    /* end_quiet_ms mode only: the controller saw the end of the marker (for
     * example 30 mm travelled without an edge); finish at the next edge-free
     * sample instead of waiting out end_quiet_ms. */
    bool end_requested;
    /* end_quiet_ms mode: the caller sets position/position_valid before each
     * feed. A capture that starts with a valid position stores its runs as
     * travelled distance instead of milliseconds (the decoder uses ratios). */
    uint32_t position;
    bool     position_valid;
    bool     by_position;
    uint32_t last_edge_position;
    uint8_t  pending; /* s3_barcode_status_t */
} s3_barcode_t;

bool s3_barcode_config_valid (const s3_barcode_config_t * p_config);
void s3_barcode_init (s3_barcode_t * p_barcode);
/* Abort partial capture on pause/fault. Completed/partial markers remain held
 * until the controller explicitly confirms physical marker clearance. */
void s3_barcode_abort (s3_barcode_t * p_barcode);
void s3_barcode_rearm (s3_barcode_t * p_barcode);
/* end_quiet_ms mode: finish the running capture at the next sample without an
 * edge (end_quiet_ms remains the backstop). Ignored when nothing is running. */
void                s3_barcode_request_end (s3_barcode_t * p_barcode);
s3_barcode_status_t s3_barcode_feed (s3_barcode_t *              p_barcode,
                                     const s3_barcode_config_t * p_config,
                                     bool black, uint32_t now_ms,
                                     s3_symbol_t * p_symbol);
/* Retry profiles may slide past bounded leading pulse pairs. A complete valid
 * *A..Z* can finish after white for max(quiet_ms, twice the longest saved run).
 * Other captures keep the original four-run quiet exit and absolute timeout.
 * Width/framing checks are identical; no letter is emitted before white.
 * The strict feed API above retains immediate overflow rejection. */
s3_barcode_status_t
s3_barcode_feed_resync (s3_barcode_t *              p_barcode,
                        const s3_barcode_config_t * p_config, bool black,
                        uint32_t now_ms, s3_symbol_t * p_symbol);
/* Widths may be time or monotonic distance units. Exactly two bounded passes
 * (forward/reverse) over the saved data; no physical rescan or allocation. */
s3_barcode_status_t s3_barcode_decode (const uint32_t * p_runs, uint32_t count,
                                       s3_symbol_t * p_symbol);

#endif

/*** end of file ***/
