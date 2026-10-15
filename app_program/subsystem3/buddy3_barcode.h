/* Observed A/Z sample profile, not an invented navigation codebook. */
#ifndef BUDDY3_BARCODE_H
#define BUDDY3_BARCODE_H

#include <stdbool.h>
#include <stdint.h>

#define S3_BARCODE_RUN_COUNT 29U

typedef struct
{
    uint32_t quiet_ms; /* Required blank surface before/after a capture. */
    uint32_t min_run_ms; /* Reject under-sampled/noisy elements. */
    uint32_t timeout_ms;
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
    char symbol; /* Only A or Z on success; zero on any failure. */
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
    bool have_sample;
    bool black;
    bool rearm_requested;
} s3_barcode_t;

bool s3_barcode_config_valid(const s3_barcode_config_t *p_config);
void s3_barcode_init(s3_barcode_t *p_barcode);
/* Abort partial capture on pause/fault. Completed/partial markers remain held
 * until the controller explicitly confirms physical marker clearance. */
void s3_barcode_abort(s3_barcode_t *p_barcode);
void s3_barcode_rearm(s3_barcode_t *p_barcode);
s3_barcode_status_t s3_barcode_feed(s3_barcode_t *p_barcode,
                                   const s3_barcode_config_t *p_config,
                                   bool black, uint32_t now_ms,
                                   s3_symbol_t *p_symbol);
/* Widths may be time or monotonic distance units. Exactly two bounded passes
 * (forward/reverse) over the saved data; no physical rescan or allocation. */
s3_barcode_status_t s3_barcode_decode(const uint32_t *p_runs, uint32_t count,
                                     s3_symbol_t *p_symbol);

#endif
