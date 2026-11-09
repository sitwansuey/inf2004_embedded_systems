#ifndef VEHICLE_BARCODE_RETRY_H
#define VEHICLE_BARCODE_RETRY_H
#include <stdint.h>
#include <stdbool.h>
/* When does the barcode re-read stop reversing, and what comes next?
 *
 * Counting all 29 elements does not end the reverse: a noise edge can make
 * the count early, before the first bar, and stopping right at the marker
 * leaves the slow pass no run-up. Keep reversing until S3's read has ended
 * (BOARD_BARCODE_END_GAP_MM after the last edge) and a further run-up, so a
 * slow final pass starts before the marker. A failed (or stalled) reverse
 * read is only final after REVERSE_MIN_MS; the distance bound always stops. */
enum { BR_NEXT_STAY, BR_NEXT_RESUME, BR_NEXT_WAIT, BR_NEXT_SLOW };
typedef struct {
    bool hit, miss;          /* S3 ok / fail counter changed this tick */
    bool full;               /* reading with >= 29 elements counted */
    bool reading;            /* S3 capture still running */
    bool odom_valid;
    bool min_done;           /* reversed for REVERSE_MIN_MS */
    bool quiet;              /* barcode state unchanged for IDLE_MS */
    uint32_t moved_um;       /* reversed so far */
    uint32_t since_edge_um;  /* travelled since the last GP28 edge */
} br_reverse_input_t;
/* *full_seen / *miss_seen latch for this reverse pass (clear on entry). */
static inline unsigned br_reverse_next(const br_reverse_input_t *in, bool *full_seen, bool *miss_seen,
    uint32_t gap_mm, uint32_t runup_mm, uint32_t max_mm)
{
    if (in->full) *full_seen = true;
    if (in->miss) *miss_seen = true;
    if (in->hit) return BR_NEXT_RESUME;
    if (in->moved_um >= max_mm * 1000U) return BR_NEXT_SLOW;
    bool beyond = !in->reading && (*full_seen || *miss_seen) && in->odom_valid &&
                  in->since_edge_um >= (gap_mm + runup_mm) * 1000U;
    if (beyond && (*full_seen || in->min_done)) return *miss_seen ? BR_NEXT_SLOW : BR_NEXT_WAIT;
    if (!*full_seen && !*miss_seen && in->min_done && in->quiet) return BR_NEXT_SLOW; /* never saw it */
    return BR_NEXT_STAY;
}
#endif
