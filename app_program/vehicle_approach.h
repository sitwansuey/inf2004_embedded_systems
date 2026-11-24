#ifndef VEHICLE_APPROACH_H
#define VEHICLE_APPROACH_H
#include <stdbool.h>
#include <stdint.h>
/* Obstacle approach: detect early, manoeuvre from about 20 cm.
 * VC_DETECT_SAMPLES new forward echoes in a row within VC_DETECT_MM while
 * tracking latch the obstacle's range. The robot follows the line on for
 * that range less stop_mm (odometry), then holds still so S5 starts its scan
 * from there (its own trigger: S5_ACQUISITION_SAMPLES echoes within
 * S5_OBSTACLE_MM). If S5 has not started within hold_ms, it follows on
 * (S5's trigger still stops it), and the approach ends slack_mm past the
 * stop point (no obstacle there). One approach per obstacle: another needs
 * a far or open forward sample first. Leaving VC_TRACK (S5's scan, a fault,
 * a STOP) ends it. */
#ifndef VC_DETECT_MM
#define VC_DETECT_MM 300U
#endif
#ifndef VC_DETECT_SAMPLES
#define VC_DETECT_SAMPLES 2U
#endif
#ifndef VC_APPROACH_STOP_MM
#define VC_APPROACH_STOP_MM 200U /* the manoeuvre distance */
#endif
#ifndef VC_APPROACH_BRAKE_MM
#define VC_APPROACH_BRAKE_MM 20U /* sample age and braking: stop this much early */
#endif
#ifndef VC_APPROACH_HOLD_MS
#define VC_APPROACH_HOLD_MS 1500U /* ~20 sonar samples standing still */
#endif
#ifndef VC_APPROACH_SLACK_MM
#define VC_APPROACH_SLACK_MM 150U
#endif
enum { APPROACH_NONE, APPROACH_DRIVE, APPROACH_HOLD };
typedef struct {
    int32_t from_um;   /* odometry at detection */
    uint32_t hold_ms;  /* hold start */
    uint16_t go_mm;    /* distance to drive before holding */
    uint16_t range_mm; /* range at detection */
    uint8_t on : 1, holding : 1, expired : 1, spent : 1;
} vehicle_approach_t;

/* One controller tick. tracking: armed, VC_TRACK, no barcode re-read.
 * detected: the detection count is complete with range_mm the newest echo.
 * far: a new forward sample beyond VC_DETECT_MM or open space. Returns
 * APPROACH_HOLD (stop this tick), APPROACH_DRIVE (approaching) or NONE. */
static inline unsigned vehicle_approach_step(vehicle_approach_t *a, bool tracking, bool detected,
    uint32_t range_mm, bool far, bool odom_valid, int32_t distance_um, uint32_t now,
    uint32_t stop_mm, uint32_t hold_ms, uint32_t slack_mm)
{
    if (!tracking) { *a = (vehicle_approach_t){0}; return APPROACH_NONE; }
    if (far && !a->on) a->spent = 0U;
    if (!a->on) {
        if (!detected || a->spent || !odom_valid) return APPROACH_NONE;
        a->on = 1U; a->spent = 1U; a->holding = 0U; a->expired = 0U;
        a->from_um = distance_um;
        a->range_mm = (uint16_t)(range_mm > 65535U ? 65535U : range_mm);
        a->go_mm = (uint16_t)(range_mm > stop_mm ? range_mm - stop_mm : 0U);
    }
    if (!odom_valid) return a->holding && !a->expired ? APPROACH_HOLD : APPROACH_DRIVE;
    int64_t moved_um = (int64_t)distance_um - a->from_um;
    uint32_t moved = moved_um > 0 ? (uint32_t)(moved_um / 1000) : 0U;
    if (moved > (uint32_t)a->go_mm + slack_mm) { a->on = 0U; return APPROACH_NONE; }
    if (!a->holding && moved >= a->go_mm) { a->holding = 1U; a->hold_ms = now; }
    if (a->holding && !a->expired) {
        if (now - a->hold_ms < hold_ms) return APPROACH_HOLD;
        a->expired = 1U;
    }
    return APPROACH_DRIVE;
}
#endif
