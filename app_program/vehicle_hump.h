#ifndef VEHICLE_HUMP_H
#define VEHICLE_HUMP_H
#include <math.h>
#include <stdbool.h>
#include <stdint.h>
/* Controller hump height for the event log: wheel distance x sin(pitch),
 * added up from where the robot left level ground. Subsystem 4's terrain
 * height starts only at a 5 deg climb and is dropped after a 60 ms timing
 * gap, so it often had no height for a 2-5 cm hump. This estimate uses the
 * same smoothed pitch and odometry:
 *   - at_base (the caller: no ramp in progress and the total tilt level)
 *     sets it back to 0, so a climb is counted from the 3 deg level limit;
 *   - each step adds the odometry distance moved (signed: reversing back
 *     down a slope takes the height off again) at the current pitch, or at
 *     the last valid pitch for up to stale_max steps;
 *   - the smoothed pitch lags the slope by the smoothing time lag_ms, which
 *     alone would put the top about 20% low on a 2-5 cm hump (it is passed
 *     while the pitch still reads a climb). For an exponential smoothing,
 *     the height still missing is speed x lag_ms x sin(pitch), added to
 *     height_um (exact at a steady speed);
 *   - peak_um keeps the highest point since the base;
 *   - it is lost until the next base after invalid odometry, a jump above
 *     max_step_um in one step, or a tilt invalid for more than stale_max
 *     steps.
 * vehicle_hump_up() is true while the robot is still up on a hump it has
 * climbed: the ramp handling then holds the crest speed cap (does not call
 * the hump over) until it is back down. */
#ifndef VC_HUMP_MAX_STEP_UM
#define VC_HUMP_MAX_STEP_UM 120000 /* as S4 terrain: more in one step is a jump */
#endif
#ifndef VC_HUMP_STALE_TICKS
#define VC_HUMP_STALE_TICKS 10U /* 200 ms of 20 ms controller ticks */
#endif
#ifndef VC_HUMP_SPEED_MS
#define VC_HUMP_SPEED_MS 200U /* speed smoothing (2 mm encoder steps) */
#endif
#ifndef VC_HUMP_MIN_UM
#define VC_HUMP_MIN_UM 5000 /* a lower climb is not a hump (as S4 terrain) */
#endif
#ifndef VC_HUMP_TOP_UM
#define VC_HUMP_TOP_UM 500000 /* a top this long without going down ends the hold */
#endif
#if VC_HUMP_STALE_TICKS > 255U || VC_HUMP_MAX_STEP_UM < 1 || VC_HUMP_MIN_UM < 1 || \
    VC_HUMP_TOP_UM < 1 || VC_HUMP_SPEED_MS < 1U
#error Invalid controller hump settings
#endif
typedef struct {
    int32_t raw_um;        /* distance x sin(smoothed pitch) since the base */
    int32_t height_um;     /* raw_um and the smoothing lag: the height now */
    int32_t peak_um;       /* highest height_um since the base */
    int32_t mark_um;       /* peak when since_peak_um restarted */
    int32_t last_um;       /* odometry distance at the previous step */
    int32_t pitch_cdeg;    /* last valid pitch */
    uint32_t last_ms;      /* time of the previous step */
    uint32_t since_peak_um;/* distance moved since the peak last rose 1 mm */
    float speed;           /* smoothed signed speed, um per ms */
    uint8_t stale;         /* steps at the last valid pitch */
    uint8_t primed : 1;    /* last_um and last_ms are set */
    uint8_t pitched : 1;   /* pitch_cdeg is set */
    uint8_t lost : 1;      /* the estimate is lost until the next base */
} vehicle_hump_t;

static inline void vehicle_hump_step(vehicle_hump_t *h, bool active, bool at_base,
    bool tilt_valid, int32_t pitch_cdeg, bool odom_valid, int32_t distance_um,
    uint32_t now_ms, uint32_t lag_ms, int32_t max_step_um, unsigned stale_max)
{
    if (!active) { *h = (vehicle_hump_t){0}; return; }
    if (tilt_valid) { h->pitch_cdeg = pitch_cdeg; h->pitched = 1U; h->stale = 0U; }
    if (at_base) {
        h->raw_um = h->height_um = h->peak_um = h->mark_um = 0;
        h->since_peak_um = 0U; h->lost = 0U;
    }
    if (!odom_valid) { h->primed = 0U; h->speed = 0.0f; if (!at_base) h->lost = 1U; return; }
    int64_t du = h->primed ? (int64_t)distance_um - h->last_um : 0;
    uint32_t dt = h->primed ? now_ms - h->last_ms : 0U;
    h->last_um = distance_um; h->last_ms = now_ms;
    bool first = !h->primed;
    h->primed = 1U;
    if (du > max_step_um || du < -(int64_t)max_step_um) {
        h->speed = 0.0f;
        if (!at_base) h->lost = 1U;
        return;
    }
    if (!first && dt) /* exponential smoothing over VC_HUMP_SPEED_MS */
        h->speed += (float)(((double)du / dt - h->speed) * dt / ((double)VC_HUMP_SPEED_MS + dt));
    if (at_base || h->lost) return;
    if (!tilt_valid) {
        if (!h->pitched || h->stale >= stale_max) { h->lost = 1U; return; }
        ++h->stale;
    }
    double s = sin((double)h->pitch_cdeg / 5729.577951308232);
    int64_t raw = (int64_t)h->raw_um + (int64_t)lround((double)du * s);
    int64_t height = raw + (int64_t)lround((double)h->speed * lag_ms * s);
    if (raw > 1000000 || raw < -1000000 || height > 1000000 || height < -1000000) {
        h->lost = 1U; return;
    }
    h->raw_um = (int32_t)raw; h->height_um = (int32_t)height;
    if (h->height_um > h->peak_um) h->peak_um = h->height_um;
    /* Still climbing (the peak rose 1 mm): the top starts again from here.
     * Pitch noise on a level top raises the peak far less than that. */
    if (h->peak_um >= h->mark_um + 1000) { h->mark_um = h->peak_um; h->since_peak_um = 0U; }
    else {
        uint64_t moved = (uint64_t)h->since_peak_um + (uint64_t)(du < 0 ? -du : du);
        h->since_peak_um = moved > UINT32_MAX ? UINT32_MAX : (uint32_t)moved;
    }
}

/* The highest point since the base in 2 mm units (rounded), or 0 when the
 * estimate is lost or there was no climb. */
static inline unsigned vehicle_hump_peak_2mm(const vehicle_hump_t *h)
{
    if (h->lost || h->peak_um <= 0) return 0U;
    return ((unsigned)h->peak_um + 1000U) / 2000U; /* at most 500: 1 m */
}

/* Still up on a hump: climbed at least min_um, not yet back down to half its
 * height, and moved less than top_um since it last rose 1 mm (a longer
 * level top, or an estimate that never comes back down, ends the hold). */
static inline bool vehicle_hump_up(const vehicle_hump_t *h, int32_t min_um, uint32_t top_um)
{
    return !h->lost && h->peak_um >= min_um && h->height_um > h->peak_um / 2 &&
           h->since_peak_um < top_um;
}
#endif
