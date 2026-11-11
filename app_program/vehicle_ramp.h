#ifndef VEHICLE_RAMP_H
#define VEHICLE_RAMP_H
#include <stdbool.h>
#include <stdint.h>
/* Accelerometer low-pass for tilt: a time-constant EMA over new samples
 * (identified by their read time), so motor vibration averages out of the
 * gravity direction. Samples whose magnitude is far from 1 g (vibration
 * spikes, jolts) are skipped. Output in milli-g, robot frame. */
typedef struct {
    int32_t f[3];      /* milli-g x16 */
    uint64_t last_us;
    uint8_t primed : 1;
} vehicle_accel_filter_t;

static inline void vehicle_accel_filter(vehicle_accel_filter_t *s, const int32_t a[3],
    uint64_t t_us, bool valid, uint32_t tau_ms, int32_t lo_mg, int32_t hi_mg, int32_t out[3])
{
    int64_t norm = (int64_t)a[0]*a[0] + (int64_t)a[1]*a[1] + (int64_t)a[2]*a[2];
    bool usable = valid && norm >= (int64_t)lo_mg*lo_mg && norm <= (int64_t)hi_mg*hi_mg;
    if (usable && (!s->primed || t_us != s->last_us)) {
        uint32_t dt = s->primed ? (uint32_t)((t_us - s->last_us) / 1000U) : 0U;
        if (!s->primed || dt >= tau_ms * 4U) {
            for (unsigned n = 0; n < 3U; ++n) s->f[n] = a[n] * 16;
        } else {
            if (!dt) dt = 1U;
            for (unsigned n = 0; n < 3U; ++n)
                s->f[n] += (int32_t)(((int64_t)(a[n] * 16 - s->f[n]) * dt) / (int64_t)(tau_ms + dt));
        }
        s->primed = 1U;
        s->last_us = t_us;
    }
    if (!s->primed) { for (unsigned n = 0; n < 3U; ++n) out[n] = a[n]; return; }
    for (unsigned n = 0; n < 3U; ++n) out[n] = (s->f[n] + (s->f[n] >= 0 ? 8 : -8)) / 16;
}

/* Ramp test: FLAT until the smoothed total tilt stays >= start for hold_ms
 * (UP), DOWN once it falls drop below its peak, DONE when it stays <= level
 * for level_ms after UP/DOWN. Inactive resets to FLAT; an invalid tilt
 * changes nothing. */
enum { RAMP_FLAT, RAMP_UP, RAMP_DOWN, RAMP_DONE };
typedef struct {
    uint32_t since_ms;
    int32_t peak;
    uint8_t state : 2;
    uint8_t timing : 1;
} vehicle_ramp_t;

static inline unsigned vehicle_ramp_step(vehicle_ramp_t *r, uint32_t now, bool active, bool valid,
    int32_t tilt, int32_t start, int32_t level, int32_t drop, uint32_t hold_ms, uint32_t level_ms)
{
    if (!active) { *r = (vehicle_ramp_t){0}; return RAMP_FLAT; }
    if (r->state == RAMP_DONE || !valid) return r->state;
    if (r->state == RAMP_FLAT) {
        if (tilt < start) r->timing = 0U;
        else if (!r->timing) { r->timing = 1U; r->since_ms = now; }
        else if (now - r->since_ms >= hold_ms) { r->state = RAMP_UP; r->peak = tilt; r->timing = 0U; }
        return r->state;
    }
    if (tilt > r->peak) r->peak = tilt;
    if (r->state == RAMP_UP && tilt <= r->peak - drop) r->state = RAMP_DOWN;
    if (tilt > level) r->timing = 0U;
    else if (!r->timing) { r->timing = 1U; r->since_ms = now; }
    else if (now - r->since_ms >= level_ms) { r->state = RAMP_DONE; r->timing = 0U; }
    return r->state;
}
#endif
