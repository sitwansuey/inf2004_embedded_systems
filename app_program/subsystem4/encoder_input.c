#include "encoder_input.h"

static bool aligned(uint32_t a, uint32_t b)
{
    return a - b <= S4_ENCODER_MAX_SKEW_MS ||
           b - a <= S4_ENCODER_MAX_SKEW_MS;
}

void s4_encoder_terrain_step(s4_encoder_input_t *s, terrain_t *t,
    const odom_config_t *c, uint32_t now, const uint8_t *payload,
    size_t length, bool available, uint64_t imu_us, const int32_t a[3],
    bool av, bool calibrated, int pzero, int rzero)
{
    edr_encoder_t encoder;
    uint32_t imu_ms = (uint32_t)(imu_us / 1000U);
    bool accepted = false;
    bool priming = false;
    bool forward = true;
    if (!s || !t || !a) return;
    s->fault = S4_INPUT_OK;
    if (!available) s->fault = S4_INPUT_UNAVAILABLE;
    else if (!edr_encoder_decode(payload, length, &encoder))
        s->fault = S4_INPUT_FORMAT;
    else if (!(encoder.flags & EDR_ENCODER_VALID))
        s->fault = S4_INPUT_UNAVAILABLE;
    else if (!edr_age_is_fresh(now, encoder.sampled_ms, S4_ENCODER_MAX_AGE_MS))
        s->fault = S4_INPUT_STALE;
    else {
        odom_t *o = &s->odometry;
        uint32_t dt = encoder.sampled_ms - o->time_ms;
        if (o->initialized && dt == 0U) {
            /* A duplicate is neither new distance nor an error. Counts must
             * agree; never divide by zero or treat changed data as a repeat. */
            if (o->left != encoder.left_counts || o->right != encoder.right_counts)
                s->fault = S4_INPUT_HISTORY;
        } else if (o->initialized && dt > S4_ENCODER_MAX_GAP_MS) {
            s->fault = S4_INPUT_HISTORY;
        } else {
            int32_t dl = 0, dr = 0;
            if (o->initialized &&
                (!edr_count_delta(encoder.left_counts, o->left, &dl) ||
                 !edr_count_delta(encoder.right_counts, o->right, &dr) ||
                 dl < 0 || dr < 0)) forward = false;
            odom_step(o, c, encoder.sampled_ms, encoder.left_counts,
                      encoder.right_counts, true);
            if (!o->valid) {
                s->fault = S4_INPUT_HISTORY;
                priming = o->initialized;
            }
        }
        /* An IMU fault does not discard otherwise usable vehicle odometry. */
        accepted = s->fault == S4_INPUT_OK;
        if (accepted && (!av || !edr_age_is_fresh(now, imu_ms, 100U) ||
                         !aligned(encoder.sampled_ms, imu_ms)))
            s->fault = S4_INPUT_SKEW;
    }
    if (!accepted) {
        /* Retain the cumulative distance; a new good record establishes a
         * baseline. Do not bridge an unknown interval with old counts. */
        s->odometry.valid = 0;
        s->odometry.speed_mm_s = 0;
        s->odometry.delta_um = 0;
        /* odom_step's first usable record already establishes its baseline. */
        if (!priming) s->odometry.initialized = 0;
    }
    terrain_step(t, imu_ms, a,
        av && s->fault == S4_INPUT_OK, s->odometry.distance_um,
        s->odometry.valid, calibrated, pzero, rzero);
    /* Includes reversing between two polls of the same cached IMU sample.
     * Navigation may still use valid odometry and tilt while height recovers. */
    if (!forward) terrain_invalidate_height(t);
}
