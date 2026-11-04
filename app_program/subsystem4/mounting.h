#ifndef S4_MOUNTING_H
#define S4_MOUNTING_H
#include <stdbool.h>
#include <stdint.h>
#include <math.h>

/* Compile-time board profiles; no additional persistent or wire fields.
 * Raw acquisition/telemetry always stays in the chip's sensor XYZ frame. */
enum { S4_MOUNT_X_FORWARD = 0, S4_MOUNT_NEG_Y_FORWARD = 1 };
#define S4_CDEG_PER_RAD 5729.577951308232

/* Transient calculation view. Permit aliasing; errors leave output untouched.
 * Negating the raw -2048 endpoint needs +2048, hence int32_t intermediates. */
static inline bool s4_body_acceleration(const int32_t sensor[3], unsigned mounting,
                                        int32_t body[3])
{
    if (!sensor || !body || mounting > S4_MOUNT_NEG_Y_FORWARD) return false;
    for (unsigned n = 0; n < 3; ++n)
        if (sensor[n] < -2048 || sensor[n] > 2047) return false;
    int32_t mapped[3] = {sensor[0], sensor[1], sensor[2]};
    if (mounting == S4_MOUNT_NEG_Y_FORWARD) {
        mapped[0] = -sensor[1];
        mapped[1] = sensor[0];
    }
    for (unsigned n = 0; n < 3; ++n) body[n] = mapped[n];
    return true;
}

/* Gravity-derived body tilt before mounting zero subtraction. Positive pitch
 * is nose-up; positive roll is left-side-up. This is not a calibrated attitude
 * or a gyro estimate. Callers separately enforce freshness and overrun flags. */
static inline bool s4_gravity_tilt(const int32_t body[3], int32_t *pitch_cdeg,
                                  int32_t *roll_cdeg)
{
    if (!body || !pitch_cdeg || !roll_cdeg || pitch_cdeg == roll_cdeg) return false;
    for (unsigned n = 0; n < 3; ++n)
        if (body[n] < -2048 || body[n] > 2048) return false;
    int64_t norm = (int64_t)body[0]*body[0] + (int64_t)body[1]*body[1] +
                   (int64_t)body[2]*body[2];
    if (norm < 800LL*800 || norm > 1200LL*1200 || body[2] <= 0) return false;
    int32_t pitch = (int32_t)lround(atan2((double)body[0],
        hypot((double)body[1], (double)body[2])) * S4_CDEG_PER_RAD);
    int32_t roll = (int32_t)lround(atan2((double)body[1], (double)body[2]) * S4_CDEG_PER_RAD);
    *pitch_cdeg = pitch;
    *roll_cdeg = roll;
    return true;
}
#endif
