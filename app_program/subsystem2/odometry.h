#ifndef CSC02_ODOMETRY_H
#define CSC02_ODOMETRY_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t ticks_per_rev, circumference_mm, track_mm;
} odom_config_t;
/* Owner-local GCC storage, never a wire/queue format. Wide config/API values
 * must be validated before packing; no addresses of packed members escape. */
typedef struct __attribute__((packed)) {
    uint32_t ticks_per_rev : 17; /* 1..100000; zero is unconfigured. */
    uint32_t circumference_mm : 11; /* 20..2000; zero is unconfigured. */
    uint32_t track_mm : 10; /* 40..1000; zero is unconfigured. */
} odom_calibration_t;
typedef struct __attribute__((packed)) {
    uint32_t time_ms, left, right;
    int32_t distance_um;
    /* Low two's-complement bits. Zero storage decodes to zero. */
    uint32_t speed_bits : 12; /* -2000..2000 mm/s */
    uint32_t delta_bits : 19; /* -200000..200000 um, interval <=100 ms */
    uint32_t turn_bits : 20; /* signed remainder -359999..359999 mdeg */
    uint8_t initialized : 1;
    uint8_t valid : 1;
} odom_t;
static inline int32_t odom_signed_value(uint32_t bits, unsigned width) {
    uint32_t sign = UINT32_C(1) << (width - 1U);
    return (int32_t)(bits ^ sign) - (int32_t)sign;
}
static inline int32_t odom_speed_mm_s(const odom_t *s) {
    return odom_signed_value(s->speed_bits,12U);
}
static inline int32_t odom_delta_um(const odom_t *s) {
    return odom_signed_value(s->delta_bits,19U);
}
static inline int32_t odom_turn_mdeg(const odom_t *s) {
    return odom_signed_value(s->turn_bits,20U);
}
static inline void odom_clear_motion(odom_t *s) {
    s->speed_bits=0; s->delta_bits=0;
}
bool odom_config_valid(const odom_config_t *c);
/* Failure leaves the caller's destination unchanged. Not portable codecs. */
bool odom_calibration_pack(const odom_config_t *c, odom_calibration_t *out);
bool odom_calibration_unpack(const odom_calibration_t *c, odom_config_t *out);
void odom_step(odom_t *s, const odom_config_t *c, uint32_t now,
               uint32_t left, uint32_t right, bool valid);
#endif
