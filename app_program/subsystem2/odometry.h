#ifndef CSC02_ODOMETRY_H
#define CSC02_ODOMETRY_H
#include <stdint.h>
#include <stdbool.h>
typedef struct {
    uint32_t ticks_per_rev, circumference_mm, track_mm;
} odom_config_t;
typedef struct {
    uint32_t time_ms, left, right;
    int32_t distance_um, speed_mm_s, delta_um, turn_mdeg;
    uint8_t initialized : 1;
    uint8_t valid : 1;
} odom_t;
bool odom_config_valid(const odom_config_t *c);
void odom_step(odom_t *s, const odom_config_t *c, uint32_t now,
               uint32_t left, uint32_t right, bool valid);
#endif
