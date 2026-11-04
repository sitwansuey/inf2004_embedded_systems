#ifndef S4_MOTION_H
#define S4_MOTION_H
#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include "../subsystem2/odometry.h"

enum { S4_MOTION_UNKNOWN, S4_MOTION_STATIONARY, S4_MOTION_MOVING,
       S4_MOTION_ACCELERATING, S4_MOTION_DECELERATING, S4_MOTION_TURNING };
enum { S4_EVENT_STATIONARY=1, S4_EVENT_ACCELERATING=2, S4_EVENT_TURNING=4,
       S4_EVENT_CLIMBING=8, S4_EVENT_DESCENDING=16, S4_EVENT_IMPACT=32 };
#define S4_MOTION_BITS 73U
#define S4_MOTION_BYTES 10U
#define S4_MOTION_TOPIC "Robopico/sensors/buddy4/motion/v1"
#define S4_MOTION_ODOMETRY 1U
#define S4_MOTION_RATE 2U
#ifndef S4_MOTION_CONFIRM_MS
#define S4_MOTION_CONFIRM_MS 100U
#endif
#ifndef S4_MOTION_ACCEL_MM_S2
#define S4_MOTION_ACCEL_MM_S2 200
#endif
#ifndef S4_MOTION_TURN_DPS
#define S4_MOTION_TURN_DPS 10
#endif
#if S4_MOTION_CONFIRM_MS < 20 || S4_MOTION_CONFIRM_MS > 1000 || \
    S4_MOTION_ACCEL_MM_S2 < 1 || S4_MOTION_ACCEL_MM_S2 > 50000 || \
    S4_MOTION_TURN_DPS < 6 || S4_MOTION_TURN_DPS > 6000
#error Invalid motion confirmation thresholds
#endif
/* Owner-local history. No compiler layout crosses a task/network boundary.
 * Events are sticky since RESET; revision wraps modulo 256; humps saturate. */
typedef struct __attribute__((packed)) {
    uint32_t sampled_ms, candidate_ms, acceleration_ms;
    uint32_t yaw_bits:20;
    uint32_t rate_code:14; /* signed dps + 6000, valid -6000..6000 */
    uint16_t acceleration_speed_bits:12;
    uint8_t motion:3, candidate:3, initialized:1, rate_valid:1;
    uint8_t events:6, height_seen:1;
    uint8_t impact_sequence:4, previous_terrain:3;
    uint8_t revision, humps;
} s4_motion_t;
static inline bool s4_motion_impact(const s4_motion_t *s) {
    return (s->events&S4_EVENT_IMPACT)!=0U;
}
typedef struct {
    uint32_t sampled_ms;
    int32_t turn_rate_dps;
    uint8_t motion, flags, events, revision, humps;
} s4_motion_result_t;
void s4_motion_step(s4_motion_t *s,uint32_t now,const odom_t *odometry,
                    uint8_t terrain,bool height_valid,uint8_t impact_sequence);
void s4_motion_snapshot(const s4_motion_t *s,uint32_t now,s4_motion_result_t *out);
bool s4_motion_encode(const s4_motion_result_t *v,uint8_t *out,size_t capacity);
bool s4_motion_decode(const uint8_t *data,size_t length,s4_motion_result_t *out);
#endif
