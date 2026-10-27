#ifndef S4_ENCODER_INPUT_H
#define S4_ENCODER_INPUT_H
#include "../shared/edr.h"
#include "../subsystem2/odometry.h"
#include "terrain.h"

/* Initial engineering assumptions, not measured timing guarantees. */
#ifndef S4_ENCODER_MAX_AGE_MS
#define S4_ENCODER_MAX_AGE_MS 60U
#endif
#ifndef S4_ENCODER_MAX_SKEW_MS
#define S4_ENCODER_MAX_SKEW_MS 30U
#endif
#ifndef S4_ENCODER_MAX_GAP_MS
#define S4_ENCODER_MAX_GAP_MS 60U
#endif
#if S4_ENCODER_MAX_AGE_MS >= 0x80000000U || S4_ENCODER_MAX_SKEW_MS >= 0x80000000U || S4_ENCODER_MAX_GAP_MS == 0 || S4_ENCODER_MAX_GAP_MS >= 0x80000000U
#error Encoder freshness bounds must fit unambiguous modular milliseconds
#endif

enum {
    S4_INPUT_OK, S4_INPUT_UNAVAILABLE, S4_INPUT_FORMAT, S4_INPUT_STALE,
    S4_INPUT_SKEW, S4_INPUT_HISTORY
};
typedef struct {
    odom_t odometry;
    uint8_t fault : 3;
} s4_encoder_input_t;

/* Controller-owned state. Exact 13-byte EDR payload, not a 15-byte envelope
 * or native struct. Both source timestamps MUST be operating milliseconds.
 * Call once per controller tick, including errors. Invalid input abandons
 * the active traversal and interval history but retains the run maximum.
 * imu_time_us is the 64-bit millisecond clock scaled by 1000 by S4.
 */
void s4_encoder_terrain_step(s4_encoder_input_t *input, terrain_t *terrain,
    const odom_config_t *calibration, uint32_t now_ms,
    const uint8_t *payload, size_t length, bool available,
    uint64_t imu_time_us, const int32_t acceleration_mg[3], bool accel_valid,
    bool mounting_calibrated, int pitch_zero, int roll_zero);
#endif
