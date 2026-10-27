#ifndef CSC02_TERRAIN_H
#define CSC02_TERRAIN_H
#include <stdint.h>
#include <stdbool.h>
enum { TERRAIN_FLAT,TERRAIN_ASCENT,TERRAIN_CREST,TERRAIN_DESCENT,TERRAIN_SHOCK,TERRAIN_INVALID };
#ifndef TERRAIN_MAX_GAP_MS
#define TERRAIN_MAX_GAP_MS 60U
#endif
#ifndef TERRAIN_FLAT_CONFIRM_SAMPLES
#define TERRAIN_FLAT_CONFIRM_SAMPLES 5U
#endif
#ifndef TERRAIN_SLOPE_CDEG
#define TERRAIN_SLOPE_CDEG 500
#endif
#ifndef TERRAIN_FLAT_CDEG
#define TERRAIN_FLAT_CDEG 300
#endif
#ifndef TERRAIN_MAX_ROLL_CDEG
#define TERRAIN_MAX_ROLL_CDEG 1500
#endif
#ifndef TERRAIN_BASE_TOLERANCE_UM
#define TERRAIN_BASE_TOLERANCE_UM 20000
#endif
#ifndef TERRAIN_MIN_HUMP_UM
#define TERRAIN_MIN_HUMP_UM 5000
#endif
#if TERRAIN_FLAT_CONFIRM_SAMPLES < 1 || TERRAIN_FLAT_CONFIRM_SAMPLES > 255
#error Invalid flat confirmation sample count
#endif
#define TERRAIN_FLAT_COUNT_BITS (TERRAIN_FLAT_CONFIRM_SAMPLES<2?1: \
    TERRAIN_FLAT_CONFIRM_SAMPLES<4?2:TERRAIN_FLAT_CONFIRM_SAMPLES<8?3: \
    TERRAIN_FLAT_CONFIRM_SAMPLES<16?4:TERRAIN_FLAT_CONFIRM_SAMPLES<32?5: \
    TERRAIN_FLAT_CONFIRM_SAMPLES<64?6:TERRAIN_FLAT_CONFIRM_SAMPLES<128?7:8)
/* GCC packed storage is task-local. No pointer to a member is exposed; the
 * Cortex-M0+ compiler generates safe accesses for unaligned packed members.
 * This compiler layout is never used as a transport representation. */
typedef struct __attribute__((packed)) {
    int32_t previous_distance_um;
    uint32_t height_code : 20; /* height_um + 20000: 0..1020000 */
    uint32_t peak_um : 20; /* 0..1000000 */
    uint32_t sample_ms;
    uint32_t pitch_code : 15; /* pitch_cdeg + 9000 */
    uint32_t roll_code : 16; /* roll_cdeg + 18000 */
    uint32_t height_mm : 10;
    uint32_t maximum_mm : 10;
    uint8_t state : 3;
    uint8_t tilt_valid : 1;
    uint8_t height_valid : 1;
    uint8_t shock : 1;
    uint8_t initialized : 1;
    uint8_t flat_samples : TERRAIN_FLAT_COUNT_BITS;
} terrain_t;
static inline int32_t terrain_pitch_cdeg(const terrain_t *s) {return (int32_t)s->pitch_code-9000;}
static inline int32_t terrain_roll_cdeg(const terrain_t *s) {return (int32_t)s->roll_code-18000;}
static inline int32_t terrain_height_um(const terrain_t *s) {
    return (int32_t)s->height_code-20000;
}
void terrain_step(terrain_t *s,uint32_t sample_ms,const int32_t accel_mg[3],
                  bool accel_valid,int32_t distance_um,bool odometry_valid,
                  bool calibrated,int pitch_zero,int roll_zero);
/* Traversal continuity can fail while gravity-derived tilt remains usable. */
void terrain_invalidate_height(terrain_t *s);
#endif
