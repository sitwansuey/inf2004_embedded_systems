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
typedef struct {
    int32_t height_um, peak_um, previous_distance_um;
    uint32_t sample_ms;
    int16_t pitch_cdeg, roll_cdeg;
    uint16_t height_mm, maximum_mm;
    uint8_t state : 3;
    uint8_t tilt_valid : 1;
    uint8_t height_valid : 1;
    uint8_t shock : 1;
    uint8_t initialized : 1;
    uint8_t flat_samples;
} terrain_t;
void terrain_step(terrain_t *s,uint32_t sample_ms,const int32_t accel_mg[3],
                  bool accel_valid,int32_t distance_um,bool odometry_valid,
                  bool calibrated,int pitch_zero,int roll_zero);
/* Traversal continuity can fail while gravity-derived tilt remains usable. */
void terrain_invalidate_height(terrain_t *s);
#endif
