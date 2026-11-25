/* Geometry for a stationary object near a plane perpendicular to forward. */
#ifndef S5_WIDTH_H
#define S5_WIDTH_H
#include "subsystem5.h"
/* cos(0..90 degrees) x 10000, shared by the width and path geometry. */
IMPORT const UW s5_cosine_q10000[91];
IMPORT BOOL s5_target_detected(const s5_status_t *sample);
IMPORT void s5_width_from_bounds(s5_status_t *result);
/* The width to REPORT (telemetry, event log), from the same bounds: each edge
 * angle less S5_BEAM_HALF_DEG (floored at 0), times the obstacle depth. Both
 * halves 0 unless object_width_valid. The plan never uses it. */
IMPORT void s5_width_reported(const s5_status_t *scan, UW *left_mm, UW *right_mm);
#endif
