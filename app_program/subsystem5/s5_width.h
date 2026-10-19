/* Geometry for a stationary object near a plane perpendicular to forward. */
#ifndef S5_WIDTH_H
#define S5_WIDTH_H
#include "subsystem5.h"
IMPORT BOOL s5_target_detected(const s5_status_t *sample);
IMPORT void s5_width_from_bounds(s5_status_t *result);
#endif
