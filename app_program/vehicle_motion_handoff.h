#ifndef VEHICLE_MOTION_HANDOFF_H
#define VEHICLE_MOTION_HANDOFF_H
#include <stdbool.h>
/* Stopped checks and real servo pans require a stop. An already-forward
 * sample can carry a forward-only avoidance phase change without braking. */
static inline bool
vehicle_sensor_pan_requires_stop (unsigned command, bool copied,
                                  int current_angle, int sampled_angle,
                                  int requested_angle)
{
    return command == 0U || !copied || current_angle != requested_angle ||
           sampled_angle != requested_angle;
}
#endif

/*** end of file ***/
