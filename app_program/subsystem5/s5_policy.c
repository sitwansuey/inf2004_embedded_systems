/* Public path assessment, kept separate from hardware. */
#include "s5_policy.h"
#include "s5_width.h"
/* In the code's positive-left convention, each outward tangent magnitude
 * is atan2(abs(x), y + offset) + asin(radius / hypot(x, y + offset)).
 * Search whole degrees using the equivalent perpendicular clearance test:
 * (y + offset) sin(theta) - abs(x) cos(theta) >= radius.
 * This avoids floating-point libraries and rounds outward for clearance. */
static INT edge_clearance_heading(INT angle, UW range_mm)
{
    INT magnitude = angle < 0 ? -angle : angle;
    if (magnitude > S5_MAX_ANGLE || range_mm == 0 || range_mm > 5000) return 0;
    unsigned long long x = (unsigned long long)range_mm * s5_cosine_q10000[90-magnitude];
    unsigned long long y = (unsigned long long)range_mm * s5_cosine_q10000[magnitude] +
        (unsigned long long)S5_SENSOR_OFFSET_MM * 10000U;
    for (INT degree = 1; degree < 90; ++degree) {
        long long clearance = (long long)(y * s5_cosine_q10000[90-degree]) -
                              (long long)(x * s5_cosine_q10000[degree]);
        long long arc = (long long)S5_BYPASS_ARC_RADIUS_MM *
                        (10000U-s5_cosine_q10000[degree]) * 10000LL;
        if (clearance-arc >= (long long)S5_CLEARANCE_RADIUS_MM * 100000000LL) return degree;
    }
    return 0;
}

EXPORT UW s5_plan_lateral_mm(const s5_status_t *sensor, BOOL left)
{
    if(!sensor || !sensor->object_width_valid || sensor->object_width_max_mm>AVOID_MAX_WIDTH_MM)return 0;
    INT bound=left?sensor->left_bound_deg:-sensor->right_bound_deg;
    UW range=left?sensor->left_bound_distance_mm:sensor->right_bound_distance_mm;
    if(bound<0 || bound>80 || !range || range>5000U)return 0;
    UW edge=range*s5_cosine_q10000[90-(UINT)bound]/10000U;
    UW half=(sensor->object_width_max_mm+1U)/2U;
    return (edge>half?edge:half)+S5_CLEARANCE_RADIUS_MM;
}

EXPORT BOOL s5_corridor_blocked(const s5_status_t *sample, UW lateral_mm, UW ahead_mm)
{
    if(!sample || !sample->valid || !sample->distance_mm || sample->distance_mm>5000U)return FALSE;
    INT magnitude=sample->sample_angle_deg<0?-sample->sample_angle_deg:sample->sample_angle_deg;
    if(magnitude>90)return FALSE;
    UW lateral=sample->distance_mm*s5_cosine_q10000[90-magnitude]/10000U;
    UW ahead=sample->distance_mm*s5_cosine_q10000[magnitude]/10000U;
    return ahead<=ahead_mm && lateral<=lateral_mm+S5_SIDE_HALF_WIDTH_MM;
}

static BOOL approach_geometry(s5_path_t *path,const s5_status_t *sensor,UINT degree,BOOL left)
{
    if(!degree || degree>80U)return FALSE;
    UW lateral=s5_plan_lateral_mm(sensor,left);
    if(!lateral)return FALSE;
    UW arc=2U*S5_BYPASS_ARC_RADIUS_MM*(10000U-s5_cosine_q10000[degree])/10000U;
    UW remaining=lateral>arc?lateral-arc:0U;
    UW sine=s5_cosine_q10000[90U-degree];
    if(!sine)return FALSE;
    UW leg=(remaining*10000U+sine-1U)/sine;
    if(leg<50U)leg=50U;
    if(leg>1000U)return FALSE;
    path->approach_mm=path->return_mm=leg;
    return TRUE;
}

/* Use both averaged edge ranges to choose the shorter feasible detour.
 * side: 0 chooses, +1 forces LEFT, -1 forces RIGHT (fails if infeasible). */
EXPORT BOOL s5_acquisition_plan_side(s5_status_t *sensor, INT side)
{
    if (!sensor) return FALSE;
    sensor->approach_plan_valid = FALSE;
    sensor->planned_rotate_deg = 0;
    sensor->planned_rotate_ms = sensor->planned_approach_mm = sensor->planned_return_mm = 0;
    s5_width_from_bounds(sensor);
    if(!sensor->object_width_valid)return FALSE;
    if ((!sensor->left_bound_valid && !sensor->right_bound_valid) ||
        sensor->obstacle_distance_mm == 0U ||
        sensor->obstacle_distance_mm > S5_OBSTACLE_MM ||
        sensor->avoidance_deg < -S5_MAX_ANGLE || sensor->avoidance_deg > S5_MAX_ANGLE)
        return FALSE;
    INT left = sensor->left_bound_valid ? edge_clearance_heading(sensor->left_bound_deg, sensor->left_bound_distance_mm) : 0;
    INT right = sensor->right_bound_valid ? edge_clearance_heading(sensor->right_bound_deg, sensor->right_bound_distance_mm) : 0;
    if(left>80)left=0;
    if(right>80)right=0;
    if (side > 0) right = 0;
    if (side < 0) left = 0;
    if (!left && !right) return FALSE;
    /* Shorter detour first: smaller centre-to-edge half; ties keep the
     * smaller heading, then LEFT. */
    BOOL choose_left = left && (!right ||
        sensor->object_left_mm < sensor->object_right_mm ||
        (sensor->object_left_mm == sensor->object_right_mm && left <= right));
    INT magnitude = choose_left ? left : right;
    sensor->avoidance_deg = choose_left ? sensor->left_bound_deg + 1 : sensor->right_bound_deg - 1;
    s5_path_t plan = {0};
    if(!approach_geometry(&plan,sensor,(UINT)magnitude,choose_left))return FALSE;
    sensor->planned_rotate_deg = choose_left ? magnitude : -magnitude;
    sensor->planned_rotate_ms = (UW)magnitude * S5_ROTATE_MS_PER_DEG;
    sensor->planned_approach_mm = plan.approach_mm;
    sensor->planned_return_mm = plan.return_mm;
    sensor->approach_plan_valid = TRUE;
    return TRUE;
}
EXPORT BOOL s5_acquisition_plan(s5_status_t *sensor)
{
    return s5_acquisition_plan_side(sensor, 0);
}

/* Pure assessment of the copied sensor state: no motor access. */
EXPORT s5_path_t s5_assess_path(const s5_status_t *sensor, BOOL available, UW now_ms)
{
    s5_path_t path = {.state = S5_PATH_UNAVAILABLE};
    if (!sensor || !available || now_ms - sensor->time_ms > S5_STALE_MS)
        return path;
    path.state = S5_PATH_STOP;
    if (sensor->state == S5_FAULT)
        path.state = S5_PATH_FAULT;
    else if (sensor->state == S5_SCAN_LEFT || sensor->state == S5_REFINE_LEFT ||
             sensor->state == S5_SCAN_RIGHT || sensor->state == S5_REFINE_RIGHT ||
             sensor->state == S5_VERIFY_CLEAR || sensor->state == S5_BLOCKED)
        path.state = S5_PATH_IDENTIFY;
    else if (sensor->valid || s5_sample_is_open(sensor))
    {
        if (sensor->state == S5_MONITOR && sensor->angle_deg == 0 && sensor->sample_angle_deg == 0 &&
            s5_direction_clear(sensor,S5_OBSTACLE_MM))
            path.state = S5_PATH_FORWARD;
        else if (sensor->state == S5_GUIDING && sensor->angle_deg == 0 &&
                 s5_direction_clear(sensor,S5_GUIDANCE_OBSTACLE_MM))
            path.state = s5_direction_clear(sensor,S5_OBSTACLE_MM) ? S5_PATH_FORWARD : S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_READY && sensor->approach_plan_valid &&
                 (sensor->left_bound_valid || sensor->right_bound_valid))
        {
            path.state = S5_PATH_READY;
            path.turn_deg = sensor->planned_rotate_deg;
            path.turn_ms = sensor->planned_rotate_ms;
            path.approach_mm = sensor->planned_approach_mm;
            path.return_mm = sensor->planned_return_mm;
        }
        else if ((sensor->state == S5_APPROACH || sensor->state == S5_REORG) &&
                 sensor->angle_deg == 0 && sensor->sample_angle_deg == 0 &&
                 s5_direction_clear(sensor,S5_GUIDANCE_OBSTACLE_MM))
            path.state = S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_OVERCOME &&
                 ((sensor->angle_deg == -90 || sensor->angle_deg == 90) ||
                  (sensor->angle_deg == 0 && s5_direction_clear(sensor,S5_GUIDANCE_OBSTACLE_MM))) &&
                 sensor->sample_angle_deg == sensor->angle_deg)
            path.state = S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_ROTATING && sensor->angle_deg == 0 &&
                 s5_direction_clear(sensor,S5_ROTATE_STOP_MM))
            path.state = S5_PATH_ROTATE_CLEAR;
    }
    return path;
}
