/* Public path assessment, kept separate from hardware. */
#include "s5_policy.h"
static const UW cosine_q10000[] = {
        10000U, 9998U, 9994U, 9986U, 9976U, 9962U, 9945U, 9925U, 9903U, 9877U,
        9848U, 9816U, 9781U, 9744U, 9703U, 9659U, 9613U, 9563U, 9511U, 9455U,
        9397U, 9336U, 9272U, 9205U, 9135U, 9063U, 8988U, 8910U, 8829U, 8746U,
        8660U, 8572U, 8480U, 8387U, 8290U, 8192U, 8090U, 7986U, 7880U, 7771U,
        7660U, 7547U, 7431U, 7314U, 7193U, 7071U, 6947U, 6820U, 6691U, 6561U,
        6428U, 6293U, 6157U, 6018U, 5878U, 5736U, 5592U, 5446U, 5299U, 5150U,
        5000U, 4848U, 4695U, 4540U, 4384U, 4226U, 4067U, 3907U, 3746U, 3584U,
        3420U, 3256U, 3090U, 2924U, 2756U, 2588U, 2419U, 2250U, 2079U, 1908U,
        1736U, 1564U, 1392U, 1219U, 1045U, 872U, 698U, 523U, 349U, 175U,
        0U
    };
/* In the code's positive-left convention, each outward tangent magnitude
 * is atan2(abs(x), y + offset) + asin(radius / hypot(x, y + offset)).
 * Search whole degrees using the equivalent perpendicular clearance test:
 * (y + offset) sin(theta) - abs(x) cos(theta) >= radius.
 * This avoids floating-point libraries and rounds outward for clearance. */
static INT edge_clearance_heading(INT angle, UW range_mm)
{
    INT magnitude = angle < 0 ? -angle : angle;
    if (magnitude > S5_MAX_ANGLE || range_mm == 0 || range_mm > 5000) return 0;
    unsigned long long x = (unsigned long long)range_mm * cosine_q10000[90-magnitude];
    unsigned long long y = (unsigned long long)range_mm * cosine_q10000[magnitude] +
        (unsigned long long)S5_SENSOR_OFFSET_MM * 10000U;
    for (INT degree = 1; degree < 90; ++degree) {
        long long clearance = (long long)(y * cosine_q10000[90-degree]) -
                              (long long)(x * cosine_q10000[degree]);
        if (clearance >= (long long)S5_CLEARANCE_RADIUS_MM * 100000000LL) return degree;
    }
    return 0;
}

static void approach_geometry(s5_path_t *path, UW depth, UINT degree)
{
    path->approach_mm = depth;
    path->return_mm = (UW)(((unsigned long long)depth * cosine_q10000[90U-degree] + 9999U) / 10000U);
}

/* Use both averaged edge ranges to choose the smaller feasible tangent. */
EXPORT BOOL s5_acquisition_plan(s5_status_t *sensor)
{
    if (!sensor) return FALSE;
    sensor->approach_plan_valid = FALSE;
    sensor->planned_rotate_deg = 0;
    sensor->planned_rotate_ms = sensor->planned_approach_mm = sensor->planned_return_mm = 0;
    if (!sensor->object_width_valid || !sensor->left_bound_valid ||
        !sensor->right_bound_valid || sensor->obstacle_distance_mm == 0U ||
        sensor->obstacle_distance_mm > S5_OBSTACLE_MM ||
        sensor->avoidance_deg < -S5_MAX_ANGLE || sensor->avoidance_deg > S5_MAX_ANGLE)
        return FALSE;
    INT left = edge_clearance_heading(sensor->left_bound_deg, sensor->left_bound_distance_mm);
    INT right = edge_clearance_heading(sensor->right_bound_deg, sensor->right_bound_distance_mm);
    if (!left && !right) return FALSE;
    BOOL choose_left = left && (!right || left <= right);
    INT magnitude = choose_left ? left : right;
    sensor->avoidance_deg = choose_left ? sensor->left_bound_deg + 1 : sensor->right_bound_deg - 1;
    s5_path_t plan = {0};
    approach_geometry(&plan, sensor->obstacle_distance_mm, (UINT)magnitude);
    sensor->planned_rotate_deg = choose_left ? magnitude : -magnitude;
    sensor->planned_rotate_ms = (UW)magnitude * S5_ROTATE_MS_PER_DEG;
    sensor->planned_approach_mm = plan.approach_mm;
    sensor->planned_return_mm = plan.return_mm;
    sensor->approach_plan_valid = TRUE;
    return TRUE;
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
    else if (sensor->valid || (sensor->state == S5_READY && sensor->acquisition_clear))
    {
        if (sensor->state == S5_MONITOR && sensor->angle_deg == 0 &&
            sensor->distance_mm > S5_OBSTACLE_MM)
            path.state = S5_PATH_FORWARD;
        else if (sensor->state == S5_GUIDING && sensor->angle_deg == 0 &&
                 sensor->distance_mm > S5_GUIDANCE_OBSTACLE_MM)
            path.state = sensor->distance_mm > S5_OBSTACLE_MM ? S5_PATH_FORWARD : S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_READY && sensor->approach_plan_valid &&
                 sensor->object_width_valid && sensor->left_bound_valid && sensor->right_bound_valid)
        {
            path.state = S5_PATH_READY;
            path.turn_deg = sensor->planned_rotate_deg;
            path.turn_ms = sensor->planned_rotate_ms;
            path.approach_mm = sensor->planned_approach_mm;
            path.return_mm = sensor->planned_return_mm;
        }
        else if ((sensor->state == S5_APPROACH || sensor->state == S5_REORG) &&
                 sensor->angle_deg == 0 && sensor->sample_angle_deg == 0 &&
                 sensor->distance_mm > S5_GUIDANCE_OBSTACLE_MM)
            path.state = S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_OVERCOME &&
                 ((sensor->angle_deg == -90 || sensor->angle_deg == 90) ||
                  (sensor->angle_deg == 0 && sensor->distance_mm > S5_GUIDANCE_OBSTACLE_MM)) &&
                 sensor->sample_angle_deg == sensor->angle_deg)
            path.state = S5_PATH_GUIDE_FORWARD;
        else if (sensor->state == S5_ROTATING && sensor->angle_deg == 0 &&
                 sensor->distance_mm > S5_ROTATE_STOP_MM)
            path.state = S5_PATH_ROTATE_CLEAR;
    }
    return path;
}

