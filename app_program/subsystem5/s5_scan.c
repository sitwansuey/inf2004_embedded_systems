/* Pure acquisition decisions: coarse outward search, fine inward bounds. */
#include "s5_scan.h"
#include "s5_policy.h"

static INT scan_blocked(s5_status_t *next)
{
    next->approach_plan_valid = FALSE;
    next->state = S5_BLOCKED;
    next->avoidance_deg = 0;
    next->object_width_valid = FALSE;
    return 0;
}

static INT scan_begin(s5_status_t *next, s5_scan_context_t *scan)
{
    if (next->distance_mm > S5_OBSTACLE_MM || next->angle_deg != 0 ||
        next->sample_angle_deg != 0)
    {
        scan->acquisition_samples = 0;
        return 0;
    }
    if (++scan->acquisition_samples < S5_ACQUISITION_SAMPLES) return 0;
    ++next->scan_id;
    next->obstacle_distance_mm = next->distance_mm;
    *scan = (s5_scan_context_t){0};
    next->approach_plan_valid = FALSE;
    next->avoidance_deg = 0;
    next->left_bound_deg = next->right_bound_deg = 0;
    next->left_bound_distance_mm = next->right_bound_distance_mm = 0;
    next->left_bound_valid = next->right_bound_valid = FALSE;
    next->object_width_valid = FALSE;
    next->object_width_mm = next->object_width_min_mm = next->object_width_max_mm = 0;
    next->state = S5_SCAN_LEFT;
    return S5_SCAN_STEP_DEG;
}

static INT scan_left(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (!object)
    {
        scan->left_clear = next->angle_deg;
        next->state = S5_REFINE_LEFT;
        return next->angle_deg - S5_REFINE_STEP_DEG;
    }
    if (next->angle_deg >= S5_MAX_ANGLE) return scan_blocked(next);
    INT target = next->angle_deg + S5_SCAN_STEP_DEG;
    return target > S5_MAX_ANGLE ? S5_MAX_ANGLE : target;
}

/* The detection reading fixes the angle; collect three new valid readings
 * at that angle before publishing the bound and moving the servo again. */
static BOOL bound_average(s5_status_t *next, s5_scan_context_t *scan, UW *mean)
{
    if (!scan->averaging_bound) {
        scan->averaging_bound = TRUE;
        scan->bound_samples = scan->bound_sum_mm = 0;
        return FALSE;
    }
    scan->bound_sum_mm += next->distance_mm;
    if (++scan->bound_samples < 3U) return FALSE;
    *mean = (scan->bound_sum_mm + 1U) / 3U;
    scan->averaging_bound = FALSE;
    return TRUE;
}

static INT refine_left(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (object || scan->averaging_bound)
    {
        next->left_bound_deg = next->angle_deg;
        if (!bound_average(next, scan, &next->left_bound_distance_mm)) return next->angle_deg;
        next->left_bound_valid = TRUE;
        next->state = S5_SCAN_RIGHT;
        /* Mirrored left bound is only the starting guess for the right. */
        return -next->left_bound_deg;
    }
    scan->left_clear = next->angle_deg;
    if (next->angle_deg <= 0) return scan_blocked(next);
    return next->angle_deg - S5_REFINE_STEP_DEG;
}

static INT scan_right(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (!object)
    {
        scan->right_clear = next->angle_deg;
        next->state = S5_REFINE_RIGHT;
        return next->angle_deg + S5_REFINE_STEP_DEG;
    }
    if (next->angle_deg <= -S5_MAX_ANGLE) return scan_blocked(next);
    INT target = next->angle_deg - S5_SCAN_STEP_DEG;
    return target < -S5_MAX_ANGLE ? -S5_MAX_ANGLE : target;
}

static INT refine_right(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (object || scan->averaging_bound)
    {
        next->right_bound_deg = next->angle_deg;
        if (!bound_average(next, scan, &next->right_bound_distance_mm)) return next->angle_deg;
        next->right_bound_valid = TRUE;
        s5_width_from_bounds(next);
        if (!next->object_width_valid) return scan_blocked(next);
        /* Use the adjacent measured clear angle, not the object-hit bound.
         * Prefer left when both routes have equal clearing angles. */
        next->avoidance_deg = scan->left_clear <= -scan->right_clear ?
            scan->left_clear : scan->right_clear;
        next->state = S5_VERIFY_CLEAR;
        return next->avoidance_deg;
    }
    scan->right_clear = next->angle_deg;
    if (next->angle_deg >= 0) return scan_blocked(next);
    return next->angle_deg + S5_REFINE_STEP_DEG;
}

EXPORT INT s5_scan_step(s5_status_t *next, s5_scan_context_t *scan)
{
    next->acquisition_clear = FALSE;
    BOOL acquiring = next->state == S5_SCAN_LEFT || next->state == S5_REFINE_LEFT ||
        next->state == S5_SCAN_RIGHT || next->state == S5_REFINE_RIGHT ||
        next->state == S5_VERIFY_CLEAR || next->state == S5_READY;
    BOOL opening = FALSE;
    if (acquiring && !next->valid && next->echo_result != S5_ECHO_OK &&
        next->sample_angle_deg == next->angle_deg) {
        if (++scan->echo_failures < S5_SCAN_ECHO_FAILURES) return next->angle_deg;
        opening = !scan->averaging_bound &&
            (next->echo_result == S5_ECHO_NO_RISE || next->echo_result == S5_ECHO_NO_FALL);
    }
    if (next->sample_angle_deg != next->angle_deg ||
        (!opening && (!next->valid || next->distance_mm == 0U || next->distance_mm > 5000U)))
    {
        next->approach_plan_valid = FALSE;
        next->state = S5_FAULT;
        next->object_width_valid = FALSE;
        next->left_bound_valid = next->right_bound_valid = FALSE;
        next->avoidance_deg = 0;
        *scan = (s5_scan_context_t){0};
        return 0;
    }
    scan->echo_failures = 0;
    BOOL object = !opening && s5_target_detected(next);
    next->acquisition_clear = acquiring && !object;
    switch (next->state)
    {
    case S5_CENTERING:
        next->state = S5_MONITOR;
        return 0;
    case S5_FAULT:
        next->state = S5_MONITOR;
        /* fall through */
    case S5_MONITOR:
        return scan_begin(next, scan);
    case S5_SCAN_LEFT:
        return scan_left(next, scan, object);
    case S5_REFINE_LEFT:
        return refine_left(next, scan, object);
    case S5_SCAN_RIGHT:
        return scan_right(next, scan, object);
    case S5_REFINE_RIGHT:
        return refine_right(next, scan, object);
    case S5_VERIFY_CLEAR:
        if (object || (!opening && next->distance_mm <= S5_GUIDANCE_OBSTACLE_MM))
            return scan_blocked(next);
        if (!s5_acquisition_plan(next)) return scan_blocked(next);
        next->state = S5_READY;
        return next->angle_deg;
    case S5_READY:
        if (object || (!opening && next->distance_mm <= S5_GUIDANCE_OBSTACLE_MM))
            return scan_blocked(next);
        return next->angle_deg;
    case S5_GUIDING:
        if (next->distance_mm > S5_OBSTACLE_MM)
        {
            next->state = S5_MONITOR;
            next->avoidance_deg = 0;
            *scan = (s5_scan_context_t){0};
        }
        return next->angle_deg;
    case S5_APPROACH: case S5_OVERCOME: case S5_REORG: case S5_ROTATING:
        return next->angle_deg;
    case S5_BLOCKED:
        next->state = S5_MONITOR;
        next->avoidance_deg = 0;
        *scan = (s5_scan_context_t){0};
        return 0;
    default:
        next->approach_plan_valid = FALSE;
        next->state = S5_FAULT;
        next->valid = FALSE;
        next->object_width_valid = FALSE;
        next->left_bound_valid = next->right_bound_valid = FALSE;
        return 0;
    }
}
