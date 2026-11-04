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
    UINT restarts=scan->motion_restarts;
    *scan = (s5_scan_context_t){0};
    scan->motion_restarts=restarts;
    scan->position_recorded=next->encoder_valid;
    scan->origin_left=next->motor1_counts_end;scan->origin_right=next->motor2_counts_end;
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

/* Always sample 30,60,90 before fine checking the first measured opening. */
static INT finish_sides(s5_status_t *next, s5_scan_context_t *scan)
{
    s5_width_from_bounds(next); /* Remains invalid when only one edge is known. */
    if (!s5_acquisition_plan(next)) return scan_blocked(next);
    next->state=S5_VERIFY_CLEAR;
    scan->plan_pending=1;
    return next->planned_rotate_deg;
}
/* Keep a failed side unknown, and advance without restarting acquisition. */
static INT side_failed(s5_status_t *next, s5_scan_context_t *scan, BOOL left)
{
    scan->averaging_bound=FALSE;scan->fine_checks=0;scan->echo_failures=0;
    if(left) {
        next->left_bound_valid=FALSE;
        next->state=S5_SCAN_RIGHT;
        return -S5_SCAN_STEP_DEG;
    }
    next->right_bound_valid=FALSE;
    return finish_sides(next,scan);
}
static INT fine_begin(s5_status_t *next, s5_scan_context_t *scan, BOOL left)
{
    scan->fine_hit=0;
    scan->fine_clear=left?scan->left_clear:-scan->right_clear;
    scan->fine_checks=0;scan->averaging_bound=FALSE;
    next->state=left?S5_REFINE_LEFT:S5_REFINE_RIGHT;
    return (left?1:-1)*(scan->fine_clear/2);
}
static INT scan_left(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (!object && !scan->left_clear) scan->left_clear=next->angle_deg;
    if (next->angle_deg<S5_MAX_ANGLE) return next->angle_deg+S5_SCAN_STEP_DEG;
    if (scan->left_clear) {
        return fine_begin(next,scan,TRUE);
    }
    /* A blocked left side must not prevent assessing the right. */
    next->state=S5_SCAN_RIGHT;
    return -S5_SCAN_STEP_DEG;
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

/* Binary refinement reaches one-degree resolution in <=7 decisions,
 * instead of slowly walking up to 89 one-degree positions. */
static INT refine_side(s5_status_t *next, s5_scan_context_t *scan, BOOL object, BOOL left)
{
    INT magnitude=left?next->angle_deg:-next->angle_deg;
    if(scan->averaging_bound && !object)return side_failed(next,scan,left);
    if(!scan->averaging_bound) {
        if(++scan->fine_checks>10U)return side_failed(next,scan,left);
        if(object)scan->fine_hit=magnitude;else scan->fine_clear=magnitude;
        if(scan->fine_clear-scan->fine_hit>1)
            return (left?1:-1)*((scan->fine_clear+scan->fine_hit)/2);
        if(!object)return (left?1:-1)*scan->fine_hit;
    }
    UW mean=0;
    if(!bound_average(next,scan,&mean))return next->angle_deg;
    if(left) {
        next->left_bound_deg=next->angle_deg;
        next->left_bound_distance_mm=mean;next->left_bound_valid=TRUE;
        next->state=S5_SCAN_RIGHT;
        return -S5_SCAN_STEP_DEG;
    }
    next->right_bound_deg=next->angle_deg;
    next->right_bound_distance_mm=mean;next->right_bound_valid=TRUE;
    return finish_sides(next,scan);
}

static INT scan_right(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (!object && !scan->right_clear) scan->right_clear=next->angle_deg;
    if (next->angle_deg> -S5_MAX_ANGLE) return next->angle_deg-S5_SCAN_STEP_DEG;
    if (scan->right_clear) {
        return fine_begin(next,scan,FALSE);
    }
    /* The verified left edge remains usable when all right checks are blocked. */
    return finish_sides(next,scan);
}

EXPORT INT s5_scan_step(s5_status_t *next, s5_scan_context_t *scan)
{
    next->acquisition_clear = FALSE;
    if (s5_stage_for_state((s5_state_t)next->state)==S5_STAGE_ACQUISITION &&
        next->state!=S5_BLOCKED && next->encoder_valid &&
        (s5_encoder_displaced(next->motor1_counts_start,next->motor1_counts_end) ||
         s5_encoder_displaced(next->motor2_counts_start,next->motor2_counts_end) ||
         (scan->position_recorded &&
          (s5_encoder_displaced(scan->origin_left,next->motor1_counts_start) ||
           s5_encoder_displaced(scan->origin_right,next->motor2_counts_start))))) {
        /* Brake settling or real pose change invalidates geometry. Hold motors
         * via ACQUIRE and retry from centre, rather than latching VC_SENSOR. */
        UINT restarts=scan->motion_restarts+1U;
        *scan=(s5_scan_context_t){0};scan->motion_restarts=restarts;
        next->approach_plan_valid=FALSE;next->valid=FALSE;
        next->left_bound_valid=next->right_bound_valid=FALSE;
        next->object_width_valid=FALSE;
        if(restarts>S5_SCAN_MOTION_RESTARTS)return scan_blocked(next);
        next->state=S5_CENTERING;
        return 0;
    }
    BOOL acquiring = next->state == S5_SCAN_LEFT || next->state == S5_REFINE_LEFT ||
        next->state == S5_SCAN_RIGHT || next->state == S5_REFINE_RIGHT ||
        next->state == S5_VERIFY_CLEAR || next->state == S5_READY;
    BOOL opening = FALSE;
    if (next->sample_angle_deg != next->angle_deg) {
        next->approach_plan_valid=FALSE;next->state=S5_FAULT;
        return 0;
    }
    if(!scan->echo_angle_valid || scan->echo_angle!=next->angle_deg) {
        scan->echo_angle=next->angle_deg;scan->echo_angle_valid=TRUE;
        scan->echo_failures=0;
    }
    if (!next->valid || next->distance_mm==0U || next->distance_mm>5000U) {
        BOOL no_return=next->echo_result==S5_ECHO_NO_RISE ||
                       next->echo_result==S5_ECHO_NO_FALL;
        if(no_return) {
            if(scan->echo_failures<S5_SCAN_ECHO_FAILURES)++scan->echo_failures;
            if(scan->echo_failures<S5_SCAN_ECHO_FAILURES)return next->angle_deg;
            opening=TRUE;
        } else {
            /* Short pulses and a pin already high are faults, not openings. */
            scan->echo_failures=0;
            if(acquiring) {
                switch(next->state) {
                case S5_SCAN_LEFT:return scan_left(next,scan,TRUE);
                case S5_SCAN_RIGHT:return scan_right(next,scan,TRUE);
                case S5_REFINE_LEFT:return side_failed(next,scan,TRUE);
                case S5_REFINE_RIGHT:return side_failed(next,scan,FALSE);
                case S5_VERIFY_CLEAR:case S5_READY:
                    return side_failed(next,scan,next->planned_rotate_deg>0);
                default:break;
                }
            }
            if(next->state==S5_BLOCKED)return 0;
            next->approach_plan_valid=FALSE;next->state=S5_FAULT;
            next->left_bound_valid=next->right_bound_valid=FALSE;
            next->object_width_valid=FALSE;*scan=(s5_scan_context_t){0};
            return 0;
        }
    } else scan->echo_failures=0;
    BOOL object = !opening && s5_target_detected(next);
    next->acquisition_clear = opening || (acquiring && !object);
    switch (next->state)
    {
    case S5_CENTERING:
        next->state = S5_MONITOR;
        return 0;
    case S5_FAULT:
        next->state = S5_MONITOR;
        /* fall through */
    case S5_MONITOR:
        if(opening) {scan->acquisition_samples=0;return 0;}
        return scan_begin(next, scan);
    case S5_SCAN_LEFT:
        return scan_left(next, scan, object);
    case S5_REFINE_LEFT:
        return refine_side(next, scan, object,TRUE);
    case S5_SCAN_RIGHT:
        return scan_right(next, scan, object);
    case S5_REFINE_RIGHT:
        return refine_side(next, scan, object,FALSE);
    case S5_VERIFY_CLEAR:
        if (object || (!opening && next->distance_mm <= S5_GUIDANCE_OBSTACLE_MM))
            return side_failed(next,scan,next->planned_rotate_deg>0);
        if (!scan->plan_pending) {
            if (!s5_acquisition_plan(next) || next->planned_rotate_deg < -80 ||
                next->planned_rotate_deg > 80) return scan_blocked(next);
            if (next->angle_deg!=next->planned_rotate_deg) {
                scan->plan_pending=1;
                return next->planned_rotate_deg;
            }
        }
        next->state = S5_READY;
        return next->angle_deg;
    case S5_READY:
        if (object || (!opening && next->distance_mm <= S5_GUIDANCE_OBSTACLE_MM))
            return scan_blocked(next);
        return next->angle_deg;
    case S5_GUIDING:
        if (opening || next->distance_mm > S5_OBSTACLE_MM)
        {
            next->state = S5_MONITOR;
            next->avoidance_deg = 0;
            *scan = (s5_scan_context_t){0};
        }
        return next->angle_deg;
    case S5_APPROACH: case S5_OVERCOME: case S5_REORG: case S5_ROTATING:
        return next->angle_deg;
    case S5_BLOCKED:
        /* Do not repeatedly restart left while the same obstacle remains. */
        if(next->angle_deg==0 && (opening || next->distance_mm>S5_OBSTACLE_MM)) {
            next->state=S5_MONITOR;*scan=(s5_scan_context_t){0};
        }
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
