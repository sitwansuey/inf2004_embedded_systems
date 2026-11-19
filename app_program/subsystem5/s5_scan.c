/* Pure acquisition decisions: coarse outward search, fine inward bounds. */
#include "s5_scan.h"
#include "s5_policy.h"

/* Halves are meaningful exactly while object_width_valid. */
static void width_clear(s5_status_t *next)
{
    next->object_width_valid = FALSE;
    next->object_left_mm = next->object_right_mm = 0;
    next->width_one_sided = FALSE;
}

static INT scan_blocked(s5_status_t *next)
{
    next->approach_plan_valid = FALSE;
    next->state = S5_BLOCKED;
    next->avoidance_deg = 0;
    width_clear(next);
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
    next->object_left_mm = next->object_right_mm = 0;
    next->width_one_sided = FALSE;
    next->state = S5_SCAN_LEFT;
    return S5_SCAN_STEP_DEG;
}

/* Plan the side the scan settled on: forced_side 0 takes the smaller
 * half-width; +1/-1 keeps LEFT/RIGHT when the other side was never clear. */
static BOOL plan_side(s5_status_t *next, s5_scan_context_t *scan)
{
    return s5_acquisition_plan_side(next,scan->forced_side) &&
        next->planned_rotate_deg >= -80 && next->planned_rotate_deg <= 80;
}
/* Publish both half-widths first, with no plan, and hold the vehicle in
 * VERIFY_CLEAR for S5_WIDTH_PUBLISH_MS before the side is chosen. */
static INT finish_sides(s5_status_t *next, s5_scan_context_t *scan)
{
    s5_width_from_bounds(next); /* Remains invalid when only one edge is known. */
    if (!next->object_width_valid) return scan_blocked(next);
    next->approach_plan_valid=FALSE;next->avoidance_deg=0;next->planned_rotate_deg=0;
    next->planned_rotate_ms=next->planned_approach_mm=next->planned_return_mm=0;
    next->state=S5_VERIFY_CLEAR;
    scan->plan_pending=0;scan->width_hold=1;scan->width_hold_ms=next->time_ms;
    return next->angle_deg;
}
/* Hold samples only advance time; the plan then pans to a fresh heading. */
static INT width_hold_step(s5_status_t *next, s5_scan_context_t *scan)
{
    if (next->time_ms - scan->width_hold_ms < S5_WIDTH_PUBLISH_MS) return next->angle_deg;
    scan->width_hold=0;
    if (!plan_side(next,scan)) return scan_blocked(next);
    scan->plan_pending=1;
    return next->planned_rotate_deg;
}
/* The chosen heading failed its check: both edges are still known, so plan
 * the other side once (if it clears) before giving up. */
static INT verify_failed(s5_status_t *next, s5_scan_context_t *scan)
{
    if (scan->plan_pending && !scan->side_retry && next->approach_plan_valid) {
        INT other = next->planned_rotate_deg > 0 ? -1 : 1;
        scan->side_retry = 1;
        if (s5_acquisition_plan_side(next, other) && next->planned_rotate_deg >= -80 &&
            next->planned_rotate_deg <= 80)
            return next->planned_rotate_deg;
    }
    return scan_blocked(next);
}
/* Only one side has a measured edge (the other was blocked through 60 deg
 * or its edge failed): the other is taken as its mirror (a symmetric width
 * for the pass check), flagged width_one_sided, and only the measured side is
 * ever planned (forced_side; side_retry stops the verify fallback). */
static INT finish_one_side(s5_status_t *next, s5_scan_context_t *scan, BOOL left_measured)
{
    if (left_measured) {
        next->right_bound_deg=-next->left_bound_deg;
        next->right_bound_distance_mm=next->left_bound_distance_mm;next->right_bound_valid=TRUE;
    } else {
        next->left_bound_deg=-next->right_bound_deg;
        next->left_bound_distance_mm=next->right_bound_distance_mm;next->left_bound_valid=TRUE;
    }
    scan->forced_side=left_measured?1:-1;scan->side_retry=1;
    next->width_one_sided=TRUE;
    return finish_sides(next,scan);
}
/* An edge is known (object angle next to the innermost clear one, averaged
 * range). A left edge sends the servo to the mirrored best clear angle on
 * the right. */
static INT side_found(s5_status_t *next, s5_scan_context_t *scan, BOOL left, INT deg, UW range)
{
    scan->averaging_bound=FALSE;scan->fine_checks=0;scan->echo_failures=0;
    if (left) {
        next->left_bound_deg=deg;next->left_bound_distance_mm=range;next->left_bound_valid=TRUE;
        next->state=S5_SCAN_RIGHT;scan->right_coarse=0;scan->coarse_object=0;
        return -scan->fine_clear;
    }
    next->right_bound_deg=deg;next->right_bound_distance_mm=range;next->right_bound_valid=TRUE;
    if (scan->right_coarse) return finish_one_side(next,scan,FALSE);
    scan->forced_side=0;
    return finish_sides(next,scan);
}
/* A side could not produce an edge. A failed left is searched on the right
 * the same way; a failed mirror walk keeps the measured left. */
static INT side_failed(s5_status_t *next, s5_scan_context_t *scan, BOOL left)
{
    scan->averaging_bound=FALSE;scan->fine_checks=0;scan->echo_failures=0;
    if (left) {
        next->left_bound_valid=FALSE;scan->right_coarse=1;scan->coarse_object=0;
        next->state=S5_SCAN_RIGHT;
        return -S5_SCAN_STEP_DEG;
    }
    next->right_bound_valid=FALSE;
    if (scan->right_coarse || !next->left_bound_valid) return scan_blocked(next);
    return finish_one_side(next,scan,TRUE);
}
/* Next fine angle: halfway between the inner object angle and the clear one
 * (bisection, ~5 pans instead of up to 20 single-degree steps, each clear
 * one up to 3 no-echo pings). Adjacent: the object angle itself, where the
 * edge is averaged. */
static INT fine_probe(const s5_scan_context_t *scan, BOOL left)
{
    return (left?1:-1)*(scan->fine_object+(scan->fine_clear-scan->fine_object)/2);
}
/* From a clear angle, search inward to the edge at 1 deg resolution. */
static INT walk_begin(s5_status_t *next, s5_scan_context_t *scan, BOOL left, INT clear, INT object)
{
    scan->fine_clear=clear;scan->fine_object=object;
    scan->fine_checks=0;scan->averaging_bound=FALSE;scan->edge_retreats=0;
    next->state=left?S5_REFINE_LEFT:S5_REFINE_RIGHT;
    if (clear<=1) return side_found(next,scan,left,0,next->obstacle_distance_mm);
    return fine_probe(scan,left);
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

/* Bisecting inward: clear becomes the new clear angle, an object short of
 * the angle next to it the new inner object angle. The object angle next to
 * the clear one is the edge and is averaged. Reaching the centre clear makes
 * the forward sample the edge; an inner object angle that reads clear leaves
 * only the forward one known. A borderline edge sample that misses the object
 * while averaging retreats the edge 1 deg inward, at most S5_EDGE_RETREATS
 * times per side. */
static INT walk_side(s5_status_t *next, s5_scan_context_t *scan, BOOL object, BOOL left)
{
    INT magnitude=left?next->angle_deg:-next->angle_deg;
    if (scan->averaging_bound && !object) {
        if (scan->edge_retreats>=S5_EDGE_RETREATS || magnitude<=1) return side_failed(next,scan,left);
        ++scan->edge_retreats;scan->averaging_bound=FALSE;
        scan->fine_clear=magnitude;scan->fine_object=magnitude-1;
        return fine_probe(scan,left);
    }
    if (!scan->averaging_bound && !object) {
        scan->fine_clear=magnitude;
        if (magnitude<=1) return side_found(next,scan,left,0,next->obstacle_distance_mm);
        if (++scan->fine_checks>(UINT)(S5_SCAN_STEP_DEG*S5_SCAN_COARSE_STEPS)) return side_failed(next,scan,left);
        if (scan->fine_object>=magnitude) scan->fine_object=0;
        return fine_probe(scan,left);
    }
    if (!scan->averaging_bound && scan->fine_clear-magnitude>1) {
        if (++scan->fine_checks>(UINT)(S5_SCAN_STEP_DEG*S5_SCAN_COARSE_STEPS)) return side_failed(next,scan,left);
        scan->fine_object=magnitude;
        return fine_probe(scan,left);
    }
    UW mean=0;
    if (!bound_average(next,scan,&mean)) return next->angle_deg;
    return side_found(next,scan,left,next->angle_deg,mean);
}

/* Coarse left: 20, 40, 60 deg until one is clear. Blocked through 60:
 * search the right the same way. */
static INT scan_left(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    if (!object) return walk_begin(next,scan,TRUE,next->angle_deg,scan->coarse_object);
    scan->coarse_object=next->angle_deg;
    if (next->angle_deg<S5_SCAN_STEP_DEG*S5_SCAN_COARSE_STEPS) return next->angle_deg+S5_SCAN_STEP_DEG;
    next->left_bound_valid=FALSE;scan->right_coarse=1;scan->coarse_object=0;
    next->state=S5_SCAN_RIGHT;
    return -S5_SCAN_STEP_DEG;
}

/* Right: the mirror check at the left's best clear angle (clear: the right
 * edge is first expected at the left edge's mirror, then bisected; the
 * object: its right edge is further out, so the right is searched outward at
 * the next of 20/40/60 deg), or the coarse search after a blocked left.
 * Object through 60 deg: a measured left is mirrored, else blocked. */
static INT scan_right(s5_status_t *next, s5_scan_context_t *scan, BOOL object)
{
    INT magnitude=-next->angle_deg;
    if (!scan->right_coarse && !scan->right_outward) {
        if (!object) return walk_begin(next,scan,FALSE,magnitude,magnitude-1);
        scan->right_outward=1;
    } else if (!object) return walk_begin(next,scan,FALSE,magnitude,scan->coarse_object);
    scan->coarse_object=magnitude;
    if (magnitude<S5_SCAN_STEP_DEG*S5_SCAN_COARSE_STEPS)
        return -(magnitude/S5_SCAN_STEP_DEG+1)*S5_SCAN_STEP_DEG;
    if (scan->right_coarse) return scan_blocked(next);
    return finish_one_side(next,scan,TRUE);
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
        width_clear(next);
        if(restarts>S5_SCAN_MOTION_RESTARTS)return scan_blocked(next);
        next->state=S5_CENTERING;
        return 0;
    }
    BOOL acquiring = next->state == S5_SCAN_LEFT || next->state == S5_REFINE_LEFT ||
        next->state == S5_SCAN_RIGHT || next->state == S5_REFINE_RIGHT ||
        next->state == S5_VERIFY_CLEAR || next->state == S5_READY;
    BOOL opening = FALSE;
    if (next->sample_angle_deg != next->angle_deg) {
        next->approach_plan_valid=FALSE;next->state=S5_FAULT;scan->width_hold=0;
        return 0;
    }
    if (scan->width_hold && next->state==S5_VERIFY_CLEAR) return width_hold_step(next,scan);
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
                case S5_VERIFY_CLEAR:return verify_failed(next,scan);
                case S5_READY:return scan_blocked(next);
                default:break;
                }
            }
            if(next->state==S5_BLOCKED)return 0;
            next->approach_plan_valid=FALSE;next->state=S5_FAULT;
            next->left_bound_valid=next->right_bound_valid=FALSE;
            width_clear(next);*scan=(s5_scan_context_t){0};
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
        return walk_side(next, scan, object,TRUE);
    case S5_SCAN_RIGHT:
        return scan_right(next, scan, object);
    case S5_REFINE_RIGHT:
        return walk_side(next, scan, object,FALSE);
    case S5_VERIFY_CLEAR:
        if (object || (!opening && next->distance_mm <= S5_GUIDANCE_OBSTACLE_MM))
            return verify_failed(next,scan);
        if (!scan->plan_pending) {
            if (!plan_side(next,scan)) return scan_blocked(next);
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
        width_clear(next);scan->width_hold=0;
        next->left_bound_valid = next->right_bound_valid = FALSE;
        return 0;
    }
}
