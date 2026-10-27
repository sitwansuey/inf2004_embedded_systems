#include "app_logging.h"

#define NAME(table, index) ((UINT)(index) < sizeof(table) / sizeof((table)[0]) ? \
                            (table)[(UINT)(index)] : "unknown")

EXPORT void app_log_state(const app_shared_state_t *state)
{
    static BOOL started;
    static app_shared_state_t previous;
    static const char *phases[] = { "cruise", "acquisition",
        "approach-rotate", "approach-cruise", "approach-rotate-back",
        "overcome-cruise", "overcome-clearance", "reorg-rotate90", "reorg-cruise",
        "reorg-rotate-minus90", "restore", "abort" };
    static const char *stages[] = { "none", "acquisition", "approach", "overcome", "reorg" };
    static const char *moves[] = { "stop", "forward", "reverse", "rotate-left",
                                  "rotate-right", "pivot-right", "pivot-left" };
    static const char *sensor_states[] = { "monitor", "scan-left", "refine-left",
        "scan-right", "refine-right", "verify-clear", "ready", "blocked", "fault",
        "rotating", "centring", "guiding", "approach", "overcome", "reorg" };
    static const char *echo[] = { "ok", "stuck-high", "no-rise", "no-fall", "too-short" };
    if (!state) return;
    if (!started || state->phase != previous.phase ||
        state->avoidance_stage != previous.avoidance_stage ||
        state->movement != previous.movement || state->power_percent != previous.power_percent) {
        tm_printf((UB *)"[STATE] t=%u seq=%u fsm=%s stage=%s motor=%s power=%u%%\n",
                  state->time_ms, state->sequence, NAME(phases, state->phase),
                  NAME(stages, state->avoidance_stage), NAME(moves, state->movement), state->power_percent);
    }
    if (!started || state->s3.time_ms != previous.s3.time_ms ||
        state->s3.available != previous.s3.available) {
        tm_printf((UB *)"[SENSOR] t=%u source=S3 sample_t=%u available=%d data=unimplemented\n",
                  state->time_ms, state->s3.time_ms, state->s3.available);
    }
    if (!started || state->s4.time_ms != previous.s4.time_ms ||
        state->s4.available != previous.s4.available) {
        tm_printf((UB *)"[SENSOR] t=%u source=S4 sample_t=%u available=%d data=unimplemented\n",
                  state->time_ms, state->s4.time_ms, state->s4.available);
    }
    if (!started || state->s5.time_ms != previous.s5.time_ms ||
        state->s5.state != previous.s5.state || state->s5.valid != previous.s5.valid ||
        state->s5_available != previous.s5_available) {
        tm_printf((UB *)"[SENSOR] t=%u source=S5 sample_t=%u age_ms=%u available=%d valid=%d "
                  "state=%s stage=%s echo=%s distance_mm=%u echo_us=%u angle_deg=%d heading_deg=%d scan=%u target_depth_mm=%u plan_valid=%d planned_rotate_deg=%d threshold_mm=%u sample_angle_deg=%d left_bound_deg=%d right_bound_deg=%d left_average_mm=%u right_average_mm=%u left_bound_valid=%d right_bound_valid=%d width_valid=%d width_mm=%u width_min_mm=%u width_max_mm=%u\n",
                  state->time_ms, state->s5.time_ms, state->time_ms - state->s5.time_ms,
                  state->s5_available, state->s5.valid, NAME(sensor_states, state->s5.state),
                  NAME(stages, s5_stage_for_state(state->s5.state)),
                  NAME(echo, state->s5.echo_result), state->s5.distance_mm, state->s5.echo_us,
                  state->s5.angle_deg, state->s5.avoidance_deg, state->s5.scan_id,
                  state->s5.obstacle_distance_mm, state->s5.approach_plan_valid,
                  state->s5.planned_rotate_deg, s5_obstacle_threshold(state->s5.state), state->s5.sample_angle_deg,
                  state->s5.left_bound_deg, state->s5.right_bound_deg,
                  state->s5.left_bound_distance_mm, state->s5.right_bound_distance_mm,
                  state->s5.left_bound_valid, state->s5.right_bound_valid,
                  state->s5.object_width_valid, state->s5.object_width_mm,
                  state->s5.object_width_min_mm, state->s5.object_width_max_mm);
    }
    previous = *state;
    started = TRUE;
}
