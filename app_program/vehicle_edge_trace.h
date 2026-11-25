#ifndef VEHICLE_EDGE_TRACE_H
#define VEHICLE_EDGE_TRACE_H
#include <stdbool.h>
#include <stdint.h>
/* Edge tracing: one straddle IR (the edge sensor) rides one edge of the line
 * at a set darkness and steers in proportion to the difference - a continuous
 * line trace, the same for straights and curves, no touch states.
 * Darkness is per mille of the sensor's calibrated white..black range.
 * Safety nets: the far sensor on black (line swung right past the gap) pivots
 * toward the far side until the edge sensor finds the edge again; the edge
 * sensor nearly fully black for deep_ms pivots toward the edge side until it
 * is back on the edge; both sensors black is a crossing: straight on until
 * the far sensor is white again. */
enum
{
    EDGE_FOLLOW,
    EDGE_PIVOT_EDGE,
    EDGE_PIVOT_FAR,
    EDGE_STRAIGHT
};
typedef struct
{
    int32_t  prev_err;
    uint32_t deep_since_ms;
    uint8_t  deep_timing : 1, crossing : 1, far_pivot : 1, edge_pivot : 1,
        primed : 1;
} edge_trace_t;
typedef struct
{
    uint8_t kind;  /* EDGE_* */
    int16_t steer; /* EDGE_FOLLOW: + right, - left (S2 follow units) */
} edge_action_t;
typedef struct
{
    int16_t  set, black, white, deep; /* darkness per mille */
    int16_t  kp_milli, kd_milli, max_steer;
    uint16_t deep_ms;
    uint8_t  edge_right; /* 1: right sensor rides the line's right edge; 0:
                            left/left */
} edge_trace_config_t;

static inline edge_action_t
edge_trace_step (edge_trace_t * s, const edge_trace_config_t * c, uint32_t now,
                 int32_t edge_dark, int32_t far_dark)
{
    edge_action_t a = {EDGE_FOLLOW, 0};
    bool edge_black = edge_dark >= c->black, far_black = far_dark >= c->black;
    /* Crossing (both black): straight until the far sensor clears. */
    if (edge_black && far_black)
    {
        s->crossing = 1U;
    }
    if (s->crossing)
    {
        if (far_dark <= c->white)
        {
            s->crossing = 0U;
        }
        else
        {
            s->far_pivot = s->edge_pivot = s->deep_timing = 0U;
            a.kind                                        = EDGE_STRAIGHT;
            return a;
        }
    }
    /* Line swung past the gap to the far sensor: pivot that way until the
     * edge sensor reaches the edge again. */
    if (far_black)
    {
        s->far_pivot = 1U;
    }
    if (s->far_pivot)
    {
        if (edge_dark >= c->set)
        {
            s->far_pivot = 0U;
        }
        else
        {
            a.kind = EDGE_PIVOT_FAR;
            return a;
        }
    }
    /* Edge sensor deep in the line for deep_ms: pivot toward the edge side
     * until it is back at the set darkness. */
    if (edge_dark >= c->deep)
    {
        if (!s->deep_timing)
        {
            s->deep_timing   = 1U;
            s->deep_since_ms = now;
        }
        else if (now - s->deep_since_ms >= c->deep_ms)
        {
            s->edge_pivot = 1U;
        }
    }
    else
    {
        s->deep_timing = 0U;
    }
    if (s->edge_pivot)
    {
        if (edge_dark <= c->set)
        {
            s->edge_pivot = 0U;
        }
        else
        {
            a.kind = EDGE_PIVOT_EDGE;
            return a;
        }
    }
    /* Proportional (+derivative) on the darkness error. Darker = the line is
     * further toward the edge sensor's side: turn that way. */
    int32_t err   = edge_dark - c->set;
    int32_t d     = s->primed ? err - s->prev_err : 0;
    s->prev_err   = err;
    s->primed     = 1U;
    int32_t steer = (c->kp_milli * err + c->kd_milli * d) / 1000;
    if (steer > c->max_steer)
    {
        steer = c->max_steer;
    }
    if (steer < -c->max_steer)
    {
        steer = -c->max_steer;
    }
    a.steer = (int16_t) (c->edge_right ? steer : -steer);
    return a;
}
#endif

/*** end of file ***/
