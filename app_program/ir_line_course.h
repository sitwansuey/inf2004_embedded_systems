#ifndef IR_LINE_COURSE_H
#define IR_LINE_COURSE_H
#include "ir_line_trace.h"

/* Bench bounds, not calibrated physical distances or turn angles. Two line
 * sensors cannot uniquely classify every corner/intersection geometry. */
#define IR_COURSE_SIDE_HOLD_MS 60U
#define IR_COURSE_WHITE_HOLD_MS 30U
#define IR_COURSE_CROSS_MAX_MS 500U
#define IR_COURSE_TURN_MAX_MS 800U
/* Bend mode: touches on one side with no touch on the other side between
 * them (however far apart) count up in bend_level; the
 * IR_COURSE_BEND_TOUCHES-th sets bend_side, bend mode on that side. It ends
 * when the other sensor sees black or both sensors are black. Not time
 * based. When the other sensor ends a confirmed bend, bend mode carries on
 * straight away to that side (S-bends); such a chained bend is confirmed by
 * a touch on its own side, otherwise the next other-side touch returns to
 * normal tracing (one swerve at most after a bend into a straight). */
#ifndef IR_COURSE_BEND_TOUCHES
#define IR_COURSE_BEND_TOUCHES 2U
#endif
enum {
    IR_COURSE_TRACK, IR_COURSE_CROSS_STRAIGHT,
    IR_COURSE_HOLD_LEFT, IR_COURSE_HOLD_RIGHT
};
typedef struct {
    uint32_t entered_ms,side_since_ms,side_last_ms,white_since_ms;
    uint8_t phase:2,side:2,side_streak:1,white_pending:1;
    uint8_t fault:4;
    uint8_t bend_level:3,bend_side:2,bend_candidate:2; /* side 1 right, 2 left; bend_side 0: no bend mode */
    uint8_t bend_chained:1; /* bend mode entered from the opposite bend, not yet confirmed */
} ir_line_course_t;

void ir_line_course_reset(ir_line_course_t *course);
/* The robot stood still for dt_ms (a STOP between steps): move every course
 * timestamp forward by it, so held-side and phase-deadline times count
 * driving time only and a resume does not jump straight to full steer.
 * Sample-based stamps never pass sample_ms, the newest line sample (a later
 * one would wrap the course's unsigned age tests). */
void ir_line_course_shift(ir_line_course_t *course,uint32_t dt_ms,uint32_t sample_ms);
/* enabled means an explicitly STARTed line run, never pulse/STOP/PREPARE.
 * Output remains a request to the existing sole motor owner. */
unsigned ir_line_course_step(ir_line_course_t *course,const ir_line_trace_t *line,
                             uint32_t now_ms,bool enabled);
#endif
