#ifndef IR_LINE_COURSE_H
#define IR_LINE_COURSE_H
#include "ir_line_trace.h"

/* Bench bounds, not calibrated physical distances or turn angles. Two line
 * sensors cannot uniquely classify every corner/intersection geometry. */
#define IR_COURSE_SIDE_HOLD_MS 60U
#define IR_COURSE_WHITE_HOLD_MS 30U
#define IR_COURSE_CROSS_MAX_MS 500U
#define IR_COURSE_TURN_MAX_MS 800U
enum {
    IR_COURSE_TRACK, IR_COURSE_CROSS_STRAIGHT,
    IR_COURSE_HOLD_LEFT, IR_COURSE_HOLD_RIGHT
};
typedef struct {
    uint32_t entered_ms,side_since_ms,side_last_ms,white_since_ms;
    uint8_t phase:2,side:2,side_streak:1,white_pending:1;
    uint8_t fault:4;
} ir_line_course_t;

void ir_line_course_reset(ir_line_course_t *course);
/* enabled means an explicitly STARTed line run, never pulse/STOP/PREPARE.
 * Output remains a request to the existing sole motor owner. */
unsigned ir_line_course_step(ir_line_course_t *course,const ir_line_trace_t *line,
                             uint32_t now_ms,bool enabled);
#endif
