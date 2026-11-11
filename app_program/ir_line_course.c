#include "ir_line_course.h"
#include <stddef.h>
#include <string.h>

void ir_line_course_reset(ir_line_course_t *s) {
    if(s!=NULL) (void)memset(s,0,sizeof(*s));
}
static uint32_t not_after(uint32_t stamp,uint32_t limit) {
    return (int32_t)(stamp-limit)>0?limit:stamp;
}
void ir_line_course_shift(ir_line_course_t *s,uint32_t dt,uint32_t sample_ms) {
    if(s==NULL || dt==0U) return;
    s->entered_ms+=dt;
    s->side_since_ms=not_after(s->side_since_ms+dt,sample_ms);
    s->side_last_ms=not_after(s->side_last_ms+dt,sample_ms);
    s->white_since_ms=not_after(s->white_since_ms+dt,sample_ms);
}
static unsigned fail(ir_line_course_t *s,unsigned reason) {
    s->fault=(uint8_t)(reason&15U);return IR_LINE_STOP;
}
static unsigned phase_direction(unsigned phase) {
    return phase==IR_COURSE_HOLD_LEFT?IR_LINE_LEFT:
           phase==IR_COURSE_HOLD_RIGHT?IR_LINE_RIGHT:IR_LINE_FORWARD;
}
unsigned ir_line_course_step(ir_line_course_t *s,const ir_line_trace_t *line,
                             uint32_t now,bool enabled) {
    if(s==NULL) return IR_LINE_STOP;
    if(!enabled) {ir_line_course_reset(s);return IR_LINE_STOP;}
    if(s->fault) return IR_LINE_STOP;
    if(line==NULL || !line->usable || !line->have_sample ||
       now-line->sample_ms>=30U || line->result.black_mask>3U)
        return fail(s,IR_SCAN_ADC_LOST);

    if(s->phase!=IR_COURSE_TRACK) {
        uint32_t bound=s->phase==IR_COURSE_CROSS_STRAIGHT?
                       IR_COURSE_CROSS_MAX_MS:IR_COURSE_TURN_MAX_MS;
        /* Partial WHITE or direction changes never restart this deadline. */
        if(now-s->entered_ms>=bound) return fail(s,IR_SCAN_COURSE_TIMEOUT);
        if(line->result.black_mask==0U && line->white_start) {
            if(!s->white_pending) {
                s->white_pending=1U;s->white_since_ms=line->sample_ms;
            } else if(line->sample_ms-s->white_since_ms>=IR_COURSE_WHITE_HOLD_MS) {
                ir_line_course_reset(s);return IR_LINE_FORWARD;
            }
        } else s->white_pending=0U;
        return phase_direction(s->phase);
    }

    unsigned mask=line->result.black_mask;
    if(mask==0U) {
        s->side_streak=0U;s->side=0U;return IR_LINE_FORWARD;
    }
    if(mask==1U || mask==2U) {
        if(!s->side_streak || s->side!=mask) {
            /* A new touch: another on the same side counts up (the third
             * enters bend mode there); the other side ends bend mode and
             * starts its own count. */
            if(s->bend_candidate==mask) {
                if(s->bend_level<7U) ++s->bend_level;
            } else {
                s->bend_level=1U;s->bend_side=0U;s->bend_candidate=(uint8_t)(mask&3U);
            }
            if(s->bend_level>=IR_COURSE_BEND_TOUCHES) s->bend_side=(uint8_t)(mask&3U);
            s->side=(uint8_t)(mask&3U);s->side_since_ms=line->sample_ms;
        }
        s->side_streak=1U;s->side_last_ms=line->sample_ms;
        return mask==2U?IR_LINE_LEFT:IR_LINE_RIGHT;
    }
    /* A sustained immediately preceding side is a corner hypothesis. A
     * centred/brief/alternating approach defaults to straight through the
     * cross. This is temporal evidence, not unique junction classification. */
    bool corner=s->side_streak &&
        line->sample_ms-s->side_since_ms>=IR_COURSE_SIDE_HOLD_MS &&
        now-s->side_last_ms<30U;
    s->bend_level=0U;s->bend_side=0U;s->bend_candidate=0U;
    s->phase=corner?(s->side==2U?IR_COURSE_HOLD_LEFT:IR_COURSE_HOLD_RIGHT):
                    IR_COURSE_CROSS_STRAIGHT;
    s->entered_ms=now;s->white_pending=0U;
    return phase_direction(s->phase);
}
