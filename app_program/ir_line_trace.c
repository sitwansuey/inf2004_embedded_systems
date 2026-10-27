/* Line-only bench decisions. No hardware, kernel calls or barcode authority. */
#include "ir_line_trace.h"
#include <stddef.h>
#include <string.h>

static bool invalidate(ir_line_trace_t *s) {
    (void)memset(s,0,sizeof(*s));return false;
}
bool ir_line_observe(ir_line_trace_t *s,const s3_line_config_t *c,
                     const uint16_t raw[3],uint8_t mask,
                     uint32_t sample_ms,uint32_t now_ms,bool source_ok) {
    if(s==NULL) return false;
    if(c==NULL || raw==NULL || !source_ok || !s3_line_config_valid(c) ||
       c->line_mask!=3U || c->position[0]!=1000 || c->position[1]!=-1000 ||
       (mask&3U)!=3U || now_ms-sample_ms>=30U) return invalidate(s);
    if(s->have_sample) {
        uint32_t elapsed=sample_ms-s->sample_ms;
        if(elapsed==0U) return s->usable;
        if(elapsed>30U) return invalidate(s);
    }
    s3_line_update(&s->line,c,raw,mask,sample_ms,&s->result);
    if(s->result.state==S3_LINE_INVALID) return invalidate(s);
    s->have_sample=true;s->usable=true;s->sample_ms=sample_ms;
    if(s->result.black_mask==0U && s->result.darkness[0]<=c->threshold_low &&
       s->result.darkness[1]<=c->threshold_low) {
        if(!s->white_start) s->white_since=sample_ms;
        s->white_start=true;
    } else s->white_start=false;
    return true;
}
bool ir_line_start_ready(const ir_line_trace_t *s) {
    return s!=NULL && s->usable && s->white_start &&
           s->sample_ms-s->white_since>=IR_LINE_START_HOLD_MS;
}
unsigned ir_line_direction(const ir_line_trace_t *s) {
    if(s==NULL || !s->usable) return IR_LINE_STOP;
    switch(s->result.black_mask) {
    case 0U:return IR_LINE_FORWARD;
    case 2U:return IR_LINE_LEFT;
    case 1U:return IR_LINE_RIGHT;
    default:return IR_LINE_STOP;
    }
}
unsigned ir_line_continuous_direction(const ir_line_trace_t *s) {
    if(s==NULL || !s->usable || s->result.black_mask>3U) return IR_LINE_STOP;
    return s->result.black_mask==2U?IR_LINE_LEFT:
           s->result.black_mask==1U?IR_LINE_RIGHT:IR_LINE_FORWARD;
}
void ir_line_enforce(ir_scan_state_t *scan,const ir_line_trace_t *line) {
    if(scan!=NULL && scan->active && scan->scan &&
       ir_line_direction(line)==IR_LINE_STOP)
        ir_scan_stop(scan,line!=NULL && line->usable?IR_SCAN_LINE_AMBIGUOUS:IR_SCAN_ADC_LOST);
}
