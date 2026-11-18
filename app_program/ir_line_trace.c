/* Straddle line observer for the vehicle controller (vehicle.c). No hardware,
 * kernel calls or barcode authority. */
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
    return true;
}
