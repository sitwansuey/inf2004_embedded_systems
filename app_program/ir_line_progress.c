#include "ir_line_progress.h"
#include <stddef.h>
#include <string.h>
bool ir_line_progress_check(ir_line_progress_t *s,unsigned command,
                            const uint32_t counts[2],uint32_t now,bool valid) {
    unsigned required=command==1U?3U:command==5U?1U:command==6U?2U:0U;
    if(s==NULL) return false;
    if(command==0U) {(void)memset(s,0,sizeof(*s));return true;}
    if(!required || !valid || counts==NULL) {(void)memset(s,0,sizeof(*s));return false;}
    bool ok=true;
    for(unsigned wheel=0U;wheel<2U;++wheel) {
        unsigned bit=1U<<wheel;
        uint32_t delta=counts[wheel]-s->counts[wheel];
        if(!s->initialized || (s->required_mask&bit)==0U ||
           (delta>0U && delta<=INT32_MAX)) s->progressed_ms[wheel]=now;
        if((required&bit)!=0U && now-s->progressed_ms[wheel]>=IR_LINE_PROGRESS_TIMEOUT_MS) ok=false;
        s->counts[wheel]=counts[wheel];
    }
    s->required_mask=(uint8_t)(required&3U);s->initialized=true;
    return ok;
}
