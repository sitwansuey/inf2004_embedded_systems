#include <stdio.h>
#include "../../app_program/ir_motor_scan.h"
#include "../../app_program/ir_line_progress.h"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL line%u: %s\n",(unsigned)__LINE__,#x);}}while(0)
static bool send(ir_scan_state_t *s,ir_scan_input_t *i,unsigned op,unsigned duration,unsigned seq) {
    uint8_t w[6]={0xD3U,(uint8_t)op,op==IR_SCAN_RUN?50U:0U,(uint8_t)duration,(uint8_t)(duration>>8),(uint8_t)seq};
    return ir_scan_command(s,i,w,i->now_ms);
}
int main(void) {
    CHECK(IR_SCAN_MAX_MS==20000U);
    CHECK(IR_SCAN_MAX_POWER==50U && IR_SCAN_POWER_BITS==6);
    /* Exercise wire validation and stored state at every power, especially
     * 31/32 and 50: the old five-bit mask silently changed 50 to 18. */
    for(unsigned power=0U;power<=255U;++power) {
        uint8_t wire[6]={0xD3U,IR_SCAN_RUN,(uint8_t)power,0x20U,0x4eU,1U};
        bool allowed=power>=1U && power<=50U;
        CHECK(ir_scan_frame_valid(wire)==allowed);
        ir_scan_state_t trial={0};
        ir_scan_input_t input={1000U,true,true,true,true};
        ir_scan_step(&trial,&input);
        CHECK(!send(&trial,&input,IR_SCAN_PREPARE,0U,0U));
        CHECK(ir_scan_command(&trial,&input,wire,input.now_ms)==allowed);
        CHECK(allowed?(trial.active && trial.power==power):(!trial.active && trial.power==0U));
    }
    uint8_t pulse[6]={0xD3U,IR_SCAN_PULSE,50U,250U,0U,1U};
    CHECK(ir_scan_frame_valid(pulse));
    pulse[3]=251U;CHECK(!ir_scan_frame_valid(pulse));
    for(unsigned duration=0U;duration<=65535U;++duration) {
        uint8_t w[6]={0xD3U,IR_SCAN_RUN,50U,(uint8_t)duration,(uint8_t)(duration>>8),0U};
        bool allowed=duration>=1U && duration<=20000U && w[3]!=115U && w[4]!=115U;
        CHECK(ir_scan_frame_valid(w)==allowed);
    }
    ir_scan_state_t scan={0};ir_scan_input_t i={1000U,true,true,true,true};
    ir_line_progress_t p={0};uint32_t counts[2]={0U,0U};
    ir_scan_step(&scan,&i);CHECK(!send(&scan,&i,IR_SCAN_PREPARE,0U,0U));
    CHECK(send(&scan,&i,IR_SCAN_RUN,20000U,1U));
    unsigned seq=1U;
    for(unsigned elapsed=10U;elapsed<20000U;elapsed+=10U) {
        i.now_ms=1000U+elapsed;++counts[0];++counts[1];
        CHECK(ir_line_progress_check(&p,1U,counts,i.now_ms,true));
        ir_scan_step(&scan,&i);CHECK(scan.active);CHECK(scan.power==50U);
        if(elapsed%50U==0U){seq=(seq+1U)&63U;CHECK(!send(&scan,&i,IR_SCAN_RENEW,0U,seq));}
    }
    i.now_ms=21000U;ir_scan_step(&scan,&i);
    CHECK(!scan.active && scan.reason==IR_SCAN_COMPLETE && scan.power==0U);
    CHECK(!send(&scan,&i,IR_SCAN_RENEW,0U,(seq+1U)&63U) && !scan.active);

    p=(ir_line_progress_t){0};counts[0]=counts[1]=100U;
    CHECK(ir_line_progress_check(&p,1U,counts,0U,true));
    CHECK(ir_line_progress_check(&p,1U,counts,749U,true));
    CHECK(!ir_line_progress_check(&p,1U,counts,750U,true));
    CHECK(ir_line_progress_check(&p,0U,counts,751U,false));
    CHECK(ir_line_progress_check(&p,1U,counts,1000U,true));
    CHECK(!ir_line_progress_check(&p,1U,counts,1750U,true));

    /* A pivot must not require the deliberately stopped wheel to move. */
    for(unsigned command=5U;command<=6U;++command) {
        p=(ir_line_progress_t){0};counts[0]=counts[1]=0U;
        for(unsigned t=0U;t<=2000U;t+=10U) {
            if(command==5U)++counts[0];else ++counts[1];
            CHECK(ir_line_progress_check(&p,command,counts,t,true));
        }
    }
    /* Switching between forward/right pivot cannot refresh a stalled left
     * wheel, since it remains required in both commands. */
    p=(ir_line_progress_t){0};counts[0]=counts[1]=0U;
    CHECK(ir_line_progress_check(&p,1U,counts,0U,true));
    for(unsigned t=10U;t<750U;t+=10U) {
        ++counts[1];CHECK(ir_line_progress_check(&p,t%20U==0U?1U:5U,counts,t,true));
    }
    CHECK(!ir_line_progress_check(&p,1U,counts,750U,true));
    p=(ir_line_progress_t){0};counts[0]=0xfffffff0U;counts[1]=0U;
    CHECK(ir_line_progress_check(&p,5U,counts,0xfffffff0U,true));
    counts[0]+=32U;CHECK(ir_line_progress_check(&p,5U,counts,0x10U,true));
    CHECK(ir_line_progress_check(&p,5U,counts,0x10U+749U,true));
    CHECK(!ir_line_progress_check(&p,5U,counts,0x10U+750U,true));
    p=(ir_line_progress_t){0};counts[0]=100U;
    CHECK(ir_line_progress_check(&p,5U,counts,0U,true));
    counts[0]=99U;CHECK(!ir_line_progress_check(&p,5U,counts,750U,true));
    CHECK(!ir_line_progress_check(&p,1U,counts,0U,false));
    CHECK(!ir_line_progress_check(&p,2U,counts,0U,true));
    CHECK(!ir_line_progress_check(NULL,1U,counts,0U,true));
    CHECK(!ir_line_progress_check(&p,1U,NULL,0U,true));
    printf("Line-only50pct/20s/progress policy: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
