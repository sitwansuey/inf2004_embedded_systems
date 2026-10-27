#include <stdio.h>
#include "../../app_program/ir_line_trace.h"
#include "../../app_program/robot_ir_config.h"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL line %u: %s\n",(unsigned)__LINE__,#x);}}while(0)
static s3_config_t cfg;
static bool sample(ir_line_trace_t *s,unsigned mask,uint32_t t) {
    uint16_t raw[3]={mask&1U?3214U:187U,mask&2U?3177U:189U,4095U};
    return ir_line_observe(s,&cfg.line,raw,7U,t,t,true);
}
static void frame(uint8_t w[6],unsigned op,unsigned power,unsigned duration,unsigned seq) {
    w[0]=0xD3U;w[1]=(uint8_t)op;w[2]=(uint8_t)power;
    w[3]=(uint8_t)duration;w[4]=(uint8_t)(duration>>8);w[5]=(uint8_t)seq;
}
int main(void) {
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    CHECK(robot_ir_apply_adc_calibration(&cfg));
    for(unsigned mask=0U;mask<4U;++mask) {
        ir_line_trace_t s={0};CHECK(sample(&s,mask,1000U));
        CHECK(ir_line_direction(&s)==(mask==0U?IR_LINE_FORWARD:mask==2U?IR_LINE_LEFT:mask==1U?IR_LINE_RIGHT:IR_LINE_STOP));
        CHECK(!ir_line_start_ready(&s));
        for(uint32_t t=1010U;t<=1100U;t+=10U) CHECK(sample(&s,mask,t));
        CHECK(ir_line_start_ready(&s)==(mask==0U));
    }
    {ir_line_trace_t s={0};uint16_t raw[3]={187U,189U,197U};
     CHECK(ir_line_observe(&s,&cfg.line,raw,3U,1000U,1000U,true));
     for(uint32_t t=1001U;t<1030U;++t) CHECK(ir_line_observe(&s,&cfg.line,raw,3U,1000U,t,true));
     CHECK(!ir_line_start_ready(&s));
     CHECK(!ir_line_observe(&s,&cfg.line,raw,3U,1000U,1030U,true));
     CHECK(ir_line_direction(&s)==IR_LINE_STOP);}
    for(unsigned valid=0U;valid<8U;++valid) {
        ir_line_trace_t s={0};uint16_t raw[3]={3214U,3177U,0U};
        CHECK(ir_line_observe(&s,&cfg.line,raw,(uint8_t)valid,1000U,1000U,true)==((valid&3U)==3U));
    }
    {ir_line_trace_t s={0};CHECK(sample(&s,0U,0xfffffff0U));
     for(unsigned n=1U;n<=10U;++n) CHECK(sample(&s,0U,0xfffffff0U+n*10U));
     CHECK(ir_line_start_ready(&s));CHECK(sample(&s,3U,100U));
     CHECK(ir_line_direction(&s)==IR_LINE_STOP && !ir_line_start_ready(&s));}
    {ir_line_trace_t s={0};CHECK(sample(&s,0U,1000U));CHECK(!sample(&s,0U,1040U));
     CHECK(ir_line_direction(&s)==IR_LINE_STOP);CHECK(sample(&s,0U,1050U));
     CHECK(!ir_line_start_ready(&s));CHECK(!sample(&s,0U,1040U));}
    {ir_line_trace_t s={0};uint16_t raw[3]={3214U,3177U,0U};
     CHECK(!ir_line_observe(&s,&cfg.line,raw,7U,1000U,999U,true));
     CHECK(!ir_line_observe(&s,&cfg.line,raw,7U,1000U,1000U,false));
     raw[0]=4096U;CHECK(!ir_line_observe(&s,&cfg.line,raw,7U,1000U,1000U,true));
     s3_line_config_t bad=cfg.line;bad.position[0]=-1000;
     CHECK(!ir_line_observe(&s,&bad,raw,7U,1000U,1000U,true));
     CHECK(!ir_line_start_ready(NULL) && ir_line_direction(NULL)==IR_LINE_STOP);}
    {ir_line_trace_t line={0};ir_scan_state_t scan={0};uint8_t w[6];
     ir_scan_input_t i={1000U,true,true,false,true};
     CHECK(sample(&line,0U,1000U));ir_scan_step(&scan,&i);
     frame(w,IR_SCAN_PREPARE,0U,0U,0U);CHECK(!ir_scan_command(&scan,&i,w,i.now_ms));
     frame(w,IR_SCAN_RUN,8U,1000U,1U);CHECK(!ir_scan_command(&scan,&i,w,i.now_ms));
     CHECK(!scan.active); /* A single WHITE sample cannot start. */
     for(uint32_t t=1010U;t<=1100U;t+=10U) CHECK(sample(&line,0U,t));
     i.now_ms=1100U;i.white=ir_line_start_ready(&line);ir_scan_step(&scan,&i);
     frame(w,IR_SCAN_PREPARE,0U,0U,0U);CHECK(!ir_scan_command(&scan,&i,w,i.now_ms));
     frame(w,IR_SCAN_RUN,8U,1000U,1U);CHECK(ir_scan_command(&scan,&i,w,i.now_ms));
     CHECK(scan.active && ir_line_direction(&line)==IR_LINE_FORWARD);
     CHECK(sample(&line,2U,1110U));ir_line_enforce(&scan,&line);
     CHECK(scan.active && ir_line_direction(&line)==IR_LINE_LEFT);
     CHECK(sample(&line,1U,1120U));ir_line_enforce(&scan,&line);
     CHECK(scan.active && ir_line_direction(&line)==IR_LINE_RIGHT);
     CHECK(sample(&line,3U,1130U));ir_line_enforce(&scan,&line);
     CHECK(!scan.active && scan.reason==IR_SCAN_LINE_AMBIGUOUS && scan.power==0U);
     CHECK(sample(&line,0U,1140U));ir_line_enforce(&scan,&line);CHECK(!scan.active);
     i.now_ms=1140U;frame(w,IR_SCAN_RENEW,0U,0U,2U);
     CHECK(!ir_scan_command(&scan,&i,w,i.now_ms) && !scan.active);}
    {ir_scan_state_t scan={0};ir_line_trace_t line={0};
     scan.active=1U;scan.scan=0U;ir_line_enforce(&scan,&line);CHECK(scan.active);}
    /* Stable WHITE is a start condition, not proof of line presence. The
     * actual centred and off-track all-white inputs cannot be distinguished. */
    {ir_line_trace_t line={0};
     for(uint32_t t=1000U;t<=2000U;t+=10U) {
         CHECK(sample(&line,0U,t));CHECK(ir_line_direction(&line)==IR_LINE_FORWARD);
     }
     CHECK(ir_line_start_ready(&line));
     uint16_t ambiguous[3]={1700U,1700U,0U};
     CHECK(ir_line_observe(&line,&cfg.line,ambiguous,7U,2010U,2010U,true));
     CHECK(!ir_line_start_ready(&line)); /* Neither strong WHITE nor BLACK. */
    }
    printf("Line trace policy: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
