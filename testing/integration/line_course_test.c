#include <stdio.h>
#include <string.h>
#include "../../app_program/ir_line_course.h"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL course line %u: %s\n",(unsigned)__LINE__,#x);}}while(0)
static const s3_line_config_t cfg={4095U,{187U,189U,197U},{3214U,3177U,2472U},
                                  {1000,-1000,0},3U,400U,600U,100U,500U};
static unsigned tick(ir_line_course_t *s,ir_line_trace_t *line,unsigned mask,uint32_t t) {
    uint16_t raw[3]={mask&1U?3214U:187U,mask&2U?3177U:189U,2472U};
    CHECK(ir_line_observe(line,&cfg,raw,7U,t,t,true));
    return ir_line_course_step(s,line,t,true);
}
static void crossing(uint32_t start) {
    ir_line_course_t s={0};ir_line_trace_t line={0};
    CHECK(tick(&s,&line,0U,start)==IR_LINE_FORWARD);
    CHECK(tick(&s,&line,3U,start+10U)==IR_LINE_FORWARD);
    CHECK(s.phase==IR_COURSE_CROSS_STRAIGHT && !s.fault);
    for(uint32_t dt=20U;dt<=100U;dt+=10U)
        CHECK(tick(&s,&line,(dt/10U)%3U+1U,start+dt)==IR_LINE_FORWARD);
    CHECK(tick(&s,&line,0U,start+110U)==IR_LINE_FORWARD);
    CHECK(tick(&s,&line,0U,start+120U)==IR_LINE_FORWARD);
    CHECK(tick(&s,&line,0U,start+130U)==IR_LINE_FORWARD);
    CHECK(s.phase==IR_COURSE_CROSS_STRAIGHT);
    CHECK(tick(&s,&line,0U,start+140U)==IR_LINE_FORWARD);
    CHECK(s.phase==IR_COURSE_TRACK && !s.fault);
}
int main(void) {
    crossing(1000U);crossing(0xfffffff0U);
    for(unsigned side=1U;side<=2U;++side) {
        ir_line_course_t s={0};ir_line_trace_t line={0};
        unsigned direction=side==2U?IR_LINE_LEFT:IR_LINE_RIGHT;
        CHECK(tick(&s,&line,0U,1000U)==IR_LINE_FORWARD);
        for(uint32_t t=1010U;t<=1070U;t+=10U) CHECK(tick(&s,&line,side,t)==direction);
        CHECK(tick(&s,&line,3U,1080U)==direction);
        CHECK(s.phase==(side==2U?IR_COURSE_HOLD_LEFT:IR_COURSE_HOLD_RIGHT));
        CHECK(tick(&s,&line,3U,1090U)==direction);
        CHECK(tick(&s,&line,side==2U?1U:2U,1100U)==direction);
        for(uint32_t t=1110U;t<=1130U;t+=10U) CHECK(tick(&s,&line,0U,t)==direction);
        CHECK(tick(&s,&line,0U,1140U)==IR_LINE_FORWARD);
        CHECK(s.phase==IR_COURSE_TRACK && !s.fault);
    }
    /* A short asymmetric crossing edge must not latch a turn. */
    for(unsigned side=1U;side<=2U;++side) {
        ir_line_course_t s={0};ir_line_trace_t line={0};
        (void)tick(&s,&line,side,1000U);(void)tick(&s,&line,side,1010U);
        CHECK(tick(&s,&line,3U,1020U)==IR_LINE_FORWARD);
        CHECK(s.phase==IR_COURSE_CROSS_STRAIGHT);
    }
    /* Alternating or WHITE-broken side history is not a sustained side. */
    for(unsigned style=0U;style<2U;++style) {
        ir_line_course_t s={0};ir_line_trace_t line={0};
        for(uint32_t t=1000U;t<=1100U;t+=10U)
            (void)tick(&s,&line,style?((t/10U)%2U?0U:2U):((t/10U)%2U?1U:2U),t);
        CHECK(tick(&s,&line,3U,1110U)==IR_LINE_FORWARD);
    }
    /* White flicker and repeated BLACK never extend a phase deadline. */
    for(unsigned corner=0U;corner<=1U;++corner) {
        ir_line_course_t s={0};ir_line_trace_t line={0};
        uint32_t entered=1080U;
        for(uint32_t t=1000U;t<entered;t+=10U) (void)tick(&s,&line,corner?2U:0U,t);
        (void)tick(&s,&line,3U,entered);
        uint32_t bound=corner?IR_COURSE_TURN_MAX_MS:IR_COURSE_CROSS_MAX_MS;
        for(uint32_t dt=10U;dt<bound;dt+=10U) {
            unsigned mask=dt%40U<20U?0U:3U;
            CHECK(tick(&s,&line,mask,entered+dt)!=IR_LINE_STOP);
            CHECK(s.entered_ms==entered);
        }
        CHECK(tick(&s,&line,3U,entered+bound)==IR_LINE_STOP);
        CHECK(s.fault==IR_SCAN_COURSE_TIMEOUT);
        CHECK(tick(&s,&line,0U,entered+bound+10U)==IR_LINE_STOP);
        CHECK(ir_line_course_step(&s,&line,entered+bound+10U,false)==IR_LINE_STOP);
        CHECK(s.phase==IR_COURSE_TRACK && !s.fault);
    }
    /* Held samples do not count as new WHITE observations. */
    {ir_line_course_t s={0};ir_line_trace_t line={0};
     (void)tick(&s,&line,3U,1000U);(void)tick(&s,&line,0U,1010U);
     for(uint32_t now=1011U;now<1040U;++now) {
         CHECK(ir_line_course_step(&s,&line,now,true)==IR_LINE_FORWARD);
         CHECK(s.phase==IR_COURSE_CROSS_STRAIGHT);
     }
     CHECK(ir_line_course_step(&s,&line,1040U,true)==IR_LINE_STOP);
     CHECK(s.fault==IR_SCAN_ADC_LOST);}
    {ir_line_course_t s={0};ir_line_trace_t line={0};
     (void)tick(&s,&line,3U,1000U);
     CHECK(ir_line_course_step(&s,&line,1001U,false)==IR_LINE_STOP);
     CHECK(s.phase==IR_COURSE_TRACK && !s.fault);
     line.usable=false;CHECK(ir_line_course_step(&s,&line,1001U,true)==IR_LINE_STOP);
     CHECK(s.fault==IR_SCAN_ADC_LOST);
     CHECK(ir_line_course_step(NULL,&line,1001U,true)==IR_LINE_STOP);
     ir_line_course_reset(&s);CHECK(ir_line_course_step(&s,NULL,1001U,true)==IR_LINE_STOP);}
    /* Course output cannot bypass the existing scan lease or motor limits. */
    {ir_scan_state_t scan={0};ir_scan_input_t input={1000U,true,true,true,true};
     uint8_t prepare[6]={0xD3U,IR_SCAN_PREPARE,0U,0U,0U,0U};
     uint8_t start[6]={0xD3U,IR_SCAN_RUN,50U,0x20U,0x4eU,1U};
     ir_line_course_t s={0};ir_line_trace_t line={0};
     ir_scan_step(&scan,&input);CHECK(!ir_scan_command(&scan,&input,prepare,1000U));
     CHECK(ir_scan_command(&scan,&input,start,1000U));
     for(uint32_t t=1000U;t<=1250U;t+=10U) {
         input.now_ms=t;ir_scan_step(&scan,&input);
         (void)tick(&s,&line,3U,t);
         unsigned output=ir_line_course_step(&s,&line,t,scan.active && scan.scan);
         CHECK(t<1250U?output==IR_LINE_FORWARD:output==IR_LINE_STOP);
     }
     CHECK(!scan.active && scan.reason==IR_SCAN_USB_LOST && scan.power==0U);}
    printf("Line course turns/cross: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
