#include <stdio.h>
#include <string.h>
#include "../../app_program/ir_motor_scan.h"
static unsigned checks,failures;
#define CHECK(x) do {checks++;if(!(x)){failures++;printf("FAIL %d: %s\n",__LINE__,#x);}}while(0)
static ir_scan_input_t input={0U,true,true,true,true};
static uint8_t wire[6];
static bool command(ir_scan_state_t *s,unsigned op,unsigned power,unsigned ms,unsigned seq) {
    wire[0]=0xD3U;wire[1]=(uint8_t)op;wire[2]=(uint8_t)power;
    wire[3]=(uint8_t)ms;wire[4]=(uint8_t)(ms>>8);wire[5]=(uint8_t)seq;
    return ir_scan_command(s,&input,wire,input.now_ms);
}
static ir_scan_state_t running(bool scan) {
    ir_scan_state_t s={0};input=(ir_scan_input_t){1000U,true,true,true,true};
    ir_scan_step(&s,&input);CHECK(!command(&s,IR_SCAN_PREPARE,0U,0U,0U));
    CHECK(s.prepared && !s.active);
    CHECK(command(&s,scan?IR_SCAN_RUN:IR_SCAN_PULSE,15U,scan?5000U:200U,1U));
    CHECK(s.active && s.power==15U);return s;
}
int main(void) {
    ir_scan_state_t s={0};
    CHECK(!command(&s,IR_SCAN_RUN,15U,5000U,0U) && !s.active);
    for(unsigned p=0U;p<256U;p++) {
        uint8_t f[6]={0xD3U,IR_SCAN_RUN,(uint8_t)p,0x88U,0x13U,0U};
        CHECK(ir_scan_frame_valid(f)==(p>0U && p<=20U));
    }
    {uint8_t f[6]={0xD3U,IR_SCAN_RUN,15U,0x71U,0x17U,0U};CHECK(!ir_scan_frame_valid(f));}
    s=running(false);
    for(input.now_ms=1010U;input.now_ms<=1200U;input.now_ms+=10U) ir_scan_step(&s,&input);
    CHECK(!s.active && s.reason==IR_SCAN_COMPLETE);
    CHECK(!command(&s,IR_SCAN_RENEW,0U,0U,2U) && !s.active);
    s=running(true);
    for(input.now_ms=1010U;input.now_ms<=1250U;input.now_ms+=10U) ir_scan_step(&s,&input);
    CHECK(!s.active && s.reason==IR_SCAN_USB_LOST);
    s=running(true);input.now_ms=1249U; /* Advance each normal tick to isolate heartbeat. */
    s.last_tick_ms=1239U;ir_scan_step(&s,&input);
    CHECK(s.active);CHECK(!command(&s,IR_SCAN_RENEW,0U,0U,2U));
    input.now_ms=1250U;ir_scan_step(&s,&input);CHECK(s.active);
    input.now_ms=1499U;s.last_tick_ms=1489U;ir_scan_step(&s,&input);
    CHECK(!s.active && s.reason==IR_SCAN_USB_LOST);
    s=running(true);input.now_ms=1200U;s.last_tick_ms=1190U;
    CHECK(!command(&s,IR_SCAN_RENEW,0U,0U,1U)); /* Duplicate does not renew. */
    input.now_ms=1250U;s.last_tick_ms=1240U;ir_scan_step(&s,&input);CHECK(!s.active);
    s=running(true);CHECK(!command(&s,IR_SCAN_RENEW,0U,0U,4U));CHECK(!s.active && s.reason==IR_SCAN_BAD_COMMAND);
    s=running(true);input.motor_ok=false;input.now_ms+=10U;ir_scan_step(&s,&input);CHECK(!s.active && s.reason==IR_SCAN_MOTOR_FAULT);
    s=running(true);input.adc_ok=false;input.now_ms+=10U;ir_scan_step(&s,&input);CHECK(!s.active && s.reason==IR_SCAN_ADC_LOST);
    s=running(true);input.front_clear=false;input.now_ms+=10U;ir_scan_step(&s,&input);CHECK(!s.active && s.reason==IR_SCAN_OBSTACLE);
    s=running(false);input.front_clear=false;input.now_ms+=10U;ir_scan_step(&s,&input);CHECK(s.active);
    s=running(true);input.now_ms+=30U;ir_scan_step(&s,&input);CHECK(!s.active && s.reason==IR_SCAN_CLOCK);
    s=running(true);ir_scan_stop(&s,IR_SCAN_REQUESTED_STOP);
    CHECK(!command(&s,IR_SCAN_RUN,15U,5000U,2U));CHECK(!command(&s,IR_SCAN_RENEW,0U,0U,3U));CHECK(!s.active);
    s=running(true);wire[1]=IR_SCAN_RENEW;wire[2]=wire[3]=wire[4]=0U;wire[5]=2U;
    CHECK(!ir_scan_command(&s,&input,wire,input.now_ms-100U));CHECK(!s.active);
    s=(ir_scan_state_t){0};input=(ir_scan_input_t){0xfffffff0U,true,true,true,true};
    ir_scan_step(&s,&input);CHECK(!command(&s,IR_SCAN_PREPARE,0U,0U,63U));
    CHECK(command(&s,IR_SCAN_PULSE,10U,200U,0U));
    for(unsigned n=1U;n<=20U;n++){input.now_ms=0xfffffff0U+n*10U;ir_scan_step(&s,&input);}
    CHECK(!s.active && s.reason==IR_SCAN_COMPLETE);
    s=running(true);
    unsigned seq=1U;
    for(input.now_ms=1010U;input.now_ms<=6000U;input.now_ms+=10U) {
        ir_scan_step(&s,&input);
        if(input.now_ms%50U==0U && s.active) {seq=(seq+1U)&63U;(void)command(&s,IR_SCAN_RENEW,0U,0U,seq);}
    }
    CHECK(!s.active && s.reason==IR_SCAN_COMPLETE); /* Heartbeat cannot extend duration. */
    s=(ir_scan_state_t){0};input=(ir_scan_input_t){1000U,true,true,false,false};
    CHECK(!command(&s,IR_SCAN_PREPARE,0U,0U,0U) && s.prepared);
    CHECK(command(&s,IR_SCAN_PULSE,15U,200U,1U)); /* Raised wheels need no WHITE/front target. */
    ir_scan_stop(&s,IR_SCAN_REQUESTED_STOP);
    CHECK(!command(&s,IR_SCAN_PREPARE,0U,0U,0U) && s.prepared);
    CHECK(!command(&s,IR_SCAN_RUN,15U,5000U,1U) && !s.active);
    printf("IR motor scan policy: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
