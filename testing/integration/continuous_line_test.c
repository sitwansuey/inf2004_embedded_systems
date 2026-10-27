#include <stdio.h>
#include "../../app_program/ir_line_trace.h"
static unsigned checks,failures;
#define CHECK(x) do {++checks;if(!(x)){++failures;printf("FAIL continuous line %u: %s\n",(unsigned)__LINE__,#x);}}while(0)
static const s3_line_config_t cfg={4095U,{187U,189U,197U},{3214U,3177U,2472U},{1000,-1000,0},3U,400U,600U,100U,500U};
int main(void) {
 CHECK(IR_SCAN_MAX_POWER==30U && IR_SCAN_POWER_BITS==5 && IR_SCAN_MAX_MS==20000U);
 for(unsigned p=0;p<=255U;++p) {
  uint8_t w[6]={0xd3U,IR_SCAN_RUN,(uint8_t)p,0x20U,0x4eU,1U};
  CHECK(ir_scan_frame_valid(w)==(p>=1U && p<=30U));
 }
 ir_scan_state_t scan={0};ir_scan_input_t input={1000U,true,true,true,true};ir_line_trace_t line={0};
 uint8_t prep[6]={0xd3U,IR_SCAN_PREPARE,0U,0U,0U,0U};
 uint8_t run[6]={0xd3U,IR_SCAN_RUN,30U,0x20U,0x4eU,1U};
 ir_scan_step(&scan,&input);CHECK(!ir_scan_command(&scan,&input,prep,1000U));CHECK(ir_scan_command(&scan,&input,run,1000U));
 unsigned seq=1U;
 for(uint32_t dt=0U;dt<20000U;dt+=10U) {
  unsigned mask=(dt/100U)%4U;
  uint16_t raw[3]={mask&1U?3214U:187U,mask&2U?3177U:189U,2472U};
  input.now_ms=1000U+dt;
  CHECK(ir_line_observe(&line,&cfg,raw,7U,input.now_ms,input.now_ms,true));
  ir_scan_step(&scan,&input);CHECK(scan.active && scan.power==30U);
  unsigned direction=ir_line_continuous_direction(&line);
  CHECK(direction==(mask==2U?IR_LINE_LEFT:mask==1U?IR_LINE_RIGHT:IR_LINE_FORWARD));
  if(dt && dt%50U==0U) {
   seq=(seq+1U)&63U;uint8_t renew[6]={0xd3U,IR_SCAN_RENEW,0U,0U,0U,(uint8_t)seq};
   CHECK(!ir_scan_command(&scan,&input,renew,input.now_ms));
  }
 }
 input.now_ms=21000U;ir_scan_step(&scan,&input);
 CHECK(!scan.active && scan.reason==IR_SCAN_COMPLETE && scan.power==0U);
 line.usable=false;CHECK(ir_line_continuous_direction(&line)==IR_LINE_STOP);
 CHECK(ir_line_continuous_direction(NULL)==IR_LINE_STOP);
 printf("Continuous30pct/20s all-pattern policy: %u checks, %u failures\n",checks,failures);
 return failures?1:0;
}
