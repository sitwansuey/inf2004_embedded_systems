#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_policy.h"
#include "../../app_program/vehicle_curve_memory.h"
static vc_input_t input(void) {
 vc_input_t i={0};i.motor_ok=i.transport_ok=i.sensor_fresh=1;
 i.terrain_valid=i.odom_valid=i.front_clear=i.line_fresh=i.steering=1;
 i.line_black_mask=2;i.line_state=1;i.bypass_enabled=1;i.bypass_steering=376;
 return i;
}
static void route(int sign) {
 vc_state_t s={0};vc_input_t i=input();assert(vc_start(&s,0,16));
 i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
 assert(s.phase==VC_ACQUIRE && !s.command);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=(int16_t)(sign*40);
 i.approach_mm=300;i.return_mm=100;vc_step(&s,&i);
 assert(s.phase==VC_TURN_OUT && !s.command);
 i.now_ms=20;i.obstacle=0;i.front_clear=1;vc_step(&s,&i);
 assert(s.command==1 && s.steering==-sign*376);
 i.now_ms=30;i.yaw_mdeg=sign*40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL);
 i.now_ms=40;vc_step(&s,&i);assert(s.power==18 && !s.steering);
 /* 2990 ms of drive must not finish the three-second interval. */
 for(unsigned n=1;n<=299;++n) {
  i.now_ms=40+n*10;i.distance_um=(int32_t)n*1000;vc_step(&s,&i);
  assert(s.phase==VC_DIAGONAL && s.power==18);
 }
 i.now_ms=3040;i.distance_um=300000;vc_step(&s,&i);
 assert(s.phase==VC_ALIGN && !s.command && s.return_mm==300);
 i.now_ms=3050;vc_step(&s,&i);assert(s.command==1 && s.steering==sign*376);
 i.yaw_mdeg=-sign*40000;i.now_ms=3060;vc_step(&s,&i);assert(s.phase==VC_RETURN);
 i.distance_um=600000;i.now_ms=3070;vc_step(&s,&i);assert(s.phase==VC_RESTORE);
 i.now_ms=3080;vc_step(&s,&i);assert(s.command==1 && s.steering==-sign*376);
 i.yaw_mdeg=0;i.now_ms=3090;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE);
 i.line_detected=0;i.steering=0;i.line_black_mask=0;i.now_ms=3100;vc_step(&s,&i);
 assert(s.phase==VC_REACQUIRE && (s.command==3 || s.command==4));
 vc_stop(&s,0);assert(!s.command && !s.power && !s.steering && !s.armed);
}
int main(void) {
 curve_memory_t m={0};
 assert(curve_memory_update(&m,10,1,1,1,1000)==250);
 assert(curve_memory_update(&m,20,1,1,3,0)==0 && !m.turning);
 assert(curve_memory_update(&m,30,1,1,0,0)==0);
 assert(curve_memory_update(&m,40,1,1,2,-1000)==0);
 assert(curve_memory_update(&m,50,1,1,5,1000)==250);
 assert(curve_memory_update(&m,60,1,0,1,1000)==0);
 route(1);route(-1);
 vc_state_t s={0};vc_input_t i=input();assert(vc_start(&s,0,16));
 i.line_black_mask=1;vc_step(&s,&i);i.line_black_mask=3;vc_step(&s,&i);
 assert(s.last_line_side==1 && s.command==1);
 i.obstacle=1;vc_step(&s,&i);i.scan_ready=1;i.turn_deg=40;i.approach_mm=300;
 i.bypass_enabled=0;vc_step(&s,&i);assert(s.fault==VC_UNCALIBRATED && !s.command);
 s=(vc_state_t){0};i=input();assert(vc_start(&s,0,16));
 i.sensor_fresh=0;vc_step(&s,&i);assert(s.fault==VC_SENSOR && !s.command);
 puts("PASS: both-black cancels bend; mirrored left/right forward bypass; 3000 ms boost; return and search; STOP, stale sensor and disabled bypass stop.");
}
