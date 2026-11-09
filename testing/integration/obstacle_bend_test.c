#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_policy.h"
#include "../../app_program/vehicle_curve_memory.h"
static vc_input_t input(void) {
 vc_input_t i={0};i.motor_ok=i.transport_ok=i.sensor_fresh=1;
 i.terrain_valid=i.odom_valid=i.front_clear=i.line_fresh=i.steering=1;
 i.line_black_mask=2;i.line_state=1;i.bypass_enabled=1;i.bypass_steering=376;i.width_valid=1;i.obstacle_width_mm=200;
 return i;
}
static void route(int sign) {
 vc_state_t s={0};vc_input_t i=input();assert(vc_start(&s,0,16));
 i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=(int16_t)(sign*40);i.approach_mm=300;i.return_mm=300;
 vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 /* Straight reverse, under the 16% cap, while the obstacle is still ahead. */
 i.now_ms=20;i.distance_um=-50000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==16 && !s.steering);
 const uint32_t t=AVOID_BACKUP_MS;
 i.now_ms=10+t;i.distance_um=-100000;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command && s.backup_mm==100 && s.pass_check_mm==275);
 i.now_ms=t+20;i.obstacle=i.scan_ready=0;i.front_clear=1;vc_step(&s,&i);assert(s.command==1 && s.steering==-sign*376 && s.power==16);
 i.now_ms=t+30;i.yaw_mdeg=sign*40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1 && s.power==16);
 i.now_ms=t+40;i.distance_um=200000;vc_step(&s,&i);assert(s.phase==VC_ALIGN && s.command==1 && s.return_mm==300);
 i.now_ms=t+50;i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_PASS && s.command==1 && !s.look_side);
 i.now_ms=t+55;i.distance_um+=274999;vc_step(&s,&i);assert(s.command==1 && !s.look_side);
 i.now_ms=t+60;i.distance_um+=1;vc_step(&s,&i);assert(s.look_side && !s.command);
 i.front_clear=0;i.side_observed=i.side_ready=i.side_clear=1;
 for(unsigned n=0;n<3;++n){i.now_ms=i.side_sample_ms=t+80U+n*20U;vc_step(&s,&i);assert(!s.command);}
 assert(s.phase==VC_TURN_IN);
 i.now_ms=t+140;i.front_clear=1;vc_step(&s,&i);assert(s.command==1 && s.steering==sign*376);
 i.now_ms=t+160;i.yaw_mdeg=-sign*40000;vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1);
 i.now_ms=t+180;i.distance_um+=300000;vc_step(&s,&i);assert(s.phase==VC_RESTORE && s.command==1);
 i.now_ms=t+200;i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command && !s.spot_scan);
 i.line_detected=i.steering=i.line_black_mask=0;i.now_ms=t+220;vc_step(&s,&i);assert(s.command==3 || s.command==4);
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
 puts("PASS: both-black cancels bend; bounded straight reverse then mirrored left/right forward bypass; width-derived travel with no boost; return and search; STOP, stale sensor and disabled bypass stop.");
}
