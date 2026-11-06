#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_policy.h"
static vc_input_t good(void) {
 vc_input_t i={0};i.motor_ok=i.transport_ok=i.sensor_fresh=i.terrain_valid=i.odom_valid=1;
 i.front_clear=i.line_fresh=i.steering=i.pose_valid=1;i.bypass_enabled=1;i.bypass_steering=376;
 return i;
}
static void backup(void) {
 vc_state_t s={0};vc_input_t i=good();assert(vc_start(&s,0,30));
 i.obstacle=1;i.front_distance_mm=200;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=20;i.distance_um=-20000;vc_step(&s,&i);assert(s.command==2 && s.power==15);
 i.now_ms=40;i.distance_um=-(int32_t)VEHICLE_BACKUP_MM*1000;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && s.await_scan && !s.command);
 i.now_ms=100;i.obstacle=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command);
 i.now_ms=200;i.acquisition_active=1;i.obstacle=1;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.await_scan);
 i.now_ms=220;i.scan_ready=1;i.turn_deg=40;i.approach_mm=400;i.return_mm=200;
 vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && s.obstacle_forward_mm==200 && s.bypass_heading==0);
 i.now_ms=240;i.yaw_mdeg=40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30};i=good();i.now_ms=VEHICLE_BACKUP_TIMEOUT_MS;
 vc_step(&s,&i);assert(!s.armed && s.fault==VC_TIMEOUT);
}
static void monitor(int sign) {
 vc_state_t s={.armed=1,.phase=VC_PASS,.maximum_power=30,.pose_return=1,
 .turn_deg=(int16_t)(sign*40),.return_angle_deg=(int16_t)(sign*40),.bypass_steering=376,.obstacle_forward_mm=200};
 vc_input_t i=good();i.along_um=400000;
 for(unsigned n=1;n<=3;++n) {
  i.now_ms=n*100;i.distance_um=(int32_t)n*75000;i.side_ready=0;
  vc_step(&s,&i);assert(s.look_side && !s.command);
  i.now_ms+=20;i.front_clear=0;i.side_ready=1;i.side_clear=1;i.side_sample_ms=n;
  if(n==2) {
   i.side_sample_ms=1;vc_step(&s,&i);assert(s.look_side && s.side_clear_samples==1);
   i.side_sample_ms=2;
  }
  vc_step(&s,&i);assert(!s.command);
  if(n<3)assert(s.phase==VC_PASS && s.side_clear_samples==n);
  i.front_clear=1;
 }
 assert(s.phase==VC_TURN_IN);
 /* A new object observation resets a run of clear views. */
 s.phase=VC_PASS;s.look_side=1;s.side_clear_samples=2;s.last_side_sample_ms=10;
 i=good();i.now_ms=500;i.side_ready=1;i.side_present=1;i.side_sample_ms=11;
 vc_step(&s,&i);assert(s.side_clear_samples==0 && s.side_seen && !s.command);
 s.look_side=0;i.front_clear=0;i.now_ms=520;vc_step(&s,&i);assert(!s.command);
 i.sensor_fresh=0;vc_step(&s,&i);assert(s.fault==VC_SENSOR && !s.command);
}
static void drive_timer(void) {
 vc_state_t s={.armed=1,.phase=VC_DIAGONAL,.maximum_power=30};vc_input_t i=good();
 unsigned views=0;
 for(unsigned n=1;n<1000 && s.phase==VC_DIAGONAL;++n) {
  i.now_ms=n*20;i.front_clear=!s.look_side;i.side_ready=s.look_side;
  if(s.look_side){i.side_sample_ms=i.now_ms;i.side_present=1;++views;}
  vc_step(&s,&i);if(s.command==1)i.distance_um+=1000;
 }
 assert(s.phase==VC_ALIGN && s.return_mm>=150 && views>0 && i.now_ms>3000);
 s=(vc_state_t){.armed=1,.phase=VC_ALIGN,.maximum_power=30,.pose_return=1,.turn_deg=40,.bypass_steering=376};
 i=good();i.now_ms=20;i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_PASS);
}
int main(void) {
 backup();monitor(1);monitor(-1);drive_timer();
 puts("PASS: bounded100mm reverse; await fresh scan; side/front alternation; three distinct clear views; object reacquisition resets count; stale/front blockage stops; driving timer excludes viewing pauses.");
}
