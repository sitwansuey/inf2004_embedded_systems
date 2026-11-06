#include <assert.h>
#include <math.h>
#include <stdio.h>
#include "../../app_program/vehicle_pose.h"
#include "../../app_program/vehicle_policy.h"
static const int32_t a[3]={0,0,1000},m[3]={300,0,0};
static void tracking(void) {
 vehicle_pose_config_t c={false,{0,0,0},{1000,1000,1000}};
 vehicle_pose_t p={0};vp_step(&p,&c,0,0,0,true,a,m,0,true);
 vp_anchor(&p);
 for(int n=1;n<=50;++n)vp_step(&p,&c,(uint32_t)n*20,n*2000,0,true,a,m,(uint64_t)n,true);
 assert(fabsf(p.x_um-100000)<10 && fabsf(p.y_um)<10 && vp_cross_track(&p)==0);
 /* A quarter circle: integrate position, not merely path length. */
 for(int n=1;n<=100;++n)vp_step(&p,&c,1000U+(uint32_t)n*20,100000+n*2000,n*900,true,a,m,(uint64_t)n+50,true);
 assert(fabsf(p.x_um-227325)<100 && fabsf(p.y_um-127325)<100);
 assert(vp_cross_track(&p)>127000 && vp_yaw(&p)>89990);
 p=(vehicle_pose_t){0};vp_step(&p,&c,0,0,179000,true,a,m,0,true);
 vp_step(&p,&c,20,2000,-179000,true,a,m,1,true);
 assert(p.x_um< -1990 && fabsf(p.y_um)<2);
 /* Calibrated compass can correct small encoder drift, once per new sample. */
 c.compass_calibrated=true;p=(vehicle_pose_t){0};
 vp_step(&p,&c,0,0,0,true,a,m,0,true);vp_step(&p,&c,20,0,0,true,a,m,1,true);
 vp_step(&p,&c,40,0,1000,true,a,m,2,true);
 assert(p.imu_assisted && vp_yaw(&p)>970 && vp_yaw(&p)<990);
 int32_t previous=vp_yaw(&p);vp_step(&p,&c,60,0,1000,true,a,m,2,true);
 assert(!p.imu_assisted && vp_yaw(&p)==previous);
 const int32_t disturbed[3]={10000,0,0};vp_step(&p,&c,80,0,1000,true,a,disturbed,3,true);
 assert(!p.imu_assisted && vp_yaw(&p)==previous);
 vp_step(&p,&c,100,0,1000,false,a,m,4,true);assert(!p.valid);
}
static vc_input_t healthy(void) {
 vc_input_t i={0};i.motor_ok=i.transport_ok=i.sensor_fresh=i.terrain_valid=i.odom_valid=1;
 i.front_clear=i.line_fresh=i.pose_valid=1;return i;
}
static void reposition(int sign) {
 vc_state_t s={.phase=VC_RETURN,.armed=1,.maximum_power=35,.pose_return=1,
 .return_angle_deg=(int16_t)(sign*40),.bypass_steering=376,.return_mm=200};
 vc_input_t i=healthy();i.now_ms=20;i.lateral_um=sign*200000;i.distance_um=999000;
 vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1);
 i.now_ms=40;i.lateral_um=sign*40000;vc_step(&s,&i);assert(s.phase==VC_RESTORE);
 i.now_ms=60;i.yaw_mdeg=-sign*40000;vc_step(&s,&i);assert(s.command==1 && s.steering==-sign*376);
 i.now_ms=80;i.yaw_mdeg=0;i.lateral_um=sign*20000;
 vc_step(&s,&i);assert(s.phase==VC_REACQUIRE);
 s=(vc_state_t){.phase=VC_RESTORE,.armed=1,.maximum_power=35,.pose_return=1,.bypass_steering=376};
 i=healthy();i.now_ms=20;i.lateral_um=sign*80000;
 vc_step(&s,&i);assert(s.phase==VC_TURN_IN && s.pose_corrections==1 && s.return_angle_deg==sign*15);
 i.now_ms=40;i.yaw_mdeg=-sign*15000;vc_step(&s,&i);assert(s.phase==VC_RETURN);
 i.now_ms=60;i.lateral_um=sign*10000;vc_step(&s,&i);assert(s.phase==VC_RESTORE);
 i.now_ms=80;i.yaw_mdeg=0;i.lateral_um=sign*5000;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE);
 s.phase=VC_RESTORE;s.pose_corrections=2;i.lateral_um=sign*80000;i.now_ms=100;
 vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_LINE && !s.command);
 s=(vc_state_t){.phase=VC_RETURN,.armed=1,.maximum_power=35,.pose_return=1};
 i=healthy();i.pose_valid=0;i.now_ms=20;vc_step(&s,&i);assert(s.fault==VC_SENSOR && !s.command);
}
int main(void) {
 tracking();reposition(1);reposition(-1);
 puts("PASS: XY displacement/heading wrap; calibrated compass fusion/disturbance fallback; cross-track return rather than mirrored distance; heading gate; bounded left/right corrections; invalid pose stops.");
}
