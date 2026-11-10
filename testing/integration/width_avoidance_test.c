#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include "../../app_program/subsystem5/s5_width.h"
#include "../../app_program/subsystem5/s5_policy.h"
#include "../../app_program/subsystem5/s5_data.h"
#include "../../app_program/subsystem1/telemetry_edr.h"
#include "../../app_program/vehicle_policy.h"
#include "../../app_program/vehicle_motion_handoff.h"
static s5_status_t measured(void){return(s5_status_t){.state=S5_READY,.scan_id=7,.angle_deg=0,.sample_angle_deg=0,.valid=1,.distance_mm=150,.obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,.left_bound_deg=30,.right_bound_deg=-30,.left_bound_distance_mm=173,.right_bound_distance_mm=173};}
static void width_packet(uint8_t packet[2]){
 s5_status_t scan=measured();s5_width_from_bounds(&scan);assert(scan.object_width_valid && scan.object_width_mm==172 && scan.object_width_max_mm==181 && scan.object_left_mm==86 && scan.object_right_mm==86);
 uint8_t private_scan[S5_DATA_SIZE];s5_status_t decoded;assert(s5_data_encode(&scan,private_scan,sizeof private_scan));assert(s5_data_decode(private_scan,sizeof private_scan,&decoded));assert(decoded.object_width_mm==172 && decoded.object_width_valid && decoded.object_left_mm==86 && decoded.object_right_mm==86);
 int64_t v[TEL_FIELD_COUNT]={0},copy[TEL_FIELD_COUNT]={0};uint8_t snapshot[TELEMETRY_BYTES],half[2];v[TEL_VERSION]=1;v[TEL_WIDTH_MM]=scan.object_width_mm;v[TEL_WIDTH_LEFT_MM]=scan.object_left_mm;v[TEL_WIDTH_RIGHT_MM]=scan.object_right_mm;v[TEL_QUALITY]=1;
 assert(telemetry_encode(snapshot,v));assert(telemetry_decode(snapshot,copy));assert(telemetry_edr_record(copy,23,packet));unsigned id,data;assert(edr_decode(packet,2,&id,&data));assert(id==0 && data==(8U<<8|34U));
 assert(telemetry_edr_record(copy,31,half) && edr_decode(half,2,&id,&data) && id==0 && data==(8U<<8|128U|11U));
 assert(telemetry_edr_record(copy,32,half) && edr_decode(half,2,&id,&data) && id==0 && data==(8U<<8|192U|11U));
 copy[TEL_QUALITY]=0;assert(!telemetry_edr_record(copy,31,half) && !telemetry_edr_record(copy,32,half));
}
static void geometry(void){
 s5_status_t scan=measured();assert(s5_acquisition_plan(&scan));assert(scan.approach_plan_valid && scan.planned_approach_mm>=50 && scan.planned_return_mm==scan.planned_approach_mm);
 double theta=fabs((double)scan.planned_rotate_deg)*3.141592653589793/180.0;
 double lateral=2.0*S5_BYPASS_ARC_RADIUS_MM*(1.0-cos(theta))+scan.planned_approach_mm*sin(theta);
 assert(lateral+1.0>=(scan.object_width_max_mm+1U)/2U+S5_CLEARANCE_RADIUS_MM);
 scan=measured();scan.right_bound_valid=0;assert(!s5_acquisition_plan(&scan) && !scan.object_width_valid);
 scan=measured();scan.left_bound_deg=70;scan.right_bound_deg=-70;scan.obstacle_distance_mm=200;assert(!s5_acquisition_plan(&scan) && !scan.object_width_valid);
}
static vc_input_t good(void){return(vc_input_t){.line_fresh=1,.steering=1,.front_clear=1,.sensor_fresh=1,.odom_valid=1,.terrain_valid=1,.motor_ok=1,.transport_ok=1,.bypass_enabled=1,.bypass_steering=376,.width_valid=1,.obstacle_width_mm=200};}
static void route(int sign){
 /* t: TURN_OUT entry, after the reverse and the stand-still. */
 const uint32_t t=10+AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MS;vc_state_t s={0};vc_input_t i=good();assert(vc_start(&s,0,30));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=(int16_t)(sign*40);i.approach_mm=i.return_mm=180;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && s.pass_check_mm==175);
 /* front_clear stays 0 and obstacle 1: the reverse is not gated by the forward sonar.
  * It ramps START -> AVOID_BACKUP_POWER -> START. */
 i.now_ms=15;i.distance_um=-1000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_START_POWER && !s.steering);
 i.now_ms=10+AVOID_BACKUP_MS/2;i.distance_um=-60000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_POWER && !s.steering);
 i.now_ms=10+AVOID_BACKUP_MS-1;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_START_POWER);
 /* Never straight from reverse to forward: stopped for AVOID_BACKUP_BRAKE_MS first. */
 i.now_ms=10+AVOID_BACKUP_MS;i.distance_um=-120000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && s.backup_braking);
 i.now_ms=t-1;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=t;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command && s.backup_mm==120 && s.pass_check_mm==175+120);
 /* The turn-out starts from standstill: soft start from AVOID_BACKUP_START_POWER. */
 i.now_ms=t+20;i.front_clear=1;i.obstacle=i.scan_ready=0;vc_step(&s,&i);assert(s.command==1 && s.power==AVOID_BACKUP_START_POWER && s.steering==-sign*376);
 i.now_ms=t+30;i.yaw_mdeg=sign*40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1 && s.power==AVOID_BACKUP_START_POWER && !s.steering);
 i.now_ms=t+40;i.distance_um=59999;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1);
 i.now_ms=t+50;i.distance_um=60000;vc_step(&s,&i);assert(s.phase==VC_ALIGN && s.command==1 && s.steering==sign*376);
 i.now_ms=t+60;i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_PASS && s.command==1 && !s.look_side && s.pass_check_mm==295);
 i.now_ms=t+70;i.distance_um=354999;vc_step(&s,&i);assert(s.command==1 && !s.look_side);
 i.now_ms=t+80;i.distance_um=355000;vc_step(&s,&i);assert(!s.command && s.look_side);
 i.front_clear=0;i.side_observed=i.side_ready=i.side_clear=1;i.side_sample_ms=i.now_ms=t+100;vc_step(&s,&i);assert(s.side_clear_count==1 && !s.command);
 i.now_ms=t+110;vc_step(&s,&i);assert(s.side_clear_count==1 && !s.command);
 i.side_sample_ms=i.now_ms=t+120;i.side_ready=0;vc_step(&s,&i);assert(!s.side_clear_count && s.look_side);
 i.side_ready=1;
 for(unsigned n=0;n<3;++n){i.side_sample_ms=i.now_ms=t+140U+n*20U;vc_step(&s,&i);assert(!s.command);if(n<2)assert(s.look_side && s.phase==VC_PASS);}
 assert(s.phase==VC_TURN_IN && !s.look_side);
 i.now_ms=t+200;i.front_clear=1;i.side_observed=0;vc_step(&s,&i);assert(s.command==1 && s.steering==sign*376);
 i.now_ms=t+220;i.yaw_mdeg=-sign*40000;vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1 && !s.steering);
 i.now_ms=t+240;i.distance_um=535000;vc_step(&s,&i);assert(s.phase==VC_RESTORE && s.command==1 && s.steering==-sign*376);
 i.now_ms=t+260;i.yaw_mdeg=0;vc_step(&s,&i);assert(s.phase==VC_REACQUIRE && !s.command && !s.spot_scan);
}
static void backup(void){
 vc_state_t s={0};vc_input_t i=good();assert(vc_start(&s,0,40));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=40;i.approach_mm=i.return_mm=180;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=20;i.distance_um=-299999;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_START_POWER && !s.steering);
 i.now_ms=30;i.distance_um=-(int32_t)AVOID_BACKUP_MAX_MM*1000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && s.backup_braking);
 i.now_ms=30+AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command && s.backup_mm==AVOID_BACKUP_MAX_MM && s.pass_check_mm==175+AVOID_BACKUP_MAX_MM);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.pass_check_mm=175};i=good();i.now_ms=50;i.distance_um=-(int32_t)AVOID_BACKUP_MAX_MM*2000-1;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && !s.command);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30};i=good();i.front_clear=0;i.obstacle=i.slope=1;i.now_ms=AVOID_BACKUP_MS/2;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==20);
 i.sensor_fresh=0;i.now_ms=AVOID_BACKUP_MS/2+10;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && !s.command && !s.armed);
 /* Stay stopped while S5 is still in its scan (its motion guard would drop
  * the published plan/widths); the 1 s reverse starts when it lets go. */
 s=(vc_state_t){0};i=good();assert(vc_start(&s,0,40));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=40;i.approach_mm=i.return_mm=180;i.sensor_acquiring=1;
 vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=20;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && !s.backup_moving);
 i.now_ms=60;i.sensor_acquiring=0;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.backup_moving);
 /* counted from the last stopped tick (20 ms), not from entry (10 ms) */
 i.now_ms=1019;i.distance_um=-100000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2);
 i.now_ms=1020;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.backup_braking && !s.command);
 i.now_ms=1020+AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && s.backup_mm==100);
 /* Ramp profile: START at both ends, AVOID_BACKUP_POWER in the middle, never above it. */
 {
  static const struct {uint32_t t;unsigned power;} ramp[]={
   {0,AVOID_BACKUP_START_POWER},
   {AVOID_BACKUP_RAMP_MS/2,AVOID_BACKUP_START_POWER+(AVOID_BACKUP_POWER-AVOID_BACKUP_START_POWER)/2},
   {AVOID_BACKUP_RAMP_MS,AVOID_BACKUP_POWER},{AVOID_BACKUP_MS/2,AVOID_BACKUP_POWER},
   {AVOID_BACKUP_MS-AVOID_BACKUP_RAMP_MS,AVOID_BACKUP_POWER},
   {AVOID_BACKUP_MS-AVOID_BACKUP_RAMP_MS/2,AVOID_BACKUP_START_POWER+(AVOID_BACKUP_POWER-AVOID_BACKUP_START_POWER)/2},
   {AVOID_BACKUP_MS-1,AVOID_BACKUP_START_POWER}};
  for(unsigned n=0;n<sizeof(ramp)/sizeof(ramp[0]);++n) {
   s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=40,.backup_moving=1,.backup_started_ms=1000};i=good();
   i.now_ms=1000+ramp[n].t;i.distance_um=-50000;vc_step(&s,&i);
   assert(s.phase==VC_BACKUP && s.command==2 && s.power==ramp[n].power);
  }
 }
 /* Still coasting after the reverse: the turn-out waits until odometry has
  * been still for AVOID_BACKUP_STILL_MS, and coasting counts as reversed. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1,.backup_started_ms=0,.pass_check_mm=175};i=good();
 i.now_ms=AVOID_BACKUP_MS;i.distance_um=-150000;vc_step(&s,&i);assert(s.backup_braking && !s.command);
 i.now_ms=AVOID_BACKUP_MS+250;i.distance_um=-150000-AVOID_BACKUP_STILL_UM-1;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MS;i.distance_um-=AVOID_BACKUP_STILL_UM; /* within STILL_UM: still */
 vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command); /* only 50 ms still */
 i.now_ms=AVOID_BACKUP_MS+250+AVOID_BACKUP_STILL_MS-1;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 i.now_ms=AVOID_BACKUP_MS+250+AVOID_BACKUP_STILL_MS;vc_step(&s,&i);
 assert(s.phase==VC_TURN_OUT && !s.command && s.backup_mm==153 && s.pass_check_mm==175+153); /* 152.001 mm rounds up */
 /* Tilt drops out while reversing (fore-aft rocking): VC_BACKUP carries on,
  * and the stand-still still ends normally; TURN_OUT then requires tilt again. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1,.backup_started_ms=0};i=good();i.terrain_valid=0;
 i.now_ms=AVOID_BACKUP_MS/2;i.distance_um=-60000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.armed);
 i.now_ms=AVOID_BACKUP_MS;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.backup_braking && s.armed);
 i.now_ms=AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && s.armed);
 i.now_ms+=10;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_TILT);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1,.backup_started_ms=0,.turn_deg=40,.bypass_steering=376};i=good();
 i.now_ms=AVOID_BACKUP_MS;i.distance_um=-60000;vc_step(&s,&i);
 i.now_ms=AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT);
 i.now_ms+=10;i.front_clear=1;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && s.command==1); /* tilt back: turns out */
 /* ...but never stands longer than AVOID_BACKUP_BRAKE_MAX_MS, even if odometry keeps creeping. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1,.backup_started_ms=0};i=good();
 i.now_ms=AVOID_BACKUP_MS;i.distance_um=-150000;vc_step(&s,&i);assert(s.backup_braking);
 for(uint32_t ms=20;ms<AVOID_BACKUP_BRAKE_MAX_MS;ms+=20) {
  i.now_ms=AVOID_BACKUP_MS+ms;i.distance_um-=AVOID_BACKUP_STILL_UM+1;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 }
 i.now_ms=AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MAX_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command);
 /* ...but never waits longer than AVOID_BACKUP_SETTLE_MS. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30};i=good();i.sensor_acquiring=1;
 i.now_ms=AVOID_BACKUP_SETTLE_MS;vc_step(&s,&i);assert(s.command==2 && s.backup_moving);
 /* Odometry moving forward while commanding reverse is a motor fault. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30};i=good();i.now_ms=20;
 i.distance_um=(int32_t)AVOID_BACKUP_WRONG_WAY_MM*1000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2);
 i.distance_um+=1;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_MOTOR && !s.command);
 /* The reverse offset delays only the first side look. */
 s=(vc_state_t){.armed=1,.phase=VC_PASS,.maximum_power=30,.look_side=1,.pass_check_mm=175+120,.backup_mm=120};
 i=good();i.front_clear=0;i.side_observed=i.side_ready=1;
 for(unsigned n=1;n<=3;++n){i.now_ms=i.side_sample_ms=n*100U;vc_step(&s,&i);}
 assert(!s.look_side && s.pass_check_mm==175 && !s.backup_mm);
}
static void occupied(void){
 vc_state_t s={.armed=1,.phase=VC_PASS,.maximum_power=30,.look_side=1,.pass_check_mm=150};vc_input_t i=good();i.front_clear=0;i.side_observed=i.side_ready=1;
 for(unsigned n=1;n<=3;++n){i.now_ms=i.side_sample_ms=n*100U;vc_step(&s,&i);assert(!s.command);if(n<3)assert(s.look_side);}
 assert(!s.look_side && s.phase==VC_PASS);i.now_ms=400;i.front_clear=1;i.side_observed=0;vc_step(&s,&i);assert(s.command==1 && s.power==AVOID_BACKUP_START_POWER);
 i.now_ms=400+AVOID_BACKUP_RAMP_MS;vc_step(&s,&i);assert(s.command==1 && s.power==25 && !s.soft_start); /* pass resumes, ramped */
 s=(vc_state_t){.armed=1,.phase=VC_PASS,.maximum_power=30,.look_side=1};i=good();i.front_clear=0;i.now_ms=4000;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && !s.command);
 assert(!vehicle_sensor_pan_requires_stop(1,1,0,0,0));assert(vehicle_sensor_pan_requires_stop(1,1,0,0,90));assert(vehicle_sensor_pan_requires_stop(0,1,0,0,0));assert(vehicle_sensor_pan_requires_stop(1,0,0,0,0));
}
/* Avoidance arcs ramp up whenever they start from standstill, and the
 * reverse's guards keep running through its stand-still. */
static void soft(void){
 vc_state_t s={.armed=1,.phase=VC_TURN_OUT,.maximum_power=30,.turn_deg=40,.bypass_steering=376,.entered_ms=1000};vc_input_t i=good();
 i.now_ms=1010;vc_step(&s,&i);assert(s.command==1 && s.power==AVOID_BACKUP_START_POWER && s.soft_start);
 i.now_ms=1010+AVOID_BACKUP_RAMP_MS/2;vc_step(&s,&i);
 assert(s.power==AVOID_BACKUP_START_POWER+(AVOID_POWER-AVOID_BACKUP_START_POWER)/2);
 i.now_ms=1010+AVOID_BACKUP_RAMP_MS;vc_step(&s,&i);assert(s.power==AVOID_POWER && !s.soft_start);
 i.now_ms+=10;vc_step(&s,&i);assert(s.power==AVOID_POWER);
 /* A front-clear hold stops it; the restart ramps again. */
 i.front_clear=0;i.now_ms+=10;vc_step(&s,&i);assert(!s.command);
 i.front_clear=1;i.now_ms+=10;vc_step(&s,&i);assert(s.command==1 && s.power==AVOID_BACKUP_START_POWER);
 /* Continuous phase changes while driving keep the ramp going, not restart it. */
 i.now_ms+=AVOID_BACKUP_RAMP_MS;vc_step(&s,&i);assert(s.power==AVOID_POWER);
 i.now_ms+=10;i.yaw_mdeg=40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1 && s.power==AVOID_POWER);
 /* Stand-still guards: over-distance and wrong-way still fault while braking. */
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1};i=good();
 i.now_ms=AVOID_BACKUP_MS;i.distance_um=-150000;vc_step(&s,&i);assert(s.backup_braking);
 i.now_ms+=10;i.distance_um=-(int32_t)AVOID_BACKUP_MAX_MM*2000-1;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && !s.command);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.backup_moving=1};i=good();
 i.now_ms=AVOID_BACKUP_MS;i.distance_um=-150000;vc_step(&s,&i);assert(s.backup_braking);
 i.now_ms+=10;i.distance_um=(int32_t)AVOID_BACKUP_WRONG_WAY_MM*1000+1;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_MOTOR && !s.command);
 /* The stand-still flag ends with it: a second obstacle reverses again. */
 s=(vc_state_t){0};i=good();assert(vc_start(&s,0,30));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=40;i.approach_mm=i.return_mm=180;vc_step(&s,&i);
 i.now_ms=20;vc_step(&s,&i);i.now_ms=10+AVOID_BACKUP_MS;i.distance_um=-100000;vc_step(&s,&i);assert(s.backup_braking);
 i.now_ms+=AVOID_BACKUP_BRAKE_MS;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.backup_braking);
 s.phase=VC_TRACK;i.now_ms+=10;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE);
 i.now_ms+=10;vc_step(&s,&i);assert(s.phase==VC_BACKUP);
 i.now_ms+=10;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && !s.backup_braking);
}
/* VC_SENSOR keeps why it faulted: S5 scan rejections are sonar (1), other
 * manoeuvre sanity checks carry 0; vc_start clears it. */
static void fault_causes(void){
 vc_state_t s={0};vc_input_t i=good();assert(vc_start(&s,0,30) && !s.fault_cause);i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE);
 i.now_ms=10;i.scan_blocked=1;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR);
 i.now_ms=20;vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault_cause==VC_CAUSE_SONAR);
 s=(vc_state_t){0};i=good();assert(vc_start(&s,0,30));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=40;i.approach_mm=180;i.width_valid=0;vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR);
 s=(vc_state_t){0};i=good();assert(vc_start(&s,0,30));i.obstacle=1;i.front_clear=0;vc_step(&s,&i);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=90;i.approach_mm=180;vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR);
 s=(vc_state_t){.armed=1,.phase=VC_DIAGONAL,.maximum_power=30,.approach_mm=100,.fault_cause=VC_CAUSE_TILT};i=good();i.distance_um=1000001;
 vc_step(&s,&i);assert(s.phase==VC_FAULT && s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_OTHER);
 s=(vc_state_t){.armed=1,.phase=VC_BACKUP,.maximum_power=30,.fault_cause=VC_CAUSE_SONAR};i=good();i.now_ms=50;i.distance_um=-(int32_t)AVOID_BACKUP_MAX_MM*2000-1;
 vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_OTHER);
 s=(vc_state_t){.armed=1,.phase=VC_PASS,.maximum_power=30,.look_side=1,.fault_cause=VC_CAUSE_ODOM};i=good();i.front_clear=0;i.now_ms=4000;
 vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_OTHER);
 s=(vc_state_t){0};i=good();assert(vc_start(&s,0,30));i.now_ms=10;i.odom_valid=0;vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_ODOM);
 vc_stop(&s,VC_OK);assert(!s.fault_cause);s.fault_cause=VC_CAUSE_SONAR;assert(vc_start(&s,20,30) && !s.fault_cause);
}
int main(int argc,char **argv){uint8_t bytes[2];width_packet(bytes);if(argc>1 && !strcmp(argv[1],"--packet")){printf("%02x%02x\n",(unsigned)bytes[0],(unsigned)bytes[1]);return 0;}geometry();route(1);route(-1);backup();occupied();soft();fault_causes();puts("PASS: estimated width -> EDR16; width-based legs; bounded reverse before turn-out; continuous mirrored forward phases; 3 fresh consistent side readings before resume; cached/invalid/occupied checks and pan stops.");return 0;}
