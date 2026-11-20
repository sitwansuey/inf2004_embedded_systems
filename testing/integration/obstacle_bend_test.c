#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_policy.h"
/* An obstacle met on a bend (the left straddle sensor on black). */
static vc_input_t input(void) {
 vc_input_t i={0};i.motor_ok=i.transport_ok=i.sensor_fresh=i.backup_ready=1;
 i.terrain_valid=i.odom_valid=i.front_clear=i.line_fresh=i.steering=1;
 i.line_black_mask=2;i.bypass_enabled=1;i.bypass_steering=376;i.width_valid=1;i.obstacle_width_mm=200;
 return i;
}
static void route(int sign) {
 vc_state_t s={0};vc_input_t i=input();assert(vc_start(&s,0,16));
 i.obstacle=1;i.front_clear=0;vc_step(&s,&i);assert(s.phase==VC_ACQUIRE && !s.command && s.avoid_yaw==0);
 i.now_ms=10;i.scan_ready=1;i.turn_deg=(int16_t)(sign*40);i.approach_mm=300;i.return_mm=300;
 vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command);
 /* Straight reverse, ramped, then under the 16% cap mid-way, while the obstacle is still ahead. */
 i.now_ms=20;i.distance_um=-10000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==AVOID_BACKUP_START_POWER && !s.steering);
 i.now_ms=20+AVOID_BACKUP_MS/2;i.distance_um=-50000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && s.command==2 && s.power==16 && !s.steering);
 uint32_t t=10+AVOID_BACKUP_MS+AVOID_BACKUP_BRAKE_MS;
 /* Reverse done: stand still, then turn out (the plan is mirrored for the return). */
 i.now_ms=20+AVOID_BACKUP_MS;i.distance_um=-100000;vc_step(&s,&i);assert(s.phase==VC_BACKUP && !s.command && s.backup_braking);
 i.now_ms=10+t;vc_step(&s,&i);assert(s.phase==VC_TURN_OUT && !s.command && s.approach_mm==300 && s.return_mm==300);
 i.now_ms=t+20;i.obstacle=i.scan_ready=0;i.front_clear=1;i.line_black_mask=0;
 vc_step(&s,&i);assert(s.command==1 && s.steering==-sign*376 && s.power==AVOID_BACKUP_START_POWER);
 i.now_ms=t+30;i.yaw_mdeg=sign*40000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1 && s.power==AVOID_BACKUP_START_POWER);
 /* End of the diagonal: stop and look to the obstacle side. */
 i.now_ms=t+40;i.distance_um=200000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && !s.command && s.look_side && s.return_mm==300);
 i.front_clear=0;i.side_observed=i.side_ready=i.side_clear=1;
 for(unsigned n=0;n<3;++n){i.now_ms=i.side_sample_ms=t+60U+n*20U;vc_step(&s,&i);assert(!s.command);}
 /* Clear: at least AVOID_CLEAR_RUN_MM and AVOID_CLEAR_RUN_MS straight on,
  * then straight back through the heading plus the 45 deg return heading
  * (AVOID_RETURN_MIN_DEG). */
 assert(s.phase==VC_DIAGONAL && s.clear_run && !s.look_side);
 i.now_ms=t+120;i.front_clear=1;i.side_observed=0;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1 && !s.steering);
 i.now_ms=t+130;i.distance_um+=AVOID_CLEAR_RUN_MM*1000;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1);
 /* The distance is done; the run lasts AVOID_CLEAR_RUN_MS from its first drive tick (t+120). */
 i.now_ms=t+119+AVOID_CLEAR_RUN_MS;vc_step(&s,&i);assert(s.phase==VC_DIAGONAL && s.command==1);
 t+=AVOID_CLEAR_RUN_MS;
 i.now_ms=t+140;vc_step(&s,&i);
 assert(s.phase==VC_TURN_IN && s.direct_return && s.command==1 && s.steering==sign*376 && s.return_mm==300+AVOID_CLEAR_RUN_MM);
 i.now_ms=t+150;i.yaw_mdeg=-sign*44000;vc_step(&s,&i);assert(s.phase==VC_TURN_IN && s.command==1);
 i.now_ms=t+160;i.yaw_mdeg=-sign*45000;vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1 && !s.steering);
 /* No line on the way back: on past the expected distance (the 320 mm
  * diagonal at 40 deg is 206 mm off the line: 291 mm at 45 deg) by
  * AVOID_RETURN_EXTRA_MM, then restore the heading and search. */
 i.now_ms=t+170;i.distance_um+=(291+AVOID_RETURN_EXTRA_MM)*1000-1;vc_step(&s,&i);assert(s.phase==VC_RETURN && s.command==1);
 i.now_ms=t+180;i.distance_um+=1;vc_step(&s,&i);
 assert(s.phase==VC_RESTORE && !s.line_found && s.restore_mdeg==sign*45000 && s.command==1 && s.steering==-sign*376);
 i.now_ms=t+190;i.yaw_mdeg=0;vc_step(&s,&i);
 assert(s.phase==VC_REACQUIRE && !s.command && s.spot_scan && s.search_right_first==(sign>0));
 /* The spot search turns toward the line's side first. */
 i.line_detected=i.steering=0;i.now_ms=t+200;vc_step(&s,&i);assert(s.command==(sign>0?4:3) && s.power<=15);
 vc_stop(&s,0);assert(!s.command && !s.power && !s.steering && !s.armed);
}
int main(void) {
 route(1);route(-1);
 vc_state_t s={0};vc_input_t i=input();assert(vc_start(&s,0,16));
 i.line_black_mask=1;vc_step(&s,&i);i.line_black_mask=3;vc_step(&s,&i);
 assert(s.last_line_side==1 && s.command==1);
 i.obstacle=1;vc_step(&s,&i);i.scan_ready=1;i.turn_deg=40;i.approach_mm=300;
 i.bypass_enabled=0;vc_step(&s,&i);assert(s.fault==VC_UNCALIBRATED && !s.command);
 /* No fresh sonar sample: hold still, a sonar fault only after VC_SONAR_GAP_MS. */
 s=(vc_state_t){0};i=input();assert(vc_start(&s,0,16));
 i.sensor_fresh=0;vc_step(&s,&i);assert(s.phase==VC_TRACK && s.armed && !s.command);
 i.now_ms+=VC_SONAR_GAP_MS;vc_step(&s,&i);assert(s.fault==VC_SENSOR && s.fault_cause==VC_CAUSE_SONAR && !s.command);
 puts("PASS: both-black keeps the last side; bounded straight reverse then mirrored left/right bypass; clear side look at the diagonal end returns straight through 2x the heading; restore and spot search; STOP, sonar gap and disabled bypass stop.");
}
