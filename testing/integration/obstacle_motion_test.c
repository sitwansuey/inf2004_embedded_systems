#include <assert.h>
#include <stdio.h>
#include <limits.h>
#include "../../app_program/subsystem5/s5_scan.h"
static s5_status_t sample(int ticks) {
 s5_status_t s={.state=S5_SCAN_LEFT,.angle_deg=30,.sample_angle_deg=30,
 .valid=1,.distance_mm=150,.obstacle_distance_mm=150,.encoder_valid=1};
 s.motor1_counts_end=ticks;return s;
}
int main(void) {
 assert(!s5_encoder_displaced(0,3));assert(s5_encoder_displaced(0,4));
 assert(!s5_encoder_displaced(INT32_MAX,INT32_MIN));
 assert(s5_encoder_displaced(INT32_MAX,INT32_MIN+3));
 s5_scan_context_t c={0};s5_status_t s=sample(1);
 assert(s5_scan_step(&s,&c)==60 && s.state==S5_SCAN_LEFT && !c.motion_restarts);
 c=(s5_scan_context_t){.position_recorded=1};s=sample(0);s.motor1_counts_start=s.motor1_counts_end=4;
 assert(s5_scan_step(&s,&c)==0 && s.state==S5_CENTERING && c.motion_restarts==1 && !s.approach_plan_valid);
 /* Starting a new acquisition must retain the retry budget. */
 s=sample(0);s.state=S5_MONITOR;s.angle_deg=s.sample_angle_deg=0;
 (void)s5_scan_step(&s,&c);assert(s5_scan_step(&s,&c)==30 && c.motion_restarts==1);
 s=sample(4);assert(s5_scan_step(&s,&c)==0 && s.state==S5_CENTERING && c.motion_restarts==2);
 s=sample(4);assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && c.motion_restarts==3 && !s.approach_plan_valid);
 puts("PASS: 1-3 encoder counts tolerated; larger displacement recentres and retries twice; retry budget survives acquisition; repeated motion blocks; encoder wrap is handled.");
}
