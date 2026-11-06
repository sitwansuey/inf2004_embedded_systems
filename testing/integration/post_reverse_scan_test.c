#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_scan.h"
int main(void) {
 s5_status_t s={.state=S5_MONITOR,.valid=1,.distance_mm=300};s5_scan_context_t c={0};
 (void)s5_scan_step(&s,&c);assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
 c.forced_acquisition=1;
 (void)s5_scan_step(&s,&c);assert(s5_scan_step(&s,&c)==30 && s.state==S5_SCAN_LEFT);
 assert(s.obstacle_distance_mm==300 && !c.forced_acquisition);
 s.angle_deg=30;
 for(unsigned n=0;n<200 && s.state!=S5_READY;++n) {
  s.sample_angle_deg=s.angle_deg;s.valid=1;s.echo_result=S5_ECHO_OK;
  s.distance_mm=s.angle_deg>=-20 && s.angle_deg<=20?300:1200;
  s.angle_deg=s5_scan_step(&s,&c);
 }
 assert(s.state==S5_READY && s.approach_plan_valid);
 puts("PASS: normal300mm does not trigger; explicitly requested post-reverse300mm scan produces a verified plan with updated geometry.");
}
