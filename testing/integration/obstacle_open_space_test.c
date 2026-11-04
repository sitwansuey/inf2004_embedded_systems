#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
static void no_return(s5_status_t *s,s5_scan_context_t *c,s5_echo_result_t echo) {
 s->sample_angle_deg=s->angle_deg;s->valid=0;s->distance_mm=0;s->echo_result=echo;
 (void)s5_scan_step(s,c);
}
static void route(int left,int right,s5_echo_result_t echo) {
 s5_status_t s={.state=S5_MONITOR,.encoder_valid=1};
 s5_scan_context_t c={0};unsigned lm=0,rm=0,steps=0;
 for(;steps<250 && s.state!=S5_READY && s.state!=S5_BLOCKED;++steps) {
  int a=s.angle_deg;
  s.sample_angle_deg=a;s.valid=1;s.distance_mm=150;s.echo_result=S5_ECHO_OK;
  if((a>20 && left)||(a< -20 && right)) {s.valid=0;s.distance_mm=0;s.echo_result=echo;}
  if(s.state==S5_SCAN_LEFT)lm|=1U<<(unsigned)(a/30-1);
  if(s.state==S5_SCAN_RIGHT)rm|=1U<<(unsigned)(-a/30-1);
  s.angle_deg=s5_scan_step(&s,&c);
 }
 assert(steps<250 && lm==7 && rm==7);
 if(!left && !right) {assert(s.state==S5_BLOCKED && !s.approach_plan_valid);return;}
 assert(s.state==S5_READY && s.approach_plan_valid && !s.valid && !s.distance_mm);
 assert(s5_sample_is_open(&s) && s5_direction_clear(&s,200));
 assert(s5_assess_path(&s,1,s.time_ms).state==S5_PATH_READY);
 if(left && !right)assert(s.planned_rotate_deg>0 && s.left_bound_valid && !s.right_bound_valid);
 if(!left && right)assert(s.planned_rotate_deg<0 && !s.left_bound_valid && s.right_bound_valid);
 if(left && right)assert(s.planned_rotate_deg>0);
}
int main(void) {
 for(unsigned e=S5_ECHO_NO_RISE;e<=S5_ECHO_NO_FALL;++e) {
  route(1,0,(s5_echo_result_t)e);route(0,1,(s5_echo_result_t)e);
  route(1,1,(s5_echo_result_t)e);route(0,0,(s5_echo_result_t)e);
  s5_status_t s={.state=S5_MONITOR};s5_scan_context_t c={0};
  no_return(&s,&c,(s5_echo_result_t)e);assert(!s5_sample_is_open(&s));
  no_return(&s,&c,(s5_echo_result_t)e);assert(!s5_sample_is_open(&s));
  no_return(&s,&c,(s5_echo_result_t)e);assert(s5_sample_is_open(&s) && s.state==S5_MONITOR);
  assert(s5_assess_path(&s,1,0).state==S5_PATH_FORWARD);
  no_return(&s,&c,(s5_echo_result_t)e);assert(s5_sample_is_open(&s));
  s.angle_deg=s.sample_angle_deg=30;no_return(&s,&c,(s5_echo_result_t)e);
  assert(!s5_sample_is_open(&s));
  s=(s5_status_t){.state=S5_APPROACH};c=(s5_scan_context_t){0};
  for(unsigned n=0;n<3;++n)no_return(&s,&c,(s5_echo_result_t)e);
  assert(s5_sample_is_open(&s) && s5_assess_path(&s,1,0).state==S5_PATH_GUIDE_FORWARD);
  s.sample_angle_deg=30;assert(!s5_direction_clear(&s,50));
 }
 for(unsigned e=S5_ECHO_STUCK_HIGH;e<=S5_ECHO_TOO_SHORT;e+=3) {
  s5_status_t s={.state=S5_MONITOR};s5_scan_context_t c={0};
  for(unsigned n=0;n<4;++n)no_return(&s,&c,(s5_echo_result_t)e);
  assert(!s5_sample_is_open(&s) && s.state==S5_FAULT);
 }
 puts("PASS: no-rise/no-fall clear after 3 settled samples; left/right/equal/blocked routes; READY verification; forward bypass guidance; angle reset; hard faults remain closed.");
}
