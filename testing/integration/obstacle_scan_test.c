#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
static void scene(int open_left,int open_right,int failure) {
 s5_status_t s={.state=S5_MONITOR,.valid=1,.encoder_valid=1};
 s5_scan_context_t c={0};unsigned left=0,right=0,flicks=0;
 for(unsigned n=0;n<400 && s.state!=S5_READY && s.state!=S5_BLOCKED;++n) {
  s.sample_angle_deg=s.angle_deg;s.valid=1;s.echo_result=S5_ECHO_OK;s.time_ms+=S5_SAMPLE_MS;
  int a=s.angle_deg;
  s.distance_mm=((a>20 && open_left)||(a< -20 && open_right))?1000:150;
  if(s.state==S5_SCAN_LEFT) {assert(a==30 || a==60 || a==90);left|=1U<<(unsigned)(a/30-1);}
  if(s.state==S5_SCAN_RIGHT) {assert(a==-30 || a==-60 || a==-90);right|=1U<<(unsigned)(-a/30-1);}
  if(s.state==S5_REFINE_LEFT)assert(left==7);
  if(s.state==S5_REFINE_RIGHT)assert(right==7);
  if((failure==1 && a>0) || (failure==2 && a==90) ||
     (failure==3 && a<0) || (failure==5 && a!=0) || (failure==4 && s.state==S5_REFINE_LEFT)) {
   s.valid=0;s.echo_result=S5_ECHO_STUCK_HIGH;s.distance_mm=0;
  }
  /* 6: one clear reading while averaging each edge; 7: four on the right edge. */
  unsigned side=s.state==S5_REFINE_LEFT?1U:s.state==S5_REFINE_RIGHT?2U:0U;
  if(c.averaging_bound && c.bound_samples==2U && side &&
     ((failure==6 && !(flicks&side)) || (failure==7 && side==2U))) {
   s.distance_mm=1000;flicks=failure==6?flicks|side:flicks+1U;
  }
  s.angle_deg=s5_scan_step(&s,&c);
 }
 assert(left==7 && right==7);
 if(failure==1 || failure==4 || failure==5)open_left=0;
 if(failure==3 || failure==5 || failure==7)open_right=0;
 if(failure==6)assert(flicks==3U && s.left_bound_deg==19 && s.right_bound_deg==-19 &&
    s.left_bound_valid && s.right_bound_valid);
 if(failure==7)assert(flicks==S5_EDGE_RETREATS+1U && s.left_bound_valid && s.left_bound_deg==20 && !s.right_bound_valid);
 if(!(open_left && open_right)) {assert(s.state==S5_BLOCKED && !s.approach_plan_valid);
 s.angle_deg=s.sample_angle_deg=0;s.valid=1;s.distance_mm=150;
 for(unsigned n=0;n<5;++n){assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED);}
 s.distance_mm=500;assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
 return;}
 assert(s.state==S5_READY && s.approach_plan_valid && s.valid);
 assert(s5_assess_path(&s,1,s.time_ms).state==S5_PATH_READY);
 if(open_left && !open_right)assert(s.planned_rotate_deg>0 && s.left_bound_valid && !s.right_bound_valid && !s.object_width_valid);
 if(!open_left && open_right)assert(s.planned_rotate_deg<0 && s.right_bound_valid && !s.left_bound_valid && !s.object_width_valid);
 if(open_left && open_right && failure!=6)assert(s.planned_rotate_deg>0 && s.object_width_valid &&
    s.object_left_mm==55 && s.object_right_mm==55 && s.object_width_mm==110);
 if(failure==6)assert(s.object_width_valid && s.object_left_mm==s.object_right_mm && s.object_width_mm<110);
}
int main(void) {
 scene(1,0,0);scene(0,1,0);scene(1,1,0);scene(0,0,0);
 scene(1,1,1);scene(1,0,2);scene(1,1,3);scene(1,1,4);scene(1,1,5);
 scene(1,1,6);scene(1,1,7);
 s5_status_t s={.state=S5_SCAN_RIGHT,.angle_deg=-60,.sample_angle_deg=-60,.echo_result=S5_ECHO_STUCK_HIGH};
 s5_scan_context_t c={0};
 for(unsigned n=0;n<3;++n)(void)s5_scan_step(&s,&c);
 assert(s.state==S5_SCAN_RIGHT && !s.approach_plan_valid && c.echo_failures==0);
 puts("PASS: 30/60/90 before refinement on each side; partial-width scans remain blocked; both blocked stop; hard left/right echo faults advance without restarting; unknown stays blocked.");
 puts("PASS: an edge flicker while averaging retreats that bound 1 deg inward and still reaches READY; S5_EDGE_RETREATS+1 flickers fail the side.");
}
