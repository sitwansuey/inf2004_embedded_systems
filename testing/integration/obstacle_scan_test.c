#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
/* An object within +-20 deg at 150 mm with a far (1000 mm) background beyond
 * on the open sides. The left is searched at 20/40/60 deg, then bisected
 * inward to its edge; the left's last clear angle is then checked on the
 * right (the mirror). Clear: the right edge is searched as well. The object
 * there: the right is searched outward (next of 20/40/60 deg) and only a
 * right blocked through 60 deg mirrors the left edge. A left blocked through
 * 60 deg searches the right at 20/40/60 deg instead, and a measured right is
 * mirrored to the left. A mirrored edge sets width_one_sided; only a measured
 * side is planned.
 * failure: hard echo faults 1 on the whole left, 3 on the whole right, 4
 * while refining the left edge, 5 at every off-centre angle; 6 one clear
 * reading while averaging an edge (once per side); 7 on every averaging of
 * the right edge. */
enum { PLAN_NONE, PLAN_LEFT, PLAN_RIGHT };
static void scene(int open_left,int open_right,int failure,unsigned plan,unsigned left_coarse,unsigned right_coarse,int one_sided) {
 s5_status_t s={.state=S5_MONITOR,.valid=1,.encoder_valid=1};
 s5_scan_context_t c={0};unsigned left=0,right=0,flicks=0;
 for(unsigned n=0;n<400 && s.state!=S5_READY && s.state!=S5_BLOCKED;++n) {
  s.sample_angle_deg=s.angle_deg;s.valid=1;s.echo_result=S5_ECHO_OK;s.time_ms+=S5_SAMPLE_MS;
  int a=s.angle_deg;
  s.distance_mm=((a>20 && open_left)||(a< -20 && open_right))?1000:150;
  if(s.state==S5_SCAN_LEFT) {assert(a==20 || a==40 || a==60);left|=1U<<(unsigned)(a/20-1);}
  if(s.state==S5_SCAN_RIGHT && c.right_coarse) {assert(a==-20 || a==-40 || a==-60);right|=1U<<(unsigned)(-a/20-1);}
  if(s.state==S5_REFINE_LEFT)assert(a>0 && a<60);
  if(s.state==S5_REFINE_RIGHT)assert(a<0 && a>-60);
  if(s.state==S5_SCAN_RIGHT && !c.right_coarse)assert(a<0 && a>=-60); /* mirror or outward search */
  if((failure==1 && a>0) || (failure==3 && a<0) || (failure==5 && a!=0) || (failure==4 && s.state==S5_REFINE_LEFT)) {
   s.valid=0;s.echo_result=S5_ECHO_STUCK_HIGH;s.distance_mm=0;
  }
  unsigned side=s.state==S5_REFINE_LEFT?1U:s.state==S5_REFINE_RIGHT?2U:0U;
  if(c.averaging_bound && c.bound_samples==2U && side &&
     ((failure==6 && !(flicks&side)) || (failure==7 && side==2U))) {
   s.distance_mm=1000;flicks=failure==6?flicks|side:flicks+1U;
  }
  s.angle_deg=s5_scan_step(&s,&c);
 }
 assert(left==left_coarse && right==right_coarse);
 if(plan==PLAN_NONE) {assert(s.state==S5_BLOCKED && !s.approach_plan_valid);
 s.angle_deg=s.sample_angle_deg=0;s.valid=1;s.distance_mm=150;
 for(unsigned n=0;n<5;++n){assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED);}
 s.distance_mm=500;assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
 return;}
 assert(s.state==S5_READY && s.approach_plan_valid && s.valid);
 assert(s5_assess_path(&s,1,s.time_ms).state==S5_PATH_READY);
 assert(plan==PLAN_LEFT?s.planned_rotate_deg>0:s.planned_rotate_deg<0);
 /* Both edges known (one may be the other's mirror), symmetric here. */
 assert(s.left_bound_valid && s.right_bound_valid && s.left_bound_deg==-s.right_bound_deg);
 assert(s.object_width_valid && s.object_left_mm==s.object_right_mm);
 assert(s.width_one_sided==one_sided && s.object_width_max_mm>=s.object_width_mm);
 /* The mirror saw the object (left edge retreated to 19): the right was
  * searched outward and measured, its own edge retreating to 19 as well. */
 if(failure==6)assert(flicks==3U && s.left_bound_deg==19 && s.object_width_mm<110);
 else if(failure==7)assert(flicks==S5_EDGE_RETREATS+1U && s.left_bound_deg==20); /* the right edge failed */
 else assert(s.left_bound_deg==20 && s.object_left_mm==55 && s.object_width_mm==110);
}
int main(void) {
 scene(1,0,0,PLAN_LEFT,3,0,1);scene(0,1,0,PLAN_RIGHT,7,3,1);scene(1,1,0,PLAN_LEFT,3,0,0);scene(0,0,0,PLAN_NONE,7,7,0);
 scene(1,1,1,PLAN_RIGHT,7,3,1);scene(1,1,3,PLAN_LEFT,3,0,1);scene(1,1,4,PLAN_RIGHT,3,3,1);scene(1,1,5,PLAN_NONE,7,7,0);
 scene(1,1,6,PLAN_LEFT,3,0,0);scene(1,1,7,PLAN_LEFT,3,0,1);
 /* Hard echo faults in the coarse right search advance without restarting;
  * blocked through 60 deg is blocked. */
 s5_status_t s={.state=S5_SCAN_RIGHT,.angle_deg=-20,.sample_angle_deg=-20,.echo_result=S5_ECHO_STUCK_HIGH};
 s5_scan_context_t c={.right_coarse=1};
 assert(s5_scan_step(&s,&c)==-40 && s.state==S5_SCAN_RIGHT && !s.approach_plan_valid && c.echo_failures==0);
 s.angle_deg=s.sample_angle_deg=-40;assert(s5_scan_step(&s,&c)==-60 && s.state==S5_SCAN_RIGHT);
 s.angle_deg=s.sample_angle_deg=-60;assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && !s.approach_plan_valid);
 puts("PASS: 20/40/60 deg coarse search, edge bisection, mirror check then outward search; one open side plans it with the edge mirrored (width_one_sided), none blocks; hard echo faults count as the object; unknown stays blocked.");
 puts("PASS: an edge flicker while averaging retreats that bound 1 deg inward and still reaches READY; S5_EDGE_RETREATS+1 flickers fail the side.");
}
