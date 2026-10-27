#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    s5_status_t s={.state=S5_MONITOR,.angle_deg=0,.sample_angle_deg=0,.valid=1,.distance_mm=201,.time_ms=UINT32_MAX-10};
    assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_FORWARD);
    s.sample_angle_deg=10;assert(s5_assess_path(&s,TRUE,20).state!=S5_PATH_FORWARD);
    s.sample_angle_deg=0;s.distance_mm=200;assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_STOP);
    s5_scan_context_t c={0};assert(s5_scan_step(&s,&c)==0);assert(s5_scan_step(&s,&c)==10);
    assert(s.state==S5_SCAN_LEFT);
    s.valid=0;s.echo_result=S5_ECHO_NO_RISE;s.angle_deg=s.sample_angle_deg=10;
    for(unsigned n=0;n<3;++n)(void)s5_scan_step(&s,&c);
    assert(s.state==S5_FAULT && !s.acquisition_clear && !s.approach_plan_valid);
    s=(s5_status_t){.state=S5_SCAN_LEFT,.valid=1,.distance_mm=150,.angle_deg=10,.sample_angle_deg=10,
        .encoder_valid=1,.motor1_counts_start=0,.motor1_counts_end=1};
    (void)s5_scan_step(&s,&c);assert(s.state==S5_FAULT);
    s=(s5_status_t){.obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,
        .left_bound_deg=20,.right_bound_deg=-20};
    s5_width_from_bounds(&s);assert(s.object_width_valid && s.object_width_mm>=109 && s.object_width_mm<=110);
    assert(s.object_width_min_mm<=s.object_width_mm && s.object_width_max_mm>s.object_width_mm);
    s.left_bound_deg=50;s5_width_from_bounds(&s);assert(!s.object_width_valid);
    /* Complete acquisition in a synthetic scene with real finite background
     * returns; verify the computed tangent is sampled before READY. */
    s=(s5_status_t){.state=S5_MONITOR,.valid=1,.encoder_valid=1};
    c=(s5_scan_context_t){0};unsigned verified_plan=0;
    for(unsigned n=0;n<200 && s.state!=S5_READY;++n) {
        s.sample_angle_deg=s.angle_deg;s.echo_result=S5_ECHO_OK;s.valid=1;
        s.distance_mm=s.angle_deg>=-20 && s.angle_deg<=20?150:1000;
        if(c.plan_pending && s.angle_deg==s.planned_rotate_deg)verified_plan=1;
        s.angle_deg=(int8_t)s5_scan_step(&s,&c);
    }
    assert(s.state==S5_READY && s.approach_plan_valid && s.object_width_valid && verified_plan);
    assert(s.object_width_mm>=109 && s.object_width_mm<=110);
    puts("INT-CR1-04 scan: wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS");
    return 0;
}
