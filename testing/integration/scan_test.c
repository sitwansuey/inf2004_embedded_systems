#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
#include <assert.h>
#include <stdio.h>
/* Full synthetic scan: widths are published (no plan) for S5_WIDTH_PUBLISH_MS
 * of sample time before the side is chosen and the tangent is verified. */
static void full_scan(UW period_ms, UW start_ms) {
    s5_status_t s={.state=S5_MONITOR,.valid=1,.encoder_valid=1,.time_ms=start_ms};
    s5_scan_context_t c={0};unsigned verified_plan=0,held=0;UW hold_start=0;
    for(unsigned n=0;n<400 && s.state!=S5_READY;++n) {
        s.sample_angle_deg=s.angle_deg;s.echo_result=S5_ECHO_OK;s.valid=1;s.time_ms+=period_ms;
        s.distance_mm=s.angle_deg>=-20 && s.angle_deg<=20?150:1000;
        if(c.plan_pending && s.angle_deg==s.planned_rotate_deg)verified_plan=1;
        INT before=s.angle_deg;
        s.angle_deg=(int8_t)s5_scan_step(&s,&c);
        if(c.width_hold) {
            if(!held++)hold_start=s.time_ms;
            assert(s.state==S5_VERIFY_CLEAR && s.angle_deg==before && s.object_width_valid);
            assert(!s.approach_plan_valid && !s.planned_rotate_deg && !s.avoidance_deg);
            assert(s.object_left_mm==55 && s.object_right_mm==55 && s.object_width_mm==110);
            assert(s.time_ms-hold_start<S5_WIDTH_PUBLISH_MS);
        }
        if(s.approach_plan_valid || s.state==S5_READY)
            assert(held && !c.width_hold && s.time_ms-hold_start>=S5_WIDTH_PUBLISH_MS);
    }
    assert(held>=S5_WIDTH_PUBLISH_MS/period_ms);
    assert(s.state==S5_READY && s.approach_plan_valid && s.object_width_valid && verified_plan);
    assert(s.object_width_mm>=109 && s.object_width_mm<=110);
}
/* Left edge sample flickers clear during averaging: the edge retreats 1 deg
 * inward (at most S5_EDGE_RETREATS times) instead of abandoning the side. */
static void flicker_scan(unsigned flickers) {
    s5_status_t s={.state=S5_MONITOR,.valid=1,.encoder_valid=1};
    s5_scan_context_t c={0};unsigned done=0;
    for(unsigned n=0;n<400 && s.state!=S5_READY && s.state!=S5_BLOCKED;++n) {
        s.sample_angle_deg=s.angle_deg;s.echo_result=S5_ECHO_OK;s.valid=1;s.time_ms+=S5_SAMPLE_MS;
        s.distance_mm=s.angle_deg>=-20 && s.angle_deg<=20?150:1000;
        if(s.state==S5_REFINE_LEFT && c.averaging_bound && c.bound_samples==1U && done<flickers)
            {s.distance_mm=1000;++done;}
        s.angle_deg=(int8_t)s5_scan_step(&s,&c);
    }
    assert(done==flickers);
    if(flickers<=S5_EDGE_RETREATS) {
        assert(s.state==S5_READY && s.approach_plan_valid && s.object_width_valid);
        assert(s.left_bound_valid && s.right_bound_valid);
        assert(s.left_bound_deg==20-(int)flickers && s.right_bound_deg==-20);
    } else {
        assert(s.state==S5_BLOCKED && !s.approach_plan_valid && !s.object_width_valid);
        assert(!s.left_bound_valid && s.right_bound_valid && s.right_bound_deg==-20);
    }
}
int main(void) {
    s5_status_t s={.state=S5_MONITOR,.angle_deg=0,.sample_angle_deg=0,.valid=1,.distance_mm=201,.time_ms=UINT32_MAX-10};
    assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_FORWARD);
    s.sample_angle_deg=10;assert(s5_assess_path(&s,TRUE,20).state!=S5_PATH_FORWARD);
    s.sample_angle_deg=0;s.distance_mm=200;assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_STOP);
    s5_scan_context_t c={0};assert(s5_scan_step(&s,&c)==0);assert(s5_scan_step(&s,&c)==30);
    assert(s.state==S5_SCAN_LEFT);
    /* Three coarse positions: 30, 60, 90 before refinement. */
    s.angle_deg=s.sample_angle_deg=30;s.obstacle_distance_mm=200;
    assert(s5_scan_step(&s,&c)==60);
    s.angle_deg=s.sample_angle_deg=-30;s.state=S5_SCAN_RIGHT;
    assert(s5_scan_step(&s,&c)==-60);
    s.state=S5_SCAN_LEFT;
    s.valid=0;s.echo_result=S5_ECHO_STUCK_HIGH;s.angle_deg=s.sample_angle_deg=10;
    for(unsigned n=0;n<3;++n)(void)s5_scan_step(&s,&c);
    assert(s.state==S5_SCAN_LEFT && !s.acquisition_clear && !s.approach_plan_valid);
    s=(s5_status_t){.state=S5_SCAN_LEFT,.valid=1,.distance_mm=150,.angle_deg=10,.sample_angle_deg=10,
        .encoder_valid=1,.motor1_counts_start=0,.motor1_counts_end=4};
    (void)s5_scan_step(&s,&c);assert(s.state==S5_CENTERING && !s.approach_plan_valid);
    s=(s5_status_t){.obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,
        .left_bound_deg=20,.right_bound_deg=-20};
    s5_width_from_bounds(&s);assert(s.object_width_valid && s.object_width_mm>=109 && s.object_width_mm<=110);
    assert(s.object_width_min_mm<=s.object_width_mm && s.object_width_max_mm>s.object_width_mm);
    s.left_bound_deg=80;s5_width_from_bounds(&s);assert(!s.object_width_valid);
    assert(!s.object_left_mm && !s.object_right_mm && !s.object_width_mm);
    /* Pythagoras halves from the averaged slant edge range: 173^2-150^2=7429. */
    s=(s5_status_t){.obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,
        .left_bound_deg=30,.right_bound_deg=-30,.left_bound_distance_mm=173,.right_bound_distance_mm=173};
    s5_width_from_bounds(&s);
    assert(s.object_width_valid && s.object_left_mm==86 && s.object_right_mm==86 && s.object_width_mm==172);
    assert(s.object_width_min_mm==173 && s.object_width_max_mm==181); /* tangent envelope unchanged */
    /* Edge range not beyond the forward depth falls back to round(d0*tan). */
    s.left_bound_distance_mm=150;s.right_bound_distance_mm=140;s5_width_from_bounds(&s);
    assert(s.object_width_valid && s.object_left_mm==87 && s.object_right_mm==87 && s.object_width_mm==174);
    s.left_bound_distance_mm=200;s5_width_from_bounds(&s);
    assert(s.object_left_mm==132 && s.object_right_mm==87 && s.object_width_mm==219);
    /* One invalid bound zeroes both halves and the total. */
    s.right_bound_valid=0;s5_width_from_bounds(&s);
    assert(!s.object_width_valid && !s.object_left_mm && !s.object_right_mm && !s.object_width_mm);
    puts("S5 width: Pythagoras halves 86/86/172, tangent fallback, invalid bound zeroes halves PASS");
    /* Complete acquisition in a synthetic scene with real finite background
     * returns; verify the computed tangent is sampled before READY. */
    full_scan(S5_SAMPLE_MS,0);full_scan(10,UINT32_MAX-200); /* includes clock wrap */
    /* A failed plan after the hold blocks; before the hold ends nothing moves. */
    s=(s5_status_t){.state=S5_VERIFY_CLEAR,.valid=1,.distance_mm=150,.angle_deg=-20,.sample_angle_deg=-20,
        .obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,.left_bound_deg=79,.right_bound_deg=-20,
        .object_width_valid=1,.object_left_mm=9,.object_right_mm=9,.time_ms=299};
    c=(s5_scan_context_t){.width_hold=1,.width_hold_ms=0};
    assert(s5_scan_step(&s,&c)==-20 && s.state==S5_VERIFY_CLEAR && c.width_hold && !s.approach_plan_valid);
    s.time_ms=300;assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && !c.width_hold);
    assert(!s.approach_plan_valid && !s.object_width_valid && !s.object_left_mm && !s.object_right_mm);
    puts("INT-CR1-04 scan: wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS");
    s=(s5_status_t){.object_width_valid=1,.left_bound_valid=1,.right_bound_valid=1,
        .obstacle_distance_mm=150,.left_bound_deg=20,.right_bound_deg=-10,
        .left_bound_distance_mm=160,.right_bound_distance_mm=160};
    assert(s5_acquisition_plan(&s) && s.planned_rotate_deg<0);
    assert(s.planned_approach_mm>=50 && s.planned_approach_mm<=1000 && s.planned_return_mm==s.planned_approach_mm);
    s.left_bound_deg=10;s.right_bound_deg=-20;
    assert(s5_acquisition_plan(&s) && s.planned_rotate_deg>0);
    s.left_bound_deg=20;s.right_bound_deg=-20;
    assert(s5_acquisition_plan(&s) && s.planned_rotate_deg>0);
    /* Side choice: the smaller centre-to-edge half (shorter detour) wins even
     * when its tangent heading is larger; equal halves keep the smaller heading. */
    s=(s5_status_t){.left_bound_valid=1,.right_bound_valid=1,.obstacle_distance_mm=150,
        .left_bound_deg=20,.right_bound_deg=-30,.left_bound_distance_mm=173,.right_bound_distance_mm=173};
    assert(s5_acquisition_plan(&s) && s.object_left_mm==86 && s.object_right_mm==86 && s.planned_rotate_deg>0);
    INT tie_heading=s.planned_rotate_deg;
    s.left_bound_distance_mm=200;
    assert(s5_acquisition_plan(&s) && s.object_left_mm==132 && s.object_right_mm==86);
    assert(s.planned_rotate_deg<0 && -s.planned_rotate_deg>tie_heading && s.avoidance_deg==-31);
    s.left_bound_distance_mm=173;s.right_bound_distance_mm=200;
    assert(s5_acquisition_plan(&s) && s.object_left_mm<s.object_right_mm && s.planned_rotate_deg>0);
    /* Forced side for the verify fallback: the other side is planned even
     * though it is the longer detour. */
    assert(s5_acquisition_plan_side(&s,-1) && s.planned_rotate_deg<0 && s.approach_plan_valid);
    assert(s5_acquisition_plan_side(&s,1) && s.planned_rotate_deg>0);
    puts("S5 side choice: smaller half-width first, ties keep the smaller heading then LEFT PASS");
    /* Half-width never drops below the bound angle's tangent: an edge range
     * 1 mm beyond depth must not read as a ~17 mm half (noise flips sides). */
    s=(s5_status_t){.left_bound_valid=1,.right_bound_valid=1,.obstacle_distance_mm=150,
        .left_bound_deg=30,.right_bound_deg=-10,.left_bound_distance_mm=150,.right_bound_distance_mm=152};
    s5_width_from_bounds(&s);UW at_depth=s.object_left_mm;
    s.left_bound_distance_mm=151;s5_width_from_bounds(&s);
    assert(at_depth==87 && s.object_left_mm==86 && s.object_right_mm==26);
    assert(s5_acquisition_plan(&s) && s.planned_rotate_deg<0); /* still the narrow right side */
    /* Verify fallback through the scan: the chosen (left) heading is blocked,
     * so the other side is planned once; a second failure blocks. */
    s=(s5_status_t){.state=S5_VERIFY_CLEAR,.valid=1,.left_bound_valid=1,.right_bound_valid=1,
        .obstacle_distance_mm=150,.left_bound_deg=20,.right_bound_deg=-30,
        .left_bound_distance_mm=173,.right_bound_distance_mm=200};
    assert(s5_acquisition_plan(&s) && s.planned_rotate_deg>0);
    c=(s5_scan_context_t){.plan_pending=1};
    s.angle_deg=s.sample_angle_deg=s.planned_rotate_deg;s.distance_mm=100; /* object at the heading */
    INT retry=s5_scan_step(&s,&c);
    assert(retry<0 && s.state==S5_VERIFY_CLEAR && s.approach_plan_valid && c.side_retry && s.object_width_valid);
    s.angle_deg=s.sample_angle_deg=retry;s.distance_mm=40; /* other side blocked too */
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && !s.approach_plan_valid);
    puts("S5 verify fallback: blocked heading re-plans the other side once PASS");
    flicker_scan(0);flicker_scan(1);flicker_scan(3);flicker_scan(4);
    puts("S5 edge flicker: 1-3 averaging misses retreat the bound inward to READY, 4 fail the side PASS");
    return 0;
}
