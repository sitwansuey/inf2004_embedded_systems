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
        /* The mirror check looks at the left's last clear angle: 21 deg
         * with no flicker (clear: the right edge is measured), 20 deg or
         * less otherwise (the object: the right is searched outward and its
         * own edge measured, not the left one mirrored). */
        assert(s.state==S5_READY && s.approach_plan_valid && s.object_width_valid && !s.width_one_sided);
        assert(s.left_bound_valid && s.right_bound_valid && s.planned_rotate_deg>0);
        assert(s.left_bound_deg==20-(int)flickers && s.right_bound_deg==-20);
    } else {
        /* The left side fails: the right is searched coarsely, measured and
         * planned, with its edge mirrored to the left. */
        assert(s.state==S5_READY && s.approach_plan_valid && s.object_width_valid && s.planned_rotate_deg<0);
        assert(s.left_bound_valid && s.right_bound_valid && s.right_bound_deg==-20 && s.left_bound_deg==20);
        assert(s.width_one_sided);
    }
}
/* Planar object face depth_mm ahead spanning left_mm/right_mm either side of
 * the forward axis; beyond it a wall at background_mm (0: no echo at all).
 * block_heading: the planned heading reads an object. Each sample costs what
 * scan_task (subsystem5.c) spends on it: the ping (the 25 ms echo timeout
 * without an echo, else <= 6 ms) and its wait (S5_SAMPLE_MS after a valid
 * sample with the servo staying, else max(S5_SETTLE_MS, S5_SERVO_MS_PER_DEG
 * per deg of pan)), rounded up to the 10 ms tick plus the kernel's extra
 * tick. Returns the ms from the scan start to READY or BLOCKED. */
typedef struct { UW depth_mm, left_mm, right_mm, background_mm; BOOL block_heading; } scene_t;
static int plan_signs; /* bit 0: a LEFT plan was published, bit 1: a RIGHT one */
static UW scene_scan(const scene_t *g, s5_status_t *out, unsigned *pings) {
    s5_status_t s={.state=S5_MONITOR,.encoder_valid=1};
    s5_scan_context_t c={0};UW t=0,start=0;unsigned n=0;
    *pings=0;plan_signs=0;
    for(;n<2000 && s.state!=S5_READY && s.state!=S5_BLOCKED;++n) {
        UINT m=(UINT)(s.angle_deg<0?-s.angle_deg:s.angle_deg);
        UW extent=s.angle_deg>=0?g->left_mm:g->right_mm;
        s.sample_angle_deg=s.angle_deg;s.time_ms=t;s.valid=1;s.echo_result=S5_ECHO_OK;
        if(g->block_heading && c.plan_pending && s.angle_deg==s.planned_rotate_deg) s.distance_mm=100;
        else if(m<90 && g->depth_mm*s5_cosine_q10000[90-m]<=extent*s5_cosine_q10000[m])
            s.distance_mm=(g->depth_mm*10000U+s5_cosine_q10000[m]/2U)/s5_cosine_q10000[m];
        else if(m<90 && g->background_mm && g->background_mm*10000U/s5_cosine_q10000[m]<=5000U)
            s.distance_mm=g->background_mm*10000U/s5_cosine_q10000[m];
        else {s.valid=0;s.distance_mm=0;s.echo_result=S5_ECHO_NO_RISE;}
        UW ping=s.valid?6U:25U;
        INT before=s.angle_deg, target=s5_scan_step(&s,&c);
        if(s.state==S5_SCAN_LEFT && before==0) {start=t;*pings=0;}
        if(s.planned_rotate_deg)plan_signs|=s.planned_rotate_deg>0?1:2;
        ++*pings;
        UW travel=(UW)(target>before?target-before:before-target);
        if(target!=before){s.valid=0;s.angle_deg=target;}
        UW wait=s.valid?(UW)S5_SAMPLE_MS:travel*S5_SERVO_MS_PER_DEG<(UW)S5_SETTLE_MS?(UW)S5_SETTLE_MS:travel*S5_SERVO_MS_PER_DEG;
        t+=ping+(wait+9U)/10U*10U+10U;
    }
    assert(n<2000);
    *out=s;return t-start;
}
/* Lateral extent (rounded mm) half way between bound angles deg and deg+1:
 * at depths >= 100 mm a degree spans >= 1.7 mm, so the edge stays inside. */
static UW extent_between(UW depth, UINT deg) {
    unsigned long long sum=(unsigned long long)s5_cosine_q10000[90-deg]*s5_cosine_q10000[deg+1]+
        (unsigned long long)s5_cosine_q10000[89-deg]*s5_cosine_q10000[deg];
    unsigned long long den=2ULL*s5_cosine_q10000[deg]*s5_cosine_q10000[deg+1];
    return (UW)((depth*sum+den/2U)/den);
}
#define SCAN_TIME_LIMIT_MS 15000U /* the controller's ACQUIRE limit is 30 s */
static void within_10pct(UW measured, UW actual) {
    assert(measured*10U>=actual*9U && measured*10U<=actual*11U);
}
/* Reviewers' geometries: one-sided widths and scan time. */
static void geometry_scans(void) {
    s5_status_t s;unsigned pings;UW ms;
    for(UW bg=0;bg<=1000U;bg+=1000U) {
        /* 100 mm box offset 30 mm right at 150 mm: the mirror check at -8 deg
         * hits the box; its right edge (28 deg) is searched, not mirrored. */
        scene_t box={150,20,80,bg,0};
        ms=scene_scan(&box,&s,&pings);
        assert(s.state==S5_READY && s.approach_plan_valid && !s.width_one_sided);
        assert(s.left_bound_deg==7 && s.right_bound_deg==-28);
        within_10pct(s.object_width_mm,100);assert(s.object_width_max_mm>=s.object_width_mm);
        assert(ms<=SCAN_TIME_LIMIT_MS && pings<=60U);
        printf("box 100 mm offset 30 mm right, background %u: width %u mm, %u pings, %u ms\n",
            (unsigned)bg,(unsigned)s.object_width_mm,pings,(unsigned)ms);
        box=(scene_t){150,80,20,bg,0};ms=scene_scan(&box,&s,&pings);
        assert(s.state==S5_READY && s.left_bound_deg==28 && s.right_bound_deg==-7 && !s.width_one_sided);
        within_10pct(s.object_width_mm,100);assert(ms<=SCAN_TIME_LIMIT_MS);
    }
    /* Off-centre object with no background echo (edges 41 and -5 deg): every
     * clear angle needs S5_SCAN_ECHO_FAILURES no-echo pings; 1 deg walks took
     * ~34 s, past the 30 s ACQUIRE limit. */
    scene_t off={150,133,14,0,0};
    ms=scene_scan(&off,&s,&pings);
    assert(s.state==S5_READY && s.left_bound_deg==41 && s.right_bound_deg==-5 && !s.width_one_sided);
    within_10pct(s.object_width_mm,147);assert(ms<=SCAN_TIME_LIMIT_MS && pings<=70U);
    printf("off-centre 41/-5 deg, no background: width %u mm, %u pings, %u ms\n",
        (unsigned)s.object_width_mm,pings,(unsigned)ms);
    /* Right blocked through 60 deg: the left edge is mirrored and flagged, the
     * planning width keeps at least the measured width, and only LEFT is ever
     * planned, even when that heading then fails its check. */
    scene_t wall={150,20,5000,0,0};
    ms=scene_scan(&wall,&s,&pings);
    assert(s.state==S5_READY && s.width_one_sided && s.left_bound_deg==7 && s.right_bound_deg==-7);
    assert(s.object_width_max_mm>=s.object_width_mm && s.object_width_mm>=2U*s.object_left_mm);
    assert(s.planned_rotate_deg>0 && plan_signs==1 && ms<=SCAN_TIME_LIMIT_MS);
    wall.block_heading=TRUE;ms=scene_scan(&wall,&s,&pings);
    assert(s.state==S5_BLOCKED && plan_signs==1 && !s.width_one_sided);
    scene_t left_wall={150,5000,20,0,1};ms=scene_scan(&left_wall,&s,&pings);
    assert(s.state==S5_BLOCKED && plan_signs==2);
    left_wall.block_heading=FALSE;ms=scene_scan(&left_wall,&s,&pings);
    assert(s.state==S5_READY && s.width_one_sided && s.planned_rotate_deg<0 && plan_signs==2);
    /* Worst case: every edge pair up to the 60 deg limit (60: blocked through
     * 60), no background echoes. Both edges always measured to the degree. */
    UW worst=0;
    for(UW depth=100;depth<=200U;depth+=100U) for(UINT left=0;left<=60U;++left) for(UINT right=0;right<=60U;++right) {
        scene_t g={depth,left<60U?extent_between(depth,left):5000U,right<60U?extent_between(depth,right):5000U,0,0};
        ms=scene_scan(&g,&s,&pings);
        assert(ms<=SCAN_TIME_LIMIT_MS);
        if(ms>worst)worst=ms;
        if(s.state!=S5_READY)continue;
        assert(s.object_width_max_mm>=s.object_width_mm);
        if(left<60U && right<60U) {
            assert(!s.width_one_sided && s.left_bound_deg==(INT)left && s.right_bound_deg==-(INT)right);
            assert(plan_signs==(s.planned_rotate_deg>0?1:2));
        } else assert(s.width_one_sided && plan_signs==(left<60U?1:2));
    }
    scene_t centred={150,55,55,0,0};ms=scene_scan(&centred,&s,&pings);
    assert(s.state==S5_READY && s.object_width_mm==110);
    printf("centred +-20 deg, no background: %u pings, %u ms; worst case over edges 0..60 deg: %u ms\n",
        pings,(unsigned)ms,(unsigned)worst);
}
int main(void) {
    s5_status_t s={.state=S5_MONITOR,.angle_deg=0,.sample_angle_deg=0,.valid=1,.distance_mm=201,.time_ms=UINT32_MAX-10};
    assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_FORWARD);
    s.sample_angle_deg=10;assert(s5_assess_path(&s,TRUE,20).state!=S5_PATH_FORWARD);
    s.sample_angle_deg=0;s.distance_mm=200;assert(s5_assess_path(&s,TRUE,20).state==S5_PATH_STOP);
    s5_scan_context_t c={0};
    /* S5_ACQUISITION_SAMPLES consecutive near echoes start the scan. A far
     * echo restarts the count; a no-echo retry neither counts nor restarts
     * it, but three in a row (open space) do. */
    for(unsigned n=1;n<S5_ACQUISITION_SAMPLES;++n)assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
    s.distance_mm=201;assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
    s.distance_mm=200;
    for(unsigned n=1;n<S5_ACQUISITION_SAMPLES;++n)assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
    s.valid=0;s.distance_mm=0;s.echo_result=S5_ECHO_NO_RISE;
    for(unsigned n=0;n<S5_SCAN_ECHO_FAILURES;++n)assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
    assert(s5_sample_is_open(&s));
    s.valid=1;s.distance_mm=200;s.echo_result=S5_ECHO_OK;
    for(unsigned n=1;n<S5_ACQUISITION_SAMPLES;++n)assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR);
    s.valid=0;s.distance_mm=0;s.echo_result=S5_ECHO_NO_FALL;
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_MONITOR); /* one retry: no reading */
    s.valid=1;s.distance_mm=200;s.echo_result=S5_ECHO_OK;
    assert(s5_scan_step(&s,&c)==S5_SCAN_STEP_DEG);
    assert(s.state==S5_SCAN_LEFT);
    /* Coarse positions 20, 40, 60 deg; blocked through 60 searches the right. */
    s.angle_deg=s.sample_angle_deg=20;s.obstacle_distance_mm=200;
    assert(s5_scan_step(&s,&c)==40 && s.state==S5_SCAN_LEFT);
    s.angle_deg=s.sample_angle_deg=60;
    assert(s5_scan_step(&s,&c)==-20 && s.state==S5_SCAN_RIGHT && c.right_coarse);
    s.angle_deg=s.sample_angle_deg=-20;
    assert(s5_scan_step(&s,&c)==-40 && s.state==S5_SCAN_RIGHT);
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
    /* Edge ranges past the next degree's plane put the Pythagoras halves
     * (17+17) outside the 5 deg tangent envelope (32): planning keeps max. */
    s=(s5_status_t){.obstacle_distance_mm=150,.left_bound_valid=1,.right_bound_valid=1,
        .left_bound_deg=5,.right_bound_deg=-5,.left_bound_distance_mm=151,.right_bound_distance_mm=151};
    s5_width_from_bounds(&s);
    assert(s.object_width_valid && s.object_width_mm==34 && s.object_width_max_mm==34 && s.object_width_min_mm==26);
    puts("S5 width: Pythagoras halves 86/86/172, tangent fallback, invalid bound zeroes halves, max never below the width PASS");
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
    puts("INT-CR1-04 scan: 3 consecutive near echoes start it (far or open resets, a no-echo retry does not), wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS");
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
    flicker_scan(0);flicker_scan(1);flicker_scan(2);flicker_scan(3);flicker_scan(4);
    puts("S5 edge flicker: 1-3 averaging misses retreat the bound inward to READY, 4 fail the side (the right is planned instead) PASS");
    geometry_scans();
    puts("S5 scan geometry: off-centre/offset objects measure both edges (within 10%), one-sided widths flagged and only the measured side planned, every edge pair to 60 deg within 15 s without background echoes PASS");
    return 0;
}
