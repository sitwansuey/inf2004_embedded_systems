#include "../../app_program/subsystem5/s5_scan.h"
#include "../../app_program/subsystem5/s5_policy.h"
#include "../../app_program/subsystem5/s5_hardware.h"
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
 * tick. Returns the ms from the scan start to READY or BLOCKED.
 * stray_deg: the first acquisition ping at that angle returns stray_mm (a
 * far multipath echo) instead. wall_left/right_mm: a wall parallel to the
 * path that far left/right. post_from/to_mm: a narrow post at the face depth
 * that far left. The nearest surface on the ray echoes. */
typedef struct { UW depth_mm, left_mm, right_mm, background_mm; BOOL block_heading;
    INT stray_deg; UW stray_mm, wall_left_mm, post_from_mm, post_to_mm, wall_right_mm; } scene_t;
static scene_t scene(UW depth, UW left, UW right, UW background, BOOL block_heading) {
    scene_t g={0};
    g.depth_mm=depth;g.left_mm=left;g.right_mm=right;g.background_mm=background;g.block_heading=block_heading;
    return g;
}
static int plan_signs; /* bit 0: a LEFT plan was published, bit 1: a RIGHT one */
static UW nearest(UW best, UW range) { return range<=5000U && (!best || range<best)?range:best; }
static UW scene_scan(const scene_t *g, s5_status_t *out, unsigned *pings) {
    s5_status_t s={.state=S5_MONITOR,.encoder_valid=1};
    s5_scan_context_t c={0};UW t=0,start=0;unsigned n=0;BOOL stray_used=FALSE;
    *pings=0;plan_signs=0;
    for(;n<2000 && s.state!=S5_READY && s.state!=S5_BLOCKED;++n) {
        UINT m=(UINT)(s.angle_deg<0?-s.angle_deg:s.angle_deg);
        UW extent=s.angle_deg>=0?g->left_mm:g->right_mm, echo=0;
        UW across=g->depth_mm*s5_cosine_q10000[90-m]; /* lateral at the face depth x cos */
        s.sample_angle_deg=s.angle_deg;s.time_ms=t;s.valid=1;s.echo_result=S5_ECHO_OK;
        if(m<90 && (across<=extent*s5_cosine_q10000[m] || (s.angle_deg>0 && g->post_to_mm &&
           across>=g->post_from_mm*s5_cosine_q10000[m] && across<=g->post_to_mm*s5_cosine_q10000[m])))
            echo=nearest(echo,(g->depth_mm*10000U+s5_cosine_q10000[m]/2U)/s5_cosine_q10000[m]);
        if(s.angle_deg>0 && g->wall_left_mm)
            echo=nearest(echo,g->wall_left_mm*10000U/s5_cosine_q10000[90-m]);
        if(s.angle_deg<0 && g->wall_right_mm)
            echo=nearest(echo,g->wall_right_mm*10000U/s5_cosine_q10000[90-m]);
        if(m<90 && g->background_mm)
            echo=nearest(echo,g->background_mm*10000U/s5_cosine_q10000[m]);
        if(g->stray_deg && s.angle_deg==g->stray_deg && !stray_used && s.state!=S5_MONITOR && s.state!=S5_CENTERING)
            {echo=g->stray_mm;stray_used=TRUE;}
        if(g->block_heading && c.plan_pending && s.angle_deg==s.planned_rotate_deg) s.distance_mm=100;
        else if(echo) s.distance_mm=echo;
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
        scene_t box=scene(150,20,80,bg,0);
        ms=scene_scan(&box,&s,&pings);
        assert(s.state==S5_READY && s.approach_plan_valid && !s.width_one_sided);
        assert(s.left_bound_deg==7 && s.right_bound_deg==-28);
        within_10pct(s.object_width_mm,100);assert(s.object_width_max_mm>=s.object_width_mm);
        assert(ms<=SCAN_TIME_LIMIT_MS && pings<=60U);
        printf("box 100 mm offset 30 mm right, background %u: width %u mm, %u pings, %u ms\n",
            (unsigned)bg,(unsigned)s.object_width_mm,pings,(unsigned)ms);
        box=scene(150,80,20,bg,0);ms=scene_scan(&box,&s,&pings);
        assert(s.state==S5_READY && s.left_bound_deg==28 && s.right_bound_deg==-7 && !s.width_one_sided);
        within_10pct(s.object_width_mm,100);assert(ms<=SCAN_TIME_LIMIT_MS);
    }
    /* Off-centre object with no background echo (edges 41 and -5 deg): every
     * clear angle needs S5_SCAN_ECHO_FAILURES no-echo pings; 1 deg walks took
     * ~34 s, past the 30 s ACQUIRE limit. */
    scene_t off=scene(150,133,14,0,0);
    ms=scene_scan(&off,&s,&pings);
    assert(s.state==S5_READY && s.left_bound_deg==41 && s.right_bound_deg==-5 && !s.width_one_sided);
    within_10pct(s.object_width_mm,147);assert(ms<=SCAN_TIME_LIMIT_MS && pings<=70U);
    printf("off-centre 41/-5 deg, no background: width %u mm, %u pings, %u ms\n",
        (unsigned)s.object_width_mm,pings,(unsigned)ms);
    /* Right blocked through 60 deg: the left edge is mirrored and flagged, the
     * planning width keeps at least the measured width, and only LEFT is ever
     * planned, even when that heading then fails its check. */
    scene_t wall=scene(150,20,5000,0,0);
    ms=scene_scan(&wall,&s,&pings);
    assert(s.state==S5_READY && s.width_one_sided && s.left_bound_deg==7 && s.right_bound_deg==-7);
    assert(s.object_width_max_mm>=s.object_width_mm && s.object_width_mm>=2U*s.object_left_mm);
    assert(s.planned_rotate_deg>0 && plan_signs==1 && ms<=SCAN_TIME_LIMIT_MS);
    wall.block_heading=TRUE;ms=scene_scan(&wall,&s,&pings);
    assert(s.state==S5_BLOCKED && plan_signs==1 && !s.width_one_sided);
    scene_t left_wall=scene(150,5000,20,0,1);ms=scene_scan(&left_wall,&s,&pings);
    assert(s.state==S5_BLOCKED && plan_signs==2);
    left_wall.block_heading=FALSE;ms=scene_scan(&left_wall,&s,&pings);
    assert(s.state==S5_READY && s.width_one_sided && s.planned_rotate_deg<0 && plan_signs==2);
    /* Worst case: every edge pair up to the 60 deg limit (60: blocked through
     * 60), no background echoes. Both edges always measured to the degree. */
    UW worst=0;
    for(UW depth=100;depth<=200U;depth+=100U) for(UINT left=0;left<=60U;++left) for(UINT right=0;right<=60U;++right) {
        scene_t g=scene(depth,left<60U?extent_between(depth,left):5000U,right<60U?extent_between(depth,right):5000U,0,0);
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
    scene_t centred=scene(150,55,55,0,0);ms=scene_scan(&centred,&s,&pings);
    assert(s.state==S5_READY && s.object_width_mm==110);
    printf("centred +-20 deg, no background: %u pings, %u ms; worst case over edges 0..60 deg: %u ms\n",
        pings,(unsigned)ms,(unsigned)worst);
}
/* O1: one far (multipath) echo at one probe must not move an edge inward:
 * an edge needs two clear angles in a row outward of it. The reviewer's
 * case: face 150 mm ahead, left edge 38 deg, right edge 30 deg, wall at
 * 1000 mm, a far echo at the 30 deg probe. Bisection alone read the left
 * edge at 29 deg (width -15%) and planned LEFT, the real wider side. */
static void stray_scans(void) {
    s5_status_t s,clean;unsigned pings;UW ms,worst=0;
    scene_t g=scene(150,extent_between(150,38),extent_between(150,30),1000,0);
    g.stray_deg=30;g.stray_mm=1000;
    ms=scene_scan(&g,&s,&pings);
    assert(s.state==S5_READY && s.left_bound_deg==38 && s.right_bound_deg==-30 && s.planned_rotate_deg<0);
    printf("stray far echo at the 30 deg probe, edges 38/-30 deg: left %d deg, plan %d deg, %u pings, %u ms\n",
        (int)s.left_bound_deg,(int)s.planned_rotate_deg,pings,(unsigned)ms);
    /* Sweep: edges 3..55 deg both sides, a far echo once at any angle
     * -60..60 (coarse, bisection, mirror or check probes). An edge reads
     * at most 1 deg inward (the far echo at the edge angle itself); the plan
     * changes side only where the two edges are 1 deg apart, and the robot
     * centre always passes the real edge by about the clearance radius
     * (8b's sweep: about 36 mm). */
    unsigned cases=0,inward=0,flips=0;int closest=1000;
    for(UINT left=3;left<=55U;++left) for(UINT right=3;right<=55U;++right) {
        scene_t base=scene(150,extent_between(150,left),extent_between(150,right),1000,0);
        (void)scene_scan(&base,&clean,&pings);
        assert(clean.state==S5_READY && clean.left_bound_deg==(INT)left && clean.right_bound_deg==-(INT)right);
        for(INT a=-60;a<=60;++a) {
            if(!a)continue;
            scene_t one=base;one.stray_deg=a;one.stray_mm=1000;
            ms=scene_scan(&one,&s,&pings);++cases;
            if(ms>worst)worst=ms;
            assert(s.state==S5_READY && s.left_bound_valid && s.right_bound_valid && !s.width_one_sided);
            INT dl=(INT)left-s.left_bound_deg, dr=(INT)right+s.right_bound_deg;
            assert(dl>=0 && dl<=1 && dr>=0 && dr<=1);
            if(dl || dr)++inward;
            if((s.planned_rotate_deg>0)!=(clean.planned_rotate_deg>0)) {
                ++flips;
                assert(left<=right+1U && right<=left+1U);
            }
            BOOL plan_left=s.planned_rotate_deg>0;
            int clearance=(int)s5_plan_lateral_mm(&s,plan_left)-(int)(plan_left?base.left_mm:base.right_mm);
            if(clearance<closest)closest=clearance;
        }
    }
    assert(worst<=SCAN_TIME_LIMIT_MS && closest>=(int)S5_CLEARANCE_RADIUS_MM-5);
    printf("stray sweep: %u scans, %u with an edge 1 deg inward, %u side flips (edges 1 deg apart), "
        "closest centre to real edge %d mm, worst %u ms\n",cases,inward,flips,closest,(unsigned)worst);
}
/* O6/O7: before a side is chosen its corridor is checked: clear echoes seen
 * on that side and 1-2 pings between its edge and its heading. An echo
 * within the planned lateral offset + S5_SIDE_HALF_WIDTH_MM (and at most
 * S5_SIDE_CORRIDOR_AHEAD_MM beyond the face) rejects the side; equal halves
 * go to the side with the farther echoes. */
static void corridor_scans(void) {
    s5_status_t s;unsigned pings;UW ms;
    /* O6: box face 150 mm ahead, left edge 25.6 deg, right edge 30 deg, a
     * 20 mm post 95-115 mm left at the face depth (32-37 deg). Bisection
     * never probes 32-37 deg and the 45 deg heading passes beyond the post:
     * LEFT was planned into it. The probe between them finds it. */
    for(UW bg=0;bg<=1000U;bg+=1000U) {
        scene_t post=scene(150,72,extent_between(150,30),bg,0);
        post.post_from_mm=95;post.post_to_mm=115;
        ms=scene_scan(&post,&s,&pings);
        assert(s.state==S5_READY && s.left_bound_deg==25 && s.right_bound_deg==-30);
        assert(s.planned_rotate_deg<0 && plan_signs==3);
        printf("post 95-115 mm left of a 25/-30 deg box, background %u: plan %d deg, %u pings, %u ms\n",
            (unsigned)bg,(int)s.planned_rotate_deg,pings,(unsigned)ms);
        post.post_to_mm=0;ms=scene_scan(&post,&s,&pings); /* no post: LEFT as before */
        assert(s.state==S5_READY && s.planned_rotate_deg>0 && plan_signs==1);
    }
    /* O7: a 100 mm box centred 150 mm ahead, a wall parallel to the path on
     * the left. Every ray reaches the wall beyond the face depth (clear), and
     * equal halves planned LEFT for walls 140-260 mm: the robot's side
     * reached ~190 mm. Now LEFT is planned only when the robot's outer side
     * (planned lateral + S5_SIDE_HALF_WIDTH_MM) stays inside the wall. */
    unsigned rights=0;
    for(UW wall=100;wall<=400U;wall+=5U) {
        scene_t g=scene(150,50,50,0,0);g.wall_left_mm=wall;
        ms=scene_scan(&g,&s,&pings);
        assert(s.state==S5_READY && s.left_bound_deg==18 && s.right_bound_deg==-18);
        assert(s.object_left_mm==s.object_right_mm && ms<=SCAN_TIME_LIMIT_MS);
        if(s.planned_rotate_deg>0) assert(s5_plan_lateral_mm(&s,TRUE)+S5_SIDE_HALF_WIDTH_MM<wall);
        else ++rights;
        if(wall>=140U && wall<=200U) assert(s.planned_rotate_deg<0);
    }
    printf("parallel wall left of a centred 100 mm box: RIGHT planned for %u of 61 walls 100-400 mm\n",rights);
    /* Walls 150 mm out on both sides: both seen in the search, both sides
     * out, blocked before any plan, not a plan into a wall. */
    scene_t both=scene(150,50,50,0,0);both.wall_left_mm=both.wall_right_mm=150;
    ms=scene_scan(&both,&s,&pings);
    assert(s.state==S5_BLOCKED && !plan_signs && ms<=SCAN_TIME_LIMIT_MS);
    /* Tie-break: a 120 mm box 150 mm ahead, a wall 250 mm left (outside the
     * left corridor, seen at the 40 deg coarse probe), nothing right: the
     * halves are equal and RIGHT has the farther echoes (none). */
    scene_t tie=scene(150,60,60,0,0);tie.wall_left_mm=250;
    ms=scene_scan(&tie,&s,&pings);
    assert(s.state==S5_READY && s.object_left_mm==s.object_right_mm && s.planned_rotate_deg<0 && plan_signs==2);
    tie.wall_left_mm=0;ms=scene_scan(&tie,&s,&pings);
    assert(s.state==S5_READY && s.planned_rotate_deg>0);
    /* The corridor test itself (also for the controller's own checks). */
    s=(s5_status_t){.valid=1,.distance_mm=200,.sample_angle_deg=30};
    assert(s5_corridor_blocked(&s,30U,200U));                         /* 100 mm across, 173 ahead */
    assert(!s5_corridor_blocked(&s,24U,200U) && s5_corridor_blocked(&s,25U,173U));
    assert(!s5_corridor_blocked(&s,100U,172U));                       /* past the window */
    s.sample_angle_deg=-30;assert(s5_corridor_blocked(&s,25U,173U));  /* either side */
    s.valid=0;assert(!s5_corridor_blocked(&s,1000U,1000U));           /* no echo: nothing seen */
}
/* O4: the VC drives on until it sees the scan, so the first scan's origin
 * and depth are taken moving and its braking restarts it. That restart is
 * free; S5_SCAN_MOTION_RESTARTS real disturbances still leave a scan. */
static void wheels_at(s5_status_t *s, INT start, INT end) {
    s->motor1_counts_start=s->motor2_counts_start=start;s->motor1_counts_end=s->motor2_counts_end=end;
}
static void begin_at(s5_status_t *s, s5_scan_context_t *c, INT wheels, INT per_sample) {
    s->angle_deg=s->sample_angle_deg=0;s->valid=1;s->distance_mm=150;s->echo_result=S5_ECHO_OK;
    if(s->state==S5_CENTERING) {wheels_at(s,wheels,wheels);assert(s5_scan_step(s,c)==0 && s->state==S5_MONITOR);}
    for(unsigned n=1;n<=S5_ACQUISITION_SAMPLES;++n) {
        wheels_at(s,wheels,wheels+per_sample/4);wheels+=per_sample;
        assert(s5_scan_step(s,c)==(n<S5_ACQUISITION_SAMPLES?0:S5_SCAN_STEP_DEG));
    }
    assert(s->state==S5_SCAN_LEFT);s->angle_deg=s->sample_angle_deg=S5_SCAN_STEP_DEG;
}
static void braking_restart(void) {
    s5_status_t s={.state=S5_MONITOR,.encoder_valid=1};s5_scan_context_t c={0};
    begin_at(&s,&c,0,12);                     /* about 585 counts/s: 12 counts a sample */
    wheels_at(&s,40,40);                      /* stopped 4 counts past the origin */
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_CENTERING && !c.motion_restarts && c.brake_restart);
    for(unsigned n=1;n<=S5_SCAN_MOTION_RESTARTS;++n) {
        begin_at(&s,&c,40,0);wheels_at(&s,45,45); /* a nudge after the origin */
        assert(s5_scan_step(&s,&c)==0 && s.state==S5_CENTERING && c.motion_restarts==n);
    }
    begin_at(&s,&c,45,0);wheels_at(&s,45,45);  /* still: the scan goes on */
    assert(s5_scan_step(&s,&c)==2*S5_SCAN_STEP_DEG && s.state==S5_SCAN_LEFT);
    s.angle_deg=s.sample_angle_deg=2*S5_SCAN_STEP_DEG;wheels_at(&s,50,50);
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && c.motion_restarts==S5_SCAN_MOTION_RESTARTS+1U);
    /* Free only at the first sample after the origin: a held robot's first
     * scan that moves later counts. */
    s=(s5_status_t){.state=S5_MONITOR,.encoder_valid=1};c=(s5_scan_context_t){0};
    begin_at(&s,&c,0,0);wheels_at(&s,0,0);
    assert(s5_scan_step(&s,&c)==2*S5_SCAN_STEP_DEG);
    s.angle_deg=s.sample_angle_deg=2*S5_SCAN_STEP_DEG;wheels_at(&s,5,5);
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_CENTERING && c.motion_restarts==1U && !c.brake_restart);
    puts("S5 braking: the first restart at the first sample after a moving origin is free; later and repeated motion counts PASS");
}
/* O5: angle to pulse with a configurable slope. The default 750 us per
 * 90 deg is the old mapping exactly; +-90 deg stays inside 750..2250 us. */
static void servo_slope(void) {
    for(INT centre=1300;centre<=1700;++centre) for(INT a=-95;a<=95;++a) {
        INT clipped=a<-90?-90:a>90?90:a, old=centre+(750*clipped)/90;
        old=old<750?750:old>2250?2250:old;
        assert(s5_servo_pulse_us(a,centre,750)==old);
        for(INT slope=500;slope<=1000;slope+=50) {
            INT us=s5_servo_pulse_us(a,centre,slope);
            assert(us>=750 && us<=2250);
        }
    }
    assert(s5_servo_pulse_us(45,1554,1000)==2054 && s5_servo_pulse_us(-45,1554,1000)==1054);
    assert(s5_servo_pulse_us(90,1554,1000)==2250 && s5_servo_pulse_us(-90,1554,600)==954);
    assert(s5_servo_pulse_us(0,1554,BOARD_SERVO_US_PER_90)==1554 && BOARD_SERVO_US_PER_90==750);
    puts("S5 servo slope: BOARD_SERVO_US_PER_90 750 keeps the old pulses, other slopes stay in 750..2250 us PASS");
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
    assert(retry<0 && s.state==S5_VERIFY_CLEAR && s.approach_plan_valid && c.left_rejected && s.object_width_valid);
    s.angle_deg=s.sample_angle_deg=retry;s.distance_mm=40; /* other side blocked too */
    assert(s5_scan_step(&s,&c)==0 && s.state==S5_BLOCKED && !s.approach_plan_valid);
    puts("S5 verify fallback: blocked heading re-plans the other side once PASS");
    flicker_scan(0);flicker_scan(1);flicker_scan(2);flicker_scan(3);flicker_scan(4);
    puts("S5 edge flicker: 1-3 averaging misses retreat the bound inward to READY, 4 fail the side (the right is planned instead) PASS");
    geometry_scans();
    puts("S5 scan geometry: off-centre/offset objects measure both edges (within 10%), one-sided widths flagged and only the measured side planned, every edge pair to 60 deg within 15 s without background echoes PASS");
    stray_scans();
    puts("S5 edge check: one far echo moves an edge at most 1 deg inward (two clear angles in a row outward of every edge) PASS");
    corridor_scans();
    puts("S5 side corridor: a post between edge and heading or a wall beside the box rejects that side; equal halves go to the farther echoes PASS");
    braking_restart();
    servo_slope();
    return 0;
}
