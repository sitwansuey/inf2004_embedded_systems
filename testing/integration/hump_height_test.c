/* The controller's hump height (app_program/vehicle_hump.h) and how it holds
 * the ramp handling (vehicle_ramp.h) until the robot is back down. */
#include <assert.h>
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/vehicle_hump.h"
#include "../../app_program/vehicle_ramp.h"

/* The robot_config.h ramp defaults. */
enum { RAMP_TILT=400, RAMP_LEVEL=300, RAMP_DROP=150, HOLD_MS=400, LEVEL_MS=400, TICK_MS=20, SMOOTH_MS=400 };
#define DEG 100 /* cdeg */

static vehicle_hump_t h;
static int32_t distance_um;
static uint32_t clock_ms,lag_ms; /* lag_ms 0: no smoothing-lag correction */
/* One 20 ms step: move du_um at pitch (cdeg); at_base given directly. */
static void step(bool at_base,bool tilt_valid,int32_t pitch,bool odom_valid,int32_t du_um)
{
    distance_um+=du_um;clock_ms+=TICK_MS;
    vehicle_hump_step(&h,true,at_base,tilt_valid,pitch,odom_valid,distance_um,clock_ms,lag_ms,
                      VC_HUMP_MAX_STEP_UM,VC_HUMP_STALE_TICKS);
}
static void reset(void) { h=(vehicle_hump_t){0};distance_um=123456;lag_ms=0;step(true,true,0,true,0); }
static double rise_um(double mm,double deg) { return mm*1000.0*sin(deg*M_PI/180.0); }

static void unit_checks(void)
{
    /* Level ground: nothing measured. */
    reset();
    for(unsigned n=0;n<50U;++n)step(true,true,50,true,4000);
    assert(h.height_um==0 && h.peak_um==0 && vehicle_hump_peak_2mm(&h)==0U && !vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM));
    /* 400 mm up at 5 deg (34.9 mm), then 400 mm down at 5 deg: the peak stays. */
    for(unsigned n=0;n<100U;++n)step(false,true,5*DEG,true,4000);
    assert(fabs(h.peak_um-rise_um(400,5))<100 && h.height_um==h.peak_um);
    assert(vehicle_hump_peak_2mm(&h)==17U); /* 34.9 mm in 2 mm */
    assert(vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM));
    for(unsigned n=0;n<49U;++n)step(false,true,-5*DEG,true,4000);
    assert(vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM)); /* just above half way down */
    step(false,true,-5*DEG,true,4000);step(false,true,-5*DEG,true,4000);
    assert(!vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM)); /* below half: back down */
    for(unsigned n=0;n<49U;++n)step(false,true,-5*DEG,true,4000); /* 100 steps down in all */
    assert(abs(h.height_um)<100 && vehicle_hump_peak_2mm(&h)==17U);
    /* The base: back to 0. */
    step(true,true,0,true,4000);
    assert(h.height_um==0 && h.peak_um==0 && vehicle_hump_peak_2mm(&h)==0U);

    /* Reversing down the climb takes the height off again (signed distance). */
    reset();
    for(unsigned n=0;n<50U;++n)step(false,true,8*DEG,true,4000);
    int32_t top=h.height_um;
    for(unsigned n=0;n<25U;++n)step(false,true,8*DEG,true,-4000);
    assert(fabs((top-h.height_um)-rise_um(100,8))<50 && h.peak_um==top);

    /* Invalid odometry: lost (no height, no hold) until the next base. */
    reset();
    for(unsigned n=0;n<50U;++n)step(false,true,8*DEG,true,4000);
    assert(vehicle_hump_peak_2mm(&h)>0U && vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM));
    step(false,true,8*DEG,false,4000);
    assert(h.lost && vehicle_hump_peak_2mm(&h)==0U && !vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM));
    for(unsigned n=0;n<20U;++n)step(false,true,8*DEG,true,4000);
    assert(h.lost && vehicle_hump_peak_2mm(&h)==0U);
    step(true,true,0,true,4000);
    for(unsigned n=0;n<10U;++n)step(false,true,8*DEG,true,4000);
    assert(!h.lost && fabs(h.peak_um-rise_um(40,8))<50); /* restarted from the base */
    /* The first step after odometry returns has no distance (no baseline). */
    reset();
    step(false,true,8*DEG,false,4000);step(true,true,0,false,4000);
    step(true,true,0,true,500000); /* a big move while at the base: baseline only */
    step(false,true,8*DEG,true,4000);
    assert(!h.lost && fabs(h.height_um-rise_um(4,8))<5);

    /* A jump above VC_HUMP_MAX_STEP_UM in one step: lost. */
    reset();
    step(false,true,8*DEG,true,4000);
    step(false,true,8*DEG,true,VC_HUMP_MAX_STEP_UM+1);
    assert(h.lost && vehicle_hump_peak_2mm(&h)==0U);
    reset();
    step(false,true,8*DEG,true,-(VC_HUMP_MAX_STEP_UM+1));
    assert(h.lost);
    reset();
    step(false,true,8*DEG,true,VC_HUMP_MAX_STEP_UM); /* exactly the limit is a move */
    assert(!h.lost && fabs(h.height_um-rise_um(VC_HUMP_MAX_STEP_UM/1000.0,8))<5);

    /* Invalid tilt: VC_HUMP_STALE_TICKS steps at the last valid pitch, then lost. */
    reset();
    step(false,true,10*DEG,true,4000);
    for(unsigned n=0;n<VC_HUMP_STALE_TICKS;++n)step(false,false,-30*DEG,true,4000);
    assert(!h.lost && fabs(h.height_um-rise_um(4.0*(1+VC_HUMP_STALE_TICKS),10))<20);
    step(false,true,10*DEG,true,4000); /* a valid one starts the count again */
    for(unsigned n=0;n<VC_HUMP_STALE_TICKS;++n)step(false,false,0,true,4000);
    assert(!h.lost);
    step(false,false,0,true,4000);
    assert(h.lost);
    /* No valid pitch yet at all: lost at once. */
    h=(vehicle_hump_t){0};distance_um=0;
    step(true,false,0,true,0);step(false,false,0,true,4000);
    assert(h.lost);

    /* A level top: held until the robot has moved VC_HUMP_TOP_UM since the
     * last 1 mm rise; pitch noise on the top does not restart it. */
    reset();
    for(unsigned n=0;n<100U;++n)step(false,true,5*DEG,true,4000);
    unsigned moved=0;
    while(vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM)) {
        step(false,true,(moved/4000U)%2U?20:-10,true,4000); /* +/-0.1..0.2 deg noise */
        moved+=4000U;assert(moved<=VC_HUMP_TOP_UM+8000U);
    }
    assert(moved>=VC_HUMP_TOP_UM-8000U);
    /* A climb below VC_HUMP_MIN_UM is not a hump: no hold, but measured. */
    reset();
    for(unsigned n=0;n<10U;++n)step(false,true,6*DEG,true,4000); /* 4.2 mm */
    assert(!vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM) && vehicle_hump_peak_2mm(&h)==2U);
    /* Down first (a dip or a down ramp): no height. */
    reset();
    for(unsigned n=0;n<50U;++n)step(false,true,-8*DEG,true,4000);
    assert(vehicle_hump_peak_2mm(&h)==0U && !vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM));
    /* Up to 1 m (500 in 2 mm); beyond it is lost. */
    reset();
    for(unsigned n=0;n<250U;++n)step(false,true,90*DEG,true,4000);
    assert(!h.lost && vehicle_hump_peak_2mm(&h)==500U);
    step(false,true,90*DEG,true,4000);
    assert(h.lost);
    /* The smoothing lag: at a steady speed (4 mm per 20 ms) the missing
     * height is speed x lag x sin(pitch), added to height_um only. */
    reset();lag_ms=400U;
    for(unsigned n=0;n<200U;++n)step(true,true,0,true,4000); /* speed settles at base */
    assert(fabsf(h.speed-200.0f)<0.5f && h.height_um==0);
    for(unsigned n=0;n<10U;++n)step(false,true,5*DEG,true,4000);
    assert(fabs(h.raw_um-rise_um(40,5))<10 && fabs((h.height_um-h.raw_um)-rise_um(80,5))<50);
    /* Reversing: the speed goes negative, so does the correction. */
    for(unsigned n=0;n<100U;++n)step(false,true,5*DEG,true,-4000);
    assert(h.speed<-199.0f && h.height_um<h.raw_um);
    /* Invalid odometry or a jump: no speed (restarts from 0). */
    step(false,true,5*DEG,false,4000);
    assert(h.speed==0.0f && h.lost);
    /* Disarmed: everything cleared. */
    vehicle_hump_step(&h,false,false,true,0,true,0,0U,0U,VC_HUMP_MAX_STEP_UM,VC_HUMP_STALE_TICKS);
    assert(h.height_um==0 && h.peak_um==0 && !h.lost && !h.primed && !h.pitched && h.speed==0.0f);
}

/* A low hump through the ramp handling, the way vehicle.c runs it: the
 * accelerometer tilt is smoothed over SMOOTH_MS; the robot moves mm_tick per
 * 20 ms tick, climbs at +deg for climb_mm, then (optionally over a level top
 * of top_mm) comes down at -deg the same distance. hold: the controller's
 * hold (vehicle_hump_up) is used, as in vehicle.c; 0 is the 8c rule (only
 * S4's climb holds it, and S4 does not track a climb below 5 deg). */
typedef struct { unsigned up_tick,down_tick,done_tick,cycles,crest_2mm,level_2mm,down_capped_ticks; } hump_run_t;
static hump_run_t run_hump(double deg,double climb_mm,double top_mm,double mm_tick,bool hold)
{
    hump_run_t out={0};
    vehicle_ramp_t ramp={0};
    h=(vehicle_hump_t){0};distance_um=0;clock_ms=0;lag_ms=SMOOTH_MS;
    double smooth=0,travelled=0,end=2*climb_mm+top_mm;
    unsigned seen=RAMP_FLAT;
    for(unsigned tick=0;tick<4000U;++tick) {
        double target=travelled<100?0:travelled<100+climb_mm?deg:travelled<100+climb_mm+top_mm?0:
                      travelled<100+end?-deg:0;
        smooth+=(target-smooth)*TICK_MS/(SMOOTH_MS+TICK_MS);
        int32_t p=(int32_t)lround(smooth*DEG),tilt=p<0?-p:p;
        step(ramp.state==RAMP_FLAT && !ramp.timing && tilt<=RAMP_LEVEL,true,p,true,(int32_t)lround(mm_tick*1000));
        travelled+=mm_tick;
        ramp.level_blocked=hold && vehicle_hump_up(&h,VC_HUMP_MIN_UM,VC_HUMP_TOP_UM);
        unsigned rs=vehicle_ramp_step(&ramp,clock_ms,true,true,tilt,p,0,RAMP_TILT,RAMP_LEVEL,RAMP_DROP,HOLD_MS,LEVEL_MS);
        if(rs==RAMP_DOWN && travelled>100+climb_mm+top_mm && travelled<100+end)++out.down_capped_ticks;
        if(rs!=seen) {
            if(rs==RAMP_UP) {++out.cycles;if(!out.up_tick)out.up_tick=tick;}
            if(rs==RAMP_DOWN && !out.down_tick) {out.down_tick=tick;out.crest_2mm=vehicle_hump_peak_2mm(&h);}
            if(rs==RAMP_DONE && !out.done_tick) {out.done_tick=tick;out.level_2mm=vehicle_hump_peak_2mm(&h);}
            seen=rs;
        }
        if(rs==RAMP_DONE) {ramp=(vehicle_ramp_t){0};seen=RAMP_FLAT;}
        if(travelled>100+end+600)break;
    }
    return out;
}

static void ramp_checks(void)
{
    /* A 3 cm hump: 6 deg over 287 mm up and down, 4 mm per tick (0.2 m/s). */
    double climb=30.0/sin(6*M_PI/180);
    hump_run_t a=run_hump(6,climb,0,4,true);
    assert(a.cycles==1U && a.up_tick && a.down_tick>a.up_tick && a.done_tick>a.down_tick);
    /* Over only after the way down (the robot is back at the bottom). */
    assert(a.done_tick*4.0>100+2*climb);
    /* Slow for the whole way down (from the crest). */
    assert(a.down_capped_ticks>=(unsigned)(0.9*climb/4));
    /* The level reports the hump's height: 30 mm = 15, the smoothing and the
     * 3 deg level limit lose a little. */
    assert(a.level_2mm>=13U && a.level_2mm<=15U && a.crest_2mm<=a.level_2mm && a.crest_2mm>=10U);
    printf("3 cm hump at 6 deg: one cycle, crest %.1f cm, level %.1f cm, slow all the way down PASS\n",
           a.crest_2mm*0.2,a.level_2mm*0.2);
    /* The 8c rule on the same hump: over at the top (the tilt is level
     * there for 400 ms), before half way down, and the rest of the way down
     * is a second "climb". */
    hump_run_t b=run_hump(6,climb,0,4,false);
    assert(b.cycles==2U && b.done_tick*4.0>100+climb && b.done_tick*4.0<100+1.5*climb &&
           b.down_capped_ticks<a.down_capped_ticks/2U);
    puts("Without the hold (8c): over near the top, most of the way down a second hump, not slowed (the bug) PASS");
    /* A 5 cm hump at 10 deg with a 100 mm level top, 6 mm per tick. */
    climb=50.0/sin(10*M_PI/180);
    a=run_hump(10,climb,100,6,true);
    assert(a.cycles==1U && a.done_tick*6.0>100+2*climb+100 && a.level_2mm>=22U && a.level_2mm<=25U);
    printf("5 cm hump at 10 deg, 10 cm top: one cycle, level %.1f cm PASS\n",a.level_2mm*0.2);
    /* A 2 cm hump at 5 deg. */
    climb=20.0/sin(5*M_PI/180);
    a=run_hump(5,climb,0,4,true);
    assert(a.cycles==1U && a.level_2mm>=8U && a.level_2mm<=10U);
    printf("2 cm hump at 5 deg: one cycle, level %.1f cm PASS\n",a.level_2mm*0.2);
    /* A long level top (a platform): the hold ends after VC_HUMP_TOP_UM. */
    climb=30.0/sin(6*M_PI/180);
    a=run_hump(6,climb,2000,4,true);
    assert(a.cycles>=1U && a.done_tick && a.done_tick*4.0<100+climb+VC_HUMP_TOP_UM/1000.0+200);
    puts("A 2 m level top: the hold ends after VC_HUMP_TOP_UM, normal speed on the platform PASS");
}

int main(void)
{
    unit_checks();
    puts("Controller hump height: rebased on level ground, signed wheel distance x sin(pitch), peak kept, lost on invalid odometry/jumps/stale tilt, cleared when disarmed PASS");
    ramp_checks();
    return 0;
}
