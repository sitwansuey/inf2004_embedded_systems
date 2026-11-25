/* Obstacle approach (app_program/vehicle_approach.h): detected at 30 cm, the
 * robot drives on to about 20 cm and holds still there for S5's scan. */
#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_approach.h"

enum { STOP=VC_APPROACH_STOP_MM+VC_APPROACH_BRAKE_MM, HOLD=VC_APPROACH_HOLD_MS, SLACK=VC_APPROACH_SLACK_MM };
static vehicle_approach_t a;
static int32_t um;
static uint32_t ms;
/* One 20 ms tick moving mm forward. */
static unsigned tick(bool tracking,bool detected,uint32_t range,bool far,bool odom,int32_t mm)
{
    um+=mm*1000;ms+=20U;
    return vehicle_approach_step(&a,tracking,detected,range,far,odom,um,ms,STOP,HOLD,SLACK);
}

int main(void)
{
    /* Nothing detected: nothing to do. */
    assert(tick(true,false,0,false,true,4)==APPROACH_NONE && !a.on);
    /* Detected at 290 mm: drive 70 mm on (to 220 mm), then hold. */
    assert(tick(true,true,290,false,true,4)==APPROACH_DRIVE && a.on && a.go_mm==290U-STOP && a.range_mm==290U);
    unsigned driven=0;
    while(tick(true,true,250,false,true,4)==APPROACH_DRIVE) {driven+=4;assert(driven<=80U);}
    assert(driven+4U>=290U-STOP && a.holding);
    /* Held (stopped: no distance) until S5 starts its scan, here within the
     * hold: leaving VC_TRACK ends the approach. */
    for(unsigned n=0;n<10U;++n)assert(tick(true,true,200,false,true,0)==APPROACH_HOLD);
    assert(tick(false,true,200,false,true,0)==APPROACH_NONE && !a.on && !a.spent);

    /* No scan within the hold: following on; the approach ends SLACK past
     * the stop point and is spent until a far or open sample. */
    a=(vehicle_approach_t){0};
    assert(tick(true,true,260,false,true,4)==APPROACH_DRIVE);
    while(tick(true,true,250,false,true,4)==APPROACH_DRIVE) {}
    uint32_t held=0;
    while(tick(true,true,220,false,true,0)==APPROACH_HOLD) held+=20U;
    assert(held+20U>=HOLD && held<=HOLD && a.expired);
    unsigned on=0;
    while(tick(true,true,220,false,true,4)==APPROACH_DRIVE) {on+=4;assert(on<=SLACK+8U);}
    assert(!a.on && a.spent && on+8U>=SLACK);
    assert(tick(true,true,200,false,true,4)==APPROACH_NONE); /* spent: S5's own trigger only */
    assert(tick(true,false,0,true,true,4)==APPROACH_NONE && !a.spent); /* far: re-armed */
    assert(tick(true,true,280,false,true,4)==APPROACH_DRIVE);

    /* Detected already within the stop distance: hold at once. */
    a=(vehicle_approach_t){0};
    assert(tick(true,true,180,false,true,4)==APPROACH_HOLD && a.go_mm==0U);
    /* Far readings while approaching (a second echo path) change nothing. */
    a=(vehicle_approach_t){0};
    assert(tick(true,true,300,false,true,4)==APPROACH_DRIVE);
    assert(tick(true,false,380,true,true,4)==APPROACH_DRIVE && a.on);
    /* No odometry: no latch; while latched it keeps its state (holding stays held). */
    a=(vehicle_approach_t){0};
    assert(tick(true,true,280,false,false,4)==APPROACH_NONE && !a.on);
    assert(tick(true,true,230,false,true,4)==APPROACH_DRIVE);
    assert(tick(true,true,230,false,true,10)==APPROACH_HOLD);
    assert(tick(true,true,230,false,false,0)==APPROACH_HOLD);
    /* Reversing (a barcode re-read is excluded by the caller) never counts. */
    a=(vehicle_approach_t){0};
    assert(tick(true,true,300,false,true,0)==APPROACH_DRIVE);
    assert(tick(true,true,300,false,true,-50)==APPROACH_DRIVE && !a.holding);
    puts("Obstacle approach: 2 echoes within 30 cm, drive on to ~20 cm, hold up to 1.5 s for S5's scan, then on; one per obstacle PASS");
    return 0;
}
