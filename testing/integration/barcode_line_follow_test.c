#include "../../app_program/barcode_line_follow.h"
#include "../../app_program/subsystem2/wheel_steering.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    barcode_line_follow_t s={0};
    assert(!barcode_line_follow_step(&s,10,true,100,100,0));
    assert(!barcode_line_follow_step(&s,20,true,100,180,0));
    s=(barcode_line_follow_t){0};
    int right=barcode_line_follow_step(&s,100,true,100,900,1);assert(right>0 && right<=100);
    assert(barcode_line_follow_step(&s,100,true,900,100,2)==right); /* Duplicate sample cannot refilter. */
    for(unsigned n=1;n<20;++n)barcode_line_follow_step(&s,100+n*10,true,900,100,2);
    assert(s.correction<0 && s.correction>=-100);
    assert(!barcode_line_follow_step(&s,300,true,900,900,3));
    assert(!s.primed);
    assert(barcode_line_follow_step(&s,310,true,0,1000,1)==100);
    assert(!barcode_line_follow_step(&s,320,false,0,1000,1) && !s.primed);
    assert(!barcode_line_follow_step(&s,330,true,1001,0,2));
    assert(!barcode_line_follow_step(&s,340,true,0,1000,4));
    assert(barcode_line_follow_step(&s,350,true,0,1000,1)==100);
    assert(!barcode_line_follow_step(&s,380,true,0,1000,1)); /* Gap is not reused. */
    assert(barcode_line_follow_step(&s,UINT32_MAX-5U,true,0,1000,1)==100);
    assert(barcode_line_follow_step(&s,4,true,0,1000,1)==100);
    for(unsigned left=0;left<=1000;left+=50)for(unsigned right_dark=0;right_dark<=1000;right_dark+=50)
      for(unsigned mask=0;mask<4;++mask) {
        barcode_line_follow_t test={0};int correction=barcode_line_follow_step(&test,10,true,
            (uint16_t)left,(uint16_t)right_dark,mask);
        assert(correction>=-100 && correction<=100);
        if(mask==3)assert(!correction);
        for(unsigned power=1;power<=30;++power)for(unsigned wheel=0;wheel<2;++wheel) {
            unsigned target=s2_steered_demand_milli(power,correction,wheel,30);
            assert(target>0 && target<=30000); /* No pivot/zero/reverse target. */
        }
      }
    puts("Barcode line correction: mirrored analog P, deadband/filter, no duplicate update, both-black/STOP/gap/wrap, positive wheel targets/cap PASS");
    return 0;
}
