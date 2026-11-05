#include "../../app_program/wheel_calibration_policy.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    wheel_cal_t s={0};
    assert(!wheel_cal_step(&s,100,true));
    assert(!wheel_cal_arm(&s,100,31,800,true));
    assert(!wheel_cal_arm(&s,100,30,3001,true));
    assert(!wheel_cal_arm(&s,100,30,0,true));
    assert(!wheel_cal_arm(&s,100,30,800,false));
    assert(wheel_cal_arm(&s,100,30,800,true));
    assert(!wheel_cal_arm(&s,110,30,1000,true)); /* Repeated commands never extend a pulse. */
    assert(wheel_cal_step(&s,899,true)==30);
    assert(!wheel_cal_step(&s,900,true) && !s.running);
    assert(!wheel_cal_step(&s,901,true));
    assert(wheel_cal_arm(&s,UINT32_MAX-20U,15,800,true));
    assert(wheel_cal_step(&s,778U,true)==15);
    assert(!wheel_cal_step(&s,779U,true));
    assert(wheel_cal_arm(&s,100,20,800,true));
    assert(!wheel_cal_step(&s,120,false) && !s.running);
    assert(!wheel_cal_step(&s,130,true));
    assert(wheel_cal_arm(&s,100,25,800,true));wheel_cal_stop(&s);
    assert(!wheel_cal_step(&s,120,true));
    assert(wheel_cal_arm(&s,100,30,3000,true));
    assert(!wheel_cal_arm(&s,200,30,3000,true));
    assert(wheel_cal_step(&s,3099,true)==30);
    assert(!wheel_cal_step(&s,3100,true) && !s.running);
    assert(wheel_cal_arm(&s,UINT32_MAX-20U,30,3000,true));
    assert(wheel_cal_step(&s,2978U,true)==30);
    assert(!wheel_cal_step(&s,2979U,true));
    puts("Wheel calibration: no boot motion, bounded forward power/lease, no renewal, expiry/wrap, faults and STOP PASS");
    return 0;
}
