#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/speed_pid.h"
#include "../../app_program/subsystem2/wheel_steering.h"
int main(void){
 speed_pid_t p={0};assert(speed_pid_step_milli(&p,101,100,20,20000,30,20,0,0)==20020U);
 p=(speed_pid_t){0};assert(speed_pid_step(&p,101,100,20,20,30,20,0,0)==20U);
 p=(speed_pid_t){0};assert(speed_pid_step_milli(&p,10000,0,20,20000,30,1000,1000,0)==30000U && p.integral==0);
 p=(speed_pid_t){0};assert(speed_pid_step_milli(&p,0,10000,20,20000,30,1000,1000,0)==0 && p.integral==0);
 assert(s2_steered_demand_milli(25,376,0,30)>s2_steered_demand_milli(25,376,1,30));
 assert(s2_steered_demand_milli(25,-376,0,30)<s2_steered_demand_milli(25,-376,1,30));
 assert(s2_steered_demand_milli(25,376,0,30)<=30000U);
 puts("PASS: fractional PI corrections retained; legacy rounding preserved; saturation anti-windup; calibrated wheel ratios remain capped.");return 0;
}
