#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/wheel_steering.h"
#include "../../app_program/subsystem2/speed_pid.h"
int main(void) {
 assert(s2_steered_demand_milli(35,0,0,35)==35000);
 assert(s2_steered_demand_milli(35,0,1,35)==35000);
 assert(s2_steered_demand_milli(35,250,0,35)==35000);
 assert(s2_steered_demand_milli(35,250,1,35)==21000);
 assert(s2_steered_demand_milli(35,-250,0,35)==21000);
 assert(s2_steered_demand_milli(35,-250,1,35)==35000);
 assert(s2_steered_demand_milli(16,250,0,35)==20000);
 assert(s2_steered_demand_milli(16,250,1,35)==12000);
 for(int turn=-400;turn<=400;++turn)for(unsigned p=1;p<=50;++p)
  for(unsigned w=0;w<2;++w)assert(s2_steered_demand_milli(p,turn,w,35)<=35000);
 speed_pid_t pid={0};
 assert(speed_pid_step(&pid,875,0,20,25,35,30,10,0)==35);
 assert(35000U*73U/100000U==25U && 35000U*98U/100000U==34U);
 puts("PASS: 35-percent straight demand; calibrated feedforward; mirrored 35/21 bend targets; all steering targets and PID outputs bounded at 35 percent.");
}
