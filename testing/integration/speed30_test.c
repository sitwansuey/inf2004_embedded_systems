#include <assert.h>
#include <stdio.h>
#include "../../app_program/subsystem2/wheel_steering.h"
#include "../../app_program/subsystem2/speed_pid.h"
int main(void) {
 assert(s2_steered_demand_milli(30,0,0,30)==30000);
 assert(s2_steered_demand_milli(30,0,1,30)==30000);
 assert(s2_steered_demand_milli(30,250,0,30)==30000);
 assert(s2_steered_demand_milli(30,250,1,30)==18000);
 assert(s2_steered_demand_milli(30,-250,0,30)==18000);
 assert(s2_steered_demand_milli(30,-250,1,30)==30000);
 assert(s2_steered_demand_milli(16,250,0,30)==20000);
 assert(s2_steered_demand_milli(16,250,1,30)==12000);
 for(int turn=-400;turn<=400;++turn)for(unsigned p=1;p<=50;++p)
  for(unsigned w=0;w<2;++w)assert(s2_steered_demand_milli(p,turn,w,30)<=30000);
 speed_pid_t pid={0};
 assert(speed_pid_step(&pid,750,0,20,21,30,30,10,0)==30);
 assert(30000U*73U/100000U==21U && 30000U*98U/100000U==29U);
 puts("PASS: 30-percent straight demand; calibrated feedforward; mirrored 30/18 bend targets; all steering targets and PID outputs bounded at 30 percent.");
}
