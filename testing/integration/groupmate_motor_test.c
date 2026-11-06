#include <assert.h>
#include <stdio.h>
#include "../../calibration/motor_speed_calibrated_20261106.h"
#include "../../app_program/subsystem2/motor_reference.h"
#include "../../app_program/subsystem2/speed_pid.h"
int main(void) {
 assert(BOARD_PID_MAX_POWER==30 && BOARD_DRIVE_POWER_CAP==30);
 assert(motor_reference_target(20000,1667,1000)==333);
 assert(motor_reference_target(20000,1667,970)==323);
 assert(motor_reference_target(30000,1667,1000)==500);
 assert(motor_reference_target(30000,1667,970)==485);
 assert(motor_reference_target(20000,1667,1000)==333); /* Reverse/pivot factor stays neutral. */
 assert(motor_reference_ff(20000,74,3)==18);
 assert(motor_reference_ff(20000,78,4)==20);
 assert(motor_reference_ff(30000,74,3)==25);
 assert(motor_reference_ff(30000,78,4)==27);
 assert(motor_reference_ff(0,74,3)==0 && motor_reference_ff(0,78,4)==0);
 speed_pid_t p={0};
 assert(speed_pid_step_milli(&p,333,332,20,18000,30,20,0,0)==18020);
 assert(motor_reference_ticks(6250,18020)==1126);
 assert(motor_reference_ticks(6250,18000)==1125);
 p=(speed_pid_t){0};assert(speed_pid_step_milli(&p,500,0,20,25000,30,20,10,0)==30000);
 assert(p.integral==0); /* Saturation must not wind the integral up. */
 for(int measured=-1000;measured<=1000;++measured) {
  p=(speed_pid_t){0};unsigned output=speed_pid_step_milli(&p,333,measured,20,18000,30,20,10,0);
  assert(output<=30000 && motor_reference_ticks(6250,output)<=1875);
 }
 puts("PASS: supplied 20/30 command targets, rounded trim plus offsets, zero demand, milli-percent PI retained through PWM ticks, cap and anti-windup.");
}
