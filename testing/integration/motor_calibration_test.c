#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_motor_calibration.h"
int main(void) {
 vc_state_t s={0};assert(vc_start(&s,100,35));
 vc_input_t i={.now_ms=100,.motor_ok=1,.transport_ok=1,.sensor_fresh=1,.terrain_valid=1,.odom_valid=1,.front_clear=1};
 vc_motor_calibration_step(&s,&i,1000);assert(s.command==1 && s.power==35 && !s.steering && s.phase==VC_PASS);
 i.now_ms=1099;vc_motor_calibration_step(&s,&i,1000);assert(s.command==1);
 i.now_ms=1100;vc_motor_calibration_step(&s,&i,1000);assert(!s.armed && !s.command && !s.power);
 s=(vc_state_t){0};assert(vc_start(&s,100,35));i.now_ms=200;i.obstacle=1;
 vc_motor_calibration_step(&s,&i,1000);assert(s.fault==VC_SENSOR && !s.command);
 s=(vc_state_t){0};assert(vc_start(&s,UINT32_MAX-100U,35));i.now_ms=1898;i.obstacle=0;
 vc_motor_calibration_step(&s,&i,2000);assert(s.command==1);
 i.now_ms=1899;vc_motor_calibration_step(&s,&i,2000);assert(!s.armed && !s.command);
 s=(vc_state_t){0};assert(vc_arm(&s,0,500,35));i.now_ms=500;
 vc_motor_calibration_step(&s,&i,2000);assert(s.fault==VC_TIMEOUT && !s.command);
 puts("PASS: straight-only calibration, 1000/2000 ms stops, timer wrap, command lease and obstacle stop.");
}
