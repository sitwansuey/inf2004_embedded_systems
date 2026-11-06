#ifndef VEHICLE_MOTOR_CALIBRATION_H
#define VEHICLE_MOTOR_CALIBRATION_H
#include "vehicle_policy.h"
/* Explicit test profile only. No line search, steering or obstacle bypass. */
static inline void vc_motor_calibration_step(vc_state_t *s,const vc_input_t *i,uint32_t duration)
{
 s->command=0;s->power=0;s->steering=0;
 if(!s->armed)return;
 if(!duration || duration>2000U || i->now_ms-s->armed_ms>=duration){vc_stop(s,VC_OK);return;}
 if(s->lease_ms && i->now_ms-s->armed_ms>=s->lease_ms){vc_stop(s,VC_TIMEOUT);return;}
 if(i->shock || i->safety_event){vc_stop(s,i->shock?VC_SHOCK:VC_LINE);return;}
 if(!i->transport_ok){vc_stop(s,VC_TRANSPORT);return;}
 if(!i->motor_ok){vc_stop(s,VC_MOTOR);return;}
 if(!i->sensor_fresh || !i->terrain_valid || !i->odom_valid || i->obstacle){vc_stop(s,VC_SENSOR);return;}
 if(!i->front_clear)return;
 s->phase=VC_PASS;s->command=1;s->power=s->maximum_power;
}
#endif
