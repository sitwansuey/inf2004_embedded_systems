/* Wheel speed regulator; gains are milli-percent per count/second.
 * State and arithmetic are local, never a native-struct wire representation. */
#ifndef SPEED_PID_H
#define SPEED_PID_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { int32_t integral, previous_error; bool primed; } speed_pid_t;
static inline unsigned speed_pid_step(speed_pid_t *s, int32_t target, int32_t measured,
    uint32_t dt, unsigned feedforward, unsigned limit, int32_t kp, int32_t ki, int32_t kd)
{
    if (!s || !dt || dt>100U || limit>100U || kp<0 || ki<0 || kd<0 ||
        kp>10000 || ki>10000 || kd>10000 || target<0 || target>100000 ||
        measured < -100000 || measured>100000) return 0;
    int32_t error=target-measured;
    int64_t integral=(int64_t)s->integral+(int64_t)error*dt;
    if(integral>1000000)integral=1000000;
    if(integral< -1000000)integral=-1000000;
    int64_t derivative=s->primed?((int64_t)error-s->previous_error)*1000/(int64_t)dt:0;
    int64_t output=(int64_t)feedforward*1000+(int64_t)kp*error+
        (int64_t)ki*integral/1000+(int64_t)kd*derivative;
    /* Integrate only when not driving further into actuator saturation. */
    if(!((output>(int64_t)limit*1000 && error>0) || (output<0 && error<0)))
        s->integral=(int32_t)integral;
    s->previous_error=error;s->primed=true;
    if(output<0)output=0;
    if(output>(int64_t)limit*1000)output=(int64_t)limit*1000;
    return (unsigned)((output+500)/1000);
}
#endif
