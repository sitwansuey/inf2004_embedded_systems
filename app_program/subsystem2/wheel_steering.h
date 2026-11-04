#ifndef S2_WHEEL_STEERING_H
#define S2_WHEEL_STEERING_H
#include <stdint.h>
/* Positive correction slows the right wheel; negative slows the left. */
static inline int32_t s2_steering_step(int32_t previous,int32_t target,uint32_t dt_ms)
{
    int32_t step=(int32_t)(dt_ms*2U);
    if(target>previous+step)return previous+step;
    if(target<previous-step)return previous-step;
    return target;
}
static inline unsigned s2_steering_scale(int32_t steering,unsigned wheel)
{
    /* Positive (right) steering raises the left demand and lowers the right. */
    return (unsigned)(wheel==0U?1000+steering:1000-steering);
}
/* A common divisor preserves inner/outer target ratios at the PWM ceiling.
 * Keep milli-percent precision, including the saved per-wheel trim. */
static inline unsigned s2_steered_demand_milli(unsigned power,int32_t steering,
    unsigned wheel,unsigned limit)
{
    unsigned magnitude=(unsigned)(steering<0?-steering:steering);
    unsigned peak=1000U+magnitude;
    unsigned scale=s2_steering_scale(steering,wheel);
    uint64_t request=(uint64_t)power*scale;
    if((uint64_t)power*peak>(uint64_t)limit*1000U)
        request=(uint64_t)limit*1000U*scale/peak;
    return (unsigned)request;
}
#endif
