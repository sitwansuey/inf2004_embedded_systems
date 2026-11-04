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
#endif
