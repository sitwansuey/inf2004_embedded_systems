#ifndef MOTOR_REFERENCE_H
#define MOTOR_REFERENCE_H
#include <stdint.h>
static inline unsigned motor_reference_target(unsigned demand_milli,unsigned scale,unsigned factor)
{
 return (unsigned)((uint64_t)demand_milli*scale*factor/100000000U);
}
static inline unsigned motor_reference_ff(unsigned demand_milli,unsigned trim,unsigned offset)
{
 if(!demand_milli)return 0;
 return (unsigned)(((uint64_t)demand_milli*trim+50000U)/100000U)+offset;
}
static inline unsigned motor_reference_ticks(unsigned wrap,unsigned pwm_milli)
{
 return (unsigned)(((uint64_t)wrap*pwm_milli+50000U)/100000U);
}
#endif
