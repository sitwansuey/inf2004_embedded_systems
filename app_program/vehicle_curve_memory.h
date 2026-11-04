#ifndef VEHICLE_CURVE_MEMORY_H
#define VEHICLE_CURVE_MEMORY_H
#include <stdint.h>
#include <stdbool.h>
typedef struct { int16_t steering; uint32_t last_black_ms; bool turning; } curve_memory_t;
/* Keep the observed turn through up to 300 ms of neutral/white readings. */
static inline int16_t curve_memory_update(curve_memory_t *m,uint32_t now,
    bool active,bool fresh,unsigned mask,int error)
{
    if(!active || !fresh) { *m=(curve_memory_t){0};return 0; }
    if(mask==1U || mask==2U) {
        int correction=error*250/1000;
        if(correction>250)correction=250;
        if(correction< -250)correction=-250;
        m->steering=(int16_t)correction;m->last_black_ms=now;m->turning=true;
    }
    if(m->turning && now-m->last_black_ms>=300U)m->turning=false;
    return m->turning?m->steering:0;
}
#endif
