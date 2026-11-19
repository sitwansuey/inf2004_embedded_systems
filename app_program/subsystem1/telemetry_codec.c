/* PRIVATE queue snapshot; field table must match tools/telemetry_schema.json (checked by testing/integration protocol tests). Never MQTT. */
#include "telemetry_codec.h"
#include <string.h>
typedef struct { int64_t min,max; uint8_t bits; } field_t;
static const field_t fields[TEL_FIELD_COUNT]={
    {1LL,1LL,4},
    {0LL,65535LL,16},
    {0LL,4294967295LL,32},
    {0LL,13LL,4},
    {0LL,8LL,4},
    {0LL,6LL,3},
    {0LL,100LL,7},
    {0LL,2LL,2},
    {0LL,255LL,8},
    {-2147483648LL,2147483647LL,32},
    {-2147483648LL,2147483647LL,32},
    {-2000LL,2000LL,12},
    {0LL,4LL,3},
    {0LL,4095LL,12},
    {0LL,4095LL,12},
    {0LL,4095LL,12},
    {0LL,7LL,3},
    {-1000LL,1000LL,11},
    {0LL,26LL,5},
    {0LL,4LL,3},
    {-2048LL,2047LL,12},
    {-2048LL,2047LL,12},
    {-2048LL,2047LL,12},
    {-2048LL,2047LL,12},
    {-2048LL,2047LL,12},
    {-2048LL,2047LL,12},
    {-9000LL,9000LL,15},
    {-18000LL,18000LL,16},
    {0LL,5LL,3},
    {0LL,1000LL,10},
    {0LL,1000LL,10},
    {0LL,5000LL,13},
    {0LL,500LL,9},
    {-90LL,90LL,8},
    {0LL,14LL,4},
    {0LL,63LL,6},
    {0LL,65535LL,16},
    {0LL,2LL,2},
    {0LL,31LL,5},
    {0LL,3LL,2},
    {0LL,1000LL,10},
    {0LL,1000LL,10},
    {0LL,2LL,2},
    {0LL,3LL,2},
    {0LL,1LL,1}
};
bool telemetry_encode(uint8_t out[TELEMETRY_BYTES],const int64_t v[TEL_FIELD_COUNT]) {
    if(!out || !v)return false;
    uint8_t tmp[TELEMETRY_BYTES]={0};unsigned bit=0;
    for(unsigned f=0;f<TEL_FIELD_COUNT;++f) {
        if(v[f]<fields[f].min || v[f]>fields[f].max)return false;
        uint64_t n=(uint64_t)(v[f]-fields[f].min);
        for(unsigned b=0;b<fields[f].bits;++b,++bit)
            tmp[bit/8]|=(uint8_t)(((n>>b)&1U)<<(bit%8));
    }
    memcpy(out,tmp,sizeof(tmp));return true;
}
bool telemetry_decode(const uint8_t in[TELEMETRY_BYTES],int64_t v[TEL_FIELD_COUNT]) {
    if(!in || !v)return false;
    int64_t tmp[TEL_FIELD_COUNT];unsigned bit=0;
    for(unsigned f=0;f<TEL_FIELD_COUNT;++f) {
        uint64_t n=0;
        for(unsigned b=0;b<fields[f].bits;++b,++bit)n|=(uint64_t)((in[bit/8]>>(bit%8))&1U)<<b;
        tmp[f]=(int64_t)n+fields[f].min;
        if(tmp[f]>fields[f].max)return false;
    }
    for(;bit<TELEMETRY_BYTES*8;++bit)if((in[bit/8]>>(bit%8))&1U)return false;
    memcpy(v,tmp,sizeof(tmp));return true;
}
void telemetry_keep_flags(uint8_t newer[TELEMETRY_BYTES],const uint8_t older[TELEMETRY_BYTES]) {
    if(!newer || !older)return;
    /* Both fields start at 0, so a value bit is the stored bit: no re-encode. */
    unsigned at[2]={1U,0U},bit=0; /* TEL_QUALITY bit 1, TEL_IMU_FAULT */
    for(unsigned f=0;f<TEL_FIELD_COUNT;++f) {
        if(f==TEL_QUALITY)at[0]+=bit;
        if(f==TEL_IMU_FAULT)at[1]+=bit;
        bit+=fields[f].bits;
    }
    for(unsigned n=0;n<2U;++n)newer[at[n]/8U]|=(uint8_t)(older[at[n]/8U]&(1U<<(at[n]%8U)));
}
