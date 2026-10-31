/* Generated from tools/telemetry_schema.json. LSB-first, offset binary. */
#include "telemetry_codec.h"
#include <string.h>
typedef struct { int64_t min,max; uint8_t bits; } field_t;
static const field_t fields[TEL_FIELD_COUNT]={
    {1LL,1LL,4},
    {0LL,65535LL,16},
    {0LL,4294967295LL,32},
    {0LL,12LL,4},
    {0LL,7LL,3},
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
    {0LL,2LL,2},
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
    {0LL,31LL,5},
    {0LL,65535LL,16}
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
