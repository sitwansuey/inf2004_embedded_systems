#include "../../result_data.h"
#include "../../packed_bits.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
int main(int argc,char **argv)
{
    if(argc==2) {
        uint8_t data[S4_RESULT_SIZE];size_t length=strlen(argv[1])/2U;
        if(length!=sizeof(data) || strlen(argv[1])%2U)return 2;
        for(unsigned n=0;n<sizeof(data);++n) {
            unsigned value;
            if(sscanf(argv[1]+n*2U,"%2x",&value)!=1)return 2;
            data[n]=(uint8_t)value;
        }
        s4_result_t v;
        if(!s4_result_decode(data,length,&v))return 2;
        printf("%u %d %d %u %u %u %u %u\n",v.calculated_ms,
            (int)v.pitch_cdeg,(int)v.roll_cdeg,v.height_mm,v.maximum_mm,v.terrain,v.flags,v.faults);
        return 0;
    }
    s4_result_t v={.calculated_ms=UINT32_MAX,.pitch_cdeg=-9000,.roll_cdeg=18000,
        .height_mm=50,.maximum_mm=75,.terrain=3,.flags=27,.faults=0},out;
    uint8_t bytes[S4_RESULT_SIZE+1U];memset(bytes,0xa5,sizeof(bytes));
    assert(s4_result_encode(&v,bytes,sizeof(bytes)) && bytes[S4_RESULT_SIZE]==0xa5);
    assert(s4_result_decode(bytes,S4_RESULT_SIZE,&out));
    assert(out.calculated_ms==v.calculated_ms && out.pitch_cdeg==v.pitch_cdeg &&
        out.roll_cdeg==v.roll_cdeg && out.maximum_mm==75 && out.faults==0);
    s4_result_t fault=v;fault.flags=0;fault.faults=S4_RESULT_FAULT_MASK;
    assert(s4_result_encode(&fault,bytes,sizeof(bytes)));
    assert(s4_result_decode(bytes,S4_RESULT_SIZE,&out) && out.faults==S4_RESULT_FAULT_MASK);
    assert(s4_result_encode(&v,bytes,sizeof(bytes)));
    for(size_t n=0;n<=14;++n)if(n!=S4_RESULT_SIZE) {
        out=v;assert(!s4_result_decode(bytes,n,&out));assert(out.maximum_mm==75);
    }
    for(unsigned bit=93;bit<96;++bit) {
        bytes[11]|=(uint8_t)(1U<<(bit-88U));
        assert(!s4_result_decode(bytes,S4_RESULT_SIZE,&out));bytes[11]&=0x1fU;
    }
    s4_bits_put(bytes,32,15,32767);assert(!s4_result_decode(bytes,12,&out));
    for(unsigned n=0;n<8;++n) {
        s4_result_t bad=v;
        switch(n) {
        case 0:bad.pitch_cdeg=9001;break;case 1:bad.roll_cdeg=-18001;break;
        case 2:bad.height_mm=1001;break;case 3:bad.maximum_mm=1001;break;
        case 4:bad.terrain=6;break;case 5:bad.flags=32;break;
        case 6:bad.faults=1;break;default:bad.maximum_mm=49;break;
        }
        memset(bytes,0xa5,sizeof(bytes));assert(!s4_result_encode(&bad,bytes,sizeof(bytes)));
        for(unsigned j=0;j<sizeof(bytes);++j)assert(bytes[j]==0xa5);
    }
    assert(!s4_result_encode(NULL,bytes,sizeof(bytes)));
    assert(!s4_result_decode(NULL,12,&out));
    puts("S4-U30 result codec: exact size, boundaries, padding, fault mapping, semantic checks and unchanged errors PASS");
    return 0;
}
