#include "../../app_program/subsystem1/telemetry_codec.h"
#include <stdio.h>
#include <inttypes.h>
#include <string.h>
int main(void) {
    int64_t values[TEL_FIELD_COUNT],again[TEL_FIELD_COUNT];uint8_t p[TELEMETRY_BYTES];
    while(scanf("%" SCNd64,&values[0])==1) {
        for(unsigned n=1;n<TEL_FIELD_COUNT;++n)if(scanf("%" SCNd64,&values[n])!=1)return 2;
        memset(p,0xA5,sizeof(p));
        if(!telemetry_encode(p,values)) {
            for(unsigned n=0;n<sizeof(p);++n)if(p[n]!=0xA5)return 3;
            puts("REJECT");continue;
        }
        if(!telemetry_decode(p,again) || memcmp(values,again,sizeof(values)))return 4;
        for(unsigned n=0;n<sizeof(p);++n)printf("%02x",p[n]);
        puts("");
        uint8_t saved=p[0];p[0]|=0x0F; /* version is reserved except encoded value zero */
        if(telemetry_decode(p,again))return 5;
        p[0]=saved;
        p[TELEMETRY_BYTES-1]|=0x80;
        if(telemetry_decode(p,again))return 6;
    }
    return 0;
}
