/* Host-only replay of real timestamped ADC copies through production barcode code.
 * Never linked into firmware; no board command, navigation map or timing change. */
#include <errno.h>
#include <inttypes.h>
#include <stdio.h>
#include <stdlib.h>
#include "../../app_program/subsystem3/buddy3_robot_ir.h"

static int parse(const char *line,uint32_t value[6])
{
    for(unsigned i=0;i<6;++i) {
        char *end;
        unsigned long long n;
        if(*line<'0' || *line>'9') return 0;
        errno=0;n=strtoull(line,&end,10);
        if(errno || n>UINT32_MAX || end==line) return 0;
        value[i]=(uint32_t)n;
        if(i<5) {if(*end!=',') return 0;line=end+1;}
        else {while(*end=='\r'||*end=='\n') ++end;if(*end) return 0;}
    }
    return value[1]<=4095U && value[2]<=4095U && value[3]<=4095U && value[4]<=7U;
}
int main(int argc,char **argv)
{
    FILE *input;
    char line[160];
    s3_config_t cfg;
    s3_barcode_t capture;
    bool black=false;
    uint32_t previous=0,dropped=0;
    unsigned rows=0,segments=0,discontinuities=0,decoded=0,matched=0,wrong=0,rejected=0;
    bool have=false;
    if(argc!=3 || (argv[2][0]!='A' && argv[2][0]!='Z') || argv[2][1]) return 1;
    input=fopen(argv[1],"r");if(!input) return 1;
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    if(!robot_ir_apply_adc_calibration(&cfg)) {fclose(input);return 1;}
    while(fgets(line,sizeof(line),input)) {
        uint32_t v[6];
        s3_symbol_t symbol;
        s3_barcode_status_t status;
        uint16_t darkness;
        if(!parse(line,v)) {fclose(input);fprintf(stderr,"Invalid CSV row %u\n",rows+1U);return 1;}
        ++rows;
        if(v[4]!=7U) {have=false;++discontinuities;continue;}
        if(have && ((v[0]-previous)==0U || (v[0]-previous)>cfg.max_sample_gap_ms || v[5]!=dropped)) {
            have=false;++discontinuities;
        }
        if(!have) {
            s3_barcode_init(&capture);black=false;
            ++segments;have=true;
        }
        previous=v[0];dropped=v[5];
        if(!s3_normalize((uint16_t)v[1U+cfg.barcode_sensor],cfg.line.white[cfg.barcode_sensor],
            cfg.line.black[cfg.barcode_sensor],cfg.line.raw_max,&darkness)) {fclose(input);return 1;}
        if(darkness>=cfg.line.threshold_high) black=true;
        else if(darkness<=cfg.line.threshold_low) black=false;
        status=s3_barcode_feed(&capture,&cfg.barcode,black,v[0],&symbol);
        if(status==S3_BARCODE_OK) {
            ++decoded;
            if(symbol.symbol==argv[2][0]) ++matched;else ++wrong;
            printf("DECODE symbol=%c reverse=%u t=%" PRIu32 " segment=%u\n",symbol.symbol,symbol.reverse?1U:0U,v[0],segments);
        }
        else if(status!=S3_BARCODE_NONE) {++rejected;printf("REJECT status=%u t=%" PRIu32 " segment=%u\n",(unsigned)status,v[0],segments);}
    }
    if(ferror(input)) {fclose(input);return 1;}
    fclose(input);
    printf("SUMMARY rows=%u segments=%u discontinuities=%u rejected=%u decoded=%u expected=%u wrong=%u max_gap_ms=%" PRIu32 " timeout_ms=%" PRIu32 "\n",
           rows,segments,discontinuities,rejected,decoded,matched,wrong,cfg.max_sample_gap_ms,cfg.barcode.timeout_ms);
    /* Separate contiguous segments can produce complete captures. No partial
     * capture or artificial timestamp is ever carried across missing data. */
    return matched>0U && wrong==0U ? 0 : 2;
}
