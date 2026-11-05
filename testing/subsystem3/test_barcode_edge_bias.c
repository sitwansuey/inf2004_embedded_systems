/* Regression fixtures model edge displacement, not a fitted symbol lookup. */
#include "../../app_program/robot_ir_config.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)
static const uint32_t a[29]={1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1};
static const uint32_t z[29]={1,3,1,1,3,1,3,1,1,1,1,3,3,1,3,1,1,1,1,1,1,3,1,1,3,1,3,1,1};
static const uint32_t measured[29]={30,140,40,70,130,60,140,70,30,70,120,60,30,60,40,160,40,60,140,80,30,190,30,100,180,110,220,100,40};
static const uint32_t rejected_f[29]={20,200,30,90,120,80,120,80,40,60,30,70,130,70,120,180,30,80,20,80,30,190,30,80,130,70,120,80,20};
static const uint32_t distorted_a[29]={90,170,30,70,130,70,130,70,30,70,120,80,20,70,30,190,30,70,130,70,30,190,30,70,130,70,130,80,20};
static const unsigned known_patterns[26]={0x109,0x049,0x148,0x019,0x118,0x058,0x00d,0x10c,0x04c,0x01c,0x103,0x043,0x142,0x013,0x112,0x052,0x007,0x106,0x046,0x016,0x181,0x0c1,0x1c0,0x091,0x190,0x0d0};

static void generate(const uint32_t *pattern, uint32_t *runs, uint32_t unit, int bias_percent)
{
    const uint32_t units[3]={unit,unit*6U/5U,unit*7U/5U};
    for (unsigned i=0U;i<29U;++i) {
        unsigned character=i/10U;
        int64_t local=(int64_t)units[character];
        int64_t bias=local*bias_percent/100;
        if (i==9U || i==19U) {
            local=(local+(int64_t)units[character+1U])/2;
            bias=local*bias_percent/100;
        }
        int64_t value=local*(int64_t)pattern[i]+((i&1U)?bias:-bias);
        CHECK(value>0 && value<=UINT32_MAX);
        runs[i]=(uint32_t)value;
    }
}
static void test_scale_bias_and_direction(void)
{
    const uint32_t units[]={40U,80U,1000U,100000000U};
    const int biases[]={-50,-35,-20,0,20,35,50};
    uint32_t runs[29],reverse[29];
    s3_symbol_t symbol;
    for (unsigned kind=0U;kind<2U;++kind)
        for (unsigned u=0U;u<sizeof(units)/sizeof(units[0]);++u)
            for (unsigned b=0U;b<sizeof(biases)/sizeof(biases[0]);++b) {
                generate(kind?z:a,runs,units[u],biases[b]);
                CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_OK);
                CHECK(symbol.symbol==(kind?'Z':'A') && !symbol.reverse);
                for (unsigned i=0U;i<29U;++i) reverse[i]=runs[28U-i];
                CHECK(s3_barcode_decode(reverse,29U,&symbol)==S3_BARCODE_OK);
                CHECK(symbol.symbol==(kind?'Z':'A') && symbol.reverse);
            }
    CHECK(s3_barcode_decode(measured,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='A');
    for (unsigned i=0U;i<29U;++i) reverse[i]=measured[28U-i];
    CHECK(s3_barcode_decode(reverse,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='A' && symbol.reverse);
}
static void set_character(uint32_t *runs,unsigned start,unsigned pattern)
{
    for (unsigned i=0U;i<9U;++i) runs[start+i]=(pattern&(1U<<(8U-i)))?120U:40U;
}
static void test_exhaustive_symbols_and_framing(void)
{
    uint32_t runs[29];
    s3_symbol_t symbol;
    unsigned accepted=0U;
    for (unsigned pattern=0U;pattern<512U;++pattern) {
        for (unsigned i=0U;i<29U;++i) runs[i]=a[i]*40U;
        set_character(runs,10U,pattern);
        s3_barcode_status_t status=s3_barcode_decode(runs,29U,&symbol);
        char expected=0;
        for(unsigned n=0;n<26;++n)if(pattern==known_patterns[n])expected=(char)('A'+(int)n);
        bool known=expected!=0;
        CHECK((status==S3_BARCODE_OK)==known);
        if (status==S3_BARCODE_OK) {
            ++accepted;CHECK(symbol.symbol==expected);
        } else CHECK(symbol.symbol==0);
        for (unsigned frame=0U;frame<2U;++frame) {
            for (unsigned i=0U;i<29U;++i) runs[i]=a[i]*40U;
            set_character(runs,frame?20U:0U,pattern);
            status=s3_barcode_decode(runs,29U,&symbol);
            CHECK((status==S3_BARCODE_OK)==(pattern==0x094U));
            if (status!=S3_BARCODE_OK) CHECK(symbol.symbol==0);
        }
    }
    CHECK(accepted==26U);
}
static void test_invalid_shapes(void)
{
    uint32_t runs[29];
    s3_symbol_t symbol;
    CHECK(s3_barcode_decode(measured,28U,&symbol)==S3_BARCODE_INCOMPLETE);
    CHECK(s3_barcode_decode(measured,30U,&symbol)==S3_BARCODE_INCOMPLETE);
    for (unsigned index=0U;index<29U;++index) {
        for (unsigned i=0U;i<29U;++i) runs[i]=a[i]*40U;
        runs[index]=0U;CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
        for (unsigned i=0U;i<29U;++i) runs[i]=a[i]*40U;
        runs[index]=UINT32_MAX;CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    }
    for (unsigned i=0U;i<29U;++i) runs[i]=100U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    for (unsigned i=0U;i<29U;++i) runs[i]=UINT32_MAX;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    generate(a,runs,100U,80);
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    generate(a,runs,100U,-80);
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    /* Three largest intervals still label A; one huge element must fail. */
    memcpy(runs,measured,sizeof(runs));runs[4]=10000U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
    memcpy(runs,measured,sizeof(runs));runs[9]=300U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK);
}
static void test_joint_edge_compensation(void)
{
    s3_symbol_t symbol;uint32_t runs[29],reverse[29],pattern[29];
    const int biases[]={-50,-35,-20,0,20,35,50};
    for(unsigned letter=0;letter<26;++letter) {
        memcpy(pattern,a,sizeof(pattern));
        for(unsigned n=0;n<9;++n)pattern[n+10U]=(known_patterns[letter]&(1U<<(8U-n)))?3U:1U;
        for(unsigned b=0;b<sizeof(biases)/sizeof(biases[0]);++b) {
            generate(pattern,runs,100U,biases[b]);
            CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_OK);
            CHECK(symbol.symbol==(char)('A'+letter) && !symbol.reverse);
            for(unsigned n=0;n<29;++n)reverse[n]=runs[28U-n];
            CHECK(s3_barcode_decode(reverse,29U,&symbol)==S3_BARCODE_OK);
            CHECK(symbol.symbol==(char)('A'+letter) && symbol.reverse);
        }
    }
    /* Exact complete F rejection from the21:47:00 hardware log. */
    for(unsigned scale=1U;scale<=1000000U;scale*=1000000U) {
        for(unsigned n=0;n<29;++n)runs[n]=rejected_f[n]*scale;
        CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='F' && !symbol.reverse);
        for(unsigned n=0;n<29;++n)reverse[n]=runs[28U-n];
        CHECK(s3_barcode_decode(reverse,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='F' && symbol.reverse);
        if(scale==1000000U)break;
    }
    CHECK(s3_barcode_decode(distorted_a,29U,&symbol)==S3_BARCODE_BAD_PATTERN && !symbol.symbol);
    /* Gap fits must share the same bounded biases as both adjacent characters. */
    memcpy(runs,rejected_f,sizeof(runs));runs[9]=10U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK && !symbol.symbol);
    memcpy(runs,rejected_f,sizeof(runs));runs[19]=200U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK && !symbol.symbol);
    memcpy(runs,rejected_f,sizeof(runs));runs[0]=0U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)!=S3_BARCODE_OK && !symbol.symbol);
    /* Each extreme gap can fit on its own, but their shared character cannot
     * simultaneously require both +25% and -25% sensor displacement. */
    for(unsigned n=0;n<29;++n)runs[n]=a[n]*100U;
    for(unsigned n=0;n<9;++n)runs[n+10U]=(0x058U&(1U<<(8U-n)))?300U:100U;
    runs[9]=150U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='F');
    runs[9]=100U;runs[19]=50U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_OK && symbol.symbol=='F');
    runs[9]=150U;
    CHECK(s3_barcode_decode(runs,29U,&symbol)==S3_BARCODE_BAD_PATTERN && !symbol.symbol);
}
static void test_calibrated_capture_budget(void)
{
    s3_config_t cfg;s3_barcode_t capture;s3_symbol_t symbol;
    s3_config_defaults(&cfg);CHECK(cfg.barcode.min_run_ms==40U);
    robot_ir_apply_layout(&cfg);CHECK(robot_ir_apply_adc_calibration(&cfg));
    CHECK(cfg.barcode.min_run_ms==BOARD_BARCODE_MIN_RUN_MS);
    CHECK(cfg.max_sample_gap_ms==30U && cfg.barcode.timeout_ms==10000U);
    s3_barcode_init(&capture);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,200U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,true,210U,&symbol);
    if (cfg.barcode.min_run_ms==0U) {
        CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,220U,&symbol)==S3_BARCODE_NONE);
        CHECK(capture.state==S3_CAPTURE_RUNNING && capture.count==1U && capture.runs[0]==10U);
    } else {
        CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,220U,&symbol)==S3_BARCODE_NOISE);
        CHECK(capture.state==S3_CAPTURE_HOLD && symbol.symbol==0);
    }
    s3_barcode_init(&capture);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,200U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,true,210U,&symbol);
    CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,230U,&symbol)==S3_BARCODE_NONE);
    CHECK(capture.count==1U && capture.runs[0]==20U);
}
static void test_no_minimum_capture(void)
{
    s3_config_t cfg;
    s3_config_defaults(&cfg);cfg.barcode.min_run_ms=0U;
    CHECK(s3_config_valid(&cfg));
    for (unsigned direction=0U;direction<2U;++direction) {
        for (unsigned unit=1U;unit<=10U;unit+=9U) {
            s3_barcode_t capture;s3_symbol_t symbol;uint32_t now=210U;
            s3_barcode_init(&capture);
            CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol)==S3_BARCODE_NONE);
            CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,200U,&symbol)==S3_BARCODE_NONE);
            CHECK(s3_barcode_feed(&capture,&cfg.barcode,true,now,&symbol)==S3_BARCODE_NONE);
            for (unsigned i=0U;i<29U;++i) {
                now+=a[direction?28U-i:i]*unit;
                CHECK(s3_barcode_feed(&capture,&cfg.barcode,(i&1U)!=0U,now,&symbol)==S3_BARCODE_NONE);
            }
            CHECK(capture.count==29U);
            CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,now+200U,&symbol)==S3_BARCODE_OK);
            CHECK(symbol.symbol=='A' && symbol.reverse==(direction!=0U));
        }
    }
    /* Removing the duration floor does not accept duplicate timestamps. */
    s3_barcode_t capture;s3_symbol_t symbol;
    s3_barcode_init(&capture);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol);
    CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol)==S3_BARCODE_NOISE);
}
int main(void)
{
    test_scale_bias_and_direction();
    test_exhaustive_symbols_and_framing();
    test_invalid_shapes();
    test_joint_edge_compensation();
    test_calibrated_capture_budget();
    test_no_minimum_capture();
    printf("S3 barcode edge-bias regressions: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
