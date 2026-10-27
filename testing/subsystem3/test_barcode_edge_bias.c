/* Regression fixtures model edge displacement, not a fitted symbol lookup. */
#include "../../app_program/subsystem3/buddy3_robot_ir.h"
#include <stdio.h>
#include <string.h>
#include <limits.h>

static unsigned checks, failures;
#define CHECK(c) do { ++checks; if (!(c)) { ++failures; printf("FAIL %d: %s\n", __LINE__, #c); } } while (0)
static const uint32_t a[29]={1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1};
static const uint32_t z[29]={1,3,1,1,3,1,3,1,1,1,1,3,3,1,3,1,1,1,1,1,1,3,1,1,3,1,3,1,1};
static const uint32_t measured[29]={30,140,40,70,130,60,140,70,30,70,120,60,30,60,40,160,40,60,140,80,30,190,30,100,180,110,220,100,40};

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
        bool known=pattern==0x109U || pattern==0x0D0U;
        CHECK((status==S3_BARCODE_OK)==known);
        if (status==S3_BARCODE_OK) {
            ++accepted;CHECK(symbol.symbol==(pattern==0x109U?'A':'Z'));
        } else CHECK(symbol.symbol==0);
        for (unsigned frame=0U;frame<2U;++frame) {
            for (unsigned i=0U;i<29U;++i) runs[i]=a[i]*40U;
            set_character(runs,frame?20U:0U,pattern);
            status=s3_barcode_decode(runs,29U,&symbol);
            CHECK((status==S3_BARCODE_OK)==(pattern==0x094U));
            if (status!=S3_BARCODE_OK) CHECK(symbol.symbol==0);
        }
    }
    CHECK(accepted==2U);
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
static void test_calibrated_capture_budget(void)
{
    s3_config_t cfg;s3_barcode_t capture;s3_symbol_t symbol;
    s3_config_defaults(&cfg);CHECK(cfg.barcode.min_run_ms==40U);
    robot_ir_apply_layout(&cfg);CHECK(robot_ir_apply_adc_calibration(&cfg));
    CHECK(cfg.barcode.min_run_ms==20U);
    CHECK(cfg.max_sample_gap_ms==30U && cfg.barcode.timeout_ms==10000U);
    s3_barcode_init(&capture);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,200U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,true,210U,&symbol);
    CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,220U,&symbol)==S3_BARCODE_NOISE);
    CHECK(capture.state==S3_CAPTURE_HOLD && symbol.symbol==0);
    s3_barcode_init(&capture);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,0U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,false,200U,&symbol);
    (void)s3_barcode_feed(&capture,&cfg.barcode,true,210U,&symbol);
    CHECK(s3_barcode_feed(&capture,&cfg.barcode,false,230U,&symbol)==S3_BARCODE_NONE);
    CHECK(capture.count==1U && capture.runs[0]==20U);
}
int main(void)
{
    test_scale_bias_and_direction();
    test_exhaustive_symbols_and_framing();
    test_invalid_shapes();
    test_calibrated_capture_budget();
    printf("S3 barcode edge-bias regressions: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
