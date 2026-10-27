/* Observed endpoints/limits plus paused interpretation through production core. */
#include <stdio.h>
#include "../../app_program/robot_ir_config.h"
static unsigned checks,failures;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; printf("FAIL line %d: %s\n",__LINE__,#c); } } while(0)
int main(void) {
    s3_config_t cfg;
    s3_core_t core;
    s3_output_t output;
    s3_sample_t sample={{187U,189U,213U},7U,20U};
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
#if BOARD_IR_CALIBRATED_BENCH
    /* Historical GP26/27 floor bounds; replacement GP28 training bounds.
     * These are labelled observations, not guaranteed sensor specifications. */
    const uint16_t low[2][3]={{177U,165U,177U},{3198U,3157U,2449U}};
    const uint16_t high[2][3]={{196U,212U,219U},{3225U,3195U,2493U}};
    CHECK(robot_ir_apply_adc_calibration(&cfg));
    CHECK(BOARD_MOTORS_DISABLED==1 && BOARD_IR_MODE==2 && !BOARD_IR_RAW_BENCH);
    for(unsigned s=0;s<2;++s) for(unsigned i=0;i<3;++i) {
        uint16_t a,b;
        CHECK(s3_normalize(low[s][i],cfg.line.white[i],cfg.line.black[i],4095U,&a));
        CHECK(s3_normalize(high[s][i],cfg.line.white[i],cfg.line.black[i],4095U,&b));
        CHECK(s ? (a>=cfg.line.threshold_high && b>=cfg.line.threshold_high) :
                  (a<=cfg.line.threshold_low && b<=cfg.line.threshold_low));
    }
    /* Independent repeat captures at the same reported mounted height;
     * no endpoint fitting uses these bounds. Full logs are replayed separately. */
    {
        const uint16_t held_out[2][2]={{186U,229U},{3046U,3090U}};
        for(unsigned s=0;s<2;++s) for(unsigned edge=0;edge<2;++edge) {
            uint16_t darkness=0U;
            CHECK(s3_normalize(held_out[s][edge],cfg.line.white[2],cfg.line.black[2],4095U,&darkness));
            CHECK(s ? darkness>=cfg.line.threshold_high : darkness<=cfg.line.threshold_low);
        }
    }
    CHECK(s3_core_init(&core,&cfg));
    for(unsigned mask=0;mask<8;++mask) {
        sample.time_ms+=20U;
        for(unsigned i=0;i<3;++i) sample.raw[i]=(mask&(1U<<i))?cfg.line.black[i]:cfg.line.white[i];
        s3_core_step(&core,&sample,&output);
        CHECK(output.line.black_mask==(mask&3U));
        CHECK(!output.steering_allowed && output.event.kind==S3_EVENT_NONE && output.error==S3_ERROR_NONE);
        CHECK(output.decoded.symbol==0 && core.mode==S3_MODE_PAUSED);
        CHECK(core.barcode_black==((mask&4U)!=0U));
        if((mask&3U)==1U) CHECK(output.line.state==S3_LINE_TRACKING && output.line.error==1000);
        if((mask&3U)==2U) CHECK(output.line.state==S3_LINE_TRACKING && output.line.error==-1000);
        if((mask&3U)==3U) CHECK(output.line.state==S3_LINE_JUNCTION);
    }
    /* Bad acquisition cannot retain the last classification as steering. */
    sample.time_ms+=20U;sample.valid_mask=0U;
    s3_core_step(&core,&sample,&output);
    CHECK(!output.steering_allowed && output.error==S3_ERROR_SENSOR);
#else
    (void)core;(void)output;(void)sample;
    CHECK(!robot_ir_apply_adc_calibration(&cfg));
    CHECK(BOARD_IR_MODE==0 && BOARD_MOTORS_DISABLED==1);
#endif
    printf("S3-U43 measured calibration: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
