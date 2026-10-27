/* Production-core regression for the robot's latched paused timing fault. */
#include <stdio.h>
#include "../../app_program/robot_ir_config.h"
static unsigned checks,failures;
#define CHECK(c) do { ++checks; if(!(c)) { ++failures; printf("FAIL line %d: %s\n",__LINE__,#c); } } while(0)

static void test_paused_forward_gaps(void)
{
    const uint32_t start[]={0U,UINT32_MAX-19U};
    const uint32_t gaps[]={20U,30U,40U,1000U,20U};
    s3_config_t cfg;
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    CHECK(robot_ir_apply_adc_calibration(&cfg));
    for(unsigned trial=0;trial<2;++trial) {
        s3_core_t core;
        s3_output_t output;
        s3_sample_t sample={{3214U,189U,213U},7U,start[trial]};
        CHECK(s3_core_init(&core,&cfg));
        s3_core_step(&core,&sample,&output);
        for(unsigned j=0;j<10;++j) {
            sample.time_ms+=gaps[j%5U];
            s3_core_step(&core,&sample,&output);
            CHECK(output.error==S3_ERROR_NONE && core.latched_error==S3_ERROR_NONE);
            CHECK(output.line.state==S3_LINE_TRACKING && output.line.error==1000);
            CHECK(!output.steering_allowed && output.event.kind==S3_EVENT_NONE);
            CHECK(core.mode==S3_MODE_PAUSED && output.decoded.symbol==0);
        }
    }
}

static void test_active_fault_stays_latched(void)
{
    s3_config_t cfg;
    s3_core_t core;
    s3_output_t output;
    s3_sample_t sample={{3214U,189U,213U},7U,20U};
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    CHECK(robot_ir_apply_adc_calibration(&cfg));
    CHECK(s3_core_init(&core,&cfg));
    CHECK(s3_core_control(&core,S3_MODE_TRACK,false));
    s3_core_step(&core,&sample,&output);
    CHECK(output.steering_allowed);
    sample.time_ms+=40U;s3_core_step(&core,&sample,&output);
    CHECK(!output.steering_allowed && output.error==S3_ERROR_STALE && output.event.kind==S3_EVENT_FAULT);
    sample.time_ms+=20U;s3_core_step(&core,&sample,&output);
    CHECK(!output.steering_allowed && output.error==S3_ERROR_STALE);
    CHECK(s3_core_control(&core,S3_MODE_PAUSED,false));
    sample.time_ms+=1000U;s3_core_step(&core,&sample,&output);
    CHECK(!output.steering_allowed && output.error==S3_ERROR_STALE && core.awaiting_controller);
    CHECK(s3_core_control(&core,S3_MODE_TRACK,false));
    sample.time_ms+=20U;s3_core_step(&core,&sample,&output);
    CHECK(output.steering_allowed && output.error==S3_ERROR_NONE);
    /* Duplicate/backwards time is rejected even with drive inactive. */
    CHECK(s3_core_control(&core,S3_MODE_PAUSED,false));
    s3_core_step(&core,&sample,&output);
    CHECK(!output.steering_allowed && output.error==S3_ERROR_STALE);
    CHECK(s3_core_init(&core,&cfg));
    sample.time_ms=100U;s3_core_step(&core,&sample,&output);
    sample.time_ms=90U;s3_core_step(&core,&sample,&output);
    CHECK(output.error==S3_ERROR_STALE && !output.steering_allowed);
    CHECK(s3_core_init(&core,&cfg));
    sample.time_ms=20U;sample.valid_mask=0U;s3_core_step(&core,&sample,&output);
    sample.time_ms+=1000U;sample.valid_mask=7U;s3_core_step(&core,&sample,&output);
    CHECK(output.error==S3_ERROR_SENSOR && !output.steering_allowed);
}

int main(void)
{
    test_paused_forward_gaps();
    test_active_fault_stays_latched();
    printf("S3-U44 paused timing: %u checks, %u failures\n",checks,failures);
    return failures?1:0;
}
