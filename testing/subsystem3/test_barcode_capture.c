/* Production sampler/capture queue: copied data and explicit overflow. */
#include "adapter_fixture.h"
#include "../../app_program/robot_ir_config.h"

static void setup_capture(void)
{
    s3_adapter_config_t cfg={0};
    reset();
    CHECK(subsystem3_init()==E_OK);
    s3_config_defaults(&cfg.algorithms);robot_ir_apply_layout(&cfg.algorithms);
    CHECK(robot_ir_apply_adc_calibration(&cfg.algorithms));
    cfg.ir=(s3_ir_config_t){S3_IR_ADC,{26U,27U,28U}};
    CHECK(s3_configure(&cfg)==E_OK && subsystem3_start()==E_OK);
    adc_auto_worker=true;caller=42;
}
static void test_copy_overflow(void)
{
    s3_sample_t sample;
    uint32_t dropped;
    setup_capture();
    CHECK(s3_capture_receive(&sample,&dropped)==E_TMOUT && sample.valid_mask==0U);
    for(unsigned i=0;i<10;++i) {
        adc_values[0]=200U+i;adc_values[1]=300U+i;adc_values[2]=400U+i;
        CHECK(s3_service_once()==E_OK);
        CHECK(status.configured && !status.output.steering_allowed && core.mode==S3_MODE_PAUSED);
        fake_now+=20U;
    }
    CHECK(capture_count==8U && capture_dropped==2U && queue_count==0U);
    adc_values[0]=4095U; /* queued values must have independent ownership. */
    for(unsigned i=0;i<8;++i) {
        CHECK(s3_capture_receive(&sample,&dropped)==E_OK);
        CHECK(dropped==0U && sample.valid_mask==7U && sample.time_ms==i*20U);
        CHECK(sample.raw[0]==200U+i && sample.raw[1]==300U+i && sample.raw[2]==400U+i);
    }
    CHECK(s3_capture_receive(&sample,&dropped)==E_TMOUT && sample.valid_mask==0U);
    CHECK(s3_capture_receive(NULL,&dropped)==E_PAR);
    CHECK(s3_capture_receive(&sample,NULL)==E_PAR);
    busy_mutex=true;
    CHECK(s3_capture_receive(&sample,&dropped)==E_TMOUT && sample.valid_mask==0U);
    busy_mutex=false;
    CHECK(s3_service_once()==E_OK && capture_count==1U);
    busy_mutex=true;
    CHECK(s3_capture_receive(&sample,&dropped)==E_OK && dropped==2U && sample.valid_mask==7U);
    busy_mutex=false;fake_now+=20U;
    CHECK(s3_service_once()==E_OK && capture_count==1U);
    malformed_capture=true;
    CHECK(s3_capture_receive(&sample,&dropped)==E_IO && sample.valid_mask==0U);
    CHECK(!status.output.steering_allowed && queue_count==0U);
}
static void test_invalid_acquisition_and_init_failure(void)
{
    s3_sample_t sample;
    uint32_t dropped;
    setup_capture();
    adc_read_result=E_IO;
    CHECK(s3_service_once()==E_IO);
    CHECK(s3_capture_receive(&sample,&dropped)==E_OK && sample.valid_mask==0U);
    CHECK(!status.output.steering_allowed && queue_count==0U);
    reset();fail_create=4U;
    CHECK(subsystem3_init()==E_SYS);
    CHECK(h_capture==0 && h_events==0 && mtx_stats==0 && mbx_samples==0 && deletes==3U);
    CHECK(s3_capture_receive(&sample,&dropped)==E_OBJ);
}
int main(void)
{
    RUN("S3-U46 capture copy/overflow/no authority",test_copy_overflow);
    RUN("S3-U47 capture invalid/init cleanup",test_invalid_acquisition_and_init_failure);
    printf("Buddy 3 barcode capture: %u groups, %u checks, %u failures\n",groups,checks,failures);
    return failures?1:0;
}
