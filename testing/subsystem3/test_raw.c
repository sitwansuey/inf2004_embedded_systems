/* Raw bring-up must never masquerade as calibrated line/barcode control. */
#include "adapter_fixture.h"
int main(void)
{
    s3_ir_config_t cfg={S3_IR_DIGITAL,{26U,27U,28U}};
    reset();
#if !BOARD_MOTORS_DISABLED
    CHECK(s3_configure_raw(&cfg)==E_OACV);
    CHECK(gpio_reads==0U && gpio_configs==0U && adc_reads==0U);
#else
    s3_status_t copy;
    CHECK(subsystem3_init()==E_OK);
    CHECK(s3_configure_raw(NULL)==E_PAR);
    cfg.mode=S3_IR_DISABLED;CHECK(s3_configure_raw(&cfg)==E_PAR);
    cfg.mode=S3_IR_DIGITAL;cfg.pin[2]=29U;
    CHECK(s3_configure_raw(&cfg)==E_PAR);
    cfg.pin[2]=28U;
    CHECK(s3_configure_raw(&cfg)==E_OK);
    CHECK(s3_controller_bind()==E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK,true,100U)==E_OACV);
    CHECK(subsystem3_start()==E_OK);
    CHECK(s3_configure_raw(&cfg)==E_OBJ);
    caller=42;gpio[26]=1;gpio[27]=0;gpio[28]=1;fake_now=10;
    CHECK(s3_service_once()==E_OK);
    CHECK(s3_status_get(&copy)==E_OK && copy.sample.valid_mask==7U);
    CHECK(copy.sample.raw[0]==1U && copy.sample.raw[1]==0U && copy.sample.raw[2]==1U);
    CHECK(!copy.configured && !copy.output.steering_allowed && queue_count==0U);
    CHECK(copy.output.error==S3_ERROR_CONFIG && core.have_sample==false);
    reset();CHECK(subsystem3_init()==E_OK);cfg.mode=S3_IR_ADC;
#if S3_TEST_ADC_ENABLED
    CHECK(s3_configure_raw(&cfg)==E_OK && subsystem3_start()==E_OK);
    caller=42;fake_now=10;CHECK(s3_service_once()==E_TMOUT);
    caller=43;CHECK(s3_adc_worker_once(TMO_POL)==E_OK);
    caller=42;fake_now=30;CHECK(s3_service_once()==E_OK);
    CHECK(s3_status_get(&copy)==E_OK && copy.sample.valid_mask==7U);
    CHECK(copy.sample.raw[0]==0U && copy.sample.raw[1]==4095U);
    CHECK(!copy.configured && !copy.output.steering_allowed && queue_count==0U);
    CHECK(gpio_configs==0U && adc_reads==3U && !core.have_sample);
    fake_now=60;CHECK(s3_service_once()==E_TMOUT);
    CHECK(s3_status_get(&copy)==E_OK && copy.sample.valid_mask==0U && !copy.output.steering_allowed);
#else
    CHECK(s3_configure_raw(&cfg)==E_NOSPT && adc_reads==0U);
#endif
#endif
    ++groups;
    (void)printf("S3-U33 raw bring-up: %u groups, %u checks, %u failures\n",groups,checks,failures);
    return failures==0U?0:1;
}
