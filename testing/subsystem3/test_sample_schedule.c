#include "adapter_fixture.h"

static void setup_sampling(void)
{
    s3_adapter_config_t cfg={0};
    reset();CHECK(subsystem3_init()==E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.ir=(s3_ir_config_t){S3_IR_ADC,{26U,27U,28U}};
    CHECK(s3_configure(&cfg)==E_OK && subsystem3_start()==E_OK);
    CHECK(s3_controller_bind()==E_OK);
    caller=42;adc_auto_worker=true;
}
static void test_cyclic_release(void)
{
    setup_sampling();
    stop_after_sleeps=6U;
    stop_after_delays=6U;model_delay_guard=true;
    consumer_task(0,NULL);
    CHECK(task_exits==1U && sleeps==6U && adc_reads==18U);
    CHECK(fake_now==60U && status.sample_time_ms==60U);
    CHECK(core.mode==S3_MODE_PAUSED && !status.output.steering_allowed);
    CHECK(wake_credits==0U);
    /* Callback never takes a state mutex, formats text or enters ADC. */
    busy_mutex=true;
    sample_cyclic(NULL);
    CHECK(wake_credits==1U && adc_reads==18U);
    busy_mutex=false;
    /* Multiple old releases are drained, then wait for a future one. */
    fake_now=105U;CHECK(s3_sample_wait()==E_OK);
    CHECK(fake_now==110U && wake_credits==0U);
    reset();CHECK(s3_sample_wait()==E_OACV);
}
static void test_cycle_failure_cleanup(void)
{
    s3_adapter_config_t cfg={0};
    reset();CHECK(subsystem3_init()==E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.ir=(s3_ir_config_t){S3_IR_ADC,{26U,27U,28U}};
    CHECK(s3_configure(&cfg)==E_OK);
    fail_cycle=true;CHECK(subsystem3_start()==E_SYS);
    CHECK(!started && h_task==0 && h_sample_cycle==0 && !fake_cycle_active && deletes==1U);
    fail_cycle=false;fail_start=true;CHECK(subsystem3_start()==E_SYS);
    CHECK(!started && h_task==0 && h_sample_cycle==0 && !fake_cycle_active && deletes==3U);
    fail_start=false;CHECK(subsystem3_start()==E_OK && fake_cycle_active);
}
static void stop_while_lock_waits(void)
{
    ID old=caller;caller=7;
    CHECK(s3_controller_control(S3_MODE_OVERRIDE,false,0U)==E_OK);
    caller=old;
}
static void clock_fails_while_lock_waits(void) { fail_time=true; }
static void test_publish_wait_safety(void)
{
    for(unsigned trial=0;trial<4;++trial) {
        setup_sampling();caller=7;
        CHECK(s3_controller_control(S3_MODE_TRACK,false,trial==0U?15U:1000U)==E_OK);
        caller=42;delay_lock_call=3U;state_lock_delay_ms=20U;
        if(trial==1U) state_lock_delay_ms=30U;
        if(trial==2U) state_lock_hook=stop_while_lock_waits;
        if(trial==3U) state_lock_hook=clock_fails_while_lock_waits;
        (void)s3_service_once();
        CHECK(!status.output.steering_allowed && queue_count==0U);
        if(trial==0U) CHECK(status.output.error==S3_ERROR_STALE);
        if(trial==1U) CHECK(status.acquisition_error==E_TMOUT && status.sample.valid_mask==0U);
        if(trial==2U) CHECK(status.generation==control.generation);
        if(trial==3U) CHECK(status.acquisition_error==E_SYS && status.sample.valid_mask==0U);
    }
}
int main(void)
{
    RUN("S3-U48 cyclic/no burst/no ADC callback",test_cyclic_release);
    RUN("S3-U49 cyclic lifecycle cleanup",test_cycle_failure_cleanup);
    RUN("S3-U50 publication wait freshness/STOP/lease/clock",test_publish_wait_safety);
    printf("Buddy3 sample schedule: %u groups, %u checks, %u failures\n",groups,checks,failures);
    return failures?1:0;
}
