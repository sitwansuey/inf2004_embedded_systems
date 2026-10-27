/* Real S3 adapter/ADC worker and production decoder. No motor or board I/O. */
#include "adapter_fixture.h"
#include "../../app_program/robot_ir_config.h"
static const unsigned sample_a[29]={1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1};
static const unsigned sample_z[29]={1,3,1,1,3,1,3,1,1,1,1,3,3,1,3,1,1,1,1,1,1,3,1,1,3,1,3,1,1};
static uint32_t observed_generation;
static void setup_bench(void) {
    s3_adapter_config_t cfg={0};reset();
    CHECK(subsystem3_init()==E_OK);
    s3_config_defaults(&cfg.algorithms);robot_ir_apply_layout(&cfg.algorithms);
    CHECK(robot_ir_apply_adc_calibration(&cfg.algorithms));
    cfg.ir=(s3_ir_config_t){S3_IR_ADC,{26U,27U,28U}};
    CHECK(s3_configure(&cfg)==E_OK && subsystem3_start()==E_OK);
    CHECK(s3_controller_bind()==E_OK);
    CHECK(s3_barcode_bench_control(true,&observed_generation)==E_OK);
    caller=42;adc_auto_worker=true;
}
static void feed(bool black,unsigned ticks) {
    for(unsigned i=0U;i<ticks;++i) {
        adc_values[0]=0U;adc_values[1]=0U; /* line modules unavailable */
        adc_values[2]=black?2472U:197U;
        CHECK(s3_service_once()==E_OK);
        CHECK(!status.output.steering_allowed && queue_count==0U);
        fake_now+=10U;
    }
}
static void test_every_sample(void) {
    for(unsigned z=0U;z<2U;++z) for(unsigned reverse=0U;reverse<2U;++reverse) {
        const unsigned *width=z?sample_z:sample_a;setup_bench();
        feed(false,30U);
        for(unsigned i=0U;i<29U;++i) {
            feed((i&1U)==0U,width[reverse?28U-i:i]*4U);
            /* Controller polling is deliberately sparse; all edges are
             * interpreted in S3 even when latest snapshots skip elements. */
            if(i%5U==0U){s3_status_t copy={0};CHECK(s3_status_get(&copy)==E_OK);}
        }
        feed(false,50U);
        s3_status_t copy={0};CHECK(s3_status_get(&copy)==E_OK);
        CHECK(copy.bench.error==E_OK && copy.bench.generation==observed_generation);
        CHECK(copy.bench.result==S3_BARCODE_OK && copy.bench.symbol.symbol==(z?'Z':'A'));
        CHECK(copy.bench.symbol.reverse==(reverse!=0U));
        CHECK(!bench_observe && core.mode==S3_MODE_PAUSED && queue_count==0U);
    }
}
static void test_short_biased_frame(void) {
    /* Observed pulse widths with synthetic endpoint ADC levels. Original raw
     * ADC capture is replayed independently; this checks sampler integration. */
    const unsigned widths[29]={30,140,40,70,130,60,140,70,30,70,120,60,30,60,40,160,40,60,140,80,30,190,30,100,180,110,220,100,40};
    setup_bench();feed(false,30U);
    for(unsigned i=0U;i<29U;++i)feed((i&1U)==0U,widths[i]/10U);
    feed(false,100U);
    s3_status_t copy={0};CHECK(s3_status_get(&copy)==E_OK);
    CHECK(copy.bench.result==S3_BARCODE_OK && copy.bench.symbol.symbol=='A');
    CHECK(!copy.output.steering_allowed && queue_count==0U && core.mode==S3_MODE_PAUSED);
}
static void test_faults_and_authority(void) {
    setup_bench();feed(false,30U);feed(true,1U);feed(false,1U);
    CHECK(status.bench.result==S3_BARCODE_NOISE && status.bench.symbol.symbol==0);
    setup_bench();feed(false,3U);fake_now+=40U;feed(false,1U);
    CHECK(status.bench.error==E_TMOUT && !bench_observe);
    setup_bench();feed(false,3U);adc_read_result=E_IO;
    CHECK(s3_service_once()==E_IO && status.bench.error==E_IO && !bench_observe);
    setup_bench();feed(false,3U);caller=8;
    CHECK(s3_barcode_bench_control(false,NULL)==E_OACV && bench_observe);
    caller=7;busy_mutex=true;CHECK(s3_barcode_bench_control(false,NULL)==E_TMOUT);
    busy_mutex=false;CHECK(s3_barcode_bench_control(false,NULL)==E_OK);
    CHECK(status.bench.generation!=observed_generation && !bench_observe);
    caller=42;feed(true,10U);CHECK(status.bench.samples==0U && status.bench.result==S3_BARCODE_NONE);
    caller=7;CHECK(s3_barcode_bench_control(true,&observed_generation)==E_OK);
    CHECK(status.bench.result==S3_BARCODE_NONE && status.bench.samples==0U);
    caller=42;feed(false,30U);CHECK(status.bench.error==E_OK);
}
static void fail_clock_in_handoff(void) {fail_time=true;}
static void test_bounded_status_handoff(void) {
    s3_status_t copy={0};setup_bench();feed(false,3U);
    caller=7;CHECK(s3_barcode_bench_status_get(&copy)==E_OK);
    CHECK(copy.bench.samples==3U && copy.sample_time_ms==20U && !locked);
    caller=8;CHECK(s3_barcode_bench_status_get(&copy)==E_OACV && copy.sample.valid_mask==0U && !locked);
    caller=7;busy_mutex=true;
    CHECK(s3_barcode_bench_status_get(&copy)==E_TMOUT && copy.sample.valid_mask==0U);
    busy_mutex=false;delay_lock_call=sample_lock_calls+1U;state_lock_delay_ms=20U;
    CHECK(s3_barcode_bench_status_get(&copy)==E_OK);
    CHECK(fake_now-copy.sample_time_ms>=S3_STATUS_MAX_AGE_MS && !copy.output.steering_allowed);
    delay_lock_call=sample_lock_calls+1U;state_lock_delay_ms=0U;state_lock_hook=fail_clock_in_handoff;
    CHECK(s3_barcode_bench_status_get(&copy)==E_SYS && copy.sample.valid_mask==0U && !locked);
    fail_time=false;CHECK(s3_barcode_bench_status_get(NULL)==E_PAR);
}
int main(void) {
    RUN("S3 motor bench every acquisition A/Z both directions, no line authority",test_every_sample);
    RUN("S3 motor bench short biased frame",test_short_biased_frame);
    RUN("S3 motor bench noise/gaps/ADC faults, ownership and cancellation",test_faults_and_authority);
    RUN("S3 motor bench bounded status handoff/clock/owner",test_bounded_status_handoff);
    printf("S3 motor scan sampler: %u groups, %u checks, %u failures\n",groups,checks,failures);
    return failures?1:0;
}
