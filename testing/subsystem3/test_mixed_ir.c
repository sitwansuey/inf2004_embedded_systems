/* Production ADC worker + GPIO reader + core. Deterministic RTOS/device fakes. */
#include "adapter_fixture.h"

static const uint32_t frame_a[29]={
    1,3,1,1,3,1,3,1,1,1,3,1,1,1,1,3,1,1,3,1,1,3,1,1,3,1,3,1,1
};
static const uint32_t frame_z[29]={
    1,3,1,1,3,1,3,1,1,1,1,3,3,1,3,1,1,1,1,1,1,3,1,1,3,1,3,1,1
};
static void setup(unsigned black_level)
{
    s3_adapter_config_t cfg={0};reset();
    CHECK(subsystem3_init()==E_OK);
    s3_config_defaults(&cfg.algorithms);
    cfg.ir=(s3_ir_config_t){S3_IR_ADC_LINE_DIGITAL_BARCODE,{26U,27U,28U}};
    cfg.algorithms.line.line_mask=3U;cfg.algorithms.line.center_white=true;
    cfg.algorithms.line.position[0]=1000;cfg.algorithms.line.position[1]=-1000;
    cfg.algorithms.line.position[2]=0;
    cfg.algorithms.line.white[0]=187U;cfg.algorithms.line.black[0]=3214U;
    cfg.algorithms.line.white[1]=189U;cfg.algorithms.line.black[1]=3177U;
    cfg.algorithms.line.white[2]=(uint16_t)(1U-black_level);
    cfg.algorithms.line.black[2]=(uint16_t)black_level;
    cfg.algorithms.barcode_sensor=2U;cfg.algorithms.barcode.min_run_ms=0U;
    CHECK(s3_configure(&cfg)==E_OK);CHECK(subsystem3_start()==E_OK);
    CHECK(s3_controller_bind()==E_OK);
    CHECK(s3_controller_control(S3_MODE_TRACK,true,1000U)==E_OK);
    caller=42;adc_auto_worker=true;
    adc_values[0]=187U;adc_values[1]=189U;
    adc_values[2]=65535U; /* Must never be requested/read in mixed mode. */
    gpio[28]=1U-black_level;
}
static void tick(unsigned level)
{
    fake_now+=10U;gpio[28]=level;
    CHECK(s3_service_once()==E_OK);
    CHECK(status.sample.valid_mask==7U && status.acquisition_error==E_OK);
    CHECK(status.sample.raw[2]==level && status.output.error==S3_ERROR_NONE);
}
static void test_inputs_and_timestamps(void)
{
    setup(1U);tick(0U);
    CHECK(gpio_configs==1U && gpio_reads==1U && adc_reads==2U);
    CHECK(adc_request.channel[0]==0U && adc_request.channel[1]==1U &&
          adc_request.channel[2]==S3_ADC_CHANNEL_UNUSED);
    CHECK(adc_reply.sample.valid_mask==3U && status.sample.valid_mask==7U);
    CHECK(status.output.barcode_white && status.output.line.black_mask==0U);
    CHECK(status.sample.time_ms==10U);
    adc_read_ms=1U;tick(1U);
    CHECK(status.sample.time_ms==20U && fake_now==22U);
    CHECK(gpio_reads==2U && adc_reads==4U && !status.output.barcode_white);
    /* GP26/27 still use their analog calibration and steering masks. */
    adc_read_ms=0U;adc_values[0]=3214U;tick(0U);
    CHECK(status.output.line.black_mask==1U && status.output.line.error==1000);
}
static void test_digital_and_adc_failures(void)
{
    setup(1U);gpio_config_error=E_IO;
    CHECK(s3_service_once()==E_IO && adc_reads==0U && status.sample.valid_mask==0U);
    setup(1U);gpio[28]=2U;
    CHECK(s3_service_once()==E_IO && status.sample.valid_mask==0U);
    CHECK(!status.output.steering_allowed && status.output.error==S3_ERROR_SENSOR);
    setup(1U);adc_read_result=E_IO;
    CHECK(s3_service_once()==E_IO && gpio_reads==0U && status.sample.valid_mask==0U);
    setup(1U);adc_read_ms=15U;
    CHECK(s3_service_once()==E_TMOUT && gpio_reads==0U && status.sample.valid_mask==0U);
    CHECK(worker_failed);
}
static void test_request_and_pin_ownership(void)
{
    s3_ir_config_t cfg={S3_IR_ADC_LINE_DIGITAL_BARCODE,{26U,27U,28U}};
    CHECK(s3_ir_config_valid(&cfg));cfg.pin[2]=29U;CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[2]=27U;CHECK(!s3_ir_config_valid(&cfg));
    cfg.pin[0]=27U;cfg.pin[1]=26U;cfg.pin[2]=28U;CHECK(!s3_ir_config_valid(&cfg));
    CHECK(DEVCONF_ADC_PIN_INIT_0 && DEVCONF_ADC_PIN_INIT_1 && !DEVCONF_ADC_PIN_INIT_2);
    for(unsigned trial=0U;trial<3U;++trial) {
        setup(1U);adc_auto_worker=false;
        CHECK(s3_service_once()==E_TMOUT && s3_ir_waiting(&ir));
        if(trial==0U)adc_request.channel[2]=2U; /* GP28 analog forbidden. */
        if(trial==1U)adc_request.channel[1]=0U; /* Duplicate channel. */
        if(trial==2U)adc_request.channel[0]=3U; /* Radio ADC forbidden. */
        caller=43;CHECK(s3_adc_worker_once(TMO_POL)==E_PAR);
        CHECK(adc_reads==0U && adc_reply.sample.valid_mask==0U);
    }
}
static void test_complete_frames(void)
{
    for(unsigned polarity=0U;polarity<2U;++polarity) {
        for(unsigned letter=0U;letter<2U;++letter) {
            for(unsigned reverse=0U;reverse<2U;++reverse) {
                const uint32_t *width=letter?frame_z:frame_a;
                setup(polarity);
                for(unsigned n=0U;n<22U;++n)tick(1U-polarity);
                tick(polarity);
                for(unsigned i=0U;i<29U;++i) {
                    unsigned steps=width[reverse?28U-i:i];
                    unsigned level=(i&1U)?1U-polarity:polarity;
                    for(unsigned n=1U;n<steps;++n)tick(level);
                    tick(1U-level);
                }
                for(unsigned n=0U;n<21U;++n)tick(1U-polarity);
                caller=7;s3_event_record_t event={0};
                CHECK(s3_event_receive(&event)==E_OK && event.event.kind==S3_EVENT_BARCODE);
                CHECK(event.event.symbol==(letter?'Z':'A'));
                CHECK(s3_event_receive(&event)==E_TMOUT);
                CHECK(gpio_configs==1U && gpio_reads*2U==adc_reads);
            }
        }
    }
}
int main(void)
{
    RUN("S3 mixed analog line / digital barcode and oldest timestamp",test_inputs_and_timestamps);
    RUN("S3 mixed DO/ADC failure isolation",test_digital_and_adc_failures);
    RUN("S3 mixed GPIO/ADC pin and request ownership",test_request_and_pin_ownership);
    RUN("S3 mixed complete A/Z both directions, both polarities",test_complete_frames);
    printf("S3 mixed input: %u groups, %u checks, %u failures\n",groups,checks,failures);
    return failures?1:0;
}
