/* Real mixed acquisition adapter with deterministic RTOS/GPIO boundaries. */
#define BOARD_IR_ADC_CALIBRATED 1
#define BOARD_IR_ADC_WHITE {187U,189U,197U}
#define BOARD_IR_ADC_BLACK {3214U,3177U,2472U}
#include "adapter_fixture.h"
#include "../../app_program/robot_ir_config.h"
static void mixed(void) {
 s3_adapter_config_t cfg={0};reset();CHECK(subsystem3_init()==E_OK);s3_config_defaults(&cfg.algorithms);
 robot_ir_apply_layout(&cfg.algorithms);CHECK(robot_ir_apply_adc_calibration(&cfg.algorithms));
 CHECK(cfg.algorithms.line.white[0]==187U && cfg.algorithms.line.black[1]==3177U);
 CHECK(cfg.algorithms.line.white[2]==0U && cfg.algorithms.line.black[2]==1U && cfg.algorithms.barcode_retry);
 cfg.ir=(s3_ir_config_t){S3_IR_ADC_LINE_DIGITAL_BARCODE,{26U,27U,28U}};
 CHECK(s3_configure(&cfg)==E_OK);CHECK(subsystem3_start()==E_OK);CHECK(s3_controller_bind()==E_OK);
 CHECK(s3_controller_control(S3_MODE_TRACK,false,100U)==E_OK);caller=42;adc_auto_worker=true;adc_read_ms=1U;
 adc_values[0]=3200U;adc_values[1]=180U;adc_values[2]=2222U;gpio[28]=1U;
 CHECK(s3_service_once()==E_OK);CHECK(gpio_configs==1U && gpio_reads==1U && adc_reads==2U);
 CHECK(adc_request.channel[0]==0U && adc_request.channel[1]==1U && adc_request.channel[2]==S3_ADC_CHANNEL_UNUSED);
 CHECK(status.sample.valid_mask==7U && status.sample.raw[0]==3200U && status.sample.raw[1]==180U && status.sample.raw[2]==1U);
 CHECK(core.barcode_black && status.output.line.black_mask==1U);
 fake_now+=10U;gpio[28]=0U;CHECK(s3_service_once()==E_OK);
 CHECK(adc_reads==4U && gpio_reads==2U && status.sample.raw[2]==0U && !core.barcode_black);
 gpio[28]=2U;fake_now+=10U;CHECK(s3_service_once()==E_IO);CHECK(!status.output.steering_allowed);
 cfg.ir.pin[2]=22U;CHECK(!s3_ir_config_valid(&cfg.ir));
 CHECK(DEVCONF_ADC_PIN_INIT_2==0);
}
int main(void){RUN("ADC line + digital GP28 barcode",mixed);printf("%u checks, %u failures\n",checks,failures);return failures?1:0;}
