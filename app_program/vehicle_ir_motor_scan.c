#include "robot_config.h"
#if BOARD_IR_MOTOR_SCAN_BENCH && !BOARD_IR_LINE_TRACE_BENCH
#include "app_state.h"
#include "ir_motor_scan.h"
#include "robot_ir_config.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"
#if S2_COMMAND_TIMEOUT_MS > 150 || S2_COMMAND_TIMEOUT_MS == 0
#error Motor scan requires the independent motor watchdog at most150ms
#endif
static void motor_scan_task(INT code,void *context) {
    ir_scan_state_t scan={0};app_state_t state={0};s3_config_t cfg;
    bool observe=false,observer_enabled=false;
    uint32_t bench_generation=0U,previous_log=0U,stopped_ms=0U;
    unsigned previous_active=99U,previous_prepared=99U,previous_reason=99U;
    (void)code;(void)context;
    s2_controller_bind();
    if(s3_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    if(!robot_ir_apply_adc_calibration(&cfg)) {s2_emergency_stop();tk_ext_tsk();return;}
    (void)s3_controller_control(S3_MODE_PAUSED,false,0U);
    (void)s3_barcode_bench_control(false,NULL);
    s2_emergency_stop();
    for(;;) {
        s2_motion_status_t motor={0};s3_status_t ir={0};s5_status_t sonar={0};SYSTIM now;
        bool motor_ok=s2_motion_get_status(&motor)==E_OK && motor.state!=S2_MOTION_FAULT;
        bool ir_ok=s3_barcode_bench_status_get(&ir)==E_OK;
        bool sonar_ok=s5_get_status(&sonar)==E_OK;
        uint16_t darkness=0U;
        if(tk_get_otm(&now)<E_OK) {
            ir_scan_stop(&scan,IR_SCAN_CLOCK);observe=false;s2_emergency_stop();tk_dly_tsk(10);continue;
        }
        ir_ok=ir_ok && ir.configured && ir.acquisition_error==E_OK && ir.sample.valid_mask==7U &&
              now.lo-ir.sample_time_ms<30U &&
              s3_normalize(ir.sample.raw[2],cfg.line.white[2],cfg.line.black[2],4095U,&darkness);
        ir_scan_input_t input={now.lo,motor_ok,ir_ok,ir_ok && darkness<=cfg.line.threshold_low,
            sonar_ok && sonar.valid && sonar.angle_deg==0 && sonar.sample_angle_deg==0 &&
            now.lo-sonar.time_ms<S5_STALE_MS && sonar.distance_mm>250U};
        bool was_active=scan.active;
        ir_scan_step(&scan,&input);
        if(was_active && !scan.active) stopped_ms=now.lo;
        for(unsigned n=0U;n<4U;++n) {
            control_message_t command;ER er=s1_command_take(&command);
            if(er==E_TMOUT) break;
            if(er<E_OK) {ir_scan_stop(&scan,IR_SCAN_BAD_COMMAND);break;}
            if(command.wire[0]!=0xD3U) {
                ir_scan_stop(&scan,IR_SCAN_REQUESTED_STOP);observe=false;break;
            }
            bool started=ir_scan_command(&scan,&input,command.wire,command.time_ms);
            if(command.wire[1]==IR_SCAN_QUERY) {
                tm_printf((UB *)"[IRSCAN READY] v=5 active=%u prepared=%u cap=20 lease=250 max_ms=6000 adc=%u motor=%u front=%u\n",
                    scan.active,scan.prepared,(unsigned)ir_ok,(unsigned)motor_ok,(unsigned)input.front_clear);
            }
            if(command.wire[1]==IR_SCAN_PREPARE && scan.prepared) {
                if(s3_barcode_bench_control(true,&bench_generation)==E_OK) {
                    observe=true;observer_enabled=true;
                } else {ir_scan_stop(&scan,IR_SCAN_ADC_LOST);observe=false;}
            }
            if(started) {
                if(!scan.scan) observe=false;
                tm_printf((UB *)"[IRSCAN START] mode=%u power=%u duration=%u\n",
                    (unsigned)scan.scan+1U,scan.power,scan.duration_ms);
            }
        }
        if(scan.active && scan.scan && !input.front_clear) ir_scan_stop(&scan,IR_SCAN_OBSTACLE);
        if(observe && !scan.active && !scan.prepared &&
           (scan.reason!=IR_SCAN_COMPLETE || now.lo-stopped_ms>500U)) observe=false;
        if(observe && ir_ok && ir.bench.generation==bench_generation) {
            if(ir.bench.error<E_OK) {
                ir_scan_stop(&scan,IR_SCAN_ADC_LOST);observe=false;
            } else if(ir.bench.result==S3_BARCODE_OK && now.lo-ir.bench.time_ms<100U) {
                state.barcode=ir.bench.symbol.symbol=='A'?1U:ir.bench.symbol.symbol=='Z'?2U:0U;
                tm_printf((UB *)"[IRSCAN DECODE] symbol=%c reverse=%u t=%u; STOP, no navigation\n",
                    ir.bench.symbol.symbol,(unsigned)ir.bench.symbol.reverse,ir.bench.time_ms);
                ir_scan_stop(&scan,IR_SCAN_DECODED);observe=false;
            } else if(ir.bench.result!=S3_BARCODE_NONE) {
                tm_printf((UB *)"[IRSCAN REJECT] status=%u t=%u\n",(unsigned)ir.bench.result,ir.bench.time_ms);
                ir_scan_stop(&scan,IR_SCAN_BAD_PATTERN);observe=false;
            }
        }
        if(!observe && observer_enabled) {
            (void)s3_barcode_bench_control(false,NULL);observer_enabled=false;
        }
        if(scan.active) {
            ER er=s2_movement_send(S2_MOVE_FORWARD,scan.power);
            if(er<E_OK && er!=E_OBJ) {ir_scan_stop(&scan,IR_SCAN_MOTOR_FAULT);s2_emergency_stop();}
        } else s2_emergency_stop();
        if(previous_active && !scan.active) stopped_ms=now.lo;
        if(now.lo-previous_log>=100U || previous_active!=scan.active || previous_prepared!=scan.prepared || previous_reason!=scan.reason) {
            s2_encoder_status_t enc={0};(void)s2_encoder_get_status(&enc);
            tm_printf((UB *)"[IRSCAN] t=%u active=%u prepared=%u mode=%u reason=%u power=%u raw=%u darkness=%u adc=%u front=%u motor_state=%u command=%u motor_power=%u m1=%d m2=%d\n",
                now.lo,scan.active,scan.prepared,(unsigned)scan.scan+1U,scan.reason,scan.power,
                (unsigned)ir.sample.raw[2],(unsigned)darkness,(unsigned)ir_ok,(unsigned)input.front_clear,
                (unsigned)motor.state,(unsigned)motor.command,(unsigned)motor.power_percent,
                enc.motor1_forward_counts,enc.motor2_forward_counts);
            previous_log=now.lo;previous_active=scan.active;previous_prepared=scan.prepared;previous_reason=scan.reason;
        }
        state.time_ms=now.lo;++state.sequence;
        state.vehicle.armed=scan.active;state.vehicle.phase=scan.active?VC_TRACK:VC_DISARMED;
        state.vehicle.command=scan.active?S2_MOVE_FORWARD:S2_MOVE_STOP;state.vehicle.power=scan.power;
        (void)app_state_publish(&state);
        tk_dly_tsk(10);
    }
}
ER vehicle_start(void) {
    ID tid=app_start_task((FP)motor_scan_task,5,TP_PRC1,"IR motor scan");return tid<0?tid:E_OK;
}
#endif
