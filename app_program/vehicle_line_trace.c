#include "robot_config.h"
#if BOARD_IR_LINE_TRACE_BENCH
#include "app_state.h"
#include "ir_line_trace.h"
#if BOARD_IR_LINE_ONLY_BENCH
#include "ir_line_progress.h"
#include "ir_line_course.h"
#endif
#include "robot_ir_config.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "subsystem5/subsystem5.h"
_Static_assert((int)S2_MOVE_FORWARD==IR_LINE_FORWARD &&
               (int)S2_MOVE_PIVOT_LEFT==IR_LINE_LEFT &&
               (int)S2_MOVE_PIVOT_RIGHT==IR_LINE_RIGHT,"S2 direction mapping changed");

static void line_trace_task(INT code,void *context) {
    ir_scan_state_t scan={0};ir_line_trace_t line={0};app_state_t state={0};s3_config_t cfg;
#if BOARD_IR_LINE_ONLY_BENCH
#if !BOARD_IR_CONTINUOUS_BENCH
    ir_line_progress_t progress={0};ir_line_course_t course={0};
    unsigned previous_course=99U;
#endif
#endif
    uint32_t previous_log=0U;
    unsigned previous_active=99U,previous_prepared=99U,previous_reason=99U,previous_mask=99U;
    (void)code;(void)context;
    s2_controller_bind();
    if(s3_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    s3_config_defaults(&cfg);robot_ir_apply_layout(&cfg);
    if(!robot_ir_apply_adc_calibration(&cfg)) {s2_emergency_stop();tk_ext_tsk();return;}
    if(s3_controller_control(S3_MODE_PAUSED,false,0U)<E_OK ||
       s3_barcode_bench_control(false,NULL)<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    s2_emergency_stop();
    for(;;) {
        s2_motion_status_t motor={0};s3_status_t ir={0};s5_status_t sonar={0};SYSTIM now;
        bool motor_ok=s2_motion_get_status(&motor)==E_OK && motor.state!=S2_MOTION_FAULT;
        bool source_ok=s3_barcode_bench_status_get(&ir)==E_OK && ir.configured && ir.acquisition_error==E_OK;
        bool sonar_ok=s5_get_status(&sonar)==E_OK;
#if BOARD_IR_LINE_ONLY_BENCH
        s2_encoder_status_t enc={0};
#if BOARD_IR_CONTINUOUS_BENCH
        (void)s2_encoder_get_status(&enc);
#else
        bool encoder_ok=s2_encoder_get_status(&enc)==E_OK && enc.valid;
#endif
#endif
        if(tk_get_otm(&now)<E_OK) {
            ir_scan_stop(&scan,IR_SCAN_CLOCK);line=(ir_line_trace_t){0};
            s2_emergency_stop();tk_dly_tsk(10);continue;
        }
        bool adc_ok=ir_line_observe(&line,&cfg.line,ir.sample.raw,ir.sample.valid_mask,
                                   ir.sample_time_ms,now.lo,source_ok);
        bool actual_front_clear=sonar_ok && sonar.valid && sonar.angle_deg==0 && sonar.sample_angle_deg==0 &&
            now.lo-sonar.time_ms<S5_STALE_MS && sonar.distance_mm>250U;
#if BOARD_IR_LINE_ONLY_BENCH
        /* This named profile explicitly has no obstacle protection. The
         * actual unavailable range is never published as a clear reading. */
#if BOARD_IR_CONTINUOUS_BENCH
        /* User-operated timed test: all usable patterns may start and steer.
         * Encoders remain diagnostics; they cannot terminate this profile. */
        ir_scan_input_t input={now.lo,motor_ok,adc_ok,adc_ok,true};
#else
        ir_scan_input_t input={now.lo,motor_ok && encoder_ok,adc_ok,ir_line_start_ready(&line),true};
#endif
#else
        ir_scan_input_t input={now.lo,motor_ok,adc_ok,ir_line_start_ready(&line),actual_front_clear};
#endif
        ir_scan_step(&scan,&input);
#if !BOARD_IR_LINE_ONLY_BENCH
        ir_line_enforce(&scan,&line);
#endif
        for(unsigned n=0U;n<4U;++n) {
            control_message_t command;ER er=s1_command_take(&command);
            if(er==E_TMOUT) break;
            if(er<E_OK) {ir_scan_stop(&scan,IR_SCAN_BAD_COMMAND);break;}
            if(command.wire[0]!=0xD3U) {ir_scan_stop(&scan,IR_SCAN_REQUESTED_STOP);break;}
            bool started=ir_scan_command(&scan,&input,command.wire,command.time_ms);
            if(command.wire[1]==IR_SCAN_QUERY) {
#if BOARD_IR_CONTINUOUS_BENCH
                tm_printf((UB *)"[IRLINE READY] v=6 active=%u prepared=%u cap=30 lease=250 max_ms=20000 adc=%u motor=%u pattern_stop=0 encoder_stop=0 start_ready=%u\n",
                    scan.active,scan.prepared,(unsigned)adc_ok,(unsigned)motor_ok,(unsigned)input.white);
#elif BOARD_IR_LINE_ONLY_BENCH
                tm_printf((UB *)"[IRLINE READY] v=5 active=%u prepared=%u cap=50 lease=250 max_ms=20000 adc=%u motor=%u sonar_required=0 encoders=%u course=1 cross=straight white_ready=%u\n",
                    scan.active,scan.prepared,(unsigned)adc_ok,(unsigned)motor_ok,
                    (unsigned)encoder_ok,(unsigned)input.white);
#else
                tm_printf((UB *)"[IRLINE READY] v=2 active=%u prepared=%u cap=20 lease=250 max_ms=6000 adc=%u motor=%u front=%u white_ready=%u\n",
                    scan.active,scan.prepared,(unsigned)adc_ok,(unsigned)motor_ok,
                    (unsigned)input.front_clear,(unsigned)input.white);
#endif
            }
            if(started) tm_printf((UB *)"[IRLINE START] mode=%u power=%u duration=%u\n",
                                 (unsigned)scan.scan+1U,scan.power,scan.duration_ms);
        }
#if !BOARD_IR_LINE_ONLY_BENCH
        ir_line_enforce(&scan,&line);
#endif
#if BOARD_IR_CONTINUOUS_BENCH
        unsigned direction=scan.active?(scan.scan?ir_line_continuous_direction(&line):IR_LINE_FORWARD):IR_LINE_STOP;
#elif BOARD_IR_LINE_ONLY_BENCH
        unsigned course_direction=ir_line_course_step(&course,&line,now.lo,scan.active && scan.scan);
        if(course.fault && scan.active && scan.scan)
            ir_scan_stop(&scan,(ir_scan_reason_t)course.fault);
        unsigned direction=scan.active?(scan.scan?course_direction:IR_LINE_FORWARD):IR_LINE_STOP;
        if(previous_course!=course.phase) {
            tm_printf((UB *)"[IRCOURSE] t=%u phase=%u direction=%u fault=%u\n",
                      now.lo,(unsigned)course.phase,direction,(unsigned)course.fault);
            previous_course=course.phase;
        }
        const uint32_t counts[2]={(uint32_t)enc.motor1_forward_counts,(uint32_t)enc.motor2_forward_counts};
        if(!ir_line_progress_check(&progress,direction,counts,now.lo,encoder_ok)) {
            ir_scan_stop(&scan,IR_SCAN_NO_PROGRESS);direction=IR_LINE_STOP;s2_emergency_stop();
        }
#else
        unsigned direction=scan.active?(scan.scan?ir_line_direction(&line):IR_LINE_FORWARD):IR_LINE_STOP;
#endif
        if(direction!=IR_LINE_STOP) {
            ER er=s2_movement_send((s2_move_command_t)direction,scan.power);
            if(er<E_OK && er!=E_OBJ) {ir_scan_stop(&scan,IR_SCAN_MOTOR_FAULT);s2_emergency_stop();}
        } else s2_emergency_stop();
        if(now.lo-previous_log>=100U || previous_active!=scan.active || previous_prepared!=scan.prepared ||
           previous_reason!=scan.reason || previous_mask!=line.result.black_mask) {
#if !BOARD_IR_LINE_ONLY_BENCH
            s2_encoder_status_t enc={0};(void)s2_encoder_get_status(&enc);
#endif
            tm_printf((UB *)"[IRLINE] t=%u active=%u prepared=%u mode=%u reason=%u power=%u raw=%u,%u darkness=%u,%u mask=%u adc=%u white_ready=%u front=%u motor_state=%u command=%u motor_power=%u m1=%d m2=%d\n",
                now.lo,scan.active,scan.prepared,(unsigned)scan.scan+1U,scan.reason,scan.power,
                (unsigned)ir.sample.raw[0],(unsigned)ir.sample.raw[1],
                (unsigned)line.result.darkness[0],(unsigned)line.result.darkness[1],
                (unsigned)line.result.black_mask,(unsigned)adc_ok,(unsigned)input.white,
                (unsigned)actual_front_clear,(unsigned)motor.state,(unsigned)motor.command,(unsigned)motor.power_percent,
                enc.motor1_forward_counts,enc.motor2_forward_counts);
            previous_log=now.lo;previous_active=scan.active;previous_prepared=scan.prepared;
            previous_reason=scan.reason;previous_mask=line.result.black_mask;
        }
        state.time_ms=now.lo;++state.sequence;
        state.vehicle.armed=scan.active;state.vehicle.phase=scan.active?VC_TRACK:VC_DISARMED;
        state.vehicle.command=scan.active?(uint8_t)(direction&7U):S2_MOVE_STOP;state.vehicle.power=scan.power;
        (void)app_state_publish(&state);
        tk_dly_tsk(10);
    }
}
ER vehicle_start(void) {
    ID tid=app_start_task((FP)line_trace_task,5,TP_PRC1,"IR line trace");return tid<0?tid:E_OK;
}
#endif
