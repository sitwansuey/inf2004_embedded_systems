#include "app_state.h"
#include "board_pins.h"
#include "vehicle_curve_memory.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/odometry.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem4/encoder_input.h"
#include "subsystem4/mounting.h"
#include "subsystem4/accel_calibration.h"
#include "subsystem2/s2_data.h"
#include "subsystem5/subsystem5.h"
#if !BOARD_IR_MOTOR_SCAN_BENCH
static void vehicle_task(INT code,void *context) {
    (void)code;(void)context;
    s2_controller_bind();
    if(s3_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    if(APP_S5_ENABLED && s5_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    app_state_t state={0};s4_encoder_input_t encoder_input={0};terrain_t terrain={0};
    s4_motion_t motion={0};
    const s4_accel_calibration_t accel_calibration=S4_BOARD_ACCEL_CALIBRATION;
    const terrain_geometry_t geometry={.track_mm=BOARD_TRACK_WIDTH_MM,
        .forward_mm=BOARD_IMU_FORWARD_MM,.left_mm=BOARD_IMU_LEFT_MM,.up_mm=BOARD_IMU_UP_MM,
        .one_sided=BOARD_TERRAIN_ONE_SIDED,.configured=BOARD_IMU_GEOMETRY_CALIBRATED};
    odom_t *odom=&encoder_input.odometry;
    odom_calibration_t calibration_record={0};
    {
        const odom_config_t measured={BOARD_TICKS_PER_REV,BOARD_WHEEL_CIRCUMFERENCE_MM,BOARD_TRACK_WIDTH_MM};
        (void)odom_calibration_pack(&measured,&calibration_record);
    }
    s3_mode_t mode=S3_MODE_PAUSED;
    INT line_curve=0;
    curve_memory_t curve_memory={0};
    uint32_t next_tick=0;unsigned previous_phase=VC_DISARMED,previous_look=0;
    BOOL requested_sensor_mode=FALSE;
    for(;;) {
        SYSTIM now;
        odom_config_t calibration={0}; /* short-lived calculation/API view */
        (void)odom_calibration_unpack(&calibration_record,&calibration);
        s2_motion_status_t motor={0};uint8_t encoder_payload[ENCODER_SNAPSHOT_SIZE]={0};s3_status_t line={0};s5_status_t scan={0};s4_sensor_status_t imu={0};
        BOOL mo=s2_motion_get_status(&motor)==E_OK;
        BOOL en=s2_encoder_snapshot(encoder_payload,sizeof(encoder_payload))==E_OK;
        BOOL li=s3_status_get(&line)==E_OK;
        BOOL so=s5_get_status(&scan)==E_OK;
        s4_sensor_cache_t imu_record={0};
        BOOL im=s4_sensor_get_record(&imu_record)==E_OK;
        /* Snapshot APIs may wait briefly. Age them against a time obtained
         * afterwards, so a newly published sample cannot appear in the future. */
        if(tk_get_otm(&now)<0) {vc_stop(&state.vehicle,VC_TRANSPORT);s2_emergency_stop();tk_dly_tsk(10);continue;}
        if(im)s4_data_snapshot(&imu_record,s4_operating_ms_to_us(((uint64_t)(UW)now.hi<<32)|now.lo),&imu);
        state.time_ms=now.lo;++state.sequence;
        int32_t body_accel[3]={0}; /* temporary view; raw telemetry is unchanged */
        BOOL mounting_ok=s4_calibrated_body(imu.latest.accel.milli_units,
            BOARD_S4_MOUNTING,&accel_calibration,body_accel);
        s4_encoder_terrain_step_model(&encoder_input,&terrain,&calibration,now.lo,
            encoder_payload,sizeof(encoder_payload),en,imu.latest.accel.read_time_us,
            body_accel,im && imu.accel_usable && !imu.latest.accel.overrun && mounting_ok,BOARD_TERRAIN_CALIBRATED,
            BOARD_PITCH_ZERO_CDEG,BOARD_ROLL_ZERO_CDEG,&geometry);
        s4_motion_step(&motion,now.lo,odom,(uint8_t)terrain.state,
            terrain.height_valid,im?imu.impact_sequence:motion.impact_sequence);
        s4_motion_result_t motion_result;
        s4_motion_snapshot(&motion,now.lo,&motion_result);
        BOOL motion_ok=s4_motion_encode(&motion_result,state.s4_motion,sizeof(state.s4_motion));
        s4_result_t result={.calculated_ms=now.lo,
            .pitch_cdeg=terrain_pitch_cdeg(&terrain),.roll_cdeg=terrain_roll_cdeg(&terrain),
            .height_mm=(uint16_t)terrain.height_mm,.maximum_mm=(uint16_t)terrain.maximum_mm,
            .terrain=(uint8_t)terrain.state,
            .flags=(uint8_t)((terrain.tilt_valid?S4_RESULT_TILT:0U)|
                (terrain.height_valid && !s4_motion_impact(&motion)?S4_RESULT_HEIGHT:0U)|((terrain.shock || s4_motion_impact(&motion))?S4_RESULT_SHOCK:0U)|
                (BOARD_TERRAIN_CALIBRATED?S4_RESULT_CALIBRATED:0U)|(odom->valid?S4_RESULT_ODOMETRY:0U)),
            .faults=(uint8_t)((!odom->valid?EDR_FAULT_ENCODER:0U)|
                ((!im || !imu.accel_usable || !mounting_ok || imu.latest.accel.overrun ||
                   terrain.shock || s4_motion_impact(&motion) || encoder_input.fault==S4_INPUT_SKEW)?EDR_FAULT_IMU:0U))};
        if(s4_motion_impact(&motion))result.flags&=(uint8_t)~S4_RESULT_TILT;
        BOOL result_ok=motion_ok && s4_result_encode(&result,state.s4_result,sizeof(state.s4_result)) &&
            s4_result_decode(state.s4_result,sizeof(state.s4_result),&result);
        if(!result_ok) {vc_stop(&state.vehicle,VC_SENSOR);s2_emergency_stop();}
        state.speed_bits=(uint16_t)odom->speed_bits;
        state.odometry_valid=odom->valid;
        /* S5 preserves full-width diagnostics. Validate before narrowing. */
        BOOL plan_usable=scan.planned_rotate_deg>=-80 && scan.planned_rotate_deg<=80 &&
            scan.planned_rotate_deg!=0 && scan.planned_approach_mm>0 &&
            scan.planned_approach_mm<=1000U && scan.planned_return_mm<=1000U;
        /* At most four copied command records per tick; STOP bypasses that queue. */
        for(unsigned n=0;n<4;++n) {
            control_message_t c;ER er=s1_command_take(&c);if(er<0)break;
            if(c.wire[1]==0) {vc_stop(&state.vehicle,VC_OK);
                s1_command_ack(c.wire[5],0,0,state.vehicle.phase,state.vehicle.fault);break;}
            if(now.lo-c.time_ms>=100U) {
                s1_command_ack(c.wire[5],c.wire[1],3,state.vehicle.phase,state.vehicle.fault);continue;
            }
            if(c.wire[1]==2 && !state.vehicle.armed && mo && motor.state!=S2_MOTION_MOVING) {
                state.vehicle=(vc_state_t){0};terrain=(terrain_t){0};encoder_input=(s4_encoder_input_t){0};
                motion=(s4_motion_t){0};motion.impact_sequence=imu.impact_sequence;
                s4_motion_snapshot(&motion,now.lo,&motion_result);
                (void)s4_motion_encode(&motion_result,state.s4_motion,sizeof(state.s4_motion));
                s4_result_t cleared={.calculated_ms=now.lo};
                result=cleared;
                result_ok=s4_result_encode(&cleared,state.s4_result,sizeof(state.s4_result));
                s1_command_ack(c.wire[5],2,0,state.vehicle.phase,state.vehicle.fault);continue;
            }
            if(c.wire[1]==2) {
                s1_command_ack(c.wire[5],2,1,state.vehicle.phase,state.vehicle.fault);continue;
            }
            if(c.wire[1]==1 || c.wire[1]==3) {
                if(state.vehicle.armed) {
                    s1_command_ack(c.wire[5],c.wire[1],1,state.vehicle.phase,state.vehicle.fault);continue;
                }
                if(BOARD_MOTORS_DISABLED) {vc_stop(&state.vehicle,VC_MOTOR);
                    s1_command_ack(c.wire[5],c.wire[1],1,state.vehicle.phase,state.vehicle.fault);continue;}
                if(c.wire[1]==3 && (!BOARD_PID_ENABLED || !BOARD_PID_CALIBRATED || !BOARD_BYPASS_VALIDATED)) {
                    vc_stop(&state.vehicle,VC_UNCALIBRATED);
                    s1_command_ack(c.wire[5],c.wire[1],1,state.vehicle.phase,state.vehicle.fault);continue;
                }
                BOOL fresh_arm=!state.vehicle.armed;
                if(!odom_config_valid(&calibration) || !BOARD_TERRAIN_CALIBRATED || !line.configured) {
                    vc_stop(&state.vehicle,VC_UNCALIBRATED);
                    s1_command_ack(c.wire[5],c.wire[1],1,state.vehicle.phase,state.vehicle.fault);continue;
                }
                if((c.wire[1]==3 ? vc_start(&state.vehicle,now.lo,c.wire[2]) :
                    vc_arm(&state.vehicle,now.lo,(unsigned)c.wire[3]|((unsigned)c.wire[4]<<8),c.wire[2])) && fresh_arm) {
                    state.vehicle.phase=VC_REACQUIRE;state.vehicle.entered_ms=now.lo;
                    state.vehicle.origin_yaw=odom_turn_mdeg(odom);
                    state.vehicle.origin_um=odom->distance_um;state.vehicle.turn_progress=0;
                    requested_sensor_mode=FALSE;
                }
                s1_command_ack(c.wire[5],c.wire[1],state.vehicle.armed?0U:1U,
                    state.vehicle.phase,state.vehicle.fault);
            }
        }
        vc_input_t input={.now_ms=now.lo,.scan_id=scan.scan_id,
            .distance_um=odom->distance_um,.yaw_mdeg=odom_turn_mdeg(odom),
            .line_error=line.output.line.error,.line_state=(uint8_t)line.output.line.state,
            .line_black_mask=(uint8_t)(line.output.line.black_mask & 3U),
            .turn_deg=plan_usable?(int16_t)scan.planned_rotate_deg:0,
            .approach_mm=plan_usable?(uint16_t)scan.planned_approach_mm:0,
            .return_mm=plan_usable?(uint16_t)scan.planned_return_mm:0,
            .sensor_state=scan.state,.odom_valid=result_ok && (result.flags&S4_RESULT_ODOMETRY),.motor_ok=mo && motor.state!=S2_MOTION_FAULT,
            .terrain_valid=result_ok && (result.flags&S4_RESULT_TILT),.shock=(result.flags&S4_RESULT_SHOCK)!=0U,
            .slope=result.terrain==TERRAIN_ASCENT || result.terrain==TERRAIN_DESCENT,
            .bypass_enabled=BOARD_BYPASS_VALIDATED,.transport_ok=1};
        input.line_fresh=li && line.configured && line.acquisition_error==E_OK && now.lo-line.sample_time_ms<30U;
        input.steering=input.line_fresh && line.output.steering_allowed;
        input.line_detected=input.line_fresh &&
            (line.output.line.state==S3_LINE_TRACKING ||
             (line.output.line.state==S3_LINE_JUNCTION && line.output.line.black_mask==3U));
        input.sensor_fresh=so && scan.state!=S5_FAULT &&
            (!scan.valid || (scan.distance_mm>0 && scan.distance_mm<=5000U)) &&
            now.lo-scan.time_ms<(scan.valid?S5_STALE_MS:1500U);
        input.front_clear=input.sensor_fresh && scan.valid && scan.angle_deg==0 && scan.sample_angle_deg==0 &&
            scan.distance_mm>((state.vehicle.phase==VC_TRACK || state.vehicle.phase==VC_ACQUIRE)?
                S5_OBSTACLE_MM:S5_GUIDANCE_OBSTACLE_MM);
        input.obstacle=input.sensor_fresh && ((scan.valid && scan.angle_deg==0 && scan.distance_mm<=S5_OBSTACLE_MM) ||
            s5_stage_for_state((s5_state_t)scan.state)==S5_STAGE_ACQUISITION);
        input.scan_ready=input.sensor_fresh && scan.state==S5_READY && plan_usable && scan.approach_plan_valid && scan.object_width_valid;
        int side_angle=state.vehicle.side?90:-90;
        input.side_ready=input.sensor_fresh && scan.valid && scan.sample_angle_deg==side_angle &&
            scan.angle_deg==side_angle;
        input.side_clear=input.side_ready && scan.distance_mm>200U;
        line_curve=curve_memory_update(&curve_memory,now.lo,
            state.vehicle.armed && state.vehicle.phase==VC_TRACK,
            input.line_fresh && line.output.error==S3_ERROR_NONE,
            line.output.line.black_mask & 3U,line.output.line.error);
        input.curve_hold=curve_memory.turning && input.line_fresh;
        for(unsigned n=0;n<S3_EVENT_CAPACITY;++n) {
            s3_event_record_t e;ER er=s3_event_receive(&e);if(er==E_TMOUT)break;
            if(er<0) {input.transport_ok=0;break;}
            if(e.event.kind==S3_EVENT_NAVIGATION) {
                input.navigation_event=1;input.nav=(uint8_t)e.event.command;
                state.navigation=(uint8_t)e.event.command;state.barcode=(e.event.symbol>='A' && e.event.symbol<='D')?(uint8_t)(e.event.symbol-'A'+1):e.event.symbol=='Z'?5:0;
            } else if(e.event.kind==S3_EVENT_RECOVERY_REQUEST) input.recovery_event=1;
            else input.safety_event=1;
        }
        if(input.curve_hold && input.recovery_event) {
            /* Acknowledge the short dropout so S3 can resume normal observations. */
            if(s3_controller_control(S3_MODE_TRACK,true,100)<E_OK)input.transport_ok=0;
        }
        vc_step(&state.vehicle,&input);
        s3_mode_t wanted=!state.vehicle.armed?S3_MODE_PAUSED:
            state.vehicle.phase==VC_TRACK?S3_MODE_TRACK:S3_MODE_OVERRIDE;
        ER control=E_OK;
        BOOL acknowledge_straight=state.vehicle.navigation_ack &&
            state.vehicle.armed && state.vehicle.phase==VC_TRACK;
        if(wanted!=mode || acknowledge_straight) {
            control=s3_controller_control(wanted,wanted==S3_MODE_TRACK,wanted==S3_MODE_TRACK?100:0);
            if(control>=E_OK) {
                mode=wanted;
                if(wanted==S3_MODE_TRACK)vc_track_requested(&state.vehicle,now.lo);
            }
        } else if(wanted==S3_MODE_TRACK)control=s3_controller_renew(100);
        if(control<0)vc_stop(&state.vehicle,VC_TRANSPORT);
        /* STOP on each phase transition before moving the range sensor. */
        unsigned phase=state.vehicle.phase, look=state.vehicle.look_side;
        if(phase!=previous_phase || look!=previous_look || !requested_sensor_mode) {
            s2_emergency_stop();
            s5_state_t sensor_mode=S5_MONITOR;int angle=0;
            if(phase==VC_TURN_OUT || phase==VC_DIAGONAL || phase==VC_ALIGN)sensor_mode=S5_APPROACH;
            else if(phase==VC_PASS) {sensor_mode=S5_OVERCOME;angle=look?side_angle:0;}
            else if(phase==VC_TURN_IN || phase==VC_RETURN || phase==VC_RESTORE)sensor_mode=S5_REORG;
            /* Acquisition runs entirely in S5; do not reset its search. */
            if(phase!=VC_ACQUIRE && so) {
                ER er=s5_navigation_set(scan.scan_id,sensor_mode,angle);
                if(er<0 && state.vehicle.armed)vc_stop(&state.vehicle,VC_TRANSPORT);
            }
            requested_sensor_mode=TRUE;previous_phase=phase;previous_look=look;
            state.vehicle.command=0;state.vehicle.power=0;
        }
        if(state.vehicle.command==S2_MOVE_STOP) {
            /* An epoch per stopped tick invalidates every old queued movement. */
            s2_emergency_stop();
        } else {
            ER er;
            if(BOARD_PID_ENABLED && state.vehicle.phase==VC_TRACK &&
               state.vehicle.command==S2_MOVE_FORWARD && input.line_fresh) {
                er=s2_follow_send(state.vehicle.power,line_curve);
            } else {
                line_curve=0;
                er=s2_movement_send((s2_move_command_t)state.vehicle.command,state.vehicle.power);
            }
            /* Motor may still be acknowledging the previous STOP. Retry next tick. */
            if(er<0 && er!=E_OBJ) {s2_emergency_stop();vc_stop(&state.vehicle,VC_TRANSPORT);}
        }
        (void)app_state_publish(&state);
        if(next_tick==0)next_tick=now.lo;
        next_tick+=10;tk_get_otm(&now);
        W wait=(W)(next_tick-now.lo);if(wait<=0) {next_tick=now.lo+10;wait=10;}
        tk_dly_tsk((UW)wait);
    }
}
ER vehicle_start(void) {
    ID tid=app_start_task((FP)vehicle_task,5,TP_PRC1,"vehicle");return tid<0?tid:E_OK;
}
#endif
/* Retired sample arbitration API: every subsystem may request STOP.
 * Positive movement comes only from the controller's checked policies. */
ER app_vehicle_request(UINT source,s2_move_command_t command,UINT power,UW lease) {
    (void)power;(void)lease;
    if(source<1 || source>5 || command!=S2_MOVE_STOP)return E_PAR;
    unsigned char stop[6]={0xC1,0,0,0,0,0};return s1_command_offer(stop,sizeof(stop));
}
