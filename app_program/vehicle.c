#include "app_state.h"
#include "board_pins.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/odometry.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem4/encoder_input.h"
#include "subsystem2/s2_data.h"
#include "subsystem5/subsystem5.h"
#if !BOARD_IR_MOTOR_SCAN_BENCH
static void vehicle_task(INT code,void *context) {
    (void)code;(void)context;
    s2_controller_bind();
    if(s3_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    if(APP_S5_ENABLED && s5_controller_bind()<E_OK) {s2_emergency_stop();tk_ext_tsk();return;}
    app_state_t state={0};s4_encoder_input_t encoder_input={0};terrain_t terrain={0};
    odom_t *odom=&encoder_input.odometry;
    odom_calibration_t calibration_record={0};
    {
        const odom_config_t measured={BOARD_TICKS_PER_REV,BOARD_WHEEL_CIRCUMFERENCE_MM,BOARD_TRACK_WIDTH_MM};
        (void)odom_calibration_pack(&measured,&calibration_record);
    }
    s3_mode_t mode=S3_MODE_PAUSED;
    uint32_t next_tick=0;unsigned previous_phase=VC_DISARMED,previous_look=0;
    BOOL requested_sensor_mode=FALSE;
    for(;;) {
        SYSTIM now;
        odom_config_t calibration={0}; /* short-lived calculation/API view */
        (void)odom_calibration_unpack(&calibration_record,&calibration);
        s2_motion_status_t motor={0};uint8_t encoder_payload[EDR_ENCODER_SIZE]={0};s3_status_t line={0};s5_status_t scan={0};s4_sensor_status_t imu={0};
        BOOL mo=s2_motion_get_status(&motor)==E_OK;
        BOOL en=s2_encoder_encode(encoder_payload,sizeof(encoder_payload))==E_OK;
        BOOL li=s3_status_get(&line)==E_OK;
        BOOL so=s5_get_status(&scan)==E_OK;
        s4_sensor_cache_t imu_record={0};
        BOOL im=s4_sensor_get_record(&imu_record)==E_OK;
        /* Snapshot APIs may wait briefly. Age them against a time obtained
         * afterwards, so a newly published sample cannot appear in the future. */
        if(tk_get_otm(&now)<0) {vc_stop(&state.vehicle,VC_TRANSPORT);s2_emergency_stop();tk_dly_tsk(10);continue;}
        if(im)s4_data_snapshot(&imu_record,s4_operating_ms_to_us(((uint64_t)(UW)now.hi<<32)|now.lo),&imu);
        state.time_ms=now.lo;++state.sequence;
        s4_encoder_terrain_step(&encoder_input,&terrain,&calibration,now.lo,
            encoder_payload,sizeof(encoder_payload),en,imu.latest.accel.read_time_us,
            imu.latest.accel.milli_units,im && imu.accel_usable && !imu.latest.accel.overrun,BOARD_TERRAIN_CALIBRATED,
            BOARD_PITCH_ZERO_CDEG,BOARD_ROLL_ZERO_CDEG);
        s4_result_t result={.calculated_ms=now.lo,
            .pitch_cdeg=terrain_pitch_cdeg(&terrain),.roll_cdeg=terrain_roll_cdeg(&terrain),
            .height_mm=(uint16_t)terrain.height_mm,.maximum_mm=(uint16_t)terrain.maximum_mm,
            .terrain=(uint8_t)terrain.state,
            .flags=(uint8_t)((terrain.tilt_valid?S4_RESULT_TILT:0U)|
                (terrain.height_valid?S4_RESULT_HEIGHT:0U)|(terrain.shock?S4_RESULT_SHOCK:0U)|
                (BOARD_TERRAIN_CALIBRATED?S4_RESULT_CALIBRATED:0U)|(odom->valid?S4_RESULT_ODOMETRY:0U)),
            .faults=(uint8_t)((!odom->valid?EDR_FAULT_ENCODER:0U)|
                ((!im || !imu.accel_usable || imu.latest.accel.overrun ||
                   terrain.shock || encoder_input.fault==S4_INPUT_SKEW)?EDR_FAULT_IMU:0U))};
        BOOL result_ok=s4_result_encode(&result,state.s4_result,sizeof(state.s4_result)) &&
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
            if(c.wire[1]==0) {vc_stop(&state.vehicle,VC_OK);break;}
            if(now.lo-c.time_ms>=100U)continue;
            if(c.wire[1]==2 && !state.vehicle.armed && mo && motor.state!=S2_MOTION_MOVING) {
                state.vehicle=(vc_state_t){0};terrain=(terrain_t){0};encoder_input=(s4_encoder_input_t){0};
                s4_result_t cleared={.calculated_ms=now.lo};
                result=cleared;
                result_ok=s4_result_encode(&cleared,state.s4_result,sizeof(state.s4_result));continue;
            }
            if(c.wire[1]==1) {
                if(BOARD_MOTORS_DISABLED) {vc_stop(&state.vehicle,VC_MOTOR);continue;}
                BOOL fresh_arm=!state.vehicle.armed;
                if(!odom_config_valid(&calibration) || !BOARD_TERRAIN_CALIBRATED || !line.configured) {
                    vc_stop(&state.vehicle,VC_UNCALIBRATED);continue;
                }
                if(vc_arm(&state.vehicle,now.lo,(unsigned)c.wire[3]|((unsigned)c.wire[4]<<8),c.wire[2]) && fresh_arm) {
                    state.vehicle.phase=VC_REACQUIRE;state.vehicle.entered_ms=now.lo;
                    requested_sensor_mode=FALSE;
                }
            }
        }
        vc_input_t input={.now_ms=now.lo,.scan_id=scan.scan_id,
            .distance_um=odom->distance_um,.yaw_mdeg=odom_turn_mdeg(odom),
            .line_error=line.output.line.error,.line_state=(uint8_t)line.output.line.state,
            .turn_deg=plan_usable?(int16_t)scan.planned_rotate_deg:0,
            .approach_mm=plan_usable?(uint16_t)scan.planned_approach_mm:0,
            .return_mm=plan_usable?(uint16_t)scan.planned_return_mm:0,
            .sensor_state=scan.state,.odom_valid=result_ok && (result.flags&S4_RESULT_ODOMETRY),.motor_ok=mo && motor.state!=S2_MOTION_FAULT,
            .terrain_valid=result_ok && (result.flags&S4_RESULT_TILT),.shock=(result.flags&S4_RESULT_SHOCK)!=0U,
            .slope=result.terrain==TERRAIN_ASCENT || result.terrain==TERRAIN_DESCENT,
            .bypass_enabled=BOARD_BYPASS_VALIDATED,.transport_ok=1};
        input.line_fresh=li && line.configured && line.acquisition_error==E_OK && now.lo-line.sample_time_ms<30U;
        input.steering=input.line_fresh && line.output.steering_allowed;
        input.sensor_fresh=so && scan.state!=S5_FAULT &&
            (!scan.valid || (scan.distance_mm>0 && scan.distance_mm<=5000U)) &&
            now.lo-scan.time_ms<(scan.valid?S5_STALE_MS:1500U);
        input.front_clear=input.sensor_fresh && scan.valid && scan.angle_deg==0 && scan.sample_angle_deg==0 &&
            scan.distance_mm>(state.vehicle.phase==VC_TRACK?S5_OBSTACLE_MM:S5_GUIDANCE_OBSTACLE_MM);
        input.obstacle=input.sensor_fresh && ((scan.valid && scan.angle_deg==0 && scan.distance_mm<=S5_OBSTACLE_MM) ||
            s5_stage_for_state((s5_state_t)scan.state)==S5_STAGE_ACQUISITION);
        input.scan_ready=input.sensor_fresh && scan.state==S5_READY && plan_usable && scan.approach_plan_valid && scan.object_width_valid;
        int side_angle=state.vehicle.side?90:-90;
        input.side_ready=input.sensor_fresh && scan.valid && scan.sample_angle_deg==side_angle &&
            scan.angle_deg==side_angle;
        input.side_clear=input.side_ready && scan.distance_mm>200U;
        for(unsigned n=0;n<S3_EVENT_CAPACITY;++n) {
            s3_event_record_t e;ER er=s3_event_receive(&e);if(er==E_TMOUT)break;
            if(er<0) {input.transport_ok=0;break;}
            if(e.event.kind==S3_EVENT_NAVIGATION) {
                input.navigation_event=1;input.nav=(uint8_t)e.event.command;
                state.navigation=(uint8_t)e.event.command;state.barcode=e.event.symbol=='A'?1:e.event.symbol=='Z'?2:0;
            } else input.safety_event=1;
        }
        vc_step(&state.vehicle,&input);
        s3_mode_t wanted=!state.vehicle.armed?S3_MODE_PAUSED:
            (state.vehicle.phase==VC_TRACK || state.vehicle.phase==VC_REACQUIRE)?S3_MODE_TRACK:S3_MODE_OVERRIDE;
        ER control=E_OK;
        BOOL acknowledge_straight=input.navigation_event && input.nav==S3_NAV_STRAIGHT &&
            state.vehicle.armed && state.vehicle.phase==VC_TRACK;
        if(wanted!=mode || acknowledge_straight) {
            control=s3_controller_control(wanted,wanted==S3_MODE_TRACK,wanted==S3_MODE_TRACK?100:0);mode=wanted;
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
            ER er=s2_movement_send((s2_move_command_t)state.vehicle.command,state.vehicle.power);
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
