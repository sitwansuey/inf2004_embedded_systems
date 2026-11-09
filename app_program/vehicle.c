#include "app_state.h"
#include "board_pins.h"
#include "vehicle_curve_memory.h"
#include "vehicle_motion_handoff.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/odometry.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem4/encoder_input.h"
#include "subsystem2/s2_data.h"
#include "subsystem5/subsystem5.h"
#include "vehicle_reverse_track.h"
#ifndef BOARD_LINE_STRADDLE
#define BOARD_LINE_STRADDLE 0 /* 1: GP26/27 straddle the line; both WHITE = centred */
#endif
#if BOARD_LINE_STRADDLE
#include "ir_line_course.h"
#include "robot_ir_config.h"
#include "vehicle_line_steer.h"
#endif
#ifndef BOARD_BARCODE_RETRY_DRIVE
#define BOARD_BARCODE_RETRY_DRIVE 0
#endif
#if BOARD_BARCODE_RETRY_DRIVE
#include "vehicle_barcode_retry.h"
/* Barcode re-read sequence, layered on VC_TRACK (the policy phase is unchanged).
 * A read ends once the robot has moved BOARD_BARCODE_END_GAP_MM (30 mm)
 * without a GP28 edge (this task asks S3 to finish it), so the robot has
 * always finished crossing the marker before it reacts:
 * 1. normal-speed read fails  -> brake, reverse at 30% speed (the decoder
 *    already accepts a backwards symbol), holding the forward-tracking
 *    heading with a small line correction (vehicle_reverse_track.h);
 *    keep reversing until the reverse read has ended (30 mm past the first
 *    bar) and a further BOARD_BARCODE_RETRY_RUNUP_MM, so a slow pass starts
 *    before the marker, then brake;
 * 2. reverse read succeeds    -> forward at normal speed back to where
 *    the first read failed, ignoring the marker; then any decoded turn runs;
 *    reverse read fails, or the barcode state does not change for
 *    BOARD_BARCODE_RETRY_IDLE_MS (3 s) while reversing or waiting
 *                             -> forward at 30% speed for a final read;
 *    while reversing, failure is never decided in the first
 *    BOARD_BARCODE_RETRY_REVERSE_MIN_MS (5 s), only REVERSE_MAX_MM stops it;
 * 3. final read (hit or miss) -> normal journey.
 * The forward sonar does not gate reversing or waiting: the policy's STOP for
 * an unclear front, or a one-sample obstacle glitch, does not interrupt the
 * backwards read. S5 is reset to MONITOR before driving forward again. */
enum { BR_IDLE, BR_BRAKE, BR_REVERSE, BR_WAIT, BR_RESUME, BR_SLOW };
static unsigned br_abs_um(int32_t a,int32_t b) {
    int64_t d=(int64_t)a-b;return (unsigned)(d<0?-d:d);
}
#endif
#ifndef VC_TILT_GRACE_MS
#define VC_TILT_GRACE_MS 300U
#endif
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
    INT line_curve=0;
    curve_memory_t curve_memory={0};
    uint32_t next_tick=0;unsigned previous_phase=VC_DISARMED,previous_look=0;
    BOOL requested_sensor_mode=FALSE;
    uint32_t tilt_ok_ms=0;BOOL tilt_seen=FALSE; /* last valid gravity tilt */
#if BOARD_BARCODE_RETRY_DRIVE
    unsigned br_state=BR_IDLE,br_next=BR_IDLE;
    uint8_t br_ok=0,br_fail=0;BOOL br_seen=FALSE,br_cooldown=FALSE;
    uint32_t br_ms=0,br_back_um=0;int32_t br_origin_um=0;
    /* Last barcode state seen (elements|result|ok|fail) and when it changed. */
    uint32_t br_signature=0,br_change_ms=0;
    /* Distance-based end of marker: element count and odometry at the last
     * GP28 edge of the running read, and whether the end was requested. */
    uint8_t br_end_elements=0,br_end_result=0;int32_t br_end_um=0;
    /* This reverse pass: whole marker counted / its read failed (latched). */
    bool br_full=false,br_rev_miss=false;
    /* Smoothed forward-tracking heading, and the copy held while reversing. */
    int32_t br_yaw_ref=0,br_hold_yaw=0;BOOL br_yaw_seeded=FALSE;
#endif
#if BOARD_LINE_STRADDLE
    /* Team straddle course logic (ir_line_trace/ir_line_course), shared with the
     * IRLineOnly bench. S3 still owns ADC acquisition; this only interprets it. */
    s3_config_t straddle_cfg;ir_line_trace_t straddle={0};ir_line_course_t course={0};
    s3_config_defaults(&straddle_cfg);robot_ir_apply_layout(&straddle_cfg);
    if(!robot_ir_apply_adc_calibration(&straddle_cfg)) {s2_emergency_stop();tk_ext_tsk();return;}
#endif
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
            if(c.wire[1]==0) {vc_stop(&state.vehicle,VC_OK);
                s1_command_ack(c.wire[5],0,0,state.vehicle.phase,state.vehicle.fault);break;}
            if(now.lo-c.time_ms>=100U) {
                s1_command_ack(c.wire[5],c.wire[1],3,state.vehicle.phase,state.vehicle.fault);continue;
            }
            if(c.wire[1]==2 && !state.vehicle.armed && mo && motor.state!=S2_MOTION_MOVING) {
                state.vehicle=(vc_state_t){0};terrain=(terrain_t){0};encoder_input=(s4_encoder_input_t){0};
                requested_sensor_mode=FALSE; /* RESET also restarts a latched S5 scan. */
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
            .bypass_steering=(uint16_t)(BOARD_TRACK_WIDTH_MM*1000U/(2U*S5_BYPASS_ARC_RADIUS_MM)),
            .bypass_enabled=BOARD_PID_ENABLED && BOARD_BYPASS_VALIDATED,.transport_ok=1};
        /* The GY-511 has no gyro: tilt is derived from gravity alone, so the
         * lateral jolt when a pivot starts briefly reads as >15 deg roll and
         * terrain drops tilt. Tolerate gaps shorter than VC_TILT_GRACE_MS;
         * a sustained loss still faults VC_SENSOR, and shock is never masked. */
        if(input.terrain_valid) {tilt_ok_ms=now.lo;tilt_seen=TRUE;}
        else if(tilt_seen && !input.shock && now.lo-tilt_ok_ms<VC_TILT_GRACE_MS) input.terrain_valid=1;
        input.line_fresh=li && line.configured && line.acquisition_error==E_OK && now.lo-line.sample_time_ms<30U;
        input.steering=input.line_fresh && line.output.steering_allowed;
        input.line_detected=input.line_fresh &&
            (line.output.line.state==S3_LINE_TRACKING ||
             (line.output.line.state==S3_LINE_JUNCTION && line.output.line.black_mask==3U));
        input.sensor_fresh=so && scan.state!=S5_FAULT &&
            (!scan.valid || (scan.distance_mm>0 && scan.distance_mm<=5000U)) &&
            now.lo-scan.time_ms<(scan.valid?S5_STALE_MS:S5_INVALID_STALE_MS);
        input.front_clear=input.sensor_fresh && scan.angle_deg==0 &&
            s5_direction_clear(&scan,(state.vehicle.phase==VC_TRACK || state.vehicle.phase==VC_ACQUIRE)?
                S5_OBSTACLE_MM:S5_GUIDANCE_OBSTACLE_MM);
        input.obstacle=input.sensor_fresh && ((scan.valid && scan.angle_deg==0 && scan.distance_mm<=S5_OBSTACLE_MM) ||
            s5_stage_for_state((s5_state_t)scan.state)==S5_STAGE_ACQUISITION);
        input.width_valid=input.sensor_fresh && scan.object_width_valid &&
            scan.object_width_max_mm<=AVOID_MAX_WIDTH_MM;
        input.obstacle_width_mm=(uint16_t)(input.width_valid?scan.object_width_max_mm:0U);
        input.scan_blocked=input.sensor_fresh && scan.state==S5_BLOCKED;
        input.sensor_acquiring=so && s5_stage_for_state((s5_state_t)scan.state)==S5_STAGE_ACQUISITION;
        input.scan_ready=input.sensor_fresh && scan.state==S5_READY && plan_usable && scan.approach_plan_valid;
        int side_angle=state.vehicle.side?90:-90;
        input.side_observed=input.sensor_fresh && scan.sample_angle_deg==side_angle && scan.angle_deg==side_angle;
        input.side_ready=input.side_observed && (scan.valid || s5_sample_is_open(&scan));
        input.side_clear=input.side_ready && s5_direction_clear(&scan,AVOID_SIDE_CLEAR_MM);
        input.side_sample_ms=scan.time_ms;
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
                state.navigation=(uint8_t)e.event.command;state.barcode=(e.event.symbol>='A' && e.event.symbol<='Z')?(uint8_t)(e.event.symbol-'A'+1):0;
            } else if(e.event.kind==S3_EVENT_RECOVERY_REQUEST) input.recovery_event=1;
            else input.safety_event=1;
        }
        if(input.curve_hold && input.recovery_event) {
            /* Acknowledge the short dropout so S3 can resume normal observations. */
            if(s3_controller_control(S3_MODE_TRACK,true,100)<E_OK)input.transport_ok=0;
        }
#if BOARD_LINE_STRADDLE
        /* Straddle geometry: both sensors WHITE is centred, so S3's edge-follow
         * SEARCHING/LOST/recovery semantics do not apply. Feed the policy a
         * steerable line whenever the straddle observer has a fresh sample. */
        BOOL straddle_ok=ir_line_observe(&straddle,&straddle_cfg.line,line.sample.raw,
            line.sample.valid_mask,line.sample_time_ms,now.lo,
            li && line.configured && line.acquisition_error==E_OK);
        input.line_fresh=input.line_fresh && straddle_ok;
        input.steering=input.line_fresh;input.line_detected=input.line_fresh;
        input.line_state=input.line_fresh?(uint8_t)S3_LINE_TRACKING:(uint8_t)S3_LINE_INVALID;
        input.line_black_mask=(uint8_t)(straddle.result.black_mask&3U);
        input.recovery_event=0;input.curve_hold=0;
        /* A crossing line puts both straddle sensors and GP28 on black: decoded
         * turns wait for it. (Edge-following S3 withholds steering until a
         * decoded turn runs, so those builds keep turning at once: 0/0.) */
        {
            /* A bend also puts both sensors on black, but only after one side
             * has touched for a while: the course's own corner test. */
            BOOL corner=course.phase!=IR_COURSE_TRACK || (course.side_streak &&
                course.side_last_ms-course.side_since_ms>=IR_COURSE_SIDE_HOLD_MS &&
                now.lo-course.side_last_ms<30U);
            input.junction=input.line_fresh && input.line_black_mask==3U &&
                line.output.barcode_black && !corner;
        }
        input.nav_offset_mm=(uint16_t)BOARD_NAV_JUNCTION_OFFSET_MM;
        input.nav_search_mm=(uint16_t)BOARD_NAV_JUNCTION_SEARCH_MM;
        input.uturn_ignore_mm=(uint16_t)BOARD_NAV_UTURN_IGNORE_MM;
#endif
        input.line_touch=input.line_fresh && input.line_black_mask!=0U;
        input.spot_scan_all=BOARD_NAV_SPOT_SCAN_ALL!=0;
#if BOARD_BARCODE_RETRY_DRIVE
        {
            BOOL hit=FALSE,miss=FALSE,full=FALSE,reading=FALSE;
            if(li && line.configured) {
                reading=line.output.barcode_result==S3_BARCODE_RESULT_READING;
                uint32_t signature=(uint32_t)line.output.barcode_elements|
                    ((uint32_t)line.output.barcode_result<<8)|
                    ((uint32_t)line.output.barcode_ok_count<<16)|
                    ((uint32_t)line.output.barcode_fail_count<<24);
                if(br_seen) {
                    hit=line.output.barcode_ok_count!=br_ok;
                    miss=line.output.barcode_fail_count!=br_fail;
                }
                if(!br_seen || signature!=br_signature) br_change_ms=now.lo;
                br_signature=signature;
                br_ok=line.output.barcode_ok_count;br_fail=line.output.barcode_fail_count;br_seen=TRUE;
                full=line.output.barcode_result==S3_BARCODE_RESULT_READING &&
                    line.output.barcode_elements>=BOARD_BARCODE_RETRY_REVERSE_ELEMENTS;
                /* End of marker by distance, independent of speed or pauses:
                 * BOARD_BARCODE_END_GAP_MM of travel with no new GP28 edge
                 * ends the read now. Reaching 29 elements does not: a noise
                 * edge can make the count early, before the first bar. */
                if(reading) {
                    /* Stopped (or stopping) past the marker after the reverse
                     * read: whatever is still being read is over. */
                    BOOL past=br_state==BR_WAIT || (br_state==BR_BRAKE && br_next==BR_WAIT);
                    if(br_end_result!=S3_BARCODE_RESULT_READING ||
                       line.output.barcode_elements!=br_end_elements) {
                        br_end_elements=line.output.barcode_elements;
                        br_end_um=odom->distance_um;
                    }
                    /* Repeat every tick until S3 finishes: one request can be
                     * refused (busy lock) or cancelled by a late edge. */
                    if(past || (input.odom_valid &&
                        br_abs_um(odom->distance_um,br_end_um)>=BOARD_BARCODE_END_GAP_MM*1000U))
                        (void)s3_controller_barcode_end();
                }
                br_end_result=line.output.barcode_result;
            }
            BOOL quiet=now.lo-br_change_ms>=BOARD_BARCODE_RETRY_IDLE_MS;
            unsigned moved=br_abs_um(odom->distance_um,br_origin_um);
            unsigned previous=br_state;
            if(!state.vehicle.armed || state.vehicle.phase!=VC_TRACK) {br_state=BR_IDLE;br_cooldown=FALSE;}
            else if(br_state!=BR_IDLE && now.lo-br_ms>=BOARD_BARCODE_RETRY_STATE_MS) br_state=BR_IDLE;
            else switch(br_state) {
            case BR_IDLE:
                /* After a sequence, the same marker may still report late;
                 * never start another re-read until it is well behind us. */
                if(br_cooldown && moved>=BOARD_BARCODE_RETRY_COOLDOWN_MM*1000U) br_cooldown=FALSE;
                if(miss && !hit && !br_cooldown) {br_state=BR_BRAKE;br_next=BR_REVERSE;}
                break;
            case BR_BRAKE:
                if(now.lo-br_ms>=BOARD_BARCODE_RETRY_BRAKE_MS) br_state=br_next;
                break;
            case BR_REVERSE: { /* run-up rule: vehicle_barcode_retry.h */
                br_reverse_input_t in={.hit=hit,.miss=miss,.full=full,.reading=reading,
                    .odom_valid=input.odom_valid,.min_done=now.lo-br_ms>=BOARD_BARCODE_RETRY_REVERSE_MIN_MS,
                    .quiet=quiet,.moved_um=moved,.since_edge_um=br_abs_um(odom->distance_um,br_end_um)};
                unsigned next=br_reverse_next(&in,&br_full,&br_rev_miss,BOARD_BARCODE_END_GAP_MM,
                    BOARD_BARCODE_RETRY_RUNUP_MM,BOARD_BARCODE_RETRY_REVERSE_MAX_MM);
                if(next!=BR_NEXT_STAY) {
                    br_back_um=moved;br_state=BR_BRAKE;
                    br_next=next==BR_NEXT_RESUME?BR_RESUME:next==BR_NEXT_WAIT?BR_WAIT:BR_SLOW;
                }
                break;
            }
            case BR_WAIT: /* stopped past the marker until S3 reports the reverse read */
                if(hit) br_state=BR_RESUME;
                else if(miss || quiet) br_state=BR_SLOW;
                break;
            case BR_RESUME: /* already read: drive past it without reacting */
                if(moved>=br_back_um) br_state=BR_IDLE;
                break;
            case BR_SLOW:
                if(hit || miss || moved>=br_back_um+BOARD_BARCODE_RETRY_SLOW_EXTRA_MM*1000U)
                    br_state=BR_IDLE;
                break;
            default: br_state=BR_IDLE;break;
            }
            if(br_state!=previous) {
                br_ms=now.lo;br_change_ms=now.lo;
                /* WAIT does not move, so RESUME/SLOW still measure from the reverse end. */
                if(previous!=BR_WAIT) br_origin_um=odom->distance_um;
                if(br_state==BR_IDLE && state.vehicle.armed && state.vehicle.phase==VC_TRACK) br_cooldown=TRUE;
                if(previous==BR_IDLE)
                    br_hold_yaw=br_yaw_seeded?br_yaw_ref:odom_turn_mdeg(odom);
                if(br_state==BR_REVERSE) {br_full=false;br_rev_miss=false;}
                /* Braking/reversing may have started and aborted S5 scans (each
                 * abort spends one of its motion restarts). Re-send MONITOR
                 * before driving forward again so the next real obstacle gets
                 * the full restart budget. */
                if(br_state==BR_RESUME || br_state==BR_SLOW || br_state==BR_IDLE)
                    requested_sensor_mode=FALSE;
            }
            /* Learn the heading only while tracking forward normally. */
            if(!state.vehicle.armed || state.vehicle.phase!=VC_TRACK || !input.odom_valid)
                br_yaw_seeded=FALSE;
            else if(br_state==BR_IDLE && previous==BR_IDLE &&
                    state.vehicle.command==S2_MOVE_FORWARD) {
                br_yaw_ref=reverse_track_yaw_ref(br_yaw_ref,odom_turn_mdeg(odom),
                    br_yaw_seeded,BOARD_BARCODE_RETRY_YAW_REF_DIV);
                br_yaw_seeded=TRUE;
            }
            /* Hold a turn decoded while backing up until the robot is back
             * where the normal-speed read would have reported it. */
            input.nav_hold=br_state==BR_BRAKE || br_state==BR_REVERSE ||
                br_state==BR_WAIT || br_state==BR_RESUME;
            /* Stopped or reversing away from anything ahead: a forward sonar
             * glitch must not abort the retry (TRACK->ACQUIRE resets it). */
            if(br_state==BR_BRAKE || br_state==BR_REVERSE || br_state==BR_WAIT) input.obstacle=0;
        }
#endif
        /* Dashboard shows the last decoded letter A..Z, mapped to a route or not. */
        if(li && line.output.last_symbol>='A' && line.output.last_symbol<='Z')
            state.barcode=(uint8_t)(line.output.last_symbol-'A'+1);
        vc_step(&state.vehicle,&input);
        /* A barcode is being read (not just a single crossing line): drive
         * straight across it, since the course's time-based cross/corner
         * limits would misjudge its bars at the slow reading speed. */
        BOOL marker_reading=li && line.configured &&
            line.output.barcode_result==S3_BARCODE_RESULT_READING && line.output.barcode_elements>=3U;
        (void)marker_reading; /* unused without straddle or the slow cap */
#if BOARD_LINE_STRADDLE
        if(!state.vehicle.armed || state.vehicle.phase!=VC_TRACK || state.vehicle.nav_offset || marker_reading)
            ir_line_course_reset(&course);
#endif
        s3_mode_t wanted=!state.vehicle.armed?S3_MODE_PAUSED:
            state.vehicle.phase==VC_TRACK?S3_MODE_TRACK:S3_MODE_OVERRIDE;
        ER control=E_OK;
        BOOL acknowledge_straight=state.vehicle.navigation_ack &&
            state.vehicle.armed && state.vehicle.phase==VC_TRACK;
        if(wanted!=mode || acknowledge_straight) {
            /* Keep S3 holding its decoded marker while that turn waits for its
             * junction (e.g. back in TRACK after an obstacle): a second marker
             * read meanwhile would be a conflicting turn. */
            control=s3_controller_control(wanted,wanted==S3_MODE_TRACK && !state.vehicle.pending_nav,
                wanted==S3_MODE_TRACK?100:0);
            if(control>=E_OK) {
                mode=wanted;
                if(wanted==S3_MODE_TRACK)vc_track_requested(&state.vehicle,now.lo);
            }
        } else if(wanted==S3_MODE_TRACK)control=s3_controller_renew(100);
        if(control<0)vc_stop(&state.vehicle,VC_TRANSPORT);
        /* Stop for actual pans/checks, not every forward-only phase label. */
        unsigned phase=state.vehicle.phase, look=state.vehicle.look_side;
        if(phase!=previous_phase || look!=previous_look || !requested_sensor_mode) {
            s5_state_t sensor_mode=S5_MONITOR;int angle=0;
            /* BACKUP keeps the latched plan: never MONITOR, which resets it. */
            if(phase==VC_BACKUP || phase==VC_TURN_OUT || phase==VC_DIAGONAL || phase==VC_ALIGN)sensor_mode=S5_APPROACH;
            else if(phase==VC_PASS) {sensor_mode=S5_OVERCOME;angle=look?side_angle:0;}
            else if(phase==VC_TURN_IN || phase==VC_RETURN || phase==VC_RESTORE)sensor_mode=S5_REORG;
            BOOL pause=vehicle_sensor_pan_requires_stop(state.vehicle.command,so,
                scan.angle_deg,scan.sample_angle_deg,angle);
            if(pause)s2_emergency_stop();
            /* Acquisition runs entirely in S5; do not reset its search. */
            if(phase!=VC_ACQUIRE && so) {
                ER er=s5_navigation_set(scan.scan_id,sensor_mode,angle);
                if(er<0 && state.vehicle.armed)vc_stop(&state.vehicle,VC_TRANSPORT);
            }
            requested_sensor_mode=TRUE;previous_phase=phase;previous_look=look;
            if(pause || !state.vehicle.armed){state.vehicle.command=0;state.vehicle.power=0;}
        }
#if BOARD_BARCODE_RETRY_DRIVE
        /* Any fault or obstacle leaves VC_TRACK (or disarms), which ends the
         * sequence; within it, the policy's sonar STOP only gates forward motion. */
        BOOL br_drive=state.vehicle.armed && state.vehicle.phase==VC_TRACK && br_state!=BR_IDLE;
        unsigned br_slow=state.vehicle.maximum_power*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U;
        if(br_slow<BOARD_BARCODE_RETRY_MIN_POWER)
            br_slow=state.vehicle.maximum_power<BOARD_BARCODE_RETRY_MIN_POWER?
                state.vehicle.maximum_power:BOARD_BARCODE_RETRY_MIN_POWER;
        if(br_drive && br_state==BR_SLOW && state.vehicle.command==S2_MOVE_FORWARD)
            state.vehicle.power=(uint8_t)br_slow; /* follow at 30% speed */
#if BOARD_BARCODE_SLOW_WHILE_READING
        /* First pass: a marker started at full speed is read on at the slow
         * speed (~0.5 mm/sample). Not in RESUME, which must cross the
         * already-read marker at normal speed. */
        if(state.vehicle.armed && state.vehicle.phase==VC_TRACK && br_state==BR_IDLE &&
           state.vehicle.command==S2_MOVE_FORWARD && marker_reading && state.vehicle.power>br_slow)
            state.vehicle.power=(uint8_t)br_slow;
#endif
        if(br_drive && (br_state==BR_BRAKE || br_state==BR_REVERSE || br_state==BR_WAIT)) {
#if BOARD_LINE_STRADDLE
            ir_line_course_reset(&course);
#endif
            line_curve=0;
            if(br_state!=BR_REVERSE) s2_emergency_stop();
            else {
#if BOARD_PID_ENABLED
                /* Heading hold plus an inverted line term; the sensors trail. */
                int32_t yaw=input.odom_valid?odom_turn_mdeg(odom):br_hold_yaw;
                int32_t steer=reverse_track_steering(yaw,br_hold_yaw,input.line_black_mask,
                    BOARD_LINE_STRADDLE && input.line_fresh,
                    BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN,BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN,
                    BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT);
                ER er=s2_reverse_follow_send((UINT)br_slow,(INT)steer);
#else
                (void)br_hold_yaw; /* heading hold needs PID */
                ER er=s2_movement_send(S2_MOVE_REVERSE,(UINT)br_slow);
#endif
                /* E_OBJ: motor still acknowledging the brake; retry next tick. */
                if(er<0 && er!=E_OBJ) {s2_emergency_stop();vc_stop(&state.vehicle,VC_TRANSPORT);}
            }
        } else
#endif
        if(state.vehicle.command==S2_MOVE_STOP) {
            /* An epoch per stopped tick invalidates every old queued movement. */
            s2_emergency_stop();
        } else {
            ER er;
#if BOARD_LINE_STRADDLE
            /* Junction offset before a decoded turn: straight across the
             * crossing line, no course pivots or smooth steering. */
            BOOL junction_offset=state.vehicle.armed && state.vehicle.phase==VC_TRACK &&
                state.vehicle.command==S2_MOVE_FORWARD && (state.vehicle.nav_offset || marker_reading);
            BOOL course_on=state.vehicle.armed && state.vehicle.phase==VC_TRACK &&
                state.vehicle.command==S2_MOVE_FORWARD && input.line_fresh && !junction_offset;
            unsigned direction=ir_line_course_step(&course,&straddle,now.lo,course_on);
            if(course_on && course.fault) {
                s2_emergency_stop();vc_stop(&state.vehicle,VC_LINE);er=E_OK;
            } else if(course_on && (direction==IR_LINE_LEFT || direction==IR_LINE_RIGHT)) {
                /* Side touches steer while the course is tracking; corner
                 * holds and long touches (tight bends) still pivot. */
                int32_t steer=LINE_STEER_PIVOT;
                if(BOARD_PID_ENABLED && BOARD_LINE_SMOOTH_STEER && course.phase==IR_COURSE_TRACK)
                    steer=line_steer_amount(now.lo-course.side_since_ms,BOARD_LINE_STEER_MIN,
                        BOARD_LINE_STEER_MAX,BOARD_LINE_STEER_RAMP_MS,BOARD_LINE_PIVOT_AFTER_MS);
                if(steer==LINE_STEER_PIVOT) {
                    line_curve=0;
                    er=s2_movement_send((s2_move_command_t)direction,state.vehicle.power);
                } else {
                    line_curve=(INT)(direction==IR_LINE_RIGHT?steer:-steer);
                    er=s2_follow_send(state.vehicle.power,line_curve);
                }
            } else if((course_on || junction_offset) && BOARD_PID_ENABLED) {
                line_curve=0;er=s2_follow_send(state.vehicle.power,0);
            } else
#endif
            if(state.vehicle.phase==VC_BACKUP && state.vehicle.command==S2_MOVE_REVERSE) {
                /* Heading hold only (no line term) on the yaw latched at ACQUIRE. */
                line_curve=0;
                if(BOARD_PID_ENABLED) {
                    int32_t yaw=input.odom_valid?odom_turn_mdeg(odom):state.vehicle.origin_yaw;
                    int32_t steer=reverse_track_steering(yaw,state.vehicle.origin_yaw,0U,false,
                        BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN,BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN,
                        BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT);
                    er=s2_reverse_follow_send(state.vehicle.power,(INT)steer);
                } else er=s2_movement_send(S2_MOVE_REVERSE,state.vehicle.power);
            } else
            if(!BOARD_LINE_STRADDLE && BOARD_PID_ENABLED && state.vehicle.phase==VC_TRACK &&
               state.vehicle.command==S2_MOVE_FORWARD && input.line_fresh) {
                er=s2_follow_send(state.vehicle.power,line_curve);
            } else if (BOARD_PID_ENABLED && state.vehicle.command==S2_MOVE_FORWARD &&
                       state.vehicle.phase>=VC_TURN_OUT && state.vehicle.phase<=VC_RESTORE) {
                line_curve=state.vehicle.steering;
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
