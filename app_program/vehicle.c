#include "app_state.h"
#include "robot_config.h"
#include "vehicle_motion_handoff.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"
#include "subsystem2/odometry.h"
#include "subsystem3/subsystem3.h"
#include "subsystem4/subsystem4.h"
#include "subsystem4/encoder_input.h"
#include "subsystem4/imu_mount.h"
#include "vehicle_obstacle_confirm.h"
#include "vehicle_ramp.h"
#include "vehicle_edge_trace.h"
#include <math.h>
#include "subsystem2/s2_data.h"
#include "subsystem5/subsystem5.h"
#include "subsystem5/s5_width.h"
#include "vehicle_reverse_track.h"
#include "ir_line_trace.h"
#include "robot_ir_config.h"
#include "vehicle_line_steer.h"
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
/* 2026-11-10: 300 ms faulted on ordinary fore-aft rocking (stop-and-go from
 * sonar dropouts, braking); a robot that has really tipped or been lifted
 * stays without tilt far longer. */
#define VC_TILT_GRACE_MS 800U
#endif
/* Bypass heading re-planned after the reverse. S5 plans from where the robot
 * stopped; reversing moves the obstacle edge backup_mm further ahead, so a
 * shallower, more diagonal heading clears it. Same tests as S5's planner
 * (s5_policy.c): after the turn-out arc the heading line passes the edge
 * (lateral x, ahead y from the axle) by S5_CLEARANCE_RADIUS_MM, and the
 * diagonal leg reaches the planned lateral offset after both arcs. FALSE
 * keeps the plan (no shallower heading works). */
static BOOL bypass_replan(UW x,UW y,UW lateral,UW backup_mm,unsigned planned,unsigned *deg,UW *leg) {
    for(unsigned t=AVOID_MIN_TURN_DEG;t<planned && t<90U;++t) {
        long long clear=(long long)(y+backup_mm)*s5_cosine_q10000[90U-t]-(long long)x*s5_cosine_q10000[t]-
            (long long)S5_BYPASS_ARC_RADIUS_MM*(10000LL-(long long)s5_cosine_q10000[t]);
        if(clear<(long long)S5_CLEARANCE_RADIUS_MM*10000LL) continue;
        UW arc=2U*S5_BYPASS_ARC_RADIUS_MM*(10000U-s5_cosine_q10000[t])/10000U;
        UW remaining=lateral>arc?lateral-arc:0U,sine=s5_cosine_q10000[90U-t];
        UW l=(remaining*10000U+sine-1U)/sine;
        if(l<50U) l=50U;
        if(l>1000U) return FALSE;
        *deg=t;*leg=l;return TRUE;
    }
    return FALSE;
}
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
    int32_t avoid_power_m=0,avoid_steer=0; /* smoothed avoidance power (milli-%) and steering */
    uint32_t next_tick=0;unsigned previous_phase=VC_DISARMED,previous_look=0;
    BOOL requested_sensor_mode=FALSE;
    uint8_t shown_ok=0;BOOL shown_seen=FALSE; /* last decode count shown on the dashboard */
    uint32_t tilt_ok_ms=0;BOOL tilt_seen=FALSE; /* last valid gravity tilt */
    vehicle_obstacle_confirm_t obstacle_confirm={0};
#if BOARD_RAMP_SPEED
    vehicle_accel_filter_t accel_filter={0};vehicle_ramp_t ramp={0};
#endif
    BOOL lift_timing=FALSE;uint32_t lift_since=0; /* robot lifted (no surface under the IRs) */
    int32_t prev_yaw=0; /* yaw at the last follower tick: edge tracing's lost-line turn */
    edge_trace_t edge={0};int32_t edge_lost_turn=0; /* edge tracing; turning while lost */
    BOOL shock_prev=FALSE; /* S4 shock in the previous tick (rising-edge count) */
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
    /* Straddle line observer (ir_line_trace.c). S3 still owns ADC acquisition;
     * this only interprets it. */
    s3_config_t straddle_cfg;ir_line_trace_t straddle={0};
    s3_config_defaults(&straddle_cfg);robot_ir_apply_layout(&straddle_cfg);
    straddle_cfg.line.threshold_high=BOARD_LINE_BLACK_PERMILLE;
    straddle_cfg.line.threshold_low=BOARD_LINE_WHITE_PERMILLE;
    if(!robot_ir_apply_adc_calibration(&straddle_cfg)) {s2_emergency_stop();tk_ext_tsk();return;}
    uint8_t read_side=0;uint32_t read_side_ms=0; /* barcode-read steering: side and since */
    s3_status_t line_last={0};uint8_t line_misses=0; /* last S3 snapshot, failed reads since */
    s5_status_t scan_last={0};uint8_t scan_misses=0; /* last S5 snapshot, failed reads since */
    /* Bypass edge latched with the plan: lateral, ahead (axle frame), target lateral offset; odometry then. */
    UW bypass_x=0,bypass_y=0,bypass_lateral=0;int32_t bypass_from_um=0;BOOL bypass_edge=FALSE;
    for(;;) {
        SYSTIM now;
        /* Telemetry view of the follower this tick: 0 straight, 1/2 steer
         * right/left, 3/4 pivot right/left; steer in S2 units. */
        uint8_t follow_code=0;int16_t follow_steer=0;
        odom_config_t calibration={0}; /* short-lived calculation/API view */
        (void)odom_calibration_unpack(&calibration_record,&calibration);
        s2_motion_status_t motor={0};uint8_t encoder_payload[ENCODER_SNAPSHOT_SIZE]={0};s3_status_t line={0};s5_status_t scan={0};s4_sensor_status_t imu={0};
        BOOL mo=s2_motion_get_status(&motor)==E_OK;
        BOOL en=s2_encoder_snapshot(encoder_payload,sizeof(encoder_payload))==E_OK;
        BOOL li=s3_status_get(&line)==E_OK;
        /* S3's status lock is polled, never waited on: a read fails when the
         * sampler was preempted while publishing (e.g. by network work). Use
         * the last snapshot for up to 3 such ticks; its sample age still
         * decides freshness, so stale line data still stops the run. */
        if(li) {line_last=line;line_misses=0;}
        else if(line_misses<3U && line_last.configured) {line=line_last;li=TRUE;++line_misses;}
        BOOL so=s5_get_status(&scan)==E_OK;
        /* S5's status lock waits only 5 ms: on a miss use the last snapshot
         * for up to 10 ticks; its sample time still decides freshness. */
        if(so) {scan_last=scan;scan_misses=0;}
        else if(scan_misses<10U && scan_last.time_ms) {scan=scan_last;so=TRUE;++scan_misses;}
        s4_sensor_cache_t imu_record={0};
        BOOL im=s4_sensor_get_record(&imu_record)==E_OK;
        /* Snapshot APIs may wait briefly. Age them against a time obtained
         * afterwards, so a newly published sample cannot appear in the future. */
        if(tk_get_otm(&now)<0) {vc_stop(&state.vehicle,VC_TRANSPORT);s2_emergency_stop();tk_dly_tsk(10);continue;}
        if(im)s4_data_snapshot(&imu_record,s4_operating_ms_to_us(((uint64_t)(UW)now.hi<<32)|now.lo),&imu);
        state.time_ms=now.lo;++state.sequence;
        int32_t accel[3];
        s4_imu_to_robot(imu.latest.accel.milli_units,accel,
            BOARD_IMU_FRONT_AXIS,BOARD_IMU_LEFT_AXIS,BOARD_IMU_UP_AXIS);
#if BOARD_RAMP_SPEED
        vehicle_accel_filter(&accel_filter,accel,imu.latest.accel.read_time_us,
            im && imu.accel_usable && !imu.latest.accel.overrun,BOARD_IMU_SMOOTH_MS,
            BOARD_IMU_SMOOTH_MIN_MG,BOARD_IMU_SMOOTH_MAX_MG,accel);
#endif
        s4_encoder_terrain_step(&encoder_input,&terrain,&calibration,now.lo,
            encoder_payload,sizeof(encoder_payload),en,imu.latest.accel.read_time_us,
            accel,im && imu.accel_usable && !imu.latest.accel.overrun,BOARD_TERRAIN_CALIBRATED,
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
            if(c.wire[1]==4) { /* MAP letter->turn, any phase: only future decodes change */
                s1_command_ack(c.wire[5],4,s3_controller_set_map(c.wire[2],c.wire[3])<E_OK?1U:0U,
                    state.vehicle.phase,state.vehicle.fault);continue;
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
        vc_input_t input={.now_ms=now.lo,
            .distance_um=odom->distance_um,.yaw_mdeg=odom_turn_mdeg(odom),
            .turn_deg=plan_usable?(int16_t)scan.planned_rotate_deg:0,
            .approach_mm=plan_usable?(uint16_t)scan.planned_approach_mm:0,
            .return_mm=plan_usable?(uint16_t)scan.planned_return_mm:0,
            .odom_valid=result_ok && (result.flags&S4_RESULT_ODOMETRY),.motor_ok=mo && motor.state!=S2_MOTION_FAULT,
            .terrain_valid=result_ok && (result.flags&S4_RESULT_TILT),.shock=(result.flags&S4_RESULT_SHOCK)!=0U,
            .slope=result.terrain==TERRAIN_ASCENT || result.terrain==TERRAIN_DESCENT,
            .bypass_steering=(uint16_t)(BOARD_TRACK_WIDTH_MM*1000U/(2U*S5_BYPASS_ARC_RADIUS_MM)),
            .bypass_enabled=BOARD_PID_ENABLED && BOARD_BYPASS_VALIDATED,.transport_ok=1};
        if(input.shock && !shock_prev)++state.shock_count;
        shock_prev=input.shock;
#if !BOARD_SHOCK_STOP
        /* Shock stays in the S4 result (dashboard impact) but does not stop
         * the run; the shock sample is then a tilt gap like any other. */
        input.shock=0;
#endif
        /* The GY-511 has no gyro: tilt is derived from gravity alone, so the
         * lateral jolt when a pivot starts briefly reads as >15 deg roll and
         * terrain drops tilt. Tolerate gaps shorter than VC_TILT_GRACE_MS;
         * a sustained loss still faults VC_SENSOR. With BOARD_SHOCK_STOP a
         * shock sample is never masked. */
        if(input.terrain_valid) {tilt_ok_ms=now.lo;tilt_seen=TRUE;}
        else if(tilt_seen && !input.shock && now.lo-tilt_ok_ms<VC_TILT_GRACE_MS) input.terrain_valid=1;
#if BOARD_BARCODE_RETRY_DRIVE
        /* The barcode re-read brakes, reverses slowly and waits in a straight
         * line. As for VC_BACKUP, its rocking and jolts must not fault it. */
        if(state.vehicle.armed && state.vehicle.phase==VC_TRACK && !input.shock &&
           (br_state==BR_BRAKE || br_state==BR_REVERSE || br_state==BR_WAIT)) input.terrain_valid=1;
#endif
        input.line_fresh=li && line.configured && line.acquisition_error==E_OK && now.lo-line.sample_time_ms<30U;
        input.sensor_fresh=so && scan.state!=S5_FAULT &&
            (!scan.valid || (scan.distance_mm>0 && scan.distance_mm<=5000U)) &&
            now.lo-scan.time_ms<(scan.valid?S5_STALE_MS:S5_INVALID_STALE_MS);
        /* An obstacle needs S5_ACQUISITION_SAMPLES consecutive new forward
         * echoes within S5_OBSTACLE_MM (S5 starts its scan on the same
         * count): one stray echo neither starts an acquisition nor stops the
         * line run. A no-echo retry is no reading (neither counts nor
         * resets); a far echo, open space (three no-echoes) or an off-axis
         * echo ends the streak. While tracking the robot also holds for one
         * echo within the 5 cm guidance distance, for an invalidated sample
         * (S5 re-centring: no usable distance) and for a no-echo right after
         * a near echo; a no-echo with no near echo before it does not stop
         * it. Avoidance phases keep the single-sample checks. */
        BOOL forward=scan.angle_deg==0 && scan.sample_angle_deg==0;
        BOOL open=forward && s5_sample_is_open(&scan);
        BOOL no_echo=!scan.valid && (scan.echo_result==S5_ECHO_NO_RISE || scan.echo_result==S5_ECHO_NO_FALL);
        BOOL obstacle_confirmed=vehicle_obstacle_confirm_step(&obstacle_confirm,so && (scan.valid || open),
            scan.time_ms,forward && scan.valid && scan.distance_mm<=S5_OBSTACLE_MM,S5_ACQUISITION_SAMPLES);
        /* Avoidance arcs (TURN_OUT..RESTORE): with the servo straight ahead,
         * only a real echo within the guidance distance holds the robot; a
         * missing echo or the re-sample after a pan does not, so the arcs do
         * not stop and restart on every gap. A side look still waits for the
         * servo to come back to the front before moving. */
        BOOL avoid_arc=state.vehicle.phase>=VC_TURN_OUT && state.vehicle.phase<=VC_RESTORE;
        input.front_clear=input.sensor_fresh && scan.angle_deg==0 &&
            (state.vehicle.phase==VC_TRACK ?
                forward && !obstacle_confirmed &&
                ((scan.valid && scan.distance_mm>S5_GUIDANCE_OBSTACLE_MM) || open ||
                 (no_echo && !obstacle_confirm.streak)) :
             avoid_arc ?
                !(scan.valid && scan.sample_angle_deg==0 && scan.distance_mm<=S5_GUIDANCE_OBSTACLE_MM) :
                s5_direction_clear(&scan,state.vehicle.phase==VC_ACQUIRE?S5_OBSTACLE_MM:S5_GUIDANCE_OBSTACLE_MM));
        input.obstacle=input.sensor_fresh && (obstacle_confirmed ||
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
        for(unsigned n=0;n<S3_EVENT_CAPACITY;++n) {
            s3_event_record_t e;ER er=s3_event_receive(&e);if(er==E_TMOUT)break;
            if(er<0) {input.transport_ok=0;break;}
            if(e.event.kind==S3_EVENT_NAVIGATION) {
                input.navigation_event=1;input.nav=(uint8_t)e.event.command;
                state.navigation=(uint8_t)e.event.command;state.barcode=(e.event.symbol>='A' && e.event.symbol<='Z')?(uint8_t)(e.event.symbol-'A'+1):0;
            } else if(e.event.kind!=S3_EVENT_RECOVERY_REQUEST) input.safety_event=1; /* straddle S3 never sends recovery */
        }
        /* The straddle follower does not use S3's line verdicts, but a latched
         * S3 transport error means its events (decoded turns) are being lost
         * and barcode capture has stopped: fault visibly instead of driving on. */
        if(state.vehicle.armed && li && line.configured && line.output.error==S3_ERROR_TRANSPORT)
            input.transport_ok=0;
        /* Straddle geometry: both sensors WHITE is centred, so S3's edge-follow
         * SEARCHING/LOST/recovery semantics do not apply. Feed the policy a
         * steerable line whenever the straddle observer has a fresh sample. */
        BOOL straddle_ok=ir_line_observe(&straddle,&straddle_cfg.line,line.sample.raw,
            line.sample.valid_mask,line.sample_time_ms,now.lo,
            li && line.configured && line.acquisition_error==E_OK);
        input.line_fresh=input.line_fresh && straddle_ok;
        input.steering=input.line_fresh;input.line_detected=input.line_fresh;
        input.line_black_mask=(uint8_t)(straddle.result.black_mask&3U);
        /* A crossing line puts both straddle sensors and GP28 on black: decoded
         * turns wait for it. */
        input.junction=input.line_fresh && input.line_black_mask==3U && line.output.barcode_black;
        input.nav_offset_mm=(uint16_t)BOARD_NAV_JUNCTION_OFFSET_MM;
        input.nav_search_mm=(uint16_t)BOARD_NAV_JUNCTION_SEARCH_MM;
        input.uturn_ignore_mm=(uint16_t)BOARD_NAV_UTURN_IGNORE_MM;
        /* The axle is over the junction and the sensors 90 mm ahead: past the
         * min angle the first straddle touch is the new line. */
        input.nav_turn_min_deg=(uint8_t)BOARD_NAV_TURN_MIN_DEG;
        input.nav_turn_max_deg=(uint8_t)BOARD_NAV_TURN_MAX_DEG;
        input.nav_uturn_min_deg=(uint8_t)BOARD_NAV_UTURN_MIN_DEG;
        input.nav_uturn_max_deg=(uint8_t)BOARD_NAV_UTURN_MAX_DEG;
        input.nav_turn_power=(uint8_t)BOARD_NAV_TURN_POWER;
        input.nav_seek_power=(uint8_t)BOARD_NAV_TURN_SEEK_POWER;
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
        /* Dashboard shows the last decoded letter A..Z, mapped to a route or not.
         * A decode with no turn (unmapped or set to Ignore) clears the shown
         * turn, so it is not paired with the previous letter's turn. */
        if(li && line.configured) {
            if(shown_seen && line.output.barcode_ok_count!=shown_ok && !input.navigation_event)
                state.navigation=0;
            shown_ok=line.output.barcode_ok_count;shown_seen=TRUE;
        }
        if(li && line.output.last_symbol>='A' && line.output.last_symbol<='Z')
            state.barcode=(uint8_t)(line.output.last_symbol-'A'+1);
        /* Lifted (no surface under the line IRs, GP28 black): in the hand the
         * tilt reading means nothing, so it never faults; BOARD_LIFT_FAULT_MS
         * of it while running stops with VC_LIFTED instead (a shorter lift
         * does not fault). */
        {
            BOOL lifted=BOARD_LIFT_FAULT_MS && li && line.configured && line.sample.raw[0]>=BOARD_LIFT_RAW &&
                line.sample.raw[1]>=BOARD_LIFT_RAW && line.output.barcode_black;
            if(!lifted) lift_timing=FALSE;
            else if(!lift_timing) {lift_timing=TRUE;lift_since=now.lo;}
            if(lifted) input.terrain_valid=1;
        }
        unsigned phase_before=state.vehicle.phase;
        vc_step(&state.vehicle,&input);
        if(phase_before==VC_ACQUIRE && state.vehicle.phase==VC_BACKUP) {
            /* Plan latched: keep its edge for the re-plan after the reverse. */
            BOOL left=scan.planned_rotate_deg>0;
            INT bound=left?scan.left_bound_deg:-scan.right_bound_deg;
            UW range=left?scan.left_bound_distance_mm:scan.right_bound_distance_mm;
            bypass_edge=bound>=0 && bound<=80 && range && range<=5000U;
            if(bypass_edge) {
                UW half=(scan.object_width_max_mm+1U)/2U;
                bypass_x=range*s5_cosine_q10000[90-bound]/10000U;
                bypass_y=range*s5_cosine_q10000[bound]/10000U+S5_SENSOR_OFFSET_MM;
                bypass_lateral=(bypass_x>half?bypass_x:half)+S5_CLEARANCE_RADIUS_MM;
                bypass_from_um=odom->distance_um;
            }
        } else if(phase_before==VC_BACKUP && state.vehicle.phase==VC_TURN_OUT && bypass_edge) {
            /* Reverse done: a shallower (diagonal) heading now clears the edge. */
            int32_t back_um=bypass_from_um-odom->distance_um;
            UW back=back_um>0?(UW)(back_um/1000):0U;
            int planned=state.vehicle.turn_deg<0?-state.vehicle.turn_deg:state.vehicle.turn_deg;
            unsigned deg=0;UW leg=0;
            if(bypass_replan(bypass_x,bypass_y,bypass_lateral,back,(unsigned)planned,&deg,&leg)) {
                tm_printf((UB *)"[BYPASS] reversed %u mm: heading %d -> %u deg, diagonal %u -> %u mm\n",
                    (unsigned)back,planned,deg,(unsigned)state.vehicle.approach_mm,(unsigned)leg);
                state.vehicle.turn_deg=(int16_t)(state.vehicle.turn_deg<0?-(int)deg:(int)deg);
                state.vehicle.approach_mm=state.vehicle.return_mm=(uint16_t)leg;
            }
            bypass_edge=FALSE;
        }
        if(BOARD_LIFT_FAULT_MS && state.vehicle.armed && lift_timing &&
           now.lo-lift_since>=BOARD_LIFT_FAULT_MS) {
            s2_emergency_stop();vc_stop(&state.vehicle,VC_LIFTED);
        }
        /* A barcode is being read (>= 3 elements): the follower is off;
         * line_marker_steer holds the line (BOARD_BARCODE_READ_LINE_STEER). */
        BOOL marker_reading=li && line.configured &&
            line.output.barcode_result==S3_BARCODE_RESULT_READING && line.output.barcode_elements>=3U;
        if(!marker_reading) read_side=0;
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
            if(phase==VC_BACKUP || phase==VC_TURN_OUT || phase==VC_DIAGONAL || phase==VC_ALIGN) {
                /* The end of the diagonal looks to the obstacle side first. */
                sensor_mode=S5_APPROACH;angle=phase==VC_DIAGONAL && look?side_angle:0;
            }
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
#if BOARD_RAMP_SPEED
        {
            int32_t p=terrain_pitch_cdeg(&terrain),r=terrain_roll_cdeg(&terrain);
            unsigned rs=vehicle_ramp_step(&ramp,now.lo,state.vehicle.armed,terrain.tilt_valid,
                (int32_t)lround(sqrt((double)p*p+(double)r*r)),p,r,BOARD_RAMP_TILT_CDEG,BOARD_RAMP_LEVEL_CDEG,
                BOARD_RAMP_CREST_DROP_CDEG,BOARD_RAMP_HOLD_MS,BOARD_RAMP_LEVEL_MS);
            if(rs==RAMP_DONE) {
                /* Level again: the bench test stops; a mission resumes normal
                 * speed and watches for the next ramp. */
                if(BOARD_RAMP_STOP_AFTER && state.vehicle.armed) {s2_emergency_stop();vc_stop(&state.vehicle,VC_OK);}
                ramp=(vehicle_ramp_t){0};
            } else if(state.vehicle.phase==VC_TRACK && state.vehicle.command==S2_MOVE_FORWARD) {
                unsigned cap=rs==RAMP_UP?BOARD_RAMP_UP_POWER:rs==RAMP_DOWN?BOARD_RAMP_DOWN_POWER:0U;
                if(cap && state.vehicle.power>cap) state.vehicle.power=(uint8_t)cap;
            }
        }
#endif
#if BOARD_BARCODE_RETRY_DRIVE
        /* Any fault or obstacle leaves VC_TRACK (or disarms), which ends the
         * sequence; within it, the policy's sonar STOP only gates forward motion. */
        BOOL br_drive=state.vehicle.armed && state.vehicle.phase==VC_TRACK && br_state!=BR_IDLE;
        unsigned br_slow=state.vehicle.maximum_power*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U;
        /* The reverse re-read needs more torque than the forward slow read. */
        unsigned br_back=br_slow>BOARD_BARCODE_RETRY_REVERSE_MIN_POWER?br_slow:
            (state.vehicle.maximum_power<BOARD_BARCODE_RETRY_REVERSE_MIN_POWER?
             state.vehicle.maximum_power:BOARD_BARCODE_RETRY_REVERSE_MIN_POWER);
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
            if(br_state!=BR_REVERSE) s2_emergency_stop();
            else {
#if BOARD_PID_ENABLED
                /* Heading hold plus an inverted line term; the sensors trail. */
                int32_t yaw=input.odom_valid?odom_turn_mdeg(odom):br_hold_yaw;
                int32_t steer=reverse_track_steering(yaw,br_hold_yaw,input.line_black_mask,
                    input.line_fresh,
                    BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN,BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN,
                    BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT);
                ER er=s2_reverse_follow_send((UINT)br_back,(INT)steer);
#else
                (void)br_hold_yaw; /* heading hold needs PID */
                ER er=s2_movement_send(S2_MOVE_REVERSE,(UINT)br_back);
#endif
                /* E_OBJ: motor still acknowledging the brake; retry next tick. */
                if(er<0 && er!=E_OBJ) {s2_emergency_stop();vc_stop(&state.vehicle,VC_TRANSPORT);}
            }
        } else
#endif
        if(state.vehicle.command!=S2_MOVE_FORWARD || state.vehicle.phase<VC_TURN_OUT ||
           state.vehicle.phase>VC_RESTORE) {avoid_power_m=0;avoid_steer=0;}
        if(state.vehicle.command==S2_MOVE_STOP) {
            /* An epoch per stopped tick invalidates every old queued movement. */
            s2_emergency_stop();
        } else {
            ER er;
            /* Junction offset before a decoded turn, or a barcode read: follower off. */
            BOOL junction_offset=state.vehicle.armed && state.vehicle.phase==VC_TRACK &&
                state.vehicle.command==S2_MOVE_FORWARD && (state.vehicle.nav_offset || marker_reading);
            BOOL follow_on=state.vehicle.armed && state.vehicle.phase==VC_TRACK &&
                state.vehicle.command==S2_MOVE_FORWARD && input.line_fresh && !junction_offset;
            /* Line follower while tracking: mask-pivot (default) or edge
             * tracing (robot_config.h requires one). */
            BOOL pivot_on=BOARD_LINE_MASK_PIVOT && follow_on;
            BOOL edge_on=!pivot_on && BOARD_LINE_EDGE_TRACE && follow_on;
            if(!edge_on) edge=(edge_trace_t){0};
            /* Heading change this tick (odometry yaw wraps at 360 deg), for
             * edge tracing's lost-line turn. */
            int32_t yaw_now=odom_turn_mdeg(odom),yaw_step=input.odom_valid?yaw_now-prev_yaw:0;
            prev_yaw=yaw_now;
            if(yaw_step>180000) yaw_step-=360000; else if(yaw_step< -180000) yaw_step+=360000;
            if(pivot_on) {
                /* Current mask only: pivot toward a black IR, else forward. */
                unsigned m=straddle.result.black_mask&3U;
                line_curve=0;follow_steer=0;
                if(m==1U || m==2U) {
                    follow_code=m==1U?3U:4U;
                    er=s2_movement_send(m==1U?S2_MOVE_PIVOT_RIGHT:S2_MOVE_PIVOT_LEFT,state.vehicle.power);
                } else {
                    follow_code=0U;
                    er=s2_follow_send(state.vehicle.power,0);
                }
            } else if(edge_on) {
                static const edge_trace_config_t edge_cfg={BOARD_LINE_EDGE_SET,BOARD_LINE_BLACK_PERMILLE,
                    BOARD_LINE_WHITE_PERMILLE,BOARD_LINE_EDGE_DEEP,BOARD_LINE_EDGE_KP_MILLI,
                    BOARD_LINE_EDGE_KD_MILLI,400,BOARD_LINE_EDGE_DEEP_MS,BOARD_LINE_EDGE_RIGHT};
                int32_t dark_r=straddle.result.darkness[0],dark_l=straddle.result.darkness[1];
                int32_t edge_dark=BOARD_LINE_EDGE_RIGHT?dark_r:dark_l,far_dark=BOARD_LINE_EDGE_RIGHT?dark_l:dark_r;
                edge_action_t act=edge_trace_step(&edge,&edge_cfg,now.lo,edge_dark,far_dark);
                /* Both IRs white while steering hard toward the far side: the
                 * line is lost; a full turn of it is a line fault. */
                if(edge_dark<100 && far_dark<100) edge_lost_turn+=yaw_step; else edge_lost_turn=0;
                if(edge_lost_turn>360000 || edge_lost_turn< -360000) {
                    s2_emergency_stop();vc_stop(&state.vehicle,VC_LINE);er=E_OK;
                } else if(act.kind==EDGE_PIVOT_EDGE || act.kind==EDGE_PIVOT_FAR) {
                    BOOL right=(act.kind==EDGE_PIVOT_EDGE)==(BOARD_LINE_EDGE_RIGHT!=0);
                    line_curve=0;follow_code=right?3U:4U;
                    er=s2_movement_send(right?S2_MOVE_PIVOT_RIGHT:S2_MOVE_PIVOT_LEFT,state.vehicle.power);
                } else {
                    int32_t steer=act.kind==EDGE_STRAIGHT?0:act.steer;
                    int32_t mag=steer<0?-steer:steer;
                    UINT power=(UINT)(state.vehicle.power*(1000-mag*BOARD_LINE_EDGE_SLOW_PERMILLE/400)/1000);
                    if(!power) power=1U;
                    line_curve=(INT)steer;follow_steer=(int16_t)steer;
                    follow_code=steer>0?1U:steer<0?2U:0U;
                    er=s2_follow_send(power,line_curve);
                }
            } else if(junction_offset && BOARD_PID_ENABLED) {
                line_curve=0;
                if(marker_reading && !state.vehicle.nav_offset && input.line_fresh) {
                    /* Reading a barcode: the follower is off, but keep the line. */
                    unsigned m=straddle.result.black_mask&3U;
                    if(m==1U || m==2U) {
                        if(read_side!=m) {read_side=(uint8_t)m;read_side_ms=now.lo;}
                        line_curve=(INT)line_marker_steer(m,now.lo-read_side_ms,BOARD_LINE_STEER_MIN,
                            BOARD_BARCODE_READ_LINE_STEER,BOARD_LINE_STEER_RAMP_MS);
                        follow_code=line_curve>0?1U:line_curve<0?2U:0U;follow_steer=(int16_t)line_curve;
                    } else read_side=0;
                }
                er=s2_follow_send(state.vehicle.power,line_curve);
            } else if(state.vehicle.phase==VC_BACKUP && state.vehicle.command==S2_MOVE_REVERSE) {
                /* Heading hold only (no line term) on the yaw latched at ACQUIRE. */
                line_curve=0;
                if(BOARD_PID_ENABLED) {
                    int32_t yaw=input.odom_valid?odom_turn_mdeg(odom):state.vehicle.origin_yaw;
                    int32_t steer=reverse_track_steering(yaw,state.vehicle.origin_yaw,0U,false,
                        BOARD_BARCODE_RETRY_REVERSE_YAW_GAIN,BOARD_BARCODE_RETRY_REVERSE_LINE_GAIN,
                        BOARD_BARCODE_RETRY_REVERSE_STEER_LIMIT);
                    er=s2_reverse_follow_send(state.vehicle.power,(INT)steer);
                } else er=s2_movement_send(S2_MOVE_REVERSE,state.vehicle.power);
            } else if (BOARD_PID_ENABLED && state.vehicle.command==S2_MOVE_FORWARD &&
                       state.vehicle.phase>=VC_TURN_OUT && state.vehicle.phase<=VC_RESTORE) {
                /* Smooth: power ramps up from 0 (decreases are immediate) and
                 * the arc steering slews, per 10 ms tick. */
                int32_t want=(int32_t)state.vehicle.power*1000;
                avoid_power_m+=(int32_t)(AVOID_POWER*1000U*10U/AVOID_RAMP_POWER_MS);
                if(avoid_power_m>want) avoid_power_m=want;
                int32_t ds=state.vehicle.steering-avoid_steer,step=(int32_t)(400U*10U/AVOID_STEER_RAMP_MS);
                if(ds>step) ds=step; else if(ds< -step) ds=-step;
                avoid_steer+=ds;
                UINT power=(UINT)((avoid_power_m+999)/1000);
                line_curve=(INT)avoid_steer;
                er=s2_follow_send(power?power:1U,line_curve);
            } else {
                line_curve=0;
                er=s2_movement_send((s2_move_command_t)state.vehicle.command,state.vehicle.power);
            }
            /* Motor may still be acknowledging the previous STOP. Retry next tick. */
            if(er<0 && er!=E_OBJ) {s2_emergency_stop();vc_stop(&state.vehicle,VC_TRANSPORT);}
        }
        state.line_follow=follow_code;state.line_steer=follow_steer;
        state.line_mask=(uint8_t)(straddle.result.black_mask&3U);
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
