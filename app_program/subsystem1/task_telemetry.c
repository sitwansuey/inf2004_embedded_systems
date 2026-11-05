#include "subsystem1.h"
#include "control.h"
#include "../app_state.h"
#include "../app_logging.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/s2_data.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include <string.h>
static int64_t bounded(int64_t n,int64_t lo,int64_t hi) {return n<lo?lo:n>hi?hi:n;}
void telemetry_task(INT code,void *context) {
    (void)code;(void)context;unsigned count=0;uint32_t previous=0;unsigned old_fault=0;
#if BOARD_BARCODE_DEMO
    uint32_t last_barcode_rejection=0;
#if TM_MQTT
    uint32_t diagnostic_ms=0,diagnostic_result=0,diagnostic_sequence=0;
    unsigned diagnostic_symbol=0,diagnostic_navigation=0,diagnostic_phase=0,diagnostic_fault=0;
    unsigned diagnostic_correction=0;
    static s3_barcode_diagnostics_t diagnostic_cache;
#endif
#endif
    for(;;) {
        /* Retained diagnostics are larger than the old snapshot. Keep this
         * single-task scratch space off the 2KB telemetry task stack. */
        static s3_status_t l;
        memset(&l,0,sizeof(l));
        static app_state_t s;static s2_motion_status_t m;static s2_encoder_status_t e;
        static s4_sensor_status_t a;static s5_status_t o;
        memset(&s,0,sizeof(s));memset(&m,0,sizeof(m));memset(&e,0,sizeof(e));
        memset(&a,0,sizeof(a));memset(&o,0,sizeof(o));
        BOOL st=app_get_state(&s)==E_OK;SYSTIM now;
        if(tk_get_otm(&now)<E_OK) {tk_dly_tsk(10);continue;}
        if(st && (now.lo-previous>=200U || s.vehicle.fault!=old_fault)) {
            BOOL mo=s2_motion_get_status(&m)==E_OK,en=s2_encoder_get_status(&e)==E_OK && e.valid;
            ER ir_copy=s3_status_get(&l);BOOL li=ir_copy==E_OK;
#if BOARD_BARCODE_DEMO
            if(li && l.barcode_diagnostics.rejected_count!=last_barcode_rejection) {
                const s3_barcode_diagnostics_t *d=&l.barcode_diagnostics;
                const char *names[]={"none","ok","incomplete","noise","overflow","timeout","pattern","unknown"};
                tm_printf((UB *)"[BARCODE REJECT] t=%u status=%u reason=%s runs=%u max_run_ms=%u cause=%s action=%s count=%u\n",
                    d->rejected_ms,(unsigned)d->status,names[d->status&7U],d->rejected_runs,d->max_run_ms,
                    d->junction_abort?"junction":"decoder",BOARD_BARCODE_RETRY?"retry":"fault",d->rejected_count);
                last_barcode_rejection=d->rejected_count;
            }
#endif
            static s4_sensor_cache_t imu_record;memset(&imu_record,0,sizeof(imu_record));
            BOOL im=s4_sensor_get_record(&imu_record)==E_OK;
            ER sonar_copy=s5_get_status(&o);BOOL so=sonar_copy==E_OK;
            if(tk_get_otm(&now)<E_OK) {tk_dly_tsk(10);continue;}
            if(im)s4_data_snapshot(&imu_record,s4_operating_ms_to_us(((uint64_t)(UW)now.hi<<32)|now.lo),&a);
            li=li && l.configured && l.acquisition_error==E_OK && now.lo-l.sample_time_ms<30U;
#if BOARD_BARCODE_DEMO && TM_MQTT
            if(ir_copy==E_OK)diagnostic_cache=l.barcode_diagnostics;
            unsigned correction_state=s.vehicle.steering<0?2U:s.vehicle.steering>0?1U:0U;
            if(now.lo-diagnostic_ms>=1000U ||
               (ir_copy==E_OK && l.barcode_diagnostics.result_id!=diagnostic_result) ||
               s.barcode!=diagnostic_symbol || s.navigation!=diagnostic_navigation ||
               s.vehicle.phase!=diagnostic_phase || s.vehicle.fault!=diagnostic_fault ||
               correction_state!=diagnostic_correction) {
                const s3_barcode_diagnostics_t *b=&diagnostic_cache;
                static barcode_diagnostic_t d;static uint8_t wire[BARCODE_DIAGNOSTIC_BYTES];
                memset(&d,0,sizeof(d));
                d.kind=b->result_id!=diagnostic_result?2:1;
                d.sequence=++diagnostic_sequence;d.time_ms=now.lo;
                d.result_id=b->result_id;d.result_ms=b->result_ms;d.started_ms=b->started_ms;
                d.rejected=b->rejected_count;d.decoded=b->decoded_count;
                d.max_gap_ms=b->max_gap_ms;d.run_count=b->result_runs;d.max_run_ms=b->result_max_run_ms;
                d.sample_ms=l.sample_time_ms;d.acquisition_error=ir_copy==E_OK?l.acquisition_error:ir_copy;
                d.left_counts=e.motor1_forward_counts;d.right_counts=e.motor2_forward_counts;
                d.status=b->result_status;d.phase=(uint8_t)s.vehicle.phase;d.fault=(uint8_t)s.vehicle.fault;
                d.power=(uint8_t)s.vehicle.power;d.navigation=s.navigation;d.symbol=(uint8_t)b->symbol;
                d.flags=(uint8_t)((b->reverse?1:0)|(b->result_junction_abort?2:0)|
                    (l.output.barcode_scanning?4:0)|(l.sample.raw[2]?8:0)|(li?16:0)|(s.vehicle.armed?32:0)|
                    (s.vehicle.steering?64:0)|(s.vehicle.steering<0?128:0));
                d.capture_state=l.capture_state;d.capture_runs=(uint8_t)(l.capture_runs>255U?255U:l.capture_runs);
                d.line_mask=l.output.line.black_mask;d.stored_symbol=s.barcode;d.valid_mask=l.sample.valid_mask;
                d.algorithm_error=l.output.error;
                memcpy(d.runs,b->runs,sizeof(d.runs));
                barcode_diag_pack(wire,&d);
                if(s1_diagnostic_offer(wire,sizeof(wire))==E_OK) {
                    diagnostic_ms=now.lo;diagnostic_result=b->result_id;
                    diagnostic_symbol=s.barcode;diagnostic_navigation=s.navigation;
                    diagnostic_phase=s.vehicle.phase;diagnostic_fault=s.vehicle.fault;
                    diagnostic_correction=correction_state;
                }
            }
#endif
            BOOL us=so && o.valid && o.distance_mm>0 && o.distance_mm<=5000U && now.lo-o.time_ms<500U;
            BOOL width_valid=so && o.object_width_valid && o.object_width_mm<=500U;
            app_log_state(&s);
            static s4_result_t terrain;memset(&terrain,0,sizeof(terrain));
            BOOL terrain_copy=st && s4_result_decode(s.s4_result,sizeof(s.s4_result),&terrain);
            BOOL terrain_fresh=terrain_copy && now.lo-terrain.calculated_ms<=60U && im && a.accel_usable;
            static int64_t v[TEL_FIELD_COUNT];static uint8_t out[TELEMETRY_BYTES];
            memset(v,0,sizeof(v));
            v[TEL_VERSION]=1;v[TEL_SEQUENCE]=(uint16_t)count++;
            v[TEL_TIME_MS]=now.lo;v[TEL_PHASE]=s.vehicle.phase;v[TEL_FAULT]=s.vehicle.fault;
            v[TEL_COMMAND]=mo?m.command:0;v[TEL_POWER]=mo?m.power_percent:0;v[TEL_MOTOR_STATE]=mo?m.state:2;
            v[TEL_AVAILABLE]=(mo?1:0)|(en?2:0)|(li?4:0)|(im&&a.accel_usable?8:0)|
                (us?16:0)|(terrain_fresh&&(terrain.flags&S4_RESULT_TILT)?32:0)|
                (terrain_fresh&&(terrain.flags&S4_RESULT_HEIGHT)?64:0)|(im&&a.mag_usable?128:0);
            v[TEL_LEFT_COUNTS]=e.motor1_forward_counts;v[TEL_RIGHT_COUNTS]=e.motor2_forward_counts;
            v[TEL_SPEED_MM_S]=bounded(app_speed_mm_s(&s),-2000,2000);v[TEL_LINE_STATE]=li?l.output.line.state:0;
            const int16_t positions[S3_SENSOR_COUNT]=BOARD_IR_POSITIONS;
            if(li) for(unsigned channel=0;channel<S3_SENSOR_COUNT;++channel) {
                unsigned field=channel==BOARD_BARCODE_SENSOR?TEL_IR2:
                    positions[channel]<0?TEL_IR1:TEL_IR0;
                v[field]=l.sample.raw[channel];
            }
            v[TEL_BLACK_MASK]=li?l.output.line.black_mask:0;v[TEL_LINE_ERROR]=li?l.output.line.error:0;
            v[TEL_BARCODE]=s.barcode;v[TEL_NAVIGATION]=s.navigation;
            v[TEL_AX_MG]=bounded(a.latest.accel.milli_units[0],-2048,2047);
            v[TEL_AY_MG]=bounded(a.latest.accel.milli_units[1],-2048,2047);
            v[TEL_AZ_MG]=bounded(a.latest.accel.milli_units[2],-2048,2047);
            v[TEL_MX_MG]=bounded(a.latest.mag.milli_units[0],-2048,2047);
            v[TEL_MY_MG]=bounded(a.latest.mag.milli_units[1],-2048,2047);
            v[TEL_MZ_MG]=bounded(a.latest.mag.milli_units[2],-2048,2047);
            v[TEL_PITCH_CDEG]=terrain.pitch_cdeg;v[TEL_ROLL_CDEG]=terrain.roll_cdeg;v[TEL_TERRAIN]=terrain.terrain;
            v[TEL_HEIGHT_MM]=terrain.height_mm;v[TEL_HIGHEST_MM]=terrain.maximum_mm;
            v[TEL_DISTANCE_MM]=us?o.distance_mm:0;v[TEL_WIDTH_MM]=width_valid?o.object_width_mm:0;
            v[TEL_SERVO_DEG]=o.angle_deg;v[TEL_SCAN_STATE]=o.state;
            v[TEL_QUALITY]=(width_valid?1:0)|((terrain.flags&S4_RESULT_SHOCK)?2:0)|(s.vehicle.armed?4:0)|
                (BOARD_BYPASS_VALIDATED?8:0)|(s.odometry_valid?16:0)|
                (so && o.state!=S5_FAULT?32:0);
            v[TEL_DROPPED]=s1_drop_count();
            v[TEL_MOTOR_ERROR]=s2_motor_error_code(mo?m.last_error:E_SYS);
            if(telemetry_encode(out,v)) {
#if TM_MQTT
                (void)s1_binary_send(out);
#endif
                if((count%5)==0 || s.vehicle.fault!=old_fault) {
                    tm_printf((UB *)"[CSC02] t=%u phase=%u fault=%u armed=%u cmd=%u power=%u flags=%u bytes=%u\n",
                        now.lo,s.vehicle.phase,s.vehicle.fault,s.vehicle.armed,m.command,m.power_percent,
                        (unsigned)v[TEL_AVAILABLE],TELEMETRY_BYTES);
                /* Passive bench diagnostics use the existing copied snapshots.
                 * No GPIO/PWM ownership, motor command or binary field changes. */
                if(BOARD_MOTORS_DISABLED && (count%5)==0) {
                    tm_printf((UB *)"[S4TERRAIN] t=%u fresh=%u tilt=%u pitch_cdeg=%d roll_cdeg=%d state=%u height_valid=%u height_mm=%u max_mm=%u odom=%u\n",
                        s.time_ms,(unsigned)terrain_fresh,(unsigned)((terrain.flags&S4_RESULT_TILT)!=0U),
                        (INT)terrain.pitch_cdeg,(INT)terrain.roll_cdeg,(unsigned)terrain.terrain,
                        (unsigned)((terrain.flags&S4_RESULT_HEIGHT)!=0U),(unsigned)terrain.height_mm,
                        (unsigned)terrain.maximum_mm,(unsigned)s.odometry_valid);
                    tm_printf((UB *)"[S2ENC] valid=%u m1=%d m2=%d m1ab=%u%u m2ab=%u%u\n",
                        (unsigned)en,e.motor1_forward_counts,e.motor2_forward_counts,
                        (unsigned)e.motor1_a_level,(unsigned)e.motor1_b_level,
                        (unsigned)e.a_level,(unsigned)e.b_level);
                    tm_printf((UB *)"[S5RAW] copied=%u t=%u state=%u fresh=%u valid=%u mm=%u angle=%d sampled=%d echo=%u error=%d\n",
                        (unsigned)so,o.time_ms,(unsigned)o.state,
                        (unsigned)(so && now.lo-o.time_ms<S5_STALE_MS),(unsigned)o.valid,
                        (unsigned)o.distance_mm,(INT)o.angle_deg,(INT)o.sample_angle_deg,(unsigned)o.echo_result,(INT)sonar_copy);
                }
                }
            }
#if BOARD_IR_CALIBRATED_BENCH || BOARD_IR_MOTOR_SCAN_BENCH || BOARD_BARCODE_DEMO
            {
                const uint16_t white[3]=BOARD_IR_ADC_WHITE,black[3]=BOARD_IR_ADC_BLACK;
                uint16_t barcode_darkness=0;
                BOOL usable=ir_copy==E_OK && l.configured && l.acquisition_error==E_OK &&
                    l.sample.valid_mask==7U && now.lo-l.sample.time_ms<S3_STATUS_MAX_AGE_MS;
                if(usable) usable=s3_normalize(l.sample.raw[2],white[2],black[2],4095U,&barcode_darkness);
                tm_printf((UB *)"[S3IR] copied=%u error=%d t=%u acquisition=%d mask=%u raw=%u,%u,%u configured=%u steering=%u fresh=%u line=%u blackmask=%u lineerror=%d barcode_darkness=%u algorithm_error=%u line_darkness=%u,%u\n",
                    (unsigned)(ir_copy==E_OK),(INT)ir_copy,l.sample.time_ms,(INT)l.acquisition_error,
                    (unsigned)l.sample.valid_mask,(unsigned)l.sample.raw[0],(unsigned)l.sample.raw[1],
                    (unsigned)l.sample.raw[2],(unsigned)l.configured,(unsigned)l.output.steering_allowed,
                    (unsigned)usable,(unsigned)l.output.line.state,(unsigned)l.output.line.black_mask,
                    (INT)l.output.line.error,(unsigned)barcode_darkness,(unsigned)l.output.error,
                    (unsigned)l.output.line.darkness[0],(unsigned)l.output.line.darkness[1]);
            }
#endif
            previous=now.lo;old_fault=s.vehicle.fault;
            if(BOARD_MOTORS_DISABLED && BOARD_IR_RAW_BENCH) {
                tm_printf((UB *)"[S3RAW] copied=%u error=%d t=%u acquisition=%d mask=%u raw=%u,%u,%u configured=%u steering=%u fresh=%u\n",
                    (unsigned)(ir_copy==E_OK),(INT)ir_copy,l.sample.time_ms,(INT)l.acquisition_error,
                    (unsigned)l.sample.valid_mask,(unsigned)l.sample.raw[0],(unsigned)l.sample.raw[1],
                    (unsigned)l.sample.raw[2],(unsigned)l.configured,(unsigned)l.output.steering_allowed,
                    (unsigned)(ir_copy==E_OK && now.lo-l.sample.time_ms<S3_STATUS_MAX_AGE_MS));
            }
        }
#if BOARD_IR_BARCODE_CAPTURE
        /* Drain copied scans outside the acquisition task. Each call is
         * nonblocking; bounded work preserves all other telemetry duties. */
        for(unsigned n=0;n<S3_CAPTURE_CAPACITY;++n) {
            s3_sample_t captured={0};uint32_t dropped=0;
            ER capture=s3_capture_receive(&captured,&dropped);
            if(capture==E_TMOUT) break;
            if(capture<E_OK) {tm_printf((UB *)"[S3BERR] error=%d\n",(INT)capture);break;}
            tm_printf((UB *)"[S3B] t=%u raw=%u,%u,%u mask=%u dropped=%u\n",
                captured.time_ms,(unsigned)captured.raw[0],(unsigned)captured.raw[1],
                (unsigned)captured.raw[2],(unsigned)captured.valid_mask,dropped);
        }
#endif
        tk_dly_tsk(10);
    }
}
