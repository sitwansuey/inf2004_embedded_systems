<<<<<<< HEAD
/* Read public snapshots only. Sensors and actuators retain their owners. */
#include "subsystem1.h"
#if defined(TM_NET_LWIP) && TM_NET_LWIP && defined(TM_MQTT) && TM_MQTT
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/s2_data.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"

/* Counts wrap explicitly; do not present these as distance or speed. */
void s1_sensor_telemetry_task(INT stacd, void *exinf)
{
    const int16_t positions[S3_SENSOR_COUNT] = BOARD_IR_POSITIONS;
    uint8_t motor_records[EDR_MOTION_SIZE], encoder_records[EDR_ENCODER_SIZE];
    s3_status_t ir;
    s4_sensor_status_t imu;
    UINT i;
    (void)stacd; (void)exinf;
    for (;;) {
        /* Startup lets other owners start. Sampling is bounded at 4 Hz.
         * Queue-full drops this observation; next pass sends latest snapshots.
         * Thirteen readings/pass fit the 16-message inbox while connected. */
        tk_dly_tsk(250);
        if (s2_encoder_encode(encoder_records, sizeof(encoder_records)) == E_OK) {
            for (i = 0; i < sizeof(encoder_records); i += EDR_RECORD_SIZE)
                (void)s1_telemetry_record(encoder_records + i, EDR_RECORD_SIZE);
        }
        if (s2_motion_encode(motor_records, sizeof(motor_records)) == E_OK) {
            for (i = 0; i < sizeof(motor_records); i += EDR_RECORD_SIZE)
                (void)s1_telemetry_record(motor_records + i, EDR_RECORD_SIZE);
        }
        if (s3_status_get(&ir) == E_OK && ir.configured &&
            ir.acquisition_error == E_OK) {
            SYSTIM now;
            if (tk_get_tim(&now) == E_OK &&
                (uint32_t)(now.lo - ir.sample_time_ms) <= S3_STATUS_MAX_AGE_MS) {
                for (i = 0; i < S3_SENSOR_COUNT; ++i) {
                    UINT id;
                    if (!(ir.sample.valid_mask & (1U << i))) continue;
                    if (i == BOARD_BARCODE_SENSOR) id = S1_SENSOR_IR_BARCODE;
                    else if (positions[i] < 0) id = S1_SENSOR_IR_LEFT;
                    else if (positions[i] > 0) id = S1_SENSOR_IR_RIGHT;
                    else continue;
                    (void)s1_telemetry_u12(id, ir.sample.raw[i]);
                }
            }
        }
        if (s4_sensor_get_status(&imu) == E_OK && imu.configured) {
            if (imu.mag_usable)
                for (i = 0; i < 3; ++i)
                    (void)s1_telemetry_magnetic(EDR_MAG_X + i, imu.latest.mag.milli_units[i]);
            if (imu.accel_usable)
                for (i = 0; i < 3; ++i)
                    (void)s1_telemetry_acceleration(EDR_ACCEL_X + i, imu.latest.accel.milli_units[i]);
        }
    }
}
#endif
=======
#include "subsystem1.h"
#include "control.h"
#include "../app_state.h"
#include "../app_logging.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
static int64_t bounded(int64_t n,int64_t lo,int64_t hi) {return n<lo?lo:n>hi?hi:n;}
void telemetry_task(INT code,void *context) {
    (void)code;(void)context;unsigned count=0;uint32_t previous=0;unsigned old_fault=0;
    for(;;) {
        app_state_t s={0};s2_motion_status_t m={0};s2_encoder_status_t e={0};s3_status_t l={0};s4_sensor_status_t a={0};s5_status_t o={0};
        BOOL st=app_get_state(&s)==E_OK;SYSTIM now;
        if(tk_get_otm(&now)<E_OK) {tk_dly_tsk(10);continue;}
        if(st && (now.lo-previous>=200U || s.vehicle.fault!=old_fault)) {
            BOOL mo=s2_motion_get_status(&m)==E_OK,en=s2_encoder_get_status(&e)==E_OK && e.valid;
            ER ir_copy=s3_status_get(&l);BOOL li=ir_copy==E_OK;
            s4_sensor_cache_t imu_record={0};
            BOOL im=s4_sensor_get_record(&imu_record)==E_OK;
            ER sonar_copy=s5_get_status(&o);BOOL so=sonar_copy==E_OK;
            if(tk_get_otm(&now)<E_OK) {tk_dly_tsk(10);continue;}
            if(im)s4_data_snapshot(&imu_record,s4_operating_ms_to_us(((uint64_t)(UW)now.hi<<32)|now.lo),&a);
            li=li && l.configured && l.acquisition_error==E_OK && now.lo-l.sample_time_ms<30U;
            BOOL us=so && o.valid && o.distance_mm>0 && o.distance_mm<=5000U && now.lo-o.time_ms<500U;
            BOOL width_valid=so && o.object_width_valid && o.object_width_mm<=500U;
            app_log_state(&s);
            s4_result_t terrain={0};
            BOOL terrain_copy=st && s4_result_decode(s.s4_result,sizeof(s.s4_result),&terrain);
            BOOL terrain_fresh=terrain_copy && now.lo-terrain.calculated_ms<=60U && im && a.accel_usable;
            int64_t v[TEL_FIELD_COUNT]={0};uint8_t out[TELEMETRY_BYTES];
            v[TEL_VERSION]=1;v[TEL_SEQUENCE]=(uint16_t)count++;
            v[TEL_TIME_MS]=now.lo;v[TEL_PHASE]=s.vehicle.phase;v[TEL_FAULT]=s.vehicle.fault;
            v[TEL_COMMAND]=mo?m.command:0;v[TEL_POWER]=mo?m.power_percent:0;v[TEL_MOTOR_STATE]=mo?m.state:2;
            v[TEL_AVAILABLE]=(mo?1:0)|(en?2:0)|(li?4:0)|(im&&a.accel_usable?8:0)|
                (us?16:0)|(terrain_fresh&&(terrain.flags&S4_RESULT_TILT)?32:0)|
                (terrain_fresh&&(terrain.flags&S4_RESULT_HEIGHT)?64:0)|(im&&a.mag_usable?128:0);
            v[TEL_LEFT_COUNTS]=e.motor1_forward_counts;v[TEL_RIGHT_COUNTS]=e.motor2_forward_counts;
            v[TEL_SPEED_MM_S]=bounded(app_speed_mm_s(&s),-2000,2000);v[TEL_LINE_STATE]=li?l.output.line.state:0;
            v[TEL_IR0]=li?l.sample.raw[0]:0;v[TEL_IR1]=li?l.sample.raw[1]:0;v[TEL_IR2]=li?l.sample.raw[2]:0;
            v[TEL_BLACK_MASK]=li?l.output.line.black_mask:0;v[TEL_LINE_ERROR]=li?l.output.line.error:0;
            v[TEL_BARCODE]=s.barcode;v[TEL_NAVIGATION]=s.navigation;
            v[TEL_AX_MG]=bounded(a.latest.accel.milli_units[0],-2048,2047);
            v[TEL_AY_MG]=bounded(a.latest.accel.milli_units[1],-2048,2047);
            v[TEL_AZ_MG]=bounded(a.latest.accel.milli_units[2],-2048,2047);
            v[TEL_MX_RAW]=im&&a.mag_usable?a.latest.mag.raw[0]:0;
            v[TEL_MY_RAW]=im&&a.mag_usable?a.latest.mag.raw[1]:0;
            v[TEL_MZ_RAW]=im&&a.mag_usable?a.latest.mag.raw[2]:0;
            v[TEL_PITCH_CDEG]=terrain.pitch_cdeg;v[TEL_ROLL_CDEG]=terrain.roll_cdeg;v[TEL_TERRAIN]=terrain.terrain;
            v[TEL_HEIGHT_MM]=terrain.height_mm;v[TEL_HIGHEST_MM]=terrain.maximum_mm;
            v[TEL_DISTANCE_MM]=us?o.distance_mm:0;v[TEL_WIDTH_MM]=width_valid?o.object_width_mm:0;
            v[TEL_SERVO_DEG]=o.angle_deg;v[TEL_SCAN_STATE]=o.state;
            v[TEL_QUALITY]=(width_valid?1:0)|((terrain.flags&S4_RESULT_SHOCK)?2:0)|(s.vehicle.armed?4:0)|
                (BOARD_BYPASS_VALIDATED?8:0)|(s.odometry_valid?16:0);
            v[TEL_DROPPED]=s1_drop_count();
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
#if BOARD_IR_CALIBRATED_BENCH || BOARD_IR_MOTOR_SCAN_BENCH
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
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
