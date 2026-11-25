#include "subsystem1.h"
#include "control.h"
#include "../app_state.h"
#include "../app_logging.h"
#include "../subsystem2/subsystem2.h"
#include "../subsystem2/s2_data.h"
#include "../subsystem3/subsystem3.h"
#include "../subsystem4/subsystem4.h"
#include "../subsystem5/subsystem5.h"
#include "../subsystem5/s5_width.h"
/* A new shock count sends an extra snapshot at once, but at most one per
 * this many ms (a hump's jolts can come every 40 ms); a later one rides the
 * next snapshot. Fault changes are never limited. */
#ifndef S1_SHOCK_EXTRA_MS
#define S1_SHOCK_EXTRA_MS 200U
#endif
#if defined(_PICO_RP2040_)
/* Watchdog heartbeat (app_watchdog.c): one per loop. Host builds: none. */
#include "../../include/tm/robot_health.h"
#define S1_HEARTBEAT() HEALTH_BEAT(HEALTH_HB_TELEMETRY)
#else
#define S1_HEARTBEAT() ((void) 0)
#endif
static int64_t
bounded (int64_t n, int64_t lo, int64_t hi)
{
    return n < lo ? lo : n > hi ? hi : n;
}
void
telemetry_task (INT code, void * context)
{
    (void) code;
    (void) context;
    unsigned count     = 0;
    uint32_t previous  = 0;
    unsigned old_fault = 0, shown_fault = 0;
    uint8_t  old_shock     = 0; /* app_state_t.shock_count last sent */
    uint32_t shock_sent_ms = 0;
    BOOL     shock_sent    = FALSE; /* last snapshot with a new shock */
    /* EDR_FAULT_IMU seen in a VC result, or app_state_t.imu_fault_count
     * changed, since the last snapshot that carried ID 12 and was queued. */
    BOOL     imu_fault      = FALSE;
    uint16_t old_imu_faults = 0;
    /* Last published width/side: a new valid width or side is sent at once,
     * so the halves reach MQTT during S5's hold before the side is chosen. */
    UW       width_scan = 0, width_left = 0, width_right = 0;
    unsigned old_side = 0;
    for (;;)
    {
#if defined(S1_HOST_TEST_STOP_REQUESTED)
        if (S1_HOST_TEST_STOP_REQUESTED())
        {
            return;
        }
#endif
        S1_HEARTBEAT();
        app_state_t         s  = {0};
        s2_motion_status_t  m  = {0};
        s2_encoder_status_t e  = {0};
        s3_status_t         l  = {0};
        s4_sensor_status_t  a  = {0};
        s5_status_t         o  = {0};
        BOOL                st = app_get_state(&s) == E_OK;
        SYSTIM              now;
        /* An IMU fault can last one 20 ms VC tick (a clipped or overrun
         * sample, skew): hold it until a snapshot has carried it. The VC's
         * count also covers ticks this loop never read. */
        {
            s4_result_t r = {0};
            if (st &&
                (s.imu_fault_count != old_imu_faults ||
                 (s4_result_decode(s.s4_result, sizeof(s.s4_result), &r) &&
                  (r.faults & EDR_FAULT_IMU))))
            {
                imu_fault = TRUE;
            }
        }
        if (tk_get_otm(&now) < E_OK)
        {
            tk_dly_tsk(10);
            continue;
        }
        ER   sonar_copy = s5_get_status(&o);
        BOOL so         = sonar_copy == E_OK;
        /* S5's own validity bound (tangent envelope <= 500 mm) plus a
         * consistency check against corrupt copies; the Pythagoras total can
         * legitimately exceed 500 and is only saturated for its field. */
        BOOL width_valid =
            so && o.object_width_valid && o.object_width_max_mm <= 500U &&
            o.object_left_mm <= 5000U && o.object_right_mm <= 5000U &&
            o.object_width_mm == o.object_left_mm + o.object_right_mm;
        /* Reported width: the edges less the sonar beam's half-angle
         * (s5_width_reported); validity and the plan stay S5's own. */
        UW shown_left = 0U, shown_right = 0U;
        if (width_valid)
        {
            s5_width_reported(&o, &shown_left, &shown_right);
        }
        unsigned side      = so && o.approach_plan_valid
                                 ? (o.planned_rotate_deg > 0   ? 1U
                                    : o.planned_rotate_deg < 0 ? 2U
                                                               : 0U)
                                 : 0U;
        BOOL     width_new = width_valid && (o.scan_id != width_scan ||
                                             o.object_left_mm != width_left ||
                                             o.object_right_mm != width_right);
        /* A shock no longer changes the fault, so a new one is sent at once. */
        BOOL shock_new = st && s.shock_count != old_shock;
        BOOL shock_extra =
            shock_new &&
            (!shock_sent || now.lo - shock_sent_ms >= S1_SHOCK_EXTRA_MS);
        if (st && (now.lo - previous >= 200U || s.vehicle.fault != old_fault ||
                   width_new || (so && side != old_side) || shock_extra))
        {
            BOOL              mo = s2_motion_get_status(&m) == E_OK,
                              en = s2_encoder_get_status(&e) == E_OK && e.valid;
            ER                ir_copy    = s3_status_get(&l);
            BOOL              li         = ir_copy == E_OK;
            s4_sensor_cache_t imu_record = {0};
            BOOL              im = s4_sensor_get_record(&imu_record) == E_OK;
            if (tk_get_otm(&now) < E_OK)
            {
                tk_dly_tsk(10);
                continue;
            }
            if (im)
            {
                s4_data_snapshot(&imu_record,
                                 s4_operating_ms_to_us(
                                     ((uint64_t) (UW) now.hi << 32) | now.lo),
                                 &a);
            }
            li      = li && l.configured && l.acquisition_error == E_OK &&
                      now.lo - l.sample_time_ms < 30U;
            BOOL us = so && o.valid && o.distance_mm > 0 &&
                      o.distance_mm <= 5000U && now.lo - o.time_ms < 500U;
            app_log_state(&s);
            s4_result_t terrain = {0};
            BOOL        terrain_copy =
                st &&
                s4_result_decode(s.s4_result, sizeof(s.s4_result), &terrain);
            BOOL terrain_fresh = terrain_copy &&
                                 now.lo - terrain.calculated_ms <= 60U && im &&
                                 a.accel_usable;
            int64_t v[TEL_FIELD_COUNT] = {0};
            uint8_t out[TELEMETRY_BYTES];
            v[TEL_VERSION]     = 1;
            v[TEL_SEQUENCE]    = (uint16_t) count++;
            v[TEL_TIME_MS]     = now.lo;
            v[TEL_PHASE]       = s.vehicle.phase;
            v[TEL_FAULT]       = s.vehicle.fault;
            v[TEL_COMMAND]     = mo ? m.command : 0;
            v[TEL_POWER]       = mo ? m.power_percent : 0;
            v[TEL_MOTOR_STATE] = mo ? m.state : 2;
            v[TEL_AVAILABLE] =
                (mo ? 1 : 0) | (en ? 2 : 0) | (li ? 4 : 0) |
                (im && a.accel_usable ? 8 : 0) | (us ? 16 : 0) |
                (terrain_fresh && (terrain.flags & S4_RESULT_TILT) ? 32 : 0) |
                (terrain_fresh && (terrain.flags & S4_RESULT_HEIGHT) ? 64 : 0) |
                (im && a.mag_usable ? 128 : 0);
            v[TEL_LEFT_COUNTS]  = e.motor1_forward_counts;
            v[TEL_RIGHT_COUNTS] = e.motor2_forward_counts;
            v[TEL_SPEED_MM_S]   = bounded(app_speed_mm_s(&s), -2000, 2000);
            const int16_t positions[S3_SENSOR_COUNT] = BOARD_IR_POSITIONS;
            if (li)
            {
                for (unsigned channel = 0; channel < S3_SENSOR_COUNT; ++channel)
                {
                    unsigned field = channel == BOARD_BARCODE_SENSOR ? TEL_IR2
                                     : positions[channel] < 0        ? TEL_IR1
                                                                     : TEL_IR0;
                    v[field]       = l.sample.raw[channel];
                }
            }
            /* S3's edge-follow view does not apply to the straddle robot:
             * report its line follower instead (app_state.h line_*). */
            v[TEL_LINE_STATE] = li ? s.line_follow : 0;
            v[TEL_BLACK_MASK] = li ? s.line_mask : 0;
            v[TEL_LINE_ERROR] = li ? bounded(s.line_steer, -1000, 1000) : 0;
            v[TEL_BARCODE]    = s.barcode;
            v[TEL_NAVIGATION] = s.navigation;
            v[TEL_AX_MG] = bounded(a.latest.accel.milli_units[0], -2048, 2047);
            v[TEL_AY_MG] = bounded(a.latest.accel.milli_units[1], -2048, 2047);
            v[TEL_AZ_MG] = bounded(a.latest.accel.milli_units[2], -2048, 2047);
            v[TEL_MX_MG] = bounded(a.latest.mag.milli_units[0], -2048, 2047);
            v[TEL_MY_MG] = bounded(a.latest.mag.milli_units[1], -2048, 2047);
            v[TEL_MZ_MG] = bounded(a.latest.mag.milli_units[2], -2048, 2047);
            v[TEL_PITCH_CDEG]  = terrain.pitch_cdeg;
            v[TEL_ROLL_CDEG]   = terrain.roll_cdeg;
            v[TEL_TERRAIN]     = terrain.terrain;
            v[TEL_HEIGHT_MM]   = terrain.height_mm;
            v[TEL_HIGHEST_MM]  = terrain.maximum_mm;
            v[TEL_DISTANCE_MM] = us ? o.distance_mm : 0;
            v[TEL_WIDTH_MM] =
                width_valid ? bounded(shown_left + shown_right, 0, 500) : 0;
            v[TEL_SERVO_DEG]  = o.angle_deg;
            v[TEL_SCAN_STATE] = o.state;
            v[TEL_QUALITY] =
                (width_valid ? 1 : 0) |
                (((terrain.flags & S4_RESULT_SHOCK) || shock_new) ? 2 : 0) |
                (s.vehicle.armed ? 4 : 0) | (BOARD_BYPASS_VALIDATED ? 8 : 0) |
                (s.odometry_valid ? 16 : 0) |
                (so && o.state != S5_FAULT ? 32 : 0);
            v[TEL_DROPPED]     = s1_drop_count();
            v[TEL_MOTOR_ERROR] = s2_motor_error_code(mo ? m.last_error : E_SYS);
            v[TEL_BARCODE_ELEMENTS] = li ? (l.output.barcode_elements > 31U
                                                ? 31U
                                                : l.output.barcode_elements)
                                         : 0;
            v[TEL_BARCODE_RESULT]   = li && l.output.barcode_result <= 3U
                                          ? l.output.barcode_result
                                          : 0;
            v[TEL_WIDTH_LEFT_MM] =
                width_valid ? bounded(shown_left, 0, 1000) : 0;
            v[TEL_WIDTH_RIGHT_MM] =
                width_valid ? bounded(shown_right, 0, 1000) : 0;
            v[TEL_BYPASS_SIDE] = side;
            v[TEL_FAULT_CAUSE] =
                s.vehicle.fault == VC_SENSOR || s.vehicle.fault == VC_LINE
                    ? s.vehicle.fault_cause
                    : 0;
            v[TEL_IMU_FAULT] = imu_fault ? 1 : 0;
            BOOL sent        = FALSE;
            if (telemetry_encode(out, v))
            {
                sent = TRUE;
#if TM_MQTT
                sent = s1_binary_send(out) == E_OK;
#endif
                if ((count % 5) == 0 || s.vehicle.fault != shown_fault)
                {
                    shown_fault = s.vehicle.fault;
                    tm_printf((UB *) "[CSC02] t=%u phase=%u fault=%u armed=%u "
                                     "cmd=%u power=%u flags=%u bytes=%u\n",
                              now.lo, s.vehicle.phase, s.vehicle.fault,
                              s.vehicle.armed, m.command, m.power_percent,
                              (unsigned) v[TEL_AVAILABLE], TELEMETRY_BYTES);
                    /* Passive bench diagnostics use the existing copied
                     * snapshots. No GPIO/PWM ownership, motor command or binary
                     * field changes. */
                    if (BOARD_MOTORS_DISABLED && (count % 5) == 0)
                    {
                        tm_printf(
                            (UB *) "[S4TERRAIN] t=%u fresh=%u tilt=%u "
                                   "pitch_cdeg=%d roll_cdeg=%d state=%u "
                                   "height_valid=%u height_mm=%u max_mm=%u "
                                   "odom=%u\n",
                            s.time_ms, (unsigned) terrain_fresh,
                            (unsigned) ((terrain.flags & S4_RESULT_TILT) != 0U),
                            (INT) terrain.pitch_cdeg, (INT) terrain.roll_cdeg,
                            (unsigned) terrain.terrain,
                            (unsigned) ((terrain.flags & S4_RESULT_HEIGHT) !=
                                        0U),
                            (unsigned) terrain.height_mm,
                            (unsigned) terrain.maximum_mm,
                            (unsigned) s.odometry_valid);
                        tm_printf((UB *) "[S2ENC] valid=%u m1=%d m2=%d "
                                         "m1ab=%u%u m2ab=%u%u\n",
                                  (unsigned) en, e.motor1_forward_counts,
                                  e.motor2_forward_counts,
                                  (unsigned) e.motor1_a_level,
                                  (unsigned) e.motor1_b_level,
                                  (unsigned) e.a_level, (unsigned) e.b_level);
                        tm_printf(
                            (UB *) "[S5RAW] copied=%u t=%u state=%u fresh=%u "
                                   "valid=%u mm=%u angle=%d sampled=%d echo=%u "
                                   "error=%d\n",
                            (unsigned) so, o.time_ms, (unsigned) o.state,
                            (unsigned) (so && now.lo - o.time_ms < S5_STALE_MS),
                            (unsigned) o.valid, (unsigned) o.distance_mm,
                            (INT) o.angle_deg, (INT) o.sample_angle_deg,
                            (unsigned) o.echo_result, (INT) sonar_copy);
                    }
                }
            }
            previous = now.lo;
            /* Changes count as sent only once queued: a failed send keeps
             * them, so the next 20 ms loop builds another snapshot. */
            if (sent)
            {
                old_fault = s.vehicle.fault;
                old_side  = side;
                if (shock_new)
                {
                    old_shock     = s.shock_count;
                    shock_sent_ms = now.lo;
                    shock_sent    = TRUE;
                }
                /* ID 12 goes only with motor status (telemetry_edr.h). */
                if (v[TEL_AVAILABLE] & 1)
                {
                    imu_fault      = FALSE;
                    old_imu_faults = s.imu_fault_count;
                }
                if (width_valid)
                {
                    width_scan  = o.scan_id;
                    width_left  = o.object_left_mm;
                    width_right = o.object_right_mm;
                }
            }
        }
        tk_dly_tsk(10);
    }
}

/*** end of file ***/
