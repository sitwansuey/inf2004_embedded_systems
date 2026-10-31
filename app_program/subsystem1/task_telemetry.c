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
