/* Recorded-posture regression fixtures, not independent angle metrology. */
#include "../../mounting.h"
#include "../../terrain.h"
#include "../../result_data.h"
#include "../../sensor_data.h"
#include "../../../subsystem1/telemetry_edr.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void mapping(void)
{
    const int32_t sensor[3] = {-40, -229, 934};
    int32_t body[3];
    assert(s4_body_acceleration(sensor, S4_MOUNT_X_FORWARD, body));
    assert(memcmp(sensor, body, sizeof(body)) == 0);
    assert(s4_body_acceleration(sensor, S4_MOUNT_NEG_Y_FORWARD, body));
    assert(body[0] == 229 && body[1] == -40 && body[2] == 934);
    assert(s4_body_acceleration(body, S4_MOUNT_NEG_Y_FORWARD, body));
    assert(body[0] == 40 && body[1] == 229 && body[2] == 934);
    const int32_t edge[3] = {-2048, -2048, 2047};
    assert(s4_body_acceleration(edge, S4_MOUNT_NEG_Y_FORWARD, body));
    assert(body[0] == 2048 && body[1] == -2048 && body[2] == 2047);
    int32_t old[3];memcpy(old, body, sizeof(old));
    const int32_t invalid[3] = {0, -2049, 1000};
    assert(!s4_body_acceleration(invalid, 1, body));
    assert(!s4_body_acceleration(sensor, 2, body));
    assert(!s4_body_acceleration(NULL, 1, body));
    assert(memcmp(old, body, sizeof(old)) == 0);
    int32_t pitch = 123, roll = 456;
    const int32_t transient[3] = {41, -130, 767};
    assert(!s4_gravity_tilt(transient, &pitch, &roll));
    assert(pitch == 123 && roll == 456);
    const int32_t up[3] = {500, 0, 866}, left[3] = {0, 500, 866};
    assert(s4_gravity_tilt(up, &pitch, &roll) && pitch >= 2999 && pitch <= 3001 && roll == 0);
    assert(s4_gravity_tilt(left, &pitch, &roll) && roll >= 2999 && roll <= 3001 && pitch == 0);
    puts("S4-U35 PASS mounting profiles, signed endpoints, aliasing, rejected inputs, gravity gate and positive nose/left tilt");
}

static void consumers(void)
{
    const int32_t poses[4][3] = {
        {-40, -229, 934}, {-43, -376, 880}, {64, -149, 945}, {-143, -156, 948}
    };
    int32_t body[3], pzero, rzero;
    assert(s4_body_acceleration(poses[0], 1, body));
    assert(s4_gravity_tilt(body, &pzero, &rzero));
    for (unsigned pose = 0; pose < 4; ++pose) {
        lsm303_frame_t frame = {0};
        frame.accel.fresh = frame.accel.valid = true;
        frame.accel.status = LSM303_DATA_OK;
        frame.accel.read_time_us = (100U + pose) * 1000U;
        for (unsigned n = 0; n < 3; ++n) frame.accel.raw[n] = (int16_t)poses[pose][n];
        s4_sensor_cache_t cache;s4_data_reset(&cache);s4_data_init_result(&cache, LSM303_OK);
        s4_data_update(&cache, &frame, LSM303_OK);
        s4_sensor_status_t raw;s4_data_snapshot(&cache, frame.accel.read_time_us, &raw);
        assert(raw.accel_usable && sizeof(cache) == 63);
        for (unsigned n = 0; n < 3; ++n) assert(raw.latest.accel.milli_units[n] == poses[pose][n]);
        assert(s4_body_acceleration(raw.latest.accel.milli_units, 1, body));
        terrain_t tilt = {0};
        terrain_step(&tilt, 100U + pose, body, true, 0, false, true, pzero, rzero);
        assert(tilt.tilt_valid && !tilt.height_valid);
        s4_result_t result = {.calculated_ms=100U + pose,
            .pitch_cdeg=terrain_pitch_cdeg(&tilt), .roll_cdeg=terrain_roll_cdeg(&tilt),
            .terrain=tilt.state, .flags=S4_RESULT_TILT|S4_RESULT_CALIBRATED,
            .faults=EDR_FAULT_ENCODER}, decoded;
        uint8_t result_bytes[S4_RESULT_SIZE];
        assert(s4_result_encode(&result, result_bytes, sizeof(result_bytes)));
        assert(s4_result_decode(result_bytes, sizeof(result_bytes), &decoded));
        assert(decoded.pitch_cdeg == result.pitch_cdeg && decoded.roll_cdeg == result.roll_cdeg);
        if (pose == 0) assert(result.pitch_cdeg == 0 && result.roll_cdeg == 0);
        if (pose == 1) assert(result.pitch_cdeg > 800);
        if (pose == 2) assert(result.roll_cdeg > 500);
        if (pose == 3) assert(result.roll_cdeg < -500);
        int64_t v[TEL_FIELD_COUNT] = {0}, received[TEL_FIELD_COUNT];
        v[TEL_VERSION]=1;v[TEL_AVAILABLE]=8U|32U;
        for (unsigned n=0;n<3;++n)v[TEL_AX_MG+n]=raw.latest.accel.milli_units[n];
        v[TEL_PITCH_CDEG]=decoded.pitch_cdeg;v[TEL_ROLL_CDEG]=decoded.roll_cdeg;
        uint8_t packed[TELEMETRY_BYTES], record[2];
        assert(telemetry_encode(packed,v) && telemetry_decode(packed,received));
        for (unsigned n=0;n<3;++n) {
            unsigned id,data;
            assert(telemetry_edr_record(received,12U+n,record));
            assert(edr_decode(record,2,&id,&data) && id==13U+n);
            assert((int)data-2048==poses[pose][n]); /* raw axes never silently rotate */
        }
        for (unsigned n=0;n<2;++n) {
            unsigned id,data;
            assert(telemetry_edr_record(received,18U+n,record));
            assert(edr_decode(record,2,&id,&data) && id==0 && (data>>8U)==3U+n);
            int expected=n?result.roll_cdeg/200:result.pitch_cdeg/100;
            assert((int)(data&255U)-128==expected); /* explicit MQTT quantization */
        }
    }
    puts("S4-U36 PASS recorded postures through compact cache, body tilt, 12-byte result, B1 snapshot and EDR16; raw axes preserved");
}

int main(void) { mapping(); consumers(); return 0; }
