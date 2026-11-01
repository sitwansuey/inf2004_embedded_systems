#include "../../app_program/subsystem1/telemetry_edr.h"
#include <assert.h>
#include <stdio.h>
int main(void) {
    int64_t v[TEL_FIELD_COUNT]={0};v[TEL_VERSION]=1;v[TEL_AVAILABLE]=255;
    v[TEL_QUALITY]=63;v[TEL_LEFT_COUNTS]=4196;v[TEL_RIGHT_COUNTS]=-1;
    v[TEL_IR0]=222;v[TEL_IR1]=111;v[TEL_IR2]=333;
    v[TEL_SERVO_DEG]=-30;v[TEL_DISTANCE_MM]=325;v[TEL_WIDTH_MM]=105;
    v[TEL_COMMAND]=1;v[TEL_POWER]=30;v[TEL_MOTOR_STATE]=1;
    v[TEL_PHASE]=1;v[TEL_BARCODE]=4;v[TEL_NAVIGATION]=4;
    v[TEL_HEIGHT_MM]=123;v[TEL_HIGHEST_MM]=456;v[TEL_SPEED_MM_S]=-360;
    v[TEL_MX_MG]=-100;v[TEL_AX_MG]=1000;v[TEL_PITCH_CDEG]=-1200;
    v[TEL_ROLL_CDEG]=400;v[TEL_TERRAIN]=2;v[TEL_LINE_ERROR]=-500;
    v[TEL_SEQUENCE]=257;v[TEL_DROPPED]=300;
    uint8_t snapshot[TELEMETRY_BYTES];int64_t decoded[TEL_FIELD_COUNT];
    assert(telemetry_encode(snapshot,v) && telemetry_decode(snapshot,decoded));
    for(unsigned n=0;n<TELEMETRY_EDR_RECORDS;++n) {
        uint8_t word[2];unsigned id,data;
        assert(telemetry_edr_record(decoded,n,word));
        assert(edr_decode(word,2,&id,&data));
        printf("%02x%02x\n",word[0],word[1]);
    }
    decoded[TEL_AVAILABLE]=0;decoded[TEL_QUALITY]=0;
    uint8_t word[2];
    assert(!telemetry_edr_record(decoded,0,word));
    assert(!telemetry_edr_record(decoded,2,word)); /* unavailable servo */
    assert(!telemetry_edr_record(decoded,11,word)); /* unavailable motor */
    decoded[TEL_AVAILABLE]=1;
    for(unsigned error=0;error<=2;++error) {
        unsigned id,data;
        decoded[TEL_MOTOR_ERROR]=error;
        assert(telemetry_encode(snapshot,decoded));
        assert(telemetry_decode(snapshot,decoded));
        assert(telemetry_edr_record(decoded,11,word));
        assert(edr_decode(word,2,&id,&data));
        assert(id==12 && data==(error==1?32U:error==2?64U:0U));
    }
    decoded[TEL_AVAILABLE]=0;
    assert(!telemetry_edr_record(decoded,7,word));
    assert(!telemetry_edr_record(decoded,15+6,word));
    assert(!telemetry_edr_record(decoded,15+13,word));
    decoded[TEL_AVAILABLE]=128; /* magnetometer is independent of obstacle width */
    assert(telemetry_edr_record(decoded,7,word));
    return 0;
}
