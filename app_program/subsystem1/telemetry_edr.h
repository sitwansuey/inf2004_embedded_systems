/* Only this EDR16 adapter crosses the MQTT boundary. Snapshot stays local. */
#ifndef TELEMETRY_EDR_H
#define TELEMETRY_EDR_H
#include "telemetry_codec.h"
#include "../shared/edr.h"
#define TELEMETRY_EDR_RECORDS 31U
/* False means the corresponding observation is unavailable; skip it. */
static inline bool telemetry_edr_record(const int64_t v[TEL_FIELD_COUNT],
                                       unsigned index, uint8_t out[2])
{
    if (!v || !out || index >= TELEMETRY_EDR_RECORDS) return false;
    unsigned id=index+1U, data=0;
    unsigned available=(unsigned)v[TEL_AVAILABLE];
    if (index < 15U) {
        switch (id) {
        case 1: case 2:
            if (!(available&2U)) return false;
            data=(unsigned)((uint64_t)v[id==1?TEL_LEFT_COUNTS:TEL_RIGHT_COUNTS]&4095U);break;
        case 3: if (!(v[TEL_QUALITY]&32)) return false; data=(unsigned)(v[TEL_SERVO_DEG]+90);break;
        case 4:
            if (!(available&16U)) return false;
            data=(unsigned)((v[TEL_DISTANCE_MM]+5)/10);break;
        case 5: case 6: case 7:
            if (!(available&4U)) return false;
            /* Board IR0 is right, IR1 left, IR2 barcode. */
            data=(unsigned)v[id==5?TEL_IR1:id==6?TEL_IR0:TEL_IR2];break;
        case 8: case 9: case 10:
            if (!(available&128U)) return false;
            data=(unsigned)(v[TEL_MX_MG+id-8U]+2048);break;
        case 11:
            if (!(available&1U)) return false;
            data=(unsigned)v[TEL_COMMAND]|((unsigned)v[TEL_POWER]<<3U)|((unsigned)v[TEL_MOTOR_STATE]<<10U);break;
        case 12: if (!(available&1U)) return false;
            data=edr_motor_fault_flags((unsigned)v[TEL_MOTOR_ERROR]);break;
        default:
            if (!(available&8U)) return false;
            data=(unsigned)(v[TEL_AX_MG+id-13U]+2048);break;
        }
    } else {
        unsigned subtype=index-15U;id=0;
        switch(subtype) {
        case 0: data=(unsigned)v[TEL_PHASE]|((unsigned)v[TEL_FAULT]<<4U);break;
        case 1: data=(unsigned)v[TEL_BARCODE]|((unsigned)v[TEL_NAVIGATION]<<5U);break;
        case 2: if (!(available&4U))return false; data=(unsigned)v[TEL_LINE_STATE]|((unsigned)v[TEL_BLACK_MASK]<<3U);break;
        case 3: if (!(available&32U))return false; data=(unsigned)(v[TEL_PITCH_CDEG]/100+128);break;
        case 4: if (!(available&32U))return false; data=(unsigned)(v[TEL_ROLL_CDEG]/200+128);break;
        case 5: data=(unsigned)v[TEL_TERRAIN];break;
        case 6: if (!(available&64U))return false; data=(unsigned)((v[TEL_HEIGHT_MM]+2)/5);break;
        case 7: data=(unsigned)((v[TEL_HIGHEST_MM]+2)/5);break;
        case 8: if (!(v[TEL_QUALITY]&1))return false; data=(unsigned)((v[TEL_WIDTH_MM]+2)/5);break;
        case 9: data=(unsigned)v[TEL_QUALITY];break;
        case 10:data=available;break;
        case 11:data=(unsigned)v[TEL_SEQUENCE]&255U;break;
        case 12:data=(unsigned)(v[TEL_DROPPED]>255?255:v[TEL_DROPPED]);break;
        case 13: if (!(v[TEL_QUALITY]&16))return false;data=(unsigned)(v[TEL_SPEED_MM_S]/20+128);break;
        case 14: if (!(available&4U))return false;data=(unsigned)(v[TEL_LINE_ERROR]/10+128);break;
        default:data=(unsigned)v[TEL_SCAN_STATE];break;
        }
        if (data>255U) return false;
        data|=subtype<<8U;
    }
    return edr_encode(id,data,out,2U);
}
#endif
