#include "result_data.h"
#include "packed_bits.h"
static bool valid(const s4_result_t *v) {
    return v && v->pitch_cdeg>=-9000 && v->pitch_cdeg<=9000 &&
        v->roll_cdeg>=-18000 && v->roll_cdeg<=18000 &&
        v->height_mm<=1000U && v->maximum_mm<=1000U && v->terrain<=5U &&
        !(v->flags&~S4_RESULT_FLAG_MASK) && !(v->faults&~S4_RESULT_FAULT_MASK) &&
        (!(v->faults&EDR_FAULT_IMU) || !(v->flags&S4_RESULT_TILT)) &&
        (!v->faults || !(v->flags&S4_RESULT_HEIGHT)) &&
        (!(v->flags&S4_RESULT_HEIGHT) ||
         ((v->flags&(S4_RESULT_TILT|S4_RESULT_CALIBRATED|S4_RESULT_ODOMETRY))==
          (S4_RESULT_TILT|S4_RESULT_CALIBRATED|S4_RESULT_ODOMETRY) &&
          v->height_mm>0U && v->maximum_mm>=v->height_mm)) &&
        (!(v->flags&S4_RESULT_TILT) ||
         ((v->flags&S4_RESULT_CALIBRATED) && !(v->flags&S4_RESULT_SHOCK)));
}
bool s4_result_encode(const s4_result_t *v,uint8_t *out,size_t capacity) {
    uint8_t bytes[S4_RESULT_SIZE]={0};
    if(!out || capacity<S4_RESULT_SIZE || !valid(v))return false;
    s4_bits_put(bytes,0,32,v->calculated_ms);
    s4_bits_put(bytes,32,15,(uint32_t)(v->pitch_cdeg+9000));
    s4_bits_put(bytes,47,16,(uint32_t)(v->roll_cdeg+18000));
    s4_bits_put(bytes,63,10,v->height_mm);
    s4_bits_put(bytes,73,10,v->maximum_mm);
    s4_bits_put(bytes,83,3,v->terrain);
    s4_bits_put(bytes,86,5,v->flags);
    s4_bits_put(bytes,91,2,v->faults>>1U);
    for(unsigned n=0;n<S4_RESULT_SIZE;++n)out[n]=bytes[n];
    return true;
}
bool s4_result_decode(const uint8_t *bytes,size_t length,s4_result_t *out) {
    if(!bytes || !out || length!=S4_RESULT_SIZE || (bytes[11]&0xe0U))return false;
    s4_result_t v={0};
    v.calculated_ms=(uint32_t)s4_bits_get(bytes,0,32);
    v.pitch_cdeg=(int32_t)s4_bits_get(bytes,32,15)-9000;
    v.roll_cdeg=(int32_t)s4_bits_get(bytes,47,16)-18000;
    v.height_mm=(uint16_t)s4_bits_get(bytes,63,10);
    v.maximum_mm=(uint16_t)s4_bits_get(bytes,73,10);
    v.terrain=(uint8_t)s4_bits_get(bytes,83,3);
    v.flags=(uint8_t)s4_bits_get(bytes,86,5);
    v.faults=(uint8_t)(s4_bits_get(bytes,91,2)<<1U);
    if(!valid(&v))return false;
    *out=v;return true;
}
