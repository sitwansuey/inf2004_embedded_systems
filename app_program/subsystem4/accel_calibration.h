#ifndef S4_ACCEL_CALIBRATION_H
#define S4_ACCEL_CALIBRATION_H
#include "mounting.h"
/* Measured per-sensor correction, applied before body-axis mapping. Bias is
 * nominal mg; gain is dimensionless thousandths. No coefficients are inferred
 * from a single flat sample. Inactive means use factory nominal scaling. */
typedef struct __attribute__((packed)) {
    int16_t bias_x:10, bias_y:10, bias_z:10; /* -500..500 mg */
    uint16_t gain_x:10, gain_y:10, gain_z:10; /* gain - 500: 0..1000 */
    uint8_t active:1;
} s4_accel_calibration_t;
static inline bool s4_calibrated_body(const int32_t raw[3],unsigned mounting,
    const s4_accel_calibration_t *c,int32_t body[3]) {
    if(!raw || !body || !c)return false;
    int32_t bias[3]={c->bias_x,c->bias_y,c->bias_z};
    int32_t gain[3]={c->gain_x+500,c->gain_y+500,c->gain_z+500},corrected[3]={0};
    for(unsigned n=0;n<3;++n) {
        if(raw[n]<-2048 || raw[n]>2047 || bias[n]<-500 || bias[n]>500 ||
           gain[n]<500 || gain[n]>1500)return false;
        int64_t product=(int64_t)(raw[n]-bias[n])*gain[n];
        corrected[n]=c->active?(int32_t)((product+(product>=0?500:-500))/1000):raw[n];
    }
    return s4_body_acceleration(corrected,mounting,body);
}
#define S4_BOARD_ACCEL_CALIBRATION { \
    .bias_x=BOARD_ACCEL_BIAS_X_MG,.bias_y=BOARD_ACCEL_BIAS_Y_MG,.bias_z=BOARD_ACCEL_BIAS_Z_MG, \
    .gain_x=BOARD_ACCEL_GAIN_X_MILLI-500,.gain_y=BOARD_ACCEL_GAIN_Y_MILLI-500, \
    .gain_z=BOARD_ACCEL_GAIN_Z_MILLI-500,.active=BOARD_ACCEL_CALIBRATED }
#endif
