/* Source compatibility only: all definitions and validation live in shared EDR. */
#ifndef S1_PACKED16_H
#define S1_PACKED16_H
#include "../shared/edr.h"
#define S1_PACKED16_TOPIC EDR_MQTT_TOPIC
#define S1_SENSOR_ENCODER_LEFT EDR_ENCODER_LEFT
#define S1_SENSOR_ENCODER_RIGHT EDR_ENCODER_RIGHT
#define S1_SENSOR_SERVO_ANGLE EDR_SERVO_ANGLE
#define S1_SENSOR_ULTRASONIC_CM EDR_ULTRASONIC_CM
#define S1_SENSOR_IR_LEFT EDR_IR_LEFT
#define S1_SENSOR_IR_RIGHT EDR_IR_RIGHT
#define S1_SENSOR_IR_BARCODE EDR_IR_BARCODE
#define S1_SENSOR_MAG_X EDR_MAG_X
#define S1_SENSOR_MAG_Y EDR_MAG_Y
#define S1_SENSOR_MAG_Z EDR_MAG_Z
#define s1_pack16 edr_pack
#define s1_unpack16 edr_unpack
static inline void s1_pack16_bytes(uint16_t word, unsigned char bytes[2])
{
    bytes[0] = (unsigned char)(word & 255U);
    bytes[1] = (unsigned char)(word >> 8U);
}
#endif
