/* Repository-wide EDR16 v1. Every external record is exactly two bytes.
 * Low nibble = reading ID; high 12 bits = value. See docs/DATA_REPRESENTATION.md.
 * No envelope, native struct copy, timestamp, or implicit byte order. */
#ifndef EDR_H
#define EDR_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#define EDR_RECORD_SIZE 2U
#define EDR_DATA_MAX 4095U
#define EDR_MQTT_TOPIC "Robopico/sensors/edr16/v1"
enum {
    EDR_ENCODER_LEFT = 1, EDR_ENCODER_RIGHT, EDR_SERVO_ANGLE,
    EDR_ULTRASONIC_CM, EDR_IR_LEFT, EDR_IR_RIGHT, EDR_IR_BARCODE,
    EDR_MAG_X, EDR_MAG_Y, EDR_MAG_Z, EDR_MOTOR_STATUS, EDR_MOTOR_FAULTS,
    EDR_ACCEL_X, EDR_ACCEL_Y, EDR_ACCEL_Z
};
#define EDR_FAULT_STALL         0x01U
#define EDR_FAULT_ENCODER       0x02U
#define EDR_FAULT_IMU           0x04U
#define EDR_FAULT_NO_ECHO       0x08U
#define EDR_FAULT_WIFI          0x10U
#define EDR_FAULT_TIMEOUT       0x20U
#define EDR_FAULT_MOTOR         0x40U
#define EDR_FAULT_MASK          0x7fU

/* Each adapter returns a pair of independent EDR16 records, not one 32-bit
 * record. Publish each two-byte slice as its own MQTT payload. */
#define EDR_MOTION_SIZE (2U * EDR_RECORD_SIZE)
#define EDR_ENCODER_SIZE (2U * EDR_RECORD_SIZE)

static inline bool edr_value_valid(unsigned int id, unsigned int data)
{
    if (id < 1U || id > 15U || data > EDR_DATA_MAX) return false;
    if (id == EDR_SERVO_ANGLE && data > 180U) return false;
    if (id == EDR_MOTOR_FAULTS && data > EDR_FAULT_MASK) return false;
    if (id == EDR_MOTOR_STATUS &&
        ((data & 7U) > 6U || ((data >> 3U) & 127U) > 100U ||
         (data >> 10U) > 2U)) return false;
    return true;
}
/* Error leaves every output untouched. Wider inputs are checked before narrowing. */
static inline bool edr_pack(unsigned int id, unsigned int data, uint16_t *word)
{
    if (!word || !edr_value_valid(id, data)) return false;
    *word = (uint16_t)((data << 4U) | id);
    return true;
}
static inline bool edr_unpack(uint16_t word, unsigned int *id, unsigned int *data)
{
    unsigned int sensor = word & 15U, value = word >> 4U;
    if (!id || !data || !edr_value_valid(sensor, value)) return false;
    *id = sensor; *data = value;
    return true;
}
static inline bool edr_encode(unsigned int id, unsigned int data,
                              uint8_t *bytes, size_t capacity)
{
    uint16_t word;
    if (!bytes || capacity < EDR_RECORD_SIZE || !edr_pack(id, data, &word)) return false;
    bytes[0] = (uint8_t)(word & 255U);
    bytes[1] = (uint8_t)(word >> 8U);
    return true;
}
static inline bool edr_decode(const uint8_t *bytes, size_t size,
                              unsigned int *id, unsigned int *data)
{
    uint16_t word;
    if (!bytes || size != EDR_RECORD_SIZE) return false;
    word = (uint16_t)((uint16_t)bytes[0] | ((uint16_t)bytes[1] << 8U));
    return edr_unpack(word, id, data);
}
/* Motor's internal 12-bit value: command[2:0], power[9:3], state[11:10].
 * This is DATA for ID 11, not a complete EDR16 record. */
bool edr_motor_pack(uint8_t command, uint8_t power, uint8_t state, uint16_t *word);
bool edr_motor_unpack(uint16_t word, uint8_t *command, uint8_t *power, uint8_t *state);
/* Local controller arithmetic uses full-width snapshots, not EDR16 counts. */
bool edr_count_delta(uint32_t current, uint32_t previous, int32_t *delta);
bool edr_age_is_fresh(uint32_t now_ms, uint32_t sampled_ms, uint32_t maximum_ms);
#endif
