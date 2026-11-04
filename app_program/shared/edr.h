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

/* External adapters return pairs of independent EDR16 records, not one
 * four-byte record. Publish each two-byte slice as its own MQTT payload. */
#define EDR16_MOTION_SIZE (2U * EDR_RECORD_SIZE)
#define EDR16_ENCODER_SIZE (2U * EDR_RECORD_SIZE)

/* Internal firmware snapshot codecs retain full-width counts and sample
 * times for odometry. These payloads and their compatibility frame envelope
 * are distinct from EDR16 and must never be published on EDR_MQTT_TOPIC. */
#define EDR_VERSION             1U
#define EDR_HEADER_SIZE         2U
#define EDR_PAYLOAD_MAX         255U
#define EDR_MOTION_SIZE         7U
#define EDR_ENCODER_SIZE        13U
#define EDR_SOURCE_MOTOR        2U
#define EDR_KIND_MOTION         0U
#define EDR_KIND_ENCODER        1U
#define EDR_ENCODER_VALID       0x01U
#define EDR_ENCODER_LEFT_A      0x02U
#define EDR_ENCODER_LEFT_B      0x04U
#define EDR_ENCODER_RIGHT_A     0x08U
#define EDR_ENCODER_RIGHT_B     0x10U
#define EDR_ENCODER_MASK        0x1fU

/* Transient local codec views; never copy native structs onto a transport. */
typedef struct
{
    uint16_t motor;
    uint8_t faults;
    uint32_t command_ms;
} edr_motion_t;

typedef struct
{
    uint32_t left_counts;
    uint32_t right_counts;
    uint32_t sampled_ms;
    uint8_t flags;
} edr_encoder_t;

typedef struct
{
    uint8_t source;
    uint8_t kind;
    size_t payload_size;
    const uint8_t *p_payload;
} edr_frame_t;

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

/* Internal little-endian payloads, not external EDR16 records. Encoders
 * accept capacity >= the fixed size; decoders require the exact size.
 * Buffers and codec objects must be distinct. False leaves outputs untouched.
 * An unavailable encoder snapshot has one canonical representation: all zero. */
bool edr_motion_encode(const edr_motion_t *motion, uint8_t *payload, size_t capacity);
bool edr_motion_decode(const uint8_t *payload, size_t size, edr_motion_t *motion);
bool edr_encoder_encode(const edr_encoder_t *encoder, uint8_t *payload, size_t capacity);
bool edr_encoder_decode(const uint8_t *payload, size_t size, edr_encoder_t *encoder);

/* Compatibility envelope for local snapshots and owner-specific codecs.
 * This is not an MQTT EDR16 frame. Source 1..5, kind 0..7, size 0..255.
 * Frame validation checks the envelope only; use the owner's payload decoder.
 * Decoding borrows the input payload; encoding payload and packet cannot
 * overlap. False leaves every output untouched. */
bool edr_frame_encode(uint8_t source, uint8_t kind,
                      const uint8_t *payload, size_t payload_size,
                      uint8_t *packet, size_t capacity, size_t *size);
bool edr_frame_decode(const uint8_t *packet, size_t size, edr_frame_t *frame);
/* Local controller arithmetic uses full-width snapshots, not EDR16 counts. */
bool edr_count_delta(uint32_t current, uint32_t previous, int32_t *delta);
bool edr_age_is_fresh(uint32_t now_ms, uint32_t sampled_ms, uint32_t maximum_ms);
#endif
