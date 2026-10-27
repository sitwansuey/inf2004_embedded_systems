/*
 * edr.h
 * Shared external data representation, version 1.
 * See docs/DATA_REPRESENTATION.md for the wire contract and ownership rules.
 */

#ifndef EDR_H
#define EDR_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define EDR_VERSION             1U
#define EDR_HEADER_SIZE         2U
#define EDR_PAYLOAD_MAX         255U
#define EDR_MOTION_SIZE         7U
#define EDR_ENCODER_SIZE        13U
#define EDR_SOURCE_MOTOR        2U
#define EDR_KIND_MOTION         0U
#define EDR_KIND_ENCODER        1U

#define EDR_FAULT_STALL         0x01U
#define EDR_FAULT_ENCODER       0x02U
#define EDR_FAULT_IMU           0x04U
#define EDR_FAULT_NO_ECHO       0x08U
#define EDR_FAULT_WIFI          0x10U
#define EDR_FAULT_TIMEOUT       0x20U
#define EDR_FAULT_MOTOR         0x40U
#define EDR_FAULT_MASK          0x7fU
#define EDR_ENCODER_VALID       0x01U
#define EDR_ENCODER_LEFT_A      0x02U
#define EDR_ENCODER_LEFT_B      0x04U
#define EDR_ENCODER_RIGHT_A     0x08U
#define EDR_ENCODER_RIGHT_B     0x10U
#define EDR_ENCODER_MASK        0x1fU

/* Transient codec views; never memcpy these structs onto a transport. */
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

/* Checked 12-bit motor word. Invalid inputs leave outputs untouched.
 * command: 0..6; power: 0..100 percent; state: 0..2.
 * Unpacked values are transient views, not shared mutable storage.
 */
bool edr_motor_pack(uint8_t command, uint8_t power, uint8_t state,
                    uint16_t *p_word);
bool edr_motor_unpack(uint16_t word, uint8_t *p_command,
                      uint8_t *p_power, uint8_t *p_state);

/* Payload encoders accept capacity >= fixed size; decoders require exact size.
 * Each buffer must be distinct from the input/output object. No allocation,
 * RTOS calls, globals, or hardware access. False leaves outputs untouched.
 */
bool edr_motion_encode(const edr_motion_t *p_motion,
                       uint8_t *p_payload, size_t capacity);
bool edr_motion_decode(const uint8_t *p_payload, size_t size,
                       edr_motion_t *p_motion);
bool edr_encoder_encode(const edr_encoder_t *p_encoder,
                        uint8_t *p_payload, size_t capacity);
bool edr_encoder_decode(const uint8_t *p_payload, size_t size,
                        edr_encoder_t *p_encoder);

/* One record per MQTT message. Source 1..5, kind 0..7, length 0..255.
 * Generic framing checks envelope only; call the owner's payload decoder too.
 * Decode returns a borrowed view valid while the input packet is alive.
 * Encoder input payload and packet storage must not overlap.
 */
bool edr_frame_encode(uint8_t source, uint8_t kind,
                      const uint8_t *p_payload, size_t payload_size,
                      uint8_t *p_packet, size_t capacity, size_t *p_size);
bool edr_frame_decode(const uint8_t *p_packet, size_t size,
                      edr_frame_t *p_frame);

/* Modulo-32 count difference; reject the ambiguous half-range (2^31).
 * Caller must ensure fewer than 2^31 actual ticks between samples.
 */
bool edr_count_delta(uint32_t current, uint32_t previous, int32_t *p_delta);

/* Low 32 bits of common kernel milliseconds. Maximum age must be <2^31.
 * Does not distinguish reboots or samples retained for an entire clock wrap.
 */
bool edr_age_is_fresh(uint32_t now_ms, uint32_t sampled_ms,
                      uint32_t maximum_ms);

#endif /* EDR_H */

/* End of file. */

