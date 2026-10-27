/*
 * edr.c
 * Portable byte codec. All wire integers are little-endian.
 */

#include "edr.h"

#include <limits.h>

#if (8 != CHAR_BIT)
#error "EDR requires eight-bit bytes"
#endif

/* stdint.h guarantees the exact widths of uint8_t, uint16_t and uint32_t. */

static void edr_write_u32(uint8_t *p_bytes, uint32_t value)
{
    /* Each mask bounds the unsigned narrowing conversion to one byte. */
    p_bytes[0] = (uint8_t)(value & UINT32_C(0xff));
    p_bytes[1] = (uint8_t)((value >> 8U) & UINT32_C(0xff));
    p_bytes[2] = (uint8_t)((value >> 16U) & UINT32_C(0xff));
    p_bytes[3] = (uint8_t)((value >> 24U) & UINT32_C(0xff));
}

static uint32_t edr_read_u32(const uint8_t *p_bytes)
{
    /* Widen before shifting; no signed promotions or unaligned accesses. */
    return (uint32_t)p_bytes[0] | ((uint32_t)p_bytes[1] << 8U) |
           ((uint32_t)p_bytes[2] << 16U) |
           ((uint32_t)p_bytes[3] << 24U);
}

static bool edr_word_is_valid(uint16_t word)
{
    const uint32_t value = word;

    return (0U == (value & 0xf000U)) && ((value & 0x0007U) <= 6U) &&
           (((value >> 3U) & 0x007fU) <= 100U) &&
           (((value >> 10U) & 0x0003U) <= 2U);
}

bool edr_motor_pack(uint8_t command, uint8_t power, uint8_t state,
                    uint16_t *p_word)
{
    bool b_valid = (NULL != p_word) && (command <= 6U) &&
                   (power <= 100U) && (state <= 2U);

    if (b_valid)
    {
        /* Widen before shifts. Validated fields total 12 unsigned bits. */
        *p_word = (uint16_t)((uint32_t)command |
                            ((uint32_t)power << 3U) |
                            ((uint32_t)state << 10U));
    }

    return b_valid;
}

bool edr_motor_unpack(uint16_t word, uint8_t *p_command,
                      uint8_t *p_power, uint8_t *p_state)
{
    const uint32_t value = word;
    bool b_valid = (NULL != p_command) && (NULL != p_power) &&
                   (NULL != p_state) && (edr_word_is_valid(word));

    if (b_valid)
    {
        /* Masks bound each narrowing conversion to the declared field. */
        *p_command = (uint8_t)(value & 0x0007U);
        *p_power = (uint8_t)((value >> 3U) & 0x007fU);
        *p_state = (uint8_t)((value >> 10U) & 0x0003U);
    }

    return b_valid;
}

bool edr_motion_encode(const edr_motion_t *p_motion,
                       uint8_t *p_payload, size_t capacity)
{
    bool b_valid = (NULL != p_motion) && (NULL != p_payload) &&
                   (capacity >= EDR_MOTION_SIZE);

    if (b_valid)
    {
        b_valid = (edr_word_is_valid(p_motion->motor)) &&
                  (0U == (p_motion->faults & ~EDR_FAULT_MASK));
    }

    if (b_valid)
    {
        const uint32_t motor = p_motion->motor;

        /* Masks bound conversions; motor contains only 12 application bits. */
        p_payload[0] = (uint8_t)(motor & 0xffU);
        p_payload[1] = (uint8_t)((motor >> 8U) & 0xffU);
        p_payload[2] = p_motion->faults;
        edr_write_u32(&p_payload[3], p_motion->command_ms);
    }

    return b_valid;
}

bool edr_motion_decode(const uint8_t *p_payload, size_t size,
                       edr_motion_t *p_motion)
{
    edr_motion_t decoded = {0};
    bool b_valid = (NULL != p_payload) && (NULL != p_motion) &&
                   (EDR_MOTION_SIZE == size);

    if (b_valid)
    {
        /* Widen before shifts; two bytes fit the uint16_t destination. */
        decoded.motor = (uint16_t)((uint32_t)p_payload[0] |
                                   ((uint32_t)p_payload[1] << 8U));
        decoded.faults = p_payload[2];
        decoded.command_ms = edr_read_u32(&p_payload[3]);
        b_valid = (edr_word_is_valid(decoded.motor)) &&
                  (0U == (decoded.faults & ~EDR_FAULT_MASK));
    }

    if (b_valid)
    {
        *p_motion = decoded;
    }

    return b_valid;
}

static bool edr_encoder_is_valid(const edr_encoder_t *p_encoder)
{
    bool b_valid = (0U == (p_encoder->flags & ~EDR_ENCODER_MASK));

    if (0U == (p_encoder->flags & EDR_ENCODER_VALID))
    {
        b_valid = b_valid && (0U == p_encoder->flags) &&
                  (0U == p_encoder->left_counts) &&
                  (0U == p_encoder->right_counts) &&
                  (0U == p_encoder->sampled_ms);
    }

    return b_valid;
}

bool edr_encoder_encode(const edr_encoder_t *p_encoder,
                        uint8_t *p_payload, size_t capacity)
{
    bool b_valid = (NULL != p_encoder) && (NULL != p_payload) &&
                   (capacity >= EDR_ENCODER_SIZE);

    if (b_valid)
    {
        b_valid = edr_encoder_is_valid(p_encoder);
    }

    if (b_valid)
    {
        edr_write_u32(&p_payload[0], p_encoder->left_counts);
        edr_write_u32(&p_payload[4], p_encoder->right_counts);
        edr_write_u32(&p_payload[8], p_encoder->sampled_ms);
        p_payload[12] = p_encoder->flags;
    }

    return b_valid;
}

bool edr_encoder_decode(const uint8_t *p_payload, size_t size,
                        edr_encoder_t *p_encoder)
{
    edr_encoder_t decoded = {0};
    bool b_valid = (NULL != p_payload) && (NULL != p_encoder) &&
                   (EDR_ENCODER_SIZE == size);

    if (b_valid)
    {
        decoded.left_counts = edr_read_u32(&p_payload[0]);
        decoded.right_counts = edr_read_u32(&p_payload[4]);
        decoded.sampled_ms = edr_read_u32(&p_payload[8]);
        decoded.flags = p_payload[12];
        b_valid = edr_encoder_is_valid(&decoded);
    }

    if (b_valid)
    {
        *p_encoder = decoded;
    }

    return b_valid;
}

bool edr_frame_encode(uint8_t source, uint8_t kind,
                      const uint8_t *p_payload, size_t payload_size,
                      uint8_t *p_packet, size_t capacity, size_t *p_size)
{
    size_t index;
    bool b_valid = (source >= 1U) && (source <= 5U) && (kind <= 7U) &&
                   (payload_size <= EDR_PAYLOAD_MAX) &&
                   ((NULL != p_payload) || (0U == payload_size)) &&
                   (NULL != p_packet) && (NULL != p_size);

    /* Check the payload bound before addition, avoiding size_t overflow. */
    if (b_valid)
    {
        b_valid = (capacity >= (EDR_HEADER_SIZE + payload_size));
    }

    if (b_valid)
    {
        /* Widen before shifts; bounded header and length each fit a byte. */
        p_packet[0] = (uint8_t)((EDR_VERSION << 6U) |
                               ((uint32_t)kind << 3U) | (uint32_t)source);
        p_packet[1] = (uint8_t)payload_size;

        for (index = 0U; index < payload_size; index++)
        {
            p_packet[EDR_HEADER_SIZE + index] = p_payload[index];
        }

        *p_size = EDR_HEADER_SIZE + payload_size;
    }

    return b_valid;
}

bool edr_frame_decode(const uint8_t *p_packet, size_t size,
                      edr_frame_t *p_frame)
{
    edr_frame_t decoded = {0};
    bool b_valid = (NULL != p_packet) && (NULL != p_frame) &&
                   (size >= EDR_HEADER_SIZE);

    if (b_valid)
    {
        const uint32_t header = p_packet[0];

        decoded.payload_size = p_packet[1];
        /* Each mask bounds the result to its unsigned field width. */
        decoded.source = (uint8_t)(header & 0x07U);
        decoded.kind = (uint8_t)((header >> 3U) & 0x07U);
        decoded.p_payload = &p_packet[EDR_HEADER_SIZE];
        b_valid = (EDR_VERSION == (header >> 6U)) &&
                  (decoded.source >= 1U) && (decoded.source <= 5U) &&
                  (size == (EDR_HEADER_SIZE + decoded.payload_size));
    }

    if (b_valid)
    {
        *p_frame = decoded;
    }

    return b_valid;
}

bool edr_count_delta(uint32_t current, uint32_t previous, int32_t *p_delta)
{
    uint32_t difference = current - previous;
    bool b_valid = (NULL != p_delta) &&
                   (UINT32_C(0x80000000) != difference);

    if (b_valid && (difference <= UINT32_C(0x7fffffff)))
    {
        /* The branch proves the value fits the signed destination. */
        *p_delta = (int32_t)difference;
    }
    else if (b_valid)
    {
        /* Excluding half-range makes the magnitude <=INT32_MAX. */
        *p_delta = -(int32_t)((~difference) + 1U);
    }
    else
    {
        /* An ambiguous interval has no representable directed result. */
    }

    return b_valid;
}

bool edr_age_is_fresh(uint32_t now_ms, uint32_t sampled_ms,
                      uint32_t maximum_ms)
{
    return (maximum_ms < UINT32_C(0x80000000)) &&
           ((now_ms - sampled_ms) <= maximum_ms);
}

/* End of file. */

