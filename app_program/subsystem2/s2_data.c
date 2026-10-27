/*
 * s2_data.c
 * Snapshot adapters; the shared codec owns all byte order and bit layout.
 */

#include "s2_data.h"
#include "subsystem2.h"

ER s2_motion_encode(uint8_t * const p_payload, const size_t capacity)
{
    s2_motion_status_t status;
    edr_motion_t motion = {0};
    uint8_t payload[EDR_MOTION_SIZE];
    ER result;

    if ((NULL == p_payload) || (capacity < EDR_MOTION_SIZE))
    {
        return E_PAR;
    }

    result = s2_motion_get_status(&status);
    if (result < E_OK)
    {
        return result;
    }

    /* The snapshot getter has decoded validated motor-word ranges. */
    if (!edr_motor_pack((uint8_t)status.command,
                        (uint8_t)status.power_percent,
                        (uint8_t)status.state, &motion.motor))
    {
        return E_SYS;
    }

    if (E_TMOUT == status.last_error)
    {
        motion.faults = EDR_FAULT_TIMEOUT;
    }
    else if (status.last_error < E_OK)
    {
        motion.faults = EDR_FAULT_MOTOR;
    }
    else
    {
        /* No motor-command fault is present in this snapshot. */
    }

    motion.command_ms = status.last_command_ms;
    if (!edr_motion_encode(&motion, payload, sizeof(payload)))
    {
        return E_SYS;
    }

    for (size_t index = 0U; index < EDR_MOTION_SIZE; ++index)
    {
        p_payload[index] = payload[index];
    }

    return E_OK;
}

ER s2_encoder_encode(uint8_t * const p_payload, const size_t capacity)
{
    s2_encoder_status_t status;
    edr_encoder_t encoder = {0};
    uint8_t payload[EDR_ENCODER_SIZE];
    ER result;

    if ((NULL == p_payload) || (capacity < EDR_ENCODER_SIZE))
    {
        return E_PAR;
    }

    result = s2_encoder_get_status(&status);
    if (result < E_OK)
    {
        return result;
    }

    if (0U != status.valid)
    {
        /* Unsigned subtraction preserves the count modulo 2^32. */
        encoder.left_counts = status.motor1_negative_pulses -
                              status.motor1_positive_pulses;
        encoder.right_counts = status.positive_pulses - status.negative_pulses;
        encoder.sampled_ms = status.sampled_ms;
        /* The getter returns each GPIO level as a single validated bit. */
        encoder.flags = (uint8_t)(EDR_ENCODER_VALID |
                         (status.motor1_a_level << 1U) |
                         (status.motor1_b_level << 2U) |
                         (status.a_level << 3U) | (status.b_level << 4U));
    }

    if (!edr_encoder_encode(&encoder, payload, sizeof(payload)))
    {
        return E_SYS;
    }

    for (size_t index = 0U; index < EDR_ENCODER_SIZE; ++index)
    {
        p_payload[index] = payload[index];
    }

    return E_OK;
}

/* End of file. */
