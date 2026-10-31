/* Public snapshots -> pairs of standard EDR16 records. */
#include "s2_data.h"
#include "subsystem2.h"

ER s2_motion_encode(uint8_t * const output, const size_t capacity)
{
    s2_motion_status_t status;
    uint8_t bytes[EDR_MOTION_SIZE];
    uint16_t motor;
    unsigned int faults = 0U;
    ER result;
    if (!output || capacity < sizeof(bytes)) return E_PAR;
    result = s2_motion_get_status(&status);
    if (result < E_OK) return result;
    /* Validate original widths before casting; never wrap an invalid input. */
    if ((unsigned int)status.command > 6U || status.power_percent > 100U ||
        (unsigned int)status.state > 2U) return E_SYS;
    if (!edr_motor_pack((uint8_t)status.command, (uint8_t)status.power_percent,
                        (uint8_t)status.state, &motor)) return E_SYS;
    if (status.last_error == E_TMOUT) faults = EDR_FAULT_TIMEOUT;
    else if (status.last_error < E_OK) faults = EDR_FAULT_MOTOR;
    if (!edr_encode(EDR_MOTOR_STATUS, motor, bytes, 2U) ||
        !edr_encode(EDR_MOTOR_FAULTS, faults, bytes + 2, 2U)) return E_SYS;
    for (size_t i = 0U; i < sizeof(bytes); ++i) output[i] = bytes[i];
    return E_OK;
}

ER s2_encoder_encode(uint8_t * const output, const size_t capacity)
{
    s2_encoder_status_t status;
    uint8_t bytes[EDR_ENCODER_SIZE];
    uint32_t left, right;
    ER result;
    if (!output || capacity < sizeof(bytes)) return E_PAR;
    result = s2_encoder_get_status(&status);
    if (result < E_OK) return result;
    if (!status.valid) return E_OBJ; /* Do not send fake zeros during startup. */
    left = status.motor1_negative_pulses - status.motor1_positive_pulses;
    right = status.positive_pulses - status.negative_pulses;
    if (!edr_encode(EDR_ENCODER_LEFT, left & EDR_DATA_MAX, bytes, 2U) ||
        !edr_encode(EDR_ENCODER_RIGHT, right & EDR_DATA_MAX, bytes + 2, 2U)) return E_SYS;
    for (size_t i = 0U; i < sizeof(bytes); ++i) output[i] = bytes[i];
    return E_OK;
}
