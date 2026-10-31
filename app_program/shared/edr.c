#include "edr.h"

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

