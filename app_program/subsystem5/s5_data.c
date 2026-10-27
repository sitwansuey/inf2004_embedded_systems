#include "s5_data.h"
#include <string.h>

bool s5_data_encode(const s5_status_t *v, uint8_t *data, size_t capacity)
{
    uint8_t result[S5_DATA_SIZE] = {0};
    if (!v || !data || capacity < S5_DATA_SIZE ||
        (unsigned)v->state > S5_REORG ||
        (unsigned)v->echo_result > S5_ECHO_TOO_SHORT)
        return false;
    if (v->angle_deg < -90 || v->angle_deg > 90) return false;
    if (v->sample_angle_deg < -90 || v->sample_angle_deg > 90) return false;
    if (v->avoidance_deg < -90 || v->avoidance_deg > 90) return false;
    if (v->planned_rotate_deg < -90 || v->planned_rotate_deg > 90) return false;
    if (v->left_bound_deg < -90 || v->left_bound_deg > 90) return false;
    if (v->right_bound_deg < -90 || v->right_bound_deg > 90) return false;
    if (v->acquisition_clear != FALSE && v->acquisition_clear != TRUE) return false;
    if (v->approach_plan_valid != FALSE && v->approach_plan_valid != TRUE) return false;
    if (v->encoder_valid != FALSE && v->encoder_valid != TRUE) return false;
    if (v->valid != FALSE && v->valid != TRUE) return false;
    if (v->left_bound_valid != FALSE && v->left_bound_valid != TRUE) return false;
    if (v->right_bound_valid != FALSE && v->right_bound_valid != TRUE) return false;
    if (v->object_width_valid != FALSE && v->object_width_valid != TRUE) return false;
    result[0] = (uint8_t)((unsigned)v->state | ((unsigned)v->echo_result << 4));
    result[1] |= (uint8_t)((unsigned)v->acquisition_clear << 0);
    result[1] |= (uint8_t)((unsigned)v->approach_plan_valid << 1);
    result[1] |= (uint8_t)((unsigned)v->encoder_valid << 2);
    result[1] |= (uint8_t)((unsigned)v->valid << 3);
    result[1] |= (uint8_t)((unsigned)v->left_bound_valid << 4);
    result[1] |= (uint8_t)((unsigned)v->right_bound_valid << 5);
    result[1] |= (uint8_t)((unsigned)v->object_width_valid << 6);
    result[2] = (uint8_t)(v->angle_deg + 90);
    result[3] = (uint8_t)(v->sample_angle_deg + 90);
    result[4] = (uint8_t)(v->avoidance_deg + 90);
    result[5] = (uint8_t)(v->planned_rotate_deg + 90);
    result[6] = (uint8_t)(v->left_bound_deg + 90);
    result[7] = (uint8_t)(v->right_bound_deg + 90);
    for (unsigned b = 0; b < 4; ++b) result[8 + b] = (uint8_t)((uint32_t)v->scan_id >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[12 + b] = (uint8_t)((uint32_t)v->planned_rotate_ms >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[16 + b] = (uint8_t)((uint32_t)v->planned_approach_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[20 + b] = (uint8_t)((uint32_t)v->planned_return_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[24 + b] = (uint8_t)((uint32_t)v->obstacle_distance_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[28 + b] = (uint8_t)((uint32_t)v->distance_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[32 + b] = (uint8_t)((uint32_t)v->echo_us >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[36 + b] = (uint8_t)((uint32_t)v->time_ms >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[40 + b] = (uint8_t)((uint32_t)v->motor1_counts_start >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[44 + b] = (uint8_t)((uint32_t)v->motor2_counts_start >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[48 + b] = (uint8_t)((uint32_t)v->motor1_counts_end >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[52 + b] = (uint8_t)((uint32_t)v->motor2_counts_end >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[56 + b] = (uint8_t)((uint32_t)v->encoder_start_ms >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[60 + b] = (uint8_t)((uint32_t)v->encoder_end_ms >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[64 + b] = (uint8_t)((uint32_t)v->left_bound_distance_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[68 + b] = (uint8_t)((uint32_t)v->right_bound_distance_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[72 + b] = (uint8_t)((uint32_t)v->object_width_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[76 + b] = (uint8_t)((uint32_t)v->object_width_min_mm >> (8U * b));
    for (unsigned b = 0; b < 4; ++b) result[80 + b] = (uint8_t)((uint32_t)v->object_width_max_mm >> (8U * b));
    memcpy(data, result, sizeof(result));
    return true;
}

bool s5_data_decode(const uint8_t *data, size_t size, s5_status_t *value)
{
    s5_status_t result = {0};
    if (!data || !value || size != S5_DATA_SIZE ||
        (data[0] & 0x80U) || (data[1] & 0x80U) ||
        (data[0] & 15U) > S5_REORG ||
        ((data[0] >> 4) & 7U) > S5_ECHO_TOO_SHORT)
        return false;
    result.state = (s5_state_t)(data[0] & 15U);
    result.echo_result = (s5_echo_result_t)((data[0] >> 4) & 7U);
    result.acquisition_clear = (data[1] >> 0) & 1U;
    result.approach_plan_valid = (data[1] >> 1) & 1U;
    result.encoder_valid = (data[1] >> 2) & 1U;
    result.valid = (data[1] >> 3) & 1U;
    result.left_bound_valid = (data[1] >> 4) & 1U;
    result.right_bound_valid = (data[1] >> 5) & 1U;
    result.object_width_valid = (data[1] >> 6) & 1U;
    if (data[2] > 180U) return false;
    result.angle_deg = (INT)data[2] - 90;
    if (data[3] > 180U) return false;
    result.sample_angle_deg = (INT)data[3] - 90;
    if (data[4] > 180U) return false;
    result.avoidance_deg = (INT)data[4] - 90;
    if (data[5] > 180U) return false;
    result.planned_rotate_deg = (INT)data[5] - 90;
    if (data[6] > 180U) return false;
    result.left_bound_deg = (INT)data[6] - 90;
    if (data[7] > 180U) return false;
    result.right_bound_deg = (INT)data[7] - 90;
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[8 + b] << (8U * b);
        result.scan_id = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[12 + b] << (8U * b);
        result.planned_rotate_ms = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[16 + b] << (8U * b);
        result.planned_approach_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[20 + b] << (8U * b);
        result.planned_return_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[24 + b] << (8U * b);
        result.obstacle_distance_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[28 + b] << (8U * b);
        result.distance_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[32 + b] << (8U * b);
        result.echo_us = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[36 + b] << (8U * b);
        result.time_ms = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[40 + b] << (8U * b);
        result.motor1_counts_start = word <= INT32_MAX ? (INT)word : -1 - (INT)(UINT32_MAX - word);
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[44 + b] << (8U * b);
        result.motor2_counts_start = word <= INT32_MAX ? (INT)word : -1 - (INT)(UINT32_MAX - word);
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[48 + b] << (8U * b);
        result.motor1_counts_end = word <= INT32_MAX ? (INT)word : -1 - (INT)(UINT32_MAX - word);
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[52 + b] << (8U * b);
        result.motor2_counts_end = word <= INT32_MAX ? (INT)word : -1 - (INT)(UINT32_MAX - word);
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[56 + b] << (8U * b);
        result.encoder_start_ms = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[60 + b] << (8U * b);
        result.encoder_end_ms = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[64 + b] << (8U * b);
        result.left_bound_distance_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[68 + b] << (8U * b);
        result.right_bound_distance_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[72 + b] << (8U * b);
        result.object_width_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[76 + b] << (8U * b);
        result.object_width_min_mm = word;
    }
    {
        uint32_t word = 0;
        for (unsigned b = 0; b < 4; ++b) word |= (uint32_t)data[80 + b] << (8U * b);
        result.object_width_max_mm = word;
    }
    *value = result;
    return true;
}
