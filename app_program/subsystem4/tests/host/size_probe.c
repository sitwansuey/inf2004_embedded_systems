/* Baseline definitions reproduce the supplied acquisition layout for sizeof
 * comparison. These are measurement fixtures, never firmware storage. */
#include "../../sensor_data.h"
#include "../../encoder_input.h"
#ifndef S4_ARM_LAYOUT
#include <stdio.h>
#endif
typedef struct {
    bool fresh, valid, overrun;
    lsm303_data_status_t status;
    int16_t raw[3]; int32_t milli_units[3]; uint64_t read_time_us;
} before_sample_t;
typedef struct {
    before_sample_t accel, mag;
} before_frame_t;
typedef struct {
    before_frame_t latest;
    lsm303_result_t init_result, poll_result;
    bool configured, accel_usable, mag_usable;
    uint32_t accel_age_ms, mag_age_ms, init_attempts, task_heartbeat;
    uint32_t accel_samples, mag_samples, accel_io_errors, mag_io_errors;
    uint32_t accel_overruns, publish_errors, legacy_lines_received;
    uint64_t observed_time_us;
} before_status_t;
#ifdef S4_ARM_LAYOUT
/* nm -S reports byte sizes for the real ARM ABI without executing target code. */
char s4_before_sample[sizeof(before_sample_t)];
char s4_after_sample[sizeof(lsm303_sample_t)];
char s4_before_status[sizeof(before_status_t)];
char s4_after_status[sizeof(s4_sensor_status_t)];
char s4_terrain_size[sizeof(terrain_t)];
char s4_input_size[sizeof(s4_encoder_input_t)];
#else
int main(void)
{
    printf("Native sizeof: sample %zu -> %zu; status %zu -> %zu; terrain %zu; encoder input %zu\n",
        sizeof(before_sample_t),sizeof(lsm303_sample_t),sizeof(before_status_t),
        sizeof(s4_sensor_status_t),sizeof(terrain_t),sizeof(s4_encoder_input_t));
    return 0;
}
#endif
