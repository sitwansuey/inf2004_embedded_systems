#ifndef VEHICLE_OBSTACLE_CONFIRM_H
#define VEHICLE_OBSTACLE_CONFIRM_H
#include <stdbool.h>
#include <stdint.h>
/* An obstacle counts only after `required` consecutive NEW forward sonar
 * samples are near. S5 measures every ~65 ms while the vehicle task polls
 * every 10-20 ms, so a sample is identified by its time and re-reading it
 * never counts twice. Any other new sample (far, open, no echo, off-axis)
 * breaks the streak. A failed status read changes nothing. */
typedef struct {
    uint32_t sample_ms;
    uint8_t streak;
    uint8_t seen : 1;
} vehicle_obstacle_confirm_t;

static inline bool vehicle_obstacle_confirm_step(vehicle_obstacle_confirm_t *c, bool read,
    uint32_t sample_ms, bool near, unsigned required)
{
    if (read && (!c->seen || sample_ms != c->sample_ms)) {
        c->seen = 1U;
        c->sample_ms = sample_ms;
        c->streak = near ? (uint8_t)(c->streak < 255U ? c->streak + 1U : 255U) : 0U;
    }
    return required && c->streak >= required;
}
#endif
