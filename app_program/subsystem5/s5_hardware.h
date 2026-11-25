/* Private S5 hardware interface. Only the S5 task may access these drivers. */
#ifndef S5_HARDWARE_H
#define S5_HARDWARE_H
#include "subsystem5.h"
/* Servo slope: pulse change for 90 deg of travel (robot_config.h may set it;
 * calibrate it with the +-45 and +-90 deg checks). */
#ifndef BOARD_SERVO_US_PER_90
#define BOARD_SERVO_US_PER_90 750
#endif
#if BOARD_SERVO_US_PER_90 < 500 || BOARD_SERVO_US_PER_90 > 1000
#error BOARD_SERVO_US_PER_90 is the servo pulse change for 90 deg: 500..1000 us
#endif
/* Angle (positive left) to pulse: centre_us at 0 deg, us_per_90 per 90 deg,
 * angles limited to +-90 deg and pulses to the 750..2250 us servo range. */
static inline INT
s5_servo_pulse_us (INT left_deg, INT centre_us, INT us_per_90)
{
    if (left_deg < -90)
    {
        left_deg = -90;
    }
    if (left_deg > 90)
    {
        left_deg = 90;
    }
    INT us = centre_us + (us_per_90 * left_deg) / 90;
    if (us < 750)
    {
        us = 750;
    }
    if (us > 2250)
    {
        us = 2250;
    }
    return us;
}
IMPORT ER               s5_hw_timer_init (void);
IMPORT void             s5_hw_io_init (void);
IMPORT void             s5_hw_servo (INT angle);
IMPORT void             s5_hw_sample (s5_status_t * sample);
IMPORT s5_echo_result_t s5_hw_measure_raw (UW * mm, UW * echo_us);
#endif

/*** end of file ***/
