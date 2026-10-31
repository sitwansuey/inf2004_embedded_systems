/* Private S5 hardware interface. Only the S5 task may access these drivers. */
#ifndef S5_HARDWARE_H
#define S5_HARDWARE_H
#include "subsystem5.h"
IMPORT ER s5_hw_timer_init(void);
IMPORT void s5_hw_io_init(void);
IMPORT void s5_hw_servo(INT angle);
IMPORT s5_echo_result_t s5_hw_measure(UW *mm);
IMPORT void s5_hw_sample(s5_status_t *sample);
IMPORT s5_echo_result_t s5_hw_measure_raw(UW *mm, UW *echo_us);
#endif
