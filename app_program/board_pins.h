/* Motor wiring from the updated subsystem2 hardware implementation. */
#ifndef BOARD_PINS_H
#define BOARD_PINS_H
#include "robot_config.h"

#ifndef BOARD_MOTOR1_A_GPIO
#define BOARD_MOTOR1_A_GPIO 8U
#endif
#ifndef BOARD_MOTOR1_B_GPIO
#define BOARD_MOTOR1_B_GPIO 9U
#endif
#ifndef BOARD_MOTOR2_A_GPIO
#define BOARD_MOTOR2_A_GPIO 10U
#endif
#ifndef BOARD_MOTOR2_B_GPIO
#define BOARD_MOTOR2_B_GPIO 11U
#endif

/* Motor 1 is the left wheel; motor 2 is the right wheel. */
#ifndef BOARD_MOTOR1_ENCODER_A_GPIO
#define BOARD_MOTOR1_ENCODER_A_GPIO 16U
#endif
#ifndef BOARD_MOTOR1_ENCODER_B_GPIO
#define BOARD_MOTOR1_ENCODER_B_GPIO 17U
#endif
#ifndef BOARD_MOTOR2_ENCODER_A_GPIO
#define BOARD_MOTOR2_ENCODER_A_GPIO 4U
#endif
#ifndef BOARD_MOTOR2_ENCODER_B_GPIO
#define BOARD_MOTOR2_ENCODER_B_GPIO 5U
#endif

#ifndef BOARD_BUZZER_GPIO
#define BOARD_BUZZER_GPIO 22U /* S5 explicit left-acquisition diagnostic only. */
#endif

#endif /* BOARD_PINS_H */
