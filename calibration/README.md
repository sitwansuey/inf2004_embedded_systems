# RoboPico calibration records

The robot's current calibration is the set of defaults in `app_program/robot_config.h`, with the IR endpoints in `app_program/robot_ir_calibration.h`. Change the values there. A git-ignored `app_program/robot_config.local.h` is only for optional per-checkout overrides (template `app_program/robot_config.local.example.h`). The files in this folder are the measurement records behind those values.

- Wheel geometry (2026-11-03): 1,889 left and 1,894 right counts over three revolutions. Use 631 counts/rev, 220 mm rollout and 113 mm tyre-centre spacing (`BOARD_TICKS_PER_REV`, `BOARD_WHEEL_CIRCUMFERENCE_MM`, `BOARD_TRACK_WIDTH_MM`).
- Speed PID and wheel trims: `motor_recalibration_2026-11-06.json` is the accepted straight-drive floor calibration (1088 vs 1094 counts in 2 s at 35%, left trim 86). A 2026-11-09 run then set `BOARD_PID_MAX_CPS` 1950, Ki 40 milli and right trim 93 (left stays 86) so the PID targets match what the feedforward delivers; the comment above the PID settings in `robot_config.h` gives the numbers. `BOARD_PID_CALIBRATED` is 1.
- IR endpoints (right, left, barcode) are in `app_program/robot_ir_calibration.h`: white 187/189/197, black 3214/3177/2472. The robot runs `BOARD_IR_MODE` 3 (calibrated ADC line on GP26/27, digital GP28 barcode).
- IMU mounting and level zeros (2026-11-11, robot frame) are `BOARD_IMU_*_AXIS`, `BOARD_PITCH_ZERO_CDEG` and `BOARD_ROLL_ZERO_CDEG` in `robot_config.h`. `imu_level_baseline_20261106.json` is the earlier sensor-frame baseline they replaced.
- Magnetometer: `imu_heading_calibration_20261106.json` and `imu_heading_calibrated_20261106.h` record a validated hard/soft-iron fit. The firmware does not use a compass heading.
- `motor_speed_calibrated_20261106.h` is a groupmate's motor-reference calibration, kept as a reference for porting their motor regulation; the firmware does not include it. `navigation.md` is the 2026-11-06 avoidance and heading-assist design note (historical).

History: the 2026-11-03/04 USB bench checkpoint (trims 73/98, `BOARD_PID_MAX_CPS` 2500, Ki 10 and a provisional right-PWM factor) was superseded by the records above. Its bench-only config snapshot and bench build profiles have been removed.
