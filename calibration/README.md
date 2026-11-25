# Robot calibration records

The current mission defaults are in `app_program/robot_config.h`; ADC
endpoints are in `app_program/robot_ir_calibration.h`. Optional per-checkout
overrides use the git-ignored `app_program/robot_config.local.h`, with
`app_program/robot_config.local.example.h` as the example. A value in that
local file overrides the corresponding default.

See [TEAM.md](../TEAM.md) for wiring and ownership, and
[navigation.md](navigation.md) for the current avoidance sequence.

## Current settings

| Setting | Default | Source |
|---|---|---|
| Wheel geometry | 631 x1 counts/revolution, 220 mm circumference, 113 mm track width | `BOARD_TICKS_PER_REV`, `BOARD_WHEEL_CIRCUMFERENCE_MM`, `BOARD_TRACK_WIDTH_MM` |
| Speed PID | Enabled and marked calibrated; scale 1950 counts/s; Kp/Ki/Kd 30/40/0 milli | `BOARD_PID_ENABLED`, `BOARD_PID_CALIBRATED`, `BOARD_PID_MAX_CPS`, `BOARD_PID_KP_MILLI`, `BOARD_PID_KI_MILLI`, `BOARD_PID_KD_MILLI` |
| Wheel trims | Left 86%, right 93% | `S2_MOTOR1_TRIM_PERCENT`, `S2_MOTOR2_TRIM_PERCENT` |
| IR mode | Mode 3: ADC line sensors, digital barcode sensor | `BOARD_IR_MODE` |
| Stored ADC endpoints, right/left/barcode order | White 187/189/197; black 3214/3177/2472 | `BOARD_IR_ADC_WHITE`, `BOARD_IR_ADC_BLACK` |
| IMU axes, robot front/left/up | Sensor −Y / +X / +Z | `BOARD_IMU_FRONT_AXIS`, `BOARD_IMU_LEFT_AXIS`, `BOARD_IMU_UP_AXIS` |
| Level zeros | Pitch +1538 cdeg, roll −340 cdeg | `BOARD_PITCH_ZERO_CDEG`, `BOARD_ROLL_ZERO_CDEG` |
| Servo | Centre 1554 µs; change of 750 µs per 90° | `BOARD_SERVO_CENTER_US`, `BOARD_SERVO_US_PER_90` |

The PID retune is dated 2026-11-09 (scale 1950 counts/s, Ki 40, right trim
93%); the current IMU zeros are dated 2026-11-11 in `robot_config.h`.

In IR mode 3, `robot_ir_config.h` replaces the barcode channel's ADC endpoints
with digital white/black levels; high means black. The stored barcode ADC
endpoints apply to mode 2. The calibration header says the two line endpoints
have not been revalidated after a module replacement. Repeat calibration
after changes to sensors, mounting, lighting or print.

## Saved measurements and reference files

- Wheel geometry, measured 2026-11-03: 1,889 left and 1,894 right x1
  counts over three wheel revolutions (about 631 per revolution), with
  220 mm rollout and 113 mm tyre-centre spacing. The wheel-geometry
  comment in `robot_config.h` points to this record.
- `motor_recalibration_2026-11-06.json` records straight-floor trials. Its
  accepted 35% trial measured 1088 left and 1094 right counts in 2 s. Its
  scale, right trim and gains predate the current defaults; the comments
  beside the PID settings in `robot_config.h` explain the later changes.
- `imu_level_baseline_20261106.json` records an earlier sensor-frame level
  baseline. Current mounting and robot-frame zeros are in `robot_config.h`.
- `imu_heading_calibration_20261106.json` and
  `imu_heading_calibrated_20261106.h` preserve a magnetic fit and static
  manual heading checks. The firmware does not include that header or use
  magnetic heading assistance. The GY-511 has acceleration and magnetic
  field sensors, with no gyroscope. The saved fit uses 223 samples, offsets
  365/−100/−155 mG and scales 959/953/1102; its manual checks report a
  7.21° worst error and 3.59° return-to-zero error.
- `motor_speed_calibrated_20261106.h` is a groupmate's motor-reference
  profile, supplied on 2026-11-06. The firmware does not include it; only
  the host check `testing/integration/groupmate_motor_test.c` does, and
  that check is not in the `test` target. Use `robot_config.h` for active
  settings.

Saved calibration is evidence for the conditions recorded, not proof of
course performance. Record new measurements and their tested commit in
[testing/](../testing/README.md), with logs and any unlisted YouTube evidence.
