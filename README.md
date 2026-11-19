# INF2004 CSC02 robot firmware

Pico W / RP2040 firmware using micro T-Kernel 3 SMP. See TEAM.md for hardware
ownership and task/interface responsibilities. The current implementation
includes the five subsystems, controller, binary telemetry and dashboard.

Firmware boots DISARMED and drives only after RESET then START. The
`app_program/robot_config.h` defaults are this robot's measured mission profile:
motors enabled, wheel trims 86% (left) and 93% (right), calibrated wheel-speed
PID, IR mode 3, terrain calibration and obstacle bypass. Physical accuracy,
full-course behavior, target timing and runtime memory still require
validation. GY-511 has an accelerometer and magnetometer, no gyroscope. The
line follower is the mask-pivot follower; edge tracing is a build option
(`BOARD_LINE_MASK_PIVOT 0`, `BOARD_LINE_EDGE_TRACE 1`). With the mask-pivot
follower, `BOARD_LINE_LOST_MM` (600 mm, 0 = off) of tracking with neither
straddle IR on black starts the bounded on-the-spot line search. See
[mission setup](docs/MISSION_SETUP.md) for calibration, the A-Z barcode
codebook (A U-turn, F left, Z right) and one-shot autonomous START.

## Build

Install the Pico SDK (including TinyUSB/CYW43 and lwIP for MQTT), Arm GNU
compiler, GNU Make and a host compiler/Python for software tests. From the
repository root on Windows with MSYS2, build the MQTT mission image:

```powershell
.\tools\build_windows.ps1 -Backend Native `
  -ArmToolchainBin "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin" `
  -PicoSdkPath "$env:USERPROFILE\.pico-sdk\sdk\2.3.0" -Profile Mqtt
```

Adjust paths to your installation. The helper builds only; it does not flash.
It runs plain `make -C build_make` (here with `NET=lwip MQTT=1`), which also
works directly with `PICO_SDK_PATH` exported and the Arm compiler on PATH; the
mission image is `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc.uf2`.
`Mqtt` requires ignored `config/wifi_secrets.h` copied from its example. The
other profiles are `Default` (no MQTT), `PassiveTest`, `SingleCore`, `Uart`,
`S2Bench` and `S4Bench`; `S4Bench` starts IMU application tasks for sensor-only
validation. Optional per-checkout overrides go in the ignored
`app_program/robot_config.local.h` (see `robot_config.local.example.h`). UART
builds conflict with sonar pins and disable Buddy 5.

## Software checks

Run these in a shell with GCC, GNU Make, Python and Arm GNU on PATH:

```sh
python -B testing/data_representation/run.py
make -C app_program/subsystem4/tests/host analyze sizes
make -C testing/integration test
make -C testing/subsystem3 test analyze check-headers arm-check verify-failure
```

Linux CI also runs ASan/UBSan; some native Windows GCC distributions lack those
runtimes. Test fixtures, schemas and CI definitions are tracked implementation
inputs. Generated reports, captures and supplementary documents stay local.

## Interfaces

B2's checked 13-byte internal encoder snapshot feeds B4 terrain processing. B4 exposes
protected 63-byte acquisition copies and a checked 12-byte terrain result;
C interfaces and exact field definitions are in subsystem4 headers/code. The
controller alone authorizes motion; B2 owns PWM/encoder IRQ, B3 owns IR, B4
owns I2C and B5 owns sonar/servo. B1 queues a private 55-byte snapshot, then the
network owner marshals each reading into its own two-byte EDR16 MQTT message.
The private snapshot is never published. Shared speed uses `app_speed_mm_s()`. Preserve
explicit lengths, validity/fault flags, operating time, STOP and lease behavior.

Dashboard: run `python app_program/external_systems/Dashboard/dashboard.py --broker YOUR_BROKER`
after installing that folder's requirements. It subscribes to
`Robopico/sensors/edr16/v1` (telemetry), `Robopico/events/edr16/v1` (the robot
event log) and `csc02/command/ack`; each reading keeps its latest value and is
marked stale after 5 s. ID 0 carries mission status, while sensor IDs 1-15 retain
their meanings. See
[the complete wire contract](docs/DATA_REPRESENTATION.md).

The professor's final ramp is approximately the smaller practice ramp's height.
The higher ramp may be disregarded. Measure geometry and calibrate the estimator;
do not pre-enter the expected answer. Supplementary guides/evidence are local
only for now. Required CR1 representation tables, RAM/message comparisons and
correctness/performance evidence must be added to GitHub before final submission.
