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
straddle IR on black starts the bounded on-the-spot line search. The
barcode codebook defaults to A U-turn, F left and Z right (the dashboard can
remap letters); START runs one autonomous mission.

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
protected 71-byte acquisition copies and a checked 12-byte terrain result;
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

Checkpoint 8c keeps the tuned follower, ramp powers and avoidance ramps. Plain
descents get the 20% slope cap. A flat hump top keeps the crest cap until
descent and level; its level event uses S4's completed-height latch. S4 counts
brief impacts and IMU faults between controller reads. Impact reporting stays
on and `BOARD_SHOCK_STOP` stays 0 (decision 1).

After a side look the robot waits stopped for a new clear forward sample.
BACKUP waits at most 1.5 s for an APPROACH/0 sample before its full 1 s reverse.

Track changes (2026-11-20): after a clear side look the robot drives straight
for 0.5 s, then returns to the line at 45 deg or more (`AVOID_CLEAR_RUN_MS`,
`AVOID_RETURN_MIN_DEG`). The crest cap is 15% (`BOARD_RAMP_DOWN_POWER`).
A shallower heading after reversing gets a stopped ray check; a blocked or
timed-out check keeps S5's verified heading. Calibrate `BOARD_SERVO_US_PER_90`
(default 750 us) as well as the centre.

Checkpoint 8c also adds a hardware watchdog after two core-0 freezes on the
robot. Once boot has finished, the board reboots (PWM off, DISARMED) if the
VC, motor, network, telemetry or IMU loop stops for over 1 s, or at once
after a panic, SDK/lwIP assert or HardFault. The next boot prints a
`[RESET] ...` line on USB naming the cause. The VC stack is 4096 bytes and
telemetry 3072 (2048 was too small for both). Wi-Fi power save is off.
A repeated ARM at the same demand now renews a leased ARM run, so
`tools/robot_console.py --arm` works as documented. A STOP still ends such
a run for good: ARM is refused until RESET or 1 s with no ARM. Three early
crashes in a row make the next boot hold with nothing started and print
the cause every 2 s. The freeze trigger itself is not yet found.
