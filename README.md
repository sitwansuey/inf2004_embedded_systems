# INF2004 CSC02 robot firmware

Pico W / RP2040 firmware using micro T-Kernel 3 SMP. See TEAM.md for hardware
ownership and task/interface responsibilities. The current implementation
includes the five subsystems, controller, binary telemetry and dashboard.

Firmware boots DISARMED with `BOARD_MOTORS_DISABLED=1`. Motor calibration trims
remain 97% (left) and 98% (right). Calibration, physical accuracy, full-course
behavior, target timing and runtime memory still require validation. GY-511 has
an accelerometer and magnetometer, no gyroscope. Motion output uses open-loop
PWM; PID speed regulation, absolute heading and audio playback are absent.

## Build

Install the Pico SDK (including TinyUSB/CYW43 and lwIP for MQTT), Arm GNU
compiler, GNU Make and a host compiler/Python for software tests. From the
repository root on Windows with MSYS2:

```powershell
.\tools\build_windows.ps1 -Backend Native `
  -ArmToolchainBin "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin" `
  -PicoSdkPath "$env:USERPROFILE\.pico-sdk\sdk\2.3.0" -Profile Default
```

Adjust paths to your installation. The helper builds only; it does not flash.
`S4Bench` starts IMU application tasks for sensor-only validation. `Mqtt`
requires ignored `config/wifi_secrets.h` copied from its example. Local board
calibration belongs in ignored `app_program/robot_config.local.h`. UART builds
conflict with sonar pins and disable Buddy 5. Explicit motion diagnostic
profiles have different limits; review their source before use.

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

B2's checked 13-byte encoder payload feeds B4 terrain processing. B4 exposes
protected 63-byte acquisition copies and a checked 12-byte terrain result;
C interfaces and exact field definitions are in subsystem4 headers/code. The
controller alone authorizes motion; B2 owns PWM/encoder IRQ, B3 owns IR, B4
owns I2C and B5 owns sonar/servo. B1 publishes the separate 50-byte aggregate
through the network owner. Shared speed uses `app_speed_mm_s()`. Preserve
explicit lengths, validity/fault flags, operating time, STOP and lease behavior.

Dashboard: run `python app_program/external_systems/Dashboard/dashboard.py --broker YOUR_BROKER`
after installing that folder's requirements. It subscribes to
`csc02/telemetry`; unavailable readings and stale data must remain distinguishable.

The professor's final ramp is approximately the smaller practice ramp's height.
The higher ramp may be disregarded. Measure geometry and calibrate the estimator;
do not pre-enter the expected answer. Supplementary guides/evidence are local
only for now. Required CR1 representation tables, RAM/message comparisons and
correctness/performance evidence must be added to GitHub before final submission.
