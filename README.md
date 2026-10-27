# INF2004 CSC02 local robot integration

This branch integrates the five subsystems on micro T-Kernel 3 and implements
Change Requirement 1. Firmware boots DISARMED. It needs measured board
configuration before motion; software tests do not establish robot accuracy.
Drive is also locked at build time by `BOARD_MOTORS_DISABLED=1`. Motor inputs
stay GPIO-low, motor PWM is not enabled and movement/ARM commands are rejected.
This takes effect on the board only after the updated firmware is built and flashed.

Start with [integration evidence](docs/INTEGRATION_REPORT.md),
[CR1 binary contract](docs/CR1_PROTOCOL.md), [ownership](TEAM.md), and
[hardware acceptance](docs/HARDWARE_ACCEPTANCE.md).

For the 27 October integration of latest main, the handoff and Buddy 4, start
with [current implementation and ownership](docs/BUDDY4_IMPLEMENTATION.md),
[physical testing](docs/BUDDY4_TESTING.md) and [S4 storage review](docs/BUDDY4_CR1_STORAGE.md).
Those records distinguish current checks from the historical evidence below.
The Windows helper also supports `-Backend Native` using the compiler and SDK
already installed by the Pico VS Code extension; see the physical testing guide
for this machine's exact command. WSL support remains available.

## Windows build

Use WSL Ubuntu with make, a native compiler, arm-none-eabi-gcc and an existing
Pico SDK containing tinyusb, cyw43-driver and (for MQTT) lwIP. The local checks
used ARM GCC 14.2.1, native GCC 15.2.0 and Pico SDK 2.3.1.

From the checkout's PowerShell terminal:

```powershell
.\tools\build_windows.ps1 -PicoSdkPath 'C:\path to\pico-sdk' -Profile Default
```

The helper safely aliases SDK paths containing spaces inside WSL, builds in
`build_make/`, and never downloads dependencies or flashes. Profiles are
Default, Mqtt, PassiveTest, SingleCore, Uart, S2Bench, S4Bench, IRRawAdc,
IRRawDigital, IRCalibrated and IRBarcodeCapture. The calibrated profiles use this robot's measured endpoints
under the mandatory drive lock for stationary observation; it does not enable
motion. See [IR calibration evidence](docs/IR_MEASURED_CALIBRATION.md).
Mqtt requires
your ignored `config/wifi_secrets.h`; the build tests used a placeholder only in
an isolated snapshot. Never flash that placeholder MQTT build as a real setup.

For a shell with a space-free `PICO_SDK_PATH`, equivalent commands include:

```sh
make -C build_make -j8
make -C build_make NET=lwip MQTT=1 -j8
make -C build_make TEST=1 -j8
make -C build_make SMP=0 -j8
make -C build_make CONSOLE=uart WIFI=none -j8
```

All profiles keep the drive lock by default, including S2Bench. The servo can
center/scan. Only a deliberate rebuild with `BOARD_MOTORS_DISABLED=0` in the
local configuration permits movement after the remaining arming/calibration
checks. S2Bench automatically moves wheels when that lock is removed; support
the robot before manually flashing it. UART
uses the ultrasonic pins, disables S5 and cannot arm the integrated controller.

Copy `app_program/robot_config.local.example.h` to ignored
`app_program/robot_config.local.h` and enter measured values. Read the hardware
acceptance procedure before enabling IR, tilt/odometry or bypass. ADC pad setup
follows BOARD_IR_MODE=2, never takes GP29, and requires calibrated endpoints.

## Tests and dashboard

```sh
make -C testing/subsystem3 test analyze check-headers arm-check verify-failure
make -C app_program/subsystem4/tests/host test sanitize
make -C testing/integration test sanitize
```

Run Buddy 2's `app_program/subsystem2/tests/host/run.ps1` with a native Windows
GCC (for example MSYS2 UCRT64). It checks eight movement configurations with
the lock explicitly removed in simulation, plus two default-lock configurations
that verify rejected movement, zero drive PWM, no PWM mux/enabling, and working
encoders. Checks include renewal, expiry, STOP saturation and controller ownership.

The read-only dashboard subscribes only to `csc02/telemetry`. Install its listed
dependencies in your preferred environment, then run:

```sh
python app_program/external_systems/Dashboard/dashboard.py --broker YOUR_BROKER
```

It binds only to 127.0.0.1:8001, validates the binary schema and marks stale and
unavailable readings. `decoder.py HEX` works offline without MQTT dependencies.
`tools/robot_console.py --port COMx --log PATH` records USB output read-only.
Its explicit --arm mode renews the short lease and sends STOP on exit. No
connected-hardware action was performed during this code-only stage.

Motion output is still open-loop PWM. Encoder-based distance/relative-turn
feedback and terrain processing are implemented, but PID speed regulation,
absolute compass heading and audio playback are not implemented. PID gains and
motor response need physical measurements; do not describe this as a validated
closed-loop speed controller or a completed hardware acceptance run.

All changes remain on the named local branch. No push, PR, remote merge or
automatic deployment is part of this workflow.
