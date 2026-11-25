# INF2004 CSC02 robot firmware

This project is firmware for a Raspberry Pi Pico W line-following robot on
micro T-Kernel 3.0, running on both RP2040 cores. It reads Code 39 barcodes,
bypasses obstacles with a servo-mounted ultrasonic ranger, and detects humps
with a GY-511 accelerometer and magnetometer. It sends binary telemetry and
robot events through an MQTT broker to a web dashboard, which can reset,
start and stop a mission and remap barcode letters.

- [TEAM.md](TEAM.md): owners, pins, peripherals, task priorities, interfaces
  IF-01 to IF-22 and the kernel object budget.
- [testing/](testing/README.md): every test case, result and saved log.
- [Videos of tests and demos](testing/README.md#videos-of-tests-and-demos): unlisted YouTube videos of hardware tests and demos.
- [Calibration](calibration/README.md): current settings and measurement records.
- [Wire format](docs/DATA_REPRESENTATION.md): EDR16 telemetry and robot events.
- Subsystem guides: [S1 telemetry](app_program/subsystem1/README.md),
  [S2 motion](app_program/subsystem2/README.md),
  [S3 line and barcodes](app_program/subsystem3/README.md),
  [S4 IMU and terrain](app_program/subsystem4/README.md),
  [S5 ranger and servo](app_program/subsystem5/README.md).

## 1. Set up the build tools

Install the Pico C SDK, the Arm GNU Toolchain (`arm-none-eabi-gcc`), GNU Make
and a host C++ compiler for `elf2uf2`. The SDK needs its `lib/tinyusb`,
`lib/cyw43-driver` and `lib/lwip` submodules for the MQTT mission build.
The SDK and compiler are separate installations. Every build of this robot
needs `PICO_SDK_PATH`, including `CONSOLE=uart WIFI=none`, except the S2 bench
build `S2_ONLY=1 CONSOLE=uart WIFI=none`: S4 compiles the SDK's I2C driver (`app_program/subsystem4/sdk.mk`; see TEAM.md section 5).
The example below uses SDK 2.3.0; the CI firmware job fetches SDK 2.2.0.

On Windows, the build helper supports WSL Ubuntu or native MSYS2. A native
build needs MSYS2's `bash` and `make` in `C:\msys64\usr\bin` (pass `-MsysBin`
if yours is elsewhere) and a host `c++` for `elf2uf2`; the helper puts
`C:\msys64\ucrt64\bin` on `PATH` for that compiler.

Before building on your own machine, copy `config/wifi_secrets.example.h` to
`config/wifi_secrets.h`. Set `WIFI_SSID`, `WIFI_PASSWORD`, `MQTT_BROKER_IP`,
`MQTT_BROKER_PORT` and a suitable `MQTT_CLIENT_ID`. The broker address must be
an IPv4 address reachable from the robot. Optional `MQTT_USERNAME` and
`MQTT_PASSWORD` enable broker login. The secrets file is git-ignored; never
commit it or put credentials in test logs.

## 2. Build, flash and read the console

Run from the repository root. This native Windows example uses the `Mqtt`
profile; change both installation paths to match your computer:

```powershell
.\tools\build_windows.ps1 -Backend Native `
  -ArmToolchainBin "$env:USERPROFILE\.pico-sdk\toolchain\15_2_Rel1\bin" `
  -PicoSdkPath "$env:USERPROFILE\.pico-sdk\sdk\2.3.0" -Profile Mqtt
```

If PowerShell refuses to run scripts, start the same command with `powershell -ExecutionPolicy Bypass -File
.\tools\build_windows.ps1` followed by the same arguments.

For WSL, install `make`, `g++` and the Arm compiler inside Ubuntu, for
example `sudo apt install -y make g++ gcc-arm-none-eabi`. The helper runs a
non-interactive `bash` that does not read `~/.bashrc`, so the compiler must be
on Ubuntu's default `PATH`; the helper sets `PICO_SDK_PATH` itself. Keep the
SDK on a Windows drive and pass its absolute Windows path:

```powershell
.\tools\build_windows.ps1 -Backend WSL -Distribution Ubuntu `
  -PicoSdkPath C:\pico\pico-sdk -Profile Mqtt
```

The helper runs `make -C build_make -j8 NET=lwip MQTT=1` and does not flash.
You can also run that command in a shell with `PICO_SDK_PATH` set and the
compiler on `PATH`. The mission image is
`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc.uf2`. The firmware turns
Wi-Fi power save off.

1. Hold **BOOTSEL** while plugging the Pico W into USB, then release it.
2. Copy the `.uf2` onto the **RPI-RP2** drive. The Pico reboots.
3. Open its USB serial port at **115200 baud**. On Windows, select the new
   COM port in PuTTY or a serial monitor. On Linux, for example, use
   `screen /dev/ttyACM0 115200`.

The mission firmware boots `DISARMED`. With the robot ready for motion, send
`r` then `g` to reset and start a mission. These USB keys send command frames:

| Key | Command |
|---|---|
| `s` | STOP |
| `a` | ARM at 30% demand with a 500 ms lease; renew to keep running |
| `r` | RESET while stopped |
| `g` | Autonomous START at 30% demand |

After a STOP ends an `a` run, ARM is refused until RESET, or until no ARM
has arrived for 1 s. The dashboard's START uses 20% demand. After the robot
connects to the broker, and after a dashboard STOP, it refuses dashboard ARM
and START until it has passed a RESET to the controller's command queue. Demand is a speed
request in PID builds; it is not the measured wheel speed or a fixed PWM duty
cycle.

## 3. Build options and crash reports

Plain `make -C build_make -j8` uses `SMP=1 CONSOLE=usb_cdc WIFI=cyw43`
with `NET=none MQTT=0 TEST=0 CRASH_REPORT=0` and encoders enabled. Give options
on every build; the profile stamp rebuilds objects when these options change.

| Helper profile | Make options added | Image in `build_make/` |
|---|---|---|
| `Default` | None | `mtk3pico_smp1_usb_cdc_wifi_enc.uf2` |
| `Mqtt` | `NET=lwip MQTT=1` (the mission build) | `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc.uf2` |
| `PassiveTest` | `TEST=1` (no MQTT) | `mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2` |
| `SingleCore` | `SMP=0` | `mtk3pico_smp0_usb_cdc_wifi_enc.uf2` |
| `Uart` | `CONSOLE=uart WIFI=none` (no S5, see below) | `mtk3pico_smp1_uart_enc.uf2` |
| `S4Bench` | `S4_ONLY=1` | `mtk3pico_smp1_usb_cdc_wifi_enc_s4.uf2` |
| none | `TEST=1 NET=lwip MQTT=1` | `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2` |
| none | `NET=lwip MQTT=1 CRASH_REPORT=1` | `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_crash_enc.uf2` |
| none | `S2_ONLY=1` | `mtk3pico_smp1_usb_cdc_wifi_s2_enc.uf2` |

S2 also has `S2_ENCODER_ENABLED` (default 1) and `S2_COMMAND_TIMEOUT_MS`
(default 1000); see the [S2 guide](app_program/subsystem2/README.md). The S5
diagnostic builds are `TEST=1` with a setting in `robot_config.local.h`; see
the [S5 guide](app_program/subsystem5/README.md). The template's other board
makefiles (`build_make/iote_*.mk`, `nucleo_h723.mk`) are not used or tested
for this robot.

`S2_ONLY=1` and `S4_ONLY=1` are bench builds that start only that
subsystem: no vehicle controller, no other subsystem and no watchdog.
`TEST=1` adds passive on-target checks: the integrated build runs
`subsystem1_test()` to `subsystem5_test()`, `S2_ONLY=1` runs `subsystem2_test()` and
`S4_ONLY=1` runs `subsystem4_test()`. They read status and round-trip codecs; they
command no motor, servo or sonar and are not hardware tests of the robot's motion. See [testing/](testing/README.md) for the host
suites and hardware test procedure.

For a crash report, build with:

```sh
make -C build_make -j8 NET=lwip MQTT=1 CRASH_REPORT=1
```

UART0 uses **GP0 TX / GP1 RX at 115200 baud**. Disconnect the sonar hardware
from those pins, connect a USB-serial adapter's RX to GP0 and GND to GND, and
read that port. This build copies every console line to UART0 and sends it
at once, so the last messages and the kernel's crash report still appear after a crash, when
the USB console has gone quiet. The image is
`build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_crash_enc.uf2`.
S5 is not built in this configuration, so the integrated robot cannot
drive: an ARM or START stops with fault 1 (sonar) within 1 s. `CONSOLE=uart` also reserves those
pins. See [TEAM.md](TEAM.md) before changing wiring.

`app_program/robot_config.h` holds the robot's mission defaults.
`app_program/robot_ir_calibration.h` holds the ADC endpoints. Optional
per-checkout overrides use the git-ignored `app_program/robot_config.local.h`;
see `app_program/robot_config.local.example.h`. For bench work,
`#define BOARD_MOTORS_DISABLED 1` there keeps the motors off and rejects
ARM and START.

The default line follower pivots toward a straddle sensor on black and drives
forward when both are white or both are black. After 600 mm with neither
sensor on black, it starts a bounded spot search (`BOARD_LINE_LOST_MM`).
Edge tracing is a build option (`BOARD_LINE_MASK_PIVOT 0`,
`BOARD_LINE_EDGE_TRACE 1`). The default barcode actions are A U-turn, F Left
and Z Right; the dashboard can remap them. See the calibration guide before changing these settings.

In the mission build, the runtime watchdog monitors the controller, motor, network, telemetry and
S4 sensor loops. After a watchdog or recorded crash reboot, the console's
`[RESET]` line reports the cause. [TEAM.md](TEAM.md) describes startup, timing,
watchdog limits and the repeated-crash boot hold.

## 4. Run the dashboard

Start an MQTT broker separately. Use its reachable address below and the
same broker in the robot's secrets file:

```powershell
python -m pip install -r app_program/external_systems/Dashboard/requirements.txt
python app_program/external_systems/Dashboard/dashboard.py --broker YOUR_BROKER --mqtt-port 1883
```

Open <http://127.0.0.1:8001>. Use **Reset robot**, then **Start test**.
**STOP** ends motion. The receiver subscribes to telemetry, robot events and
command replies. See the [dashboard guide](app_program/external_systems/Dashboard/README.md)
for credentials, display behaviour and barcode mappings.

## 5. Run the host tests

Run from the repository root with Python 3, native GCC and GNU Make on
`PATH` (use `python3` where `python` is not found; on Windows with MSYS2,
GNU Make may be called `mingw32-make`). The S3 `arm-check` target also needs
the Arm compiler; the S4 suite runs `python3`; browser display checks need
Node.js. GCC analysis targets use `-fanalyzer`.

```sh
python -B testing/data_representation/run.py
python -B app_program/subsystem2/tests/host/run.py
make -C testing/subsystem3 test analyze check-headers arm-check verify-failure
make -C app_program/subsystem4/tests/host analyze sizes
python -B testing/integration/run.py
python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py"
```

The first command already runs the S2 host suite and
the dashboard Python tests; the second and last commands run them on their own.
`make -C testing/integration test` is the Make entry point for integration
checks. [testing/README.md](testing/README.md) explains dependencies, skipped
checks and how to save results. Test records and logs are tracked. Host tests
check software with simulated hardware; physical results belong in the
subsystem and system test records.

`make -C app_program/subsystem4/tests/host sanitize` and
`make -C testing/integration sanitize` add AddressSanitizer and
UndefinedBehaviorSanitizer. They need a compiler and C library that support
them, such as GCC or Clang on Linux; MinGW GCC on Windows does not.

### Continuous integration

Three GitHub Actions workflows in `.github/workflows/` run on Ubuntu 24.04 for
pushes to `main` and for pull requests (the data representation workflow on
every push):

| Workflow | Runs |
|---|---|
| `Buddy 3 CI` (`buddy3-ci.yml`) | the S3 `test`, `analyze`, `check-headers` and `verify-failure` targets, then `arm-check`, a default firmware build and a mission (`NET=lwip MQTT=1`) firmware build with SDK 2.2.0 |
| `Buddy 4 integration` (`buddy4-integration.yml`) | `make -C app_program/subsystem4/tests/host analyze sizes`, `make -C testing/integration test` and both `sanitize` targets |
| `Data representation contract` (`data-representation.yml`) | `python -B testing/data_representation/run.py` with GCC and with Clang |

The workflows use Linux GCC and glibc; a host test that passes with MinGW on
Windows can still fail there, and the other way round, so check the workflow
result as well as a local run. The CI does not build the `TEST=1` image
and runs nothing on the robot.

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
