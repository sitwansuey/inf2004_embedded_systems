# Repository-wide EDR16

All binary telemetry uses the shared [16-bit EDR contract](docs/DATA_REPRESENTATION.md):
4-bit reading ID plus 12-bit data, exactly two little-endian bytes per record.
Buddy 2's adapters, Buddy 1's MQTT collector and the desktop decoder use this
contract. The old variable-length EDR envelope has been replaced. Local control
snapshots keep their full-width counters/timestamps. Run `check_data.cmd` or
`check_data.sh` with a native compiler; see the test log for actual verification.

# micro T-Kernel 3.0 project for Raspberry Pi Pico W

A starting point for your project: micro T-Kernel 3.0 running on **both cores**
of the Pico W's RP2040, with a USB serial console and the WiFi driver built in.

For this branch's motor/dual-encoder firmware, see
[Subsystem 2](app_program/subsystem2/README.md). Build it from `build_make/` with
`make S2_ONLY=1 TEST=1 S2_ENCODER_ENABLED=1 -j8` after completing the setup below.
Raise the wheels before flashing the test image. The full application still
contains sample interfaces that must be reconciled with the team's controller.

Before implementing or integrating a buddy's binary-data interface, run the
hardware-free [shared data contract check](testing/data_representation/README.md):

**Windows:** double-click `check_data.cmd` in the repository root. The window
stays open so you can read PASS/FAIL. **Linux/WSL/macOS:** run:

```sh
sh check_data.sh
```

All three stages must finish with `EDR CHECK PASS`. This needs Python and native
GCC/Clang, and checks the shared byte contract plus Buddy 2's adapters/watchdog.
Both launchers save `edr-check-report.json` in the repository root.

You need a **Raspberry Pi Pico W**, a USB cable, and a computer running
macOS, Linux, or Windows with **WSL (Ubuntu)**. On Windows, run every
command below inside the WSL Ubuntu terminal.

---

## 1. Set up the Pico C SDK and the compiler

You need two separate things:

| What | Why | Where it comes from |
|---|---|---|
| **Pico C SDK** | USB and WiFi driver source code | `git clone` (step 1.2) |
| **Arm GNU Toolchain** (`arm-none-eabi-gcc`) | the compiler | a separate download (step 1.3) |

> ⚠️ **The SDK does NOT include the compiler.** You must install the compiler
> on its own and put its `bin` folder on your `PATH` (step 1.4).

**1.1 Install the basic tools: git, make and a host C++ compiler**

```sh
# macOS
xcode-select --install

# Linux / WSL (Ubuntu)
sudo apt update && sudo apt install -y git make g++
```

**1.2 Download the Pico C SDK** (any folder works; this guide uses `~/pico`)

```sh
mkdir -p ~/pico && cd ~/pico
git clone --branch 2.2.0 https://github.com/raspberrypi/pico-sdk.git
cd pico-sdk
git submodule update --init lib/tinyusb lib/cyw43-driver
```

The `submodule` line is required: it downloads the USB and WiFi drivers.
(SDK 2.3.0 has also been tested and works.)

**1.3 Install the Arm GNU Toolchain (the compiler)**

- **macOS / Windows (WSL) / Linux:** download from
  <https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads>.
  Pick the **AArch32 bare-metal target (arm-none-eabi)** package for your
  computer. For WSL, choose the **Linux x86_64** `.tar.xz`.
  Extract it, e.g. to `~/tools/arm-gnu-toolchain-<version>/`.
- **Linux / WSL shortcut:** `sudo apt install -y gcc-arm-none-eabi`
  installs it straight onto your PATH. If you use this, skip the PATH line in 1.4.

Find the folder that contains `arm-none-eabi-gcc`. It is the toolchain's
**`bin`** folder, e.g. `~/tools/arm-gnu-toolchain-15.3.rel1/bin`.

**1.4 Tell your shell where the compiler and SDK are**

Add these two lines to the end of `~/.zshrc` (macOS) or `~/.bashrc` (Linux/WSL),
**changing both paths to match your computer**:

```sh
export PATH="$HOME/tools/arm-gnu-toolchain-15.3.rel1/bin:$PATH"   # folder that contains arm-none-eabi-gcc
export PICO_SDK_PATH="$HOME/pico/pico-sdk"                         # the SDK from step 1.2
```

Open a **new** terminal, then check both:

```sh
arm-none-eabi-gcc --version     # must print a version, not "command not found"
ls $PICO_SDK_PATH               # must list the SDK files
```

If `arm-none-eabi-gcc` is "not found", your `PATH` does not point to the
toolchain's `bin` folder. Fix that before going further.

---

## 2. Clone this repo and build the `.uf2` file

**2.1 Clone your project**

```sh
cd ~/pico
git clone <URL-of-this-repo> my-project
cd my-project
```

**2.2 Build**

```sh
cd build_make
make -j8
```

A successful build ends with a line like
`tools/elf2uf2 mtk3pico_smp1_usb_cdc_wifi.elf mtk3pico_smp1_usb_cdc_wifi.uf2`.
Your firmware is **`build_make/mtk3pico_smp1_usb_cdc_wifi.uf2`**.

If `make` stops with an error, it tells you which one is wrong:
`arm-none-eabi-gcc not found` means fix `PATH`, and
`PICO_SDK_PATH is not set` means fix `PICO_SDK_PATH` (step 1.4).

To build a single-core, UART-only or test version, see section 3.

**2.3 Flash it onto the Pico W**

1. Hold the **BOOTSEL** button and plug the Pico W into USB, then release.
   A drive called **RPI-RP2** appears.
2. Copy the `.uf2` file onto that drive. On WSL, run `explorer.exe .` in
   `build_make` to open the folder in Windows. The Pico reboots by itself.

**2.4 See the output**

Open the Pico's USB serial port at 115200 baud:

```sh
screen /dev/tty.usbmodem* 115200      # macOS   (quit: Ctrl-A then K)
screen /dev/ttyACM0 115200            # Linux
```

On Windows, use PuTTY or the VS Code *Serial Monitor* on the new COM port.
You should see the task reports. Connect an LED with a 330 Ω resistor from
**GP19** to GND to see subsystem 1's LED blink.

**2.5 Write your own code**

Your code lives in **`app_program/`**. Every `.c` file there, and in its
subfolders, is compiled automatically.

- **`app_main.c` → `usermain()` is where the program starts.** It first calls
  every subsystem's `subsystemN_init()` (create kernel objects), then every
  `subsystemN_start()` (start tasks). Read its comments first.
- **One folder per student:** `app_program/subsystem1/` … `subsystem5/`.
  Subsystems 1–4 hold a working example, one task each (LED, producer,
  consumer, monitor). Together they use the main micro T-Kernel APIs and
  pass data between subsystems. Subsystem 5 is an empty template.
  Replace the examples with your own subsystems.
- `subsystemN.h` is that subsystem's public interface: the object IDs and
  data formats the other subsystems may use.
- `app.h` holds the definitions shared by the whole team.
- **Drivers go in your subsystem folder too**, as a `drv_<name>.c` /
  `drv_<name>.h` pair, e.g. `subsystem2/drv_ultrasonic.c`. They are compiled
  automatically. Do not add drivers under `device/`: that folder holds the
  kernel's own serial, I2C and ADC drivers, and adding to it means editing
  kernel and makefile files shared by the whole team.

**Working as a team:** record owners, pins, priorities and the interfaces
between subsystems in **`TEAM.md`**, and record every test in **`testing/`**
(see `testing/README.md`). `make TEST=1` builds a test image that runs all
the subsystem tests at power-up. Short videos of tests and demos are shared
as unlisted YouTube links, listed in **`videos/`**.

Pins: use **GP0–GP22 and GP26–GP28**. GP0/GP1 are free unless your team
builds with `CRASH_REPORT=1` (section 3), which needs them. Never use
GP23/GP24/GP25/GP29: they are wired to the Pico W radio. The Pico W's
on-board LED is also on the radio, so use an external LED instead.

After editing, run `make -j8` again in `build_make` and copy the new `.uf2`.

---

## 3. Build options: `SMP`, `CONSOLE`, `WIFI`, `TEST`, `CRASH_REPORT`

`make` on its own uses the defaults. To build differently, put options
after `make`. They can be combined in any order:

```sh
make -j8                          # defaults: SMP=1 CONSOLE=usb_cdc WIFI=cyw43 TEST=0 CRASH_REPORT=0
make SMP=0 -j8                    # single-core build
make CONSOLE=uart WIFI=none -j8   # no USB console, radio off (no Pico SDK needed)
make TEST=1 -j8                   # firmware with the subsystem self-tests
make CRASH_REPORT=1 -j8           # also see crash reports, on GP0/GP1 (uses those pins)
```

To set several options at once, list them all, separated by spaces. For
example, to track down a crash in the test build while running on one core only:

```sh
make SMP=0 TEST=1 CRASH_REPORT=1 -j8
#  -> build_make/mtk3pico_smp0_usb_cdc_wifi_crash_test.uf2
```

Any option you leave out keeps its default (here `CONSOLE=usb_cdc` and
`WIFI=cyw43`). Options are not remembered: give them on every `make`, as
the next plain `make` goes back to the defaults.

| Option | Values (**default**) | What it does | Image name | C macro your code can test |
|---|---|---|---|---|
| `SMP` | `0`, **`1`** | `1` runs the kernel on both RP2040 cores; `0` uses core 0 only | `smp1` / `smp0` | `TK_SUPPORT_SMP` (1 or 0) |
| `CONSOLE` | **`usb_cdc`**, `uart` | where `tm_printf` output goes. `usb_cdc` = USB cable; `uart` = UART0 only (GP0 TX / GP1 RX, needs a USB-serial adapter) | `_usb_cdc` / `_uart` | `TM_CONSOLE_USB_CDC` |
| `WIFI` | **`cyw43`**, `none` | `cyw43` powers up the Pico W radio and its driver task; `none` leaves the radio off | `_wifi` / *(nothing)* | `TM_WIFI_CYW43` |
| `TEST` | **`0`**, `1` | `1` compiles every `subsystemN_test.c` and runs the tests at power-up | `_test` / *(nothing)* | `APP_TEST` (1 or 0) |
| `CRASH_REPORT` | **`0`**, `1` | `1` also copies the console to UART0 on **GP0 (TX) / GP1 (RX)**, 115200 baud, so you can see the last messages and the kernel's crash report (see below). **Uses GP0/GP1.** `CONSOLE=uart` always does this. | `_crash` / *(nothing)* | `TM_CONSOLE_UART` (1 or 0) |

**Why `CRASH_REPORT` exists.** USB output is not sent the moment you call
`tm_printf`: it waits in a buffer until the kernel's USB task sends it. If the
program crashes, no task runs again, so the last messages and the kernel's
crash report never reach your USB terminal. The terminal just goes quiet.
UART output *is* sent at once, even inside the crash handler. So when you
need to know why the Pico crashed or froze, build with `CRASH_REPORT=1`,
connect a USB-serial adapter (adapter RX → GP0, adapter GND → GND), and read
it at 115200 baud. Normal `tm_printf` output on USB works the same with
either setting. If a subsystem has claimed GP0/GP1 in `TEAM.md`, unplug that
hardware before using `CRASH_REPORT=1`.

The options are part of the image name, e.g.
`mtk3pico_smp1_usb_cdc_wifi_test.uf2`, so you can always tell which build
you are flashing. Switching options needs no `make clean`: the makefile
notices and recompiles everything.

Using an option in C code:

```c
#if TK_SUPPORT_SMP
    ctsk.tskatr |= TA_ASSPRC;   /* core affinity only exists on the dual-core build */
#endif

#if APP_TEST
    /* code that only exists in the test build */
#endif
```

### Where the options live in the makefile

All five are handled in **`build_make/pico_rp2040.mk`**. To find each piece, search the file for the
names in the left column (e.g. `grep -n "TEST" build_make/pico_rp2040.mk`).
Each option goes through the same steps:

| Step | What to look for | Example for `TEST` |
|---|---|---|
| 1. Default value | `NAME ?= value` near the top of the file | `TEST ?= 0` |
| 2. Allowed values | an `$(error Unknown ...)` check just below it | `$(error Unknown TEST '$(TEST)'; use 0 or 1)` |
| 3. Image-name suffix | `NAME_SUFFIX :=` and the `EXE_FILE :=` line | `TEST_SUFFIX := _test` |
| 4. C macro for the code | a `CFLAGS += -D...` line | `CFLAGS += -DAPP_TEST=$(TEST)` |
| 5. Rebuild on change | the `PROFILE_ID :=` line | `...-test$(TEST)` |
| 6. Extra source files (`CONSOLE`, `WIFI` only) | an `ifeq ($(CONSOLE),usb_cdc)` or `ifeq ($(WIFI),cyw43)` block that adds the USB/radio files and SDK include paths | — |

- **Changing a default:** edit the `?=` line from step 1, e.g.
  `CONSOLE ?= uart`. Every `make` without that option then uses the new value.
- **Adding your own option** (say `DEBUG=1` to turn on extra prints): copy
  how `TEST` is done. Add steps 1 to 5 next to the `TEST` lines, then
  use `#if APP_DEBUG` in your code. **Step 5 matters:** without it, switching
  the option can quietly link files that were compiled with the old setting.
- **Kernel settings** such as the maximum number of tasks, semaphores or
  mailboxes are not make options. They are in `config/config.h` (`CNF_MAX_...`).

The makefile also has `NET=lwip` and `MQTT=1` (TCP/IP and MQTT over WiFi).
They need the SDK's `lib/lwip` submodule and a `config/wifi_secrets.h`
(copy `config/wifi_secrets.example.h`). See the shared configuration guide below
and `app_program/subsystem1/README.md` for Buddy 1 integration.

## 4. Shared configuration and interfaces for all buddies

All five buddies compile into **one firmware image**, so they share the same
kernel and build configuration. There is no single file containing every
setting: `config/config.h` is the central RTOS configuration, while
`app_program/app.h` is the common application header. Keep subsystem-specific
settings in that subsystem's own header.

| File | Purpose | Who uses it? |
|---|---|---|
| `config/config.h` | Kernel limits, timer settings and system configuration | All subsystems indirectly through kernel headers |
| `config/config_func.h` | Enables kernel API functions | Whole firmware |
| `config/config_device.h` | Enables/configures built-in device drivers | Whole firmware; coordinate changes with peripheral owners |
| `config/config_tm.h` | Console and T-Monitor settings | Whole firmware |
| `app_program/app.h` | Common RTOS/board includes, task helper, stack size and subsystem lifecycle declarations | Every buddy's `subsystemN.h` includes it |
| `app_program/subsystemN/subsystemN.h` | That buddy's constants and public APIs | Its owner and other subsystems calling those APIs |
| `app_program/subsystem1/telemetry_topics.h` | Shared telemetry reading IDs, exact MQTT topics and message size limits | Buddy 1 and callers through `subsystem1.h` |
| `config/wifi_secrets.h` | Local Wi-Fi credentials, broker address/port, client ID, heartbeat topic and optional authentication | Network service and Buddy 1 startup diagnostics; ignored by Git |
| `config/wifi_secrets.example.h` | Versioned template for the local network settings | Whoever builds/configures the robot |
| `build_make/pico_rp2040.mk` | Shared build profiles: SMP, console, Wi-Fi, networking, MQTT and tests | Everyone building the firmware |
| `TEAM.md` | Agreed pin/peripheral ownership, priorities, interfaces and object budget | All buddies; this is documentation, not a compiled header |

### What each buddy includes

Inside a subsystem's public header:

```c
#include "../app.h"
```

Inside its implementation, include its own public header. When sending dashboard
telemetry, also include Buddy 1's public interface:

```c
#include "subsystem3.h"                  /* example: Buddy 3 */
#include "../subsystem1/subsystem1.h"

/* In task context, after subsystem initialization: */
ER er = s1_telemetry_integer(S1_LEFT_IR_IR_LEFT, 650);
if (er < E_OK) {
    /* Apply a bounded retry/drop policy; do not block navigation on telemetry. */
}
```

The value above is an example; send actual measurements in the real task.
All buddies use the same reading IDs from `telemetry_topics.h`; do not create
independent topic strings in each subsystem. Buddy 1 copies readings into its
16-message RTOS buffer. The existing Wi-Fi service task polls that buffer and
publishes MQTT. Only that service task may call lwIP/MQTT functions.

Control requests and safety events still go directly to the Vehicle Controller
through the team's agreed interfaces. The telemetry buffer is for external
reporting and does not replace the controller's command/safety inbox.

### Local settings versus shared settings

Keep your own sensor thresholds, sampling periods, calibration and private pins
in your subsystem header (or a private configuration header in that folder).
Record physical pin/peripheral ownership in `TEAM.md` before using it. Changes
to common headers, kernel limits, build defaults and cross-subsystem interfaces
must be coordinated with the team so everyone builds compatible code.

The current motor module includes `app_program/board_pins.h`, but that shared
header is **missing from this checkout**. Its host-test `board_pins.h` is only a
test fixture. The team needs to provide a hardware pin header consistent with
the agreed wiring before integrating the motor module; do not treat the test
fixture as a verified robot pin configuration.

For network builds, fill in your local `config/wifi_secrets.h`, then run from
`build_make` with the SDK/toolchain configured:

```sh
make -j8 NET=lwip MQTT=1
```

The Pico needs the broker computer's **LAN IPv4 address** in `MQTT_BROKER_IP`.
`localhost` would refer to the Pico itself. The desktop dashboard has separate
Python runtime settings: it defaults to `localhost:1883`, appropriate when the
broker is on that desktop. It does not automatically read the firmware's C
configuration headers.

```powershell
python -m pip install -r app_program/external_systems/Dashboard/requirements.txt
python app_program/external_systems/Dashboard/dashboard.py --broker localhost --mqtt-port 1883
```

Open http://127.0.0.1:8001. The dashboard subscribes to `Robopico/sensors/#` and
`Robopico/Actuators/#`. If it runs on a different computer or Raspberry Pi,
replace `localhost` with the broker's reachable address.

---

*micro T-Kernel 3.0 © Ken Sakamura / TRON Forum, distributed under the
T-License 2.2. This dual-core RP2040 port is derived from the official
μT-Kernel 3.0 BSP (`tron-forum/mtk3_bsp`, branch `pico_rp2040`).*
