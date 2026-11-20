# Testing

All test records for the robot live in this folder. A test only counts as
done when it is **written down here with its result and a saved log**.

```
testing/
├── README.md              this guide
├── subsystem1/            one folder per subsystem, kept by its owner
│   ├── TEST_LOG.md        test cases + results
│   └── logs/              saved console output, photos, scope captures
├── ...                    (subsystem3/ and subsystem5/ also hold host test sources)
├── subsystem5/
├── integration/           tests across two or more subsystems (shared), plus host tests
├── system/                tests of the whole robot against its requirements (shared)
├── data_representation/   EDR16 wire-format checks (host) and their TEST_LOG.md
└── evidence/              review evidence from October, kept as it is
```

| Folder | Owner | Covers |
|---|---|---|
| `subsystem1/` | Buddy 1 (2500238) | binary telemetry, command/telemetry buffers, dashboard, LED on GP19 |
| `subsystem2/` | Buddy 2 (2500613) | motor PWM/PID, encoders, motor watchdogs |
| `subsystem3/` | Buddy 3 (2500383) | IR line sensors, GP28 barcode sampler, navigation events |
| `subsystem4/` | Buddy 4 (2500243) | GY-511 IMU acquisition, terrain/hump algorithm |
| `subsystem5/` | Buddy 5 (2502324) | HC-SR04 sonar and servo, edge scan |
| `integration/`, `system/` | whole team | vehicle controller (`vehicle.c`), watchdog supervisor, whole-robot behaviour |

Some `TEST_LOG.md` files were restored from October and keep an older table
layout. Leave those tables as they are. Add new test cases and results to the
*Test cases* and *Results* tables in the same file.

## Three levels of test

| Level | ID format | Tests what | Written by | Where |
|---|---|---|---|---|
| Unit | `S1-U01`, `S2-U01`, … | one subsystem on its own | the subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-01`, … | an interface between subsystems (TEAM.md section 5) | the owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-01`, … | the finished robot against a requirement (FR1-FR13, NFR1-NFR8) | whole team | `system/TEST_LOG.md` |

Test IDs never change and are never reused, even if a test is dropped. Mark a
dropped test "retired - do not reuse" in its row. Do not delete the row. New
IDs continue the numbering. Older Buddy 3 rows also use `S3-M` (manual),
`S3-B` (build), `S3-I` (integration probe) and `S3-CI` (failure propagation)
IDs. Those IDs stay.

Values for the *Method* column:

- **auto**: a `TEST_CHECK` in an on-target test image. Say which image
  (integrated, S2_ONLY or S4_ONLY, see *Test images*).
- **auto (host)**: a host test on a PC. Give the exact command and the test or
  recipe name.
- **manual**: a person follows numbered steps and measures the result (LED,
  scope, tape measure, stopwatch, dashboard). Give a measurable expected result.

## Test images

The on-target tests are built only into a test image (`TEST=1`). A normal
`make` leaves them out. Build in `build_make` (the root README shows how to
set up the compiler and `PICO_SDK_PATH`):

| Image | Command | UF2 file | What runs |
|---|---|---|---|
| Integrated, passive | `make TEST=1 NET=lwip MQTT=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2` | The whole robot starts DISARMED. Then the S1, S3, S4 and S5 checks run in turn. No motor moves. |
| Motor bench | `make TEST=1 S2_ONLY=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_s2_enc_test.uf2` | Only S2 starts. Each motor move runs for 2 s at 30 %, then the command watchdog check runs. **Raise the wheels first.** |
| IMU bench | `make TEST=1 S4_ONLY=1 -j8` | `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2` | Only S4 starts. The S4 checks run with no other subsystem running. |

- The integrated image needs your local `config/wifi_secrets.h` (copy
  `config/wifi_secrets.example.h` and fill it in). Never commit it.
- Each image prints `[TEST] === test run start ===`, then one
  `[TEST] <ID> PASS|FAIL` line per check and a `SUMMARY` line per subsystem,
  then `[TEST] === test run end ===`.
- A check may print `SKIP` instead (for example `S4-U06` when the IMU is not
  found). A SKIP is not a PASS.
- Between its `[TEST]` lines, the S2 test also prints `[S2 TEST]` and
  `[S2 ENCODER]` progress lines.
- An `ERROR <what>: <code>` line means a kernel call failed. Note it in the
  result row.
- You can switch between images freely. The makefile throws away objects
  built for another image, and each image has its own file name.
- `tools/build_windows.ps1 -Profile S2Bench` builds the same motor bench image.

## How to run and record a test

1. **Define it first.** Add a row to the *Test cases* table in the right
   `TEST_LOG.md`: ID, what it checks, the requirement or interface, method,
   steps, and the expected result.
2. **Automate it if you can.** Add a `TEST_CHECK` with the same ID to
   `app_program/subsystemN/subsystemN_test.c` (see `app_program/test_report.h`).
   Checks in the integrated image must stay passive: they read state and
   wait, and never command a motor. A check that moves the robot belongs in
   the motor bench image. Keep test functions small: they run in the initial
   task, which has a 1 KB stack. A host test goes in that subsystem's host
   suite (see *Host tests*). A test that needs eyes or instruments (the LED
   blinks, a wheel turns, the robot stops before a box) is a *manual* test.
   Write its steps in the table instead.
3. **Build the test image and flash it.**
   ```sh
   cd build_make
   make TEST=1 NET=lwip MQTT=1 -j8   # integrated, passive
   make TEST=1 S2_ONLY=1 -j8         # motor bench, wheels raised
   make TEST=1 S4_ONLY=1 -j8         # IMU bench
   ```
   Hold BOOTSEL while you plug in the USB cable, then copy the `.uf2` file to
   the `RPI-RP2` drive. The Pico restarts. The tests start right after boot
   and take a few seconds.
4. **Save the console output** to `logs/`, named `YYYY-MM-DD_<what>.txt`.
   The USB console runs at 115200 8N1. On Windows, find the port in Device
   Manager under *Ports (COM & LPT)*. Run the commands below from the
   repository root.
   - **Windows, PuTTY:** in *Session*, choose connection type *Serial*, your
     COM port and speed 115200. In *Session → Logging*, choose *All session
     output* and set the log file to a new file in the repository's
     `testing\integration\logs\` folder, for example
     `2026-11-21_passive_test.txt`. Then open the session.
   - **Windows, Python:** install pyserial once with
     `pip install -r tools/robot_console_requirements.txt`, then run
     ```sh
     python tools/robot_console.py --port COM5 --seconds 60 --log testing/integration/logs/2026-11-21_passive_test.txt
     ```
     It only listens. `--seconds` is the recording time, at most 120
     (default 15). Never add `--arm` (it drives the robot) or `--reset` to a
     test run. It puts a time stamp before each block it reads, which can
     split a line. Use PuTTY when you need clean lines.
   - **macOS / Linux:**
     ```sh
     cat /dev/cu.usbmodem1101 | tee testing/integration/logs/2026-11-21_passive_test.txt   # macOS
     cat /dev/ttyACM0 | tee testing/integration/logs/2026-11-21_passive_test.txt           # Linux
     ```
     Press Ctrl-C when it is done.

   The USB port disappears when the Pico resets and comes back a moment
   later. Open the terminal as soon as the port is back, because output
   printed before that can be lost. If the log has no
   `[TEST] === test run start ===` line, reset the Pico and try again.

   The integrated image prints the S1, S3, S4 and S5 results in one log. Save
   it once in `integration/logs/` and link it from each subsystem's result
   row. For manual tests, save photos or scope screenshots in `logs/` the
   same way (keep each file under a few MB). Videos go in `videos/` instead
   (see below).
5. **Record the result.** Add a row to the *Results* table: date, test ID,
   the commit you tested (`git rev-parse --short HEAD`), who ran it,
   PASS/FAIL, and the log file. Say which image you used in *Notes*. Commit
   the log and the table together.

## Host tests

Host tests build the robot's C and Python code on a PC, with stand-ins for
the kernel and the hardware. They check logic, not hardware accuracy. None of
them flashes or starts a robot. Run them from the repository root in a shell
with GCC, GNU Make and Python on PATH (MSYS2 or Git Bash on Windows):

| Command | Covers |
|---|---|
| `python -B testing/data_representation/run.py` (or `check_data.cmd` / `check_data.sh`) | EDR16 C/Python codecs, the S1 telemetry queue, the S5 status codec, every Dashboard `test_*.py`, and the S2 host profiles |
| `make -C testing/subsystem3 test analyze check-headers arm-check verify-failure` | IR algorithms, adapter, ADC, ARM/header checks |
| `python -B app_program/subsystem2/tests/host/run.py` | B2 adapters and watchdog in nine host profiles (also stage 3 of the EDR check) |
| `make -C app_program/subsystem4/tests/host analyze sizes` | B4 driver, compact storage, checked encoder/terrain/result path, GCC analysis; `sanitize` adds ASan/UBSan |
| `make -C testing/integration test`, or `python -B testing/integration/run.py` | controller, avoidance, queue and telemetry/MQTT boundaries; `run.py` runs the same recipes with any native compiler (Windows included) |
| `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py"` | the dashboard unit tests on their own (decoder, commands, events, obstacle width) |

- `verify-failure` makes one check fail on purpose, to prove that a failure
  stops the run. Its wrapper passing is the expected result.
- The Python runners take `--compiler PATH` when your native GCC/Clang is
  not on PATH.
- Some native Windows GCC builds lack the ASan/UBSan runtimes. GitHub CI
  (`.github/workflows/`) runs the subsystem 3, subsystem 4, integration and
  EDR16 suites on every push and pull request, with ASan/UBSan for
  subsystem 4 and integration.
- Save the output like a console log:
  `<command> 2>&1 | tee testing/<folder>/logs/YYYY-MM-DD_<what>.txt`.
- `app_program/robot_config.h` defaults are the robot's mission profile. Host
  builds define `ROBOT_CONFIG_NO_LOCAL`, so a git-ignored
  `robot_config.local.h` never leaks into a test. A test that needs another
  scenario passes explicit `-D` flags (for example `-DBOARD_IR_MODE=2` or
  `-DBOARD_MOTORS_DISABLED=1`).

## Videos of tests and demos

A manual test (an LED, a motor, a sensor reacting) is best shown with a short
video. Record 15–60 s showing the test ID and the hardware, share it as an
**unlisted YouTube** link (the only way videos are shared), list it in
**`videos/README.md`**, and put the link in the *Log / evidence* column of
the test's result row. `videos/README.md` explains how.

## Rules

- **Never edit or delete an old result.** If a test fails, record the FAIL,
  fix the code, re-run, and add a new PASS row. The history is the evidence.
- **Always record the commit.** A result is meaningless without the exact
  code it was run on. Commit your work *before* testing.
- **A FAIL needs a note** saying what went wrong and, later, which commit fixed it.
- **Logs and results go into git**, in this folder, next to the test log
  that uses them.
- Never commit `config/wifi_secrets.h`. Check a log for passwords before you
  commit it.
- Motor tests run only in the motor bench image, with the wheels raised.
- Hardware numbers (RAM use, latency, STOP timing, calibration) come from the
  final firmware on the robot. A host test does not replace them.
- Before merging a change into `main`, re-run your subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
