# Testing

All test records for the robot live here. A test counts as done only when
its result and saved evidence are recorded. `TEST_LOG.md` files and the
contents of `logs/` are tracked in git.

```text
testing/
├── subsystem1/ … subsystem5/  TEST_LOG.md and logs/ (S3 and S5 also hold host test sources)
├── integration/              shared interface tests, host sources and logs
├── system/                   whole-robot tests and logs
└── data_representation/      EDR16 host checks and TEST_LOG.md
```

See [TEAM.md](../TEAM.md) for subsystem owners and interfaces IF-01 to IF-22.
The records are [S1](subsystem1/TEST_LOG.md), [S2](subsystem2/TEST_LOG.md),
[S3](subsystem3/TEST_LOG.md), [S4](subsystem4/TEST_LOG.md),
[S5](subsystem5/TEST_LOG.md), [integration](integration/TEST_LOG.md),
[system](system/TEST_LOG.md) and [data representation](data_representation/TEST_LOG.md).

## Three levels of test

| Level | ID format | What it checks | Written by | Record |
|---|---|---|---|---|
| Unit | `S<n>-U<nn>` | One subsystem | Subsystem owner | `subsystemN/TEST_LOG.md` |
| Integration | `INT-<nn>` | An interface in `TEAM.md` | Owners on both sides | `integration/TEST_LOG.md` |
| System | `SYS-<nn>` | The complete robot against FR1-FR13 and NFR1-NFR8 | Whole team | `system/TEST_LOG.md` |

From now on, keep IDs stable and never reuse them. The IDs were renumbered
on 2026-11-21 and, in every log except S2, again on 2026-11-25; some old
numbers now name different tests. Each log's **ID changes on 2026-11-21**
and **ID changes on 2026-11-25** tables, applied in date order, map older
console output and results;
keep the saved output unchanged. S3 also uses `S3-B` (build) and
`S3-CI` (failure propagation) IDs; older S3 rows also use `S3-I` (integration
probe) IDs.

Use **auto (host)** for PC tests, **auto** for on-target `TEST_CHECK` checks,
**manual** for observations and measurements on the robot, and
**manual (video)** when a filmed run is the evidence, as in the core system
tests. Name the exact command or build and give a measurable expected result.

## How to run and record a test

1. **Define it first.** Add a case to the correct `TEST_LOG.md`: ID, purpose,
   requirement or interface, method, steps and expected result.
2. **Choose the test path.** Use the host entry points below for software
   checks. The integrated `TEST=1` image calls only `subsystem3_test()`
   (`app_program/subsystem3/subsystem3_test.c`), a passive `S3-U01`
   normalization check. It does not test all five subsystems at boot. To add
   an on-target check, put a `TEST_CHECK` (see `app_program/test_report.h`) in
   a test function and call it from the `#if APP_TEST` block in
   `app_program/app_main.c`. That file is shared code, so agree the change
   with the team. Keep the check passive and small: it runs in the initial
   task, which has a 4 KB stack in a `TEST=1` image. Write manual steps for
   measurements such as stopping distance, wheel direction, scan accuracy and
   hump height.
3. **Run the selected command or build and flash the required image.** For
   the integrated passive check, with the SDK and Arm compiler set up:

   ```sh
   make -C build_make TEST=1 NET=lwip MQTT=1 -j8
   ```

   This produces `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`.
   It needs the builder's git-ignored `config/wifi_secrets.h`, copied from
   `config/wifi_secrets.example.h`. Without Wi-Fi secrets,
   `.\tools\build_windows.ps1 -Backend Native -ArmToolchainBin <Arm bin folder> -PicoSdkPath <SDK path> -Profile PassiveTest`
   builds the same check without MQTT, as
   `build_make/mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2`. You can switch
   between images freely: the makefile discards objects built for another
   profile, and each image has its own file name. Hold BOOTSEL while plugging
   in USB and copy the image to `RPI-RP2`. The robot boots `DISARMED`; the
   passive check sends no motion command. For manual mission tests, use the
   mission image (`NET=lwip MQTT=1`, or `-Profile Mqtt`) and the steps in the
   case. See the [root build guide](../README.md).
4. **Save the output and measurements.** Save the output to a new file in that
   area's `logs/` folder, named `YYYY-MM-DD_<what>.txt` with the run date and a
   short description. For shared runs, save one log in `integration/logs/` or
   `system/logs/` and link it from each result that uses it.

   The USB console is 115200 baud. In PuTTY, choose connection type **Serial**
   and the Pico's COM port; find it in Device Manager under **Ports (COM & LPT)**.
   Enable **Session → Logging → All session output** before running the test.
   For a read-only Python capture, install pyserial once with
   `python -m pip install -r tools/robot_console_requirements.txt`, then run
   from the repository root:

   ```sh
   python tools/robot_console.py --port COM5 --seconds 60 --log testing/subsystem3/logs/YYYY-MM-DD_passive.txt
   ```

   Change the port and filename. The tool defaults to 15 s and accepts up to
   120 s. It timestamps read blocks, which may split console lines. Leave
   out `--arm` and `--reset` for passive capture; `--arm` drives the robot.
   It does not reconnect automatically after a reset. Reopen the port as soon
   as it returns: the check runs right after start-up and does not wait for
   the console. For an on-target check, capture the `[TEST]` start marker,
   `S3-U01` result, summary and end marker. Missing output is not a PASS; if
   the start marker is missing, reset the Pico and capture again. On macOS or
   Linux, `cat /dev/cu.usbmodem1101 | tee <log>` (macOS) or
   `cat /dev/ttyACM0 | tee <log>` (Linux) also works; press Ctrl-C when done.

   Save manual measurements, photos and scope captures with the run's log.
   Keep image files under a few MB. Share videos as unlisted YouTube links
   as described below. For host tests, run the command in Git Bash or MSYS2
   and copy its output to a new log, for example:

   ```bash
   python -B testing/integration/run.py 2>&1 | tee testing/integration/logs/YYYY-MM-DD_host.txt
   echo "exit code: ${PIPESTATUS[0]}"
   ```

   `${PIPESTATUS[0]}` is the runner's exit code (`$?` would be `tee`'s).
   Check it as well as the output. Record skipped checks and missing
   dependencies; a SKIP or compile-only run is not an executed test PASS.
5. **Record the result.** Append the actual date, test ID, tested commit
   (`git rev-parse --short HEAD`), tester, result and evidence path to the
   *Results* table. Include build options, local overrides and relevant
   measurements in *Notes*. Commit the result and evidence together.

**Result values**

- **PASS**: every step was run and every expected result was seen (in the log, the console or the video).
- **FAIL**: the test ran and at least one expected result was not met.
- **INCOMPLETE**: the test ran, but not every step was done, or the evidence does not show every expected result (a partial run).
- **BLOCKED**: the test could not be run (a tool or the hardware was missing); used in some October records.

On 2026-11-25 every test whose latest result was INCOMPLETE was removed (each affected log lists the removed IDs under **ID changes on 2026-11-25**; S2 had none), the tests filmed for the video set were narrowed to what their video shows, and the IDs were renumbered so each series runs with no gaps.

ID history note (audit, 2026-11-25): in git, both renumberings arrived in one commit, `5f93cb6`, which also deleted result rows, including two FAIL rows (S1-U11 and SYS-14 in the old numbering). Those rows are restored word for word in a *Restored result rows* table under *Results* in each affected log. The S1-S5, integration and system logs now end with *Audit notes*: an *ID history (git record)* table mapping the IDs at `d18dd38` (`ed197b3` for the system log) to today's IDs, and notes on unclear commit cells and missing evidence files.

## Host tests

Run from the repository root with Python 3 and native GCC on `PATH`.
Make targets also need GNU Make and a compatible shell; use MSYS2 or WSL on
Windows. Use `python3` where `python` is not found. The S3 `arm-check` target
needs `arm-none-eabi-gcc`. The `analyze` targets use GCC's `-fanalyzer`. These checks do not flash the robot.

| Entry point | Coverage |
|---|---|
| `python -B testing/data_representation/run.py` | Three stages: Python EDR16/S5/dashboard tests; C codec and S1 queue tests; S2's nine host profiles |
| `.\check_data.cmd --no-pause` (Windows) or `sh check_data.sh` | Root launchers for the same EDR16 runner; write `edr-check-report.json` (git-ignored) in the repository root |
| `python -B app_program/subsystem2/tests/host/run.py` | Motor commands, encoders, STOP, command expiry, PID and failure paths in nine simulated profiles |
| `make -C testing/subsystem3 test analyze check-headers arm-check verify-failure` | IR/barcode algorithms and adapters, ADC, kernel headers, Arm compilation and failure propagation |
| `make -C app_program/subsystem4/tests/host analyze sizes` (needs `python3` on `PATH`) | Sensor drivers, compact records, terrain/result and odometry checks; GCC analysis and host sizes |
| `python -B testing/integration/run.py` (any OS, Windows included) or `make -C testing/integration test` (needs `python3` on `PATH`) | Vehicle policy and controller, avoidance, barcode retries, hump estimate, line tracing, watchdog policy, S5 sonar and servo drivers, queues, MQTT and telemetry-to-dashboard routing |
| `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py"` | Dashboard decoding, commands, robot events, widths and browser field routing |

The three Python runners accept `--compiler PATH` for native GCC or Clang.
The S5 Python test invokes the compiler named by `CC`. Browser field routing
needs Node.js on `PATH`. The dashboard's C event-packing test needs `gcc`;
its width-producer test is skipped unless `testing/integration/.build/width_avoidance`
(`.exe` on Windows) exists, built by either integration entry point. Check
skips before calling a run complete.

The `app_program/robot_config.h` defaults are the robot's mission profile.
Host builds define `ROBOT_CONFIG_NO_LOCAL`, so a git-ignored
`robot_config.local.h` never changes a host test. A test that needs another
scenario passes explicit `-D` flags (for example `-DBOARD_IR_MODE=2` or
`-DBOARD_MOTORS_DISABLED=1`).

`verify-failure` deliberately fails a check and verifies that the failure is
reported. The wrapper succeeding is the expected result. S4 and integration
also have `sanitize` targets for ASan/UBSan; some Windows toolchains lack
those runtimes. The workflows in `.github/workflows/` define the S3, S4,
integration and EDR16 CI checks. A workflow definition is not a saved run result.

The records contain results from specific commits and historical formats.
Use the result row and linked log;
do not treat an old PASS as evidence for a later build.

## Videos of tests and demos

Record 15–60 s showing the test ID, hardware and result. Upload as an
**unlisted YouTube** video, list it in [videos/README.md](../videos/README.md),
and add the link to the test result's *Log / evidence* column. Videos are
shared only through those links; do not commit video files.

## Rules

- Never edit or delete an old result. Record a failure, fix the code, rerun
  and append a new result. Mark retired cases without reusing their IDs.
- Commit the code before testing and always record that commit. Record local
  build overrides too, so the run can be repeated.
- A FAIL needs a note explaining what failed and, later, the fixing commit.
- Keep results and saved logs in git. Check evidence for credentials before
  adding it; never commit `config/wifi_secrets.h`.
- Follow the case's hardware setup. Support the robot with its wheels clear
  for stationary motor checks; driving and course tests need a clear course.
- Measure physical accuracy, STOP timing, task timing and memory use on the
  final firmware where the case requires it. Host checks do not replace them.
- Before merging into `main`, rerun the affected subsystem's unit tests.
  Before a demo or hand-in, run every test and record the full set.
