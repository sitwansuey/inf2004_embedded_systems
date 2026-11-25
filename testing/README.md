# Testing

The robot's test cases and results are in `TEST_LOG.md` files. Saved
console output and other evidence are linked from the result rows.

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

## Test coverage

| Level | ID format | What it checks | Record |
|---|---|---|---|
| Unit | `S<n>-U<nn>` | One subsystem | `subsystemN/TEST_LOG.md` |
| Integration | `INT-<nn>` | An interface in `TEAM.md` | `integration/TEST_LOG.md` |
| System | `SYS-<nn>` | The complete robot against FR1-FR13 and NFR1-NFR8 | `system/TEST_LOG.md` |

Test IDs were renumbered on 2026-11-21. The 2026-11-25 renumbering affected
every log except S2. The two **ID changes** tables in each affected log map
older results and console output in date order. S3 also uses `S3-B` for
build checks and `S3-CI` for failure propagation. Earlier integration
probes use `S3-I`.

The result tables distinguish host automation, passive on-target
`TEST_CHECK` results, manual measurements and video evidence.

## On-target test builds

The integrated `TEST=1` image runs all five subsystem self-tests. It
checks published status, S3-U01 normalization and S1/S5 codec round trips.
`TEST=1 S2_ONLY=1` and `TEST=1 S4_ONLY=1` run just the corresponding
subsystem. The initial task has a 4 KB stack in these images. The robot
boots `DISARMED`, and these checks issue no motor, servo or sonar commands.

```sh
make -C build_make TEST=1 NET=lwip MQTT=1 -j8
```

This builds `build_make/mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`
using the local Wi-Fi configuration. The Windows `PassiveTest` profile
builds `build_make/mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2` without MQTT.
Manual mission tests use the MQTT mission image. The root README documents
the build profiles and flashing steps. [Build guide](../README.md#2-build-flash-and-read-the-console)

### Console capture

The USB serial console runs at 115200 baud. The passive image prints
`[TEST]` results at boot. PuTTY's serial logging and the repository's
Python tool both capture the output. The Python tool depends on `pyserial` from `tools/robot_console_requirements.txt`.

```sh
python tools/robot_console.py --port COM5 --seconds 60 --log testing/subsystem3/logs/YYYY-MM-DD_passive.txt
```

The example uses a read-only capture. The input flags `--arm` and
`--reset` send commands, and the port must be reopened following a reset.
A complete passive capture includes the start marker, individual test
results, summary and end marker. Physical cases have their own procedures
and evidence in the linked test logs.

**Result values**

- **PASS**: every step was run and every expected result was seen (in the log, the console or the video).
- **FAIL**: the test ran and at least one expected result was not met.
- **INCOMPLETE**: the test ran, but not every step was done, or the evidence does not show every expected result (a partial run).
- **BLOCKED**: the test could not be run (a tool or the hardware was missing); used in some October records.

On 2026-11-25 every test whose latest result was INCOMPLETE was removed (each affected log lists the removed IDs under **ID changes on 2026-11-25**; S2 had none), the tests filmed for the video set were narrowed to what their video shows, and the IDs were renumbered so each series runs with no gaps.

ID history note (audit, 2026-11-25): in git, both renumberings arrived in one commit, `5f93cb6`, which also deleted result rows, including two FAIL rows (S1-U11 and SYS-14 in the old numbering). Those rows are restored in a *Restored result rows* table under *Results* in each affected log. The S1-S5, integration and system logs now end with *Audit notes*: an *ID history (git record)* table mapping the IDs at `d18dd38` (`ed197b3` for the system log) to today's IDs, and notes on unclear commit cells and missing evidence files.

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
(`.exe` on Windows) exists, built by either integration entry point.

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

Each result applies to the tested revision and setup named in its row.
Machine-specific workspace paths in some archived build logs are redacted.
The compiler commands, options and recorded outcomes are retained.

### Verified host results

The local BARR-C working tree based on `fe14666` passed the following checks
on 2026-11-25 with GCC 16.2.0 under MSYS2.

| Suite | Result | Evidence |
|---|---|---|
| EDR16, Buddy 2 and dashboard | PASS across all three EDR16 stages, including nine motor profiles and 44 dashboard tests | [EDR16 host log](integration/logs/2026-11-25_edr_host_gcc.txt) |
| Buddy 3 | PASS for host tests, GCC analysis, kernel header checks and failure propagation | [Buddy 3 host log](subsystem3/logs/2026-11-25_host_gcc_retest.txt) |
| Buddy 4 sensors | PASS for host tests, GCC analysis and size checks | [Buddy 4 host log](subsystem4/logs/2026-11-25_host_gcc_retest.txt) |
| Integration | PASS after building 35 binaries, including all five vehicle controller profiles | [Integration host log](integration/logs/2026-11-25_host_gcc_retest.txt) |

### Completion of Pico W and sanitizer checks

The project member reported on 2026-11-25 that the checks open after the
GCC host run were completed and cleared.

| Check | Status |
|---|---|
| Normal Pico W firmware and `TEST=1` builds | Completed, reported by project member |
| Buddy 4 and integration sanitizer runs | Completed, reported by project member |
| S1-U21 to S1-U23, passive on-robot checks | Completed, reported by project member |
| S2-U29 to S2-U33, passive on-robot checks | Completed, reported by project member |
| S3-U66 and S3-U67, passive on-robot checks | Completed, reported by project member |
| S4-U51 to S4-U55, passive on-robot checks | Completed, reported by project member |
| S5-U16 to S5-U20, passive on-robot checks | Completed, reported by project member |

This status is based on the member's report. The tested revision, build
options and individual captures for these runs were not supplied. Earlier
results remain in the dated test logs.

## Videos of tests and demos

### Video index

| Date | Subsystem | Test ID / demo | Link | Notes |
|---|---|---|---|---|
| 2026-11-11 | System | SYS-07 | <https://youtu.be/6-5GLf1cn3A> | Barcode reading and turns |
| 2026-11-20 | System | SYS-03 | <https://youtu.be/MKfX71MbV-Y> | Line tracing |
| 2026-11-20 | System | SYS-04 | <https://www.youtube.com/watch?v=SqGCaNN2gXc> | Obstacle stop and bypass |
| 2026-11-20 | System | SYS-06 | <https://youtu.be/s9e-ajBT7jo> | Turnover fault |
| 2026-11-24 | S1 | S1-U07 | <https://youtu.be/pBzQhiXhdmc> | MQTT broker restart |
| 2026-11-24 | System | SYS-08 | <https://youtu.be/ow1gfLBMz7s> | Network drop and recovery |
| 2026-11-25 | S4 | S4-U36 | <https://youtu.be/rOlxtJtGZTo> | IMU pitch and roll updates on the dashboard |
| 2026-11-25 | System | SYS-05 | <https://youtu.be/qSg2gh02zTQ> | Hump detection and height |
