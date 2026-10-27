# Buddy 4: GY-511 in the team micro T-Kernel architecture

This replaces subsystem 4's example monitor with the sensor-acquisition part
of Buddy 4. It adapts our earlier starter's tested LSM303DLHC driver to the
uploaded team's **existing shared startup, two-core scheduler and folder layout**.

This milestone supplies raw/scaled accelerometer and magnetometer data and a
snapshot interface. The chip has **no gyroscope**. Mounting calibration,
pitch/roll/heading, estimated turn rate, impact/unsafe-tilt events, hump height
and highest-valid-height tracking still need implementation and hardware data.
No fusion or height accuracy is claimed by this raw-data milestone.

## Build your part first

Use the compiler/SDK setup in the root README. Development checks use **Pico
SDK 2.2.0** and Arm GNU GCC 13.2.1. In a WSL/Linux/macOS terminal:

```sh
cd build_make
make -j4 S4_ONLY=1 TEST=1 WIFI=none
```

`PICO_SDK_PATH` must point to the SDK. Output:

```text
mtk3pico_smp1_usb_cdc_s4_test.uf2
```

This uses **the same `app_program/app_main.c` and its two-phase init/start
flow** as the team build. It selects only S4's application sources, while
retaining the kernel, USB console and both cores. Other application subsystems
are not initialized or started; in particular no S2 motor test runs.
`WIFI=none` avoids needing the radio for this sensor bench test and works on
an RP2040 Pico or Pico W. Pico 2 / RP2350 is a different target.

Optional profiles:

```sh
# Same S4 test with the team's built-in Pico W radio service enabled.
# Needs the SDK lib/cyw43-driver submodule; does not add an S4 network task.
make -j4 S4_ONLY=1 TEST=1

# Single-core diagnostic fallback, still using the same lifecycle.
make -j4 S4_ONLY=1 TEST=1 WIFI=none SMP=0

# Continuous raw-data bench image without on-target self-tests.
make -j4 S4_ONLY=1 WIFI=none
```

The build profile and output filename include `_s4`, so switching back to
`S4_ONLY=0` discards incompatible objects. A clean build is still appropriate
when changing SDK paths/versions or flags not encoded in that profile.

## What changed from the earlier starter

| Earlier starter | This team adaptation |
| --- | --- |
| Its own stand-alone `usermain()` | Existing team `usermain()` calls `subsystem4_init()` / `subsystem4_start()` |
| Separate `buddy4/` folder and build profile | Drivers and tasks live in `app_program/subsystem4/` |
| Single-core-only readout | One I2C-owning task at priority 18, pinned to core 1 for SMP; priority-19 reporting task |
| Local print-loop cache | `s4_sensor_get_status()` copies a snapshot under an internal priority-inheritance mutex |
| Hardware-timer sample timestamps | Kernel `tk_get_otm()` timestamps, with millisecond resolution, scaled to us for the driver API |
| Independent SDK clock/time imports | Reuses the WiFi port's hooks when WiFi is built; imports fallback SDK modules only when needed |
| Stand-alone test record | Host tests plus team-style `[TEST] S4-Uxx PASS/FAIL` on-target tests and `testing/subsystem4/TEST_LOG.md` |

`drv_lsm303dlhc.c/.h` retains the tested decoding, sensor identity/configuration
checks, accelerometer auto-increment, magnetic X/Z/Y remapping, clipping checks,
independent channel freshness and failed-read handling from the earlier starter.

## Ownership and lifecycle

`subsystem4_init()` creates a snapshot mutex and the retained IF-03 message
buffer/event flag. It does **no sensor I/O**. `subsystem4_start()` uses the
team's `app_start_task()` helper to create:

- `s4_sensor_task`: priority **18**, `TP_PRC2` (physical core 1). It alone
  initializes and accesses I2C1, GP2 and GP3, initializes the sensor, polls it
  after a 20 ms delay, and publishes status. In SMP=0 the helper ignores affinity.
- `monitor_task`: priority **19**, any core. It reads snapshots, drains the
  legacy S3 summary buffer, and prints labelled CSV about five times per second.
  It does not touch I2C, S2 internals, S3 statistics or S1 task state.

The existing 2048-byte application-task stack is retained. Stack headroom,
poll timing and scheduling under the final workload need bench measurement.
There is no exact-period or worst-case-execution-time guarantee here.

The adapter calls real Pico SDK I2C/GPIO functions. A register-pointer write
and its repeated-start read share one **5 ms transaction deadline**. An I/O
error resets I2C1's controller state before retry. A whole initialization/poll
contains several transactions and may take longer than 5 ms. SDK IRQ dispatch,
runtime startup, hardware alarm allocation and hardware spinlock allocation
are not added. The kernel still owns startup and scheduling.

The port's fixed clocks are assumed: `clk_sys=125 MHz`, `clk_ref=12 MHz`.
If the team changes these, revise `drv_sdk_clock.c` and timer-tick setup.

## Read data from another subsystem

In task context, include `../subsystem4/subsystem4.h`, then:

```c
s4_sensor_status_t sensor;
ER result = s4_sensor_get_status(&sensor);

if ((result == E_OK) && sensor.accel_usable)
{
    INT ax_mg = (INT)sensor.latest.accel.milli_units[0];
    /* Consume the value and its timestamp here. */
    (void)ax_mg;
}
```

`E_OK` means the snapshot was copied, **not** that the sensor is connected.
The getter returns `E_PAR` for NULL, `E_OBJ` before initialization, or a
kernel error such as `E_TMOUT` if its bounded 5 ms mutex wait fails. It leaves
the caller's output unchanged on failure. Never hold S4's internal lock or
call its I2C driver from another task or an ISR.

| Field | Meaning |
| --- | --- |
| `configured`, `init_result` | Whether the most recent sensor configuration succeeded; result values are in `drv_lsm303dlhc.h` |
| `latest.accel` / `latest.mag` | Last complete XYZ sample, raw counts, scaled units, original read timestamp and latest poll status |
| `accel_usable` / `mag_usable` | Configured, complete, unclipped and no more than 100 ms old at retrieval |
| `*_age_ms` | Age of the last complete sample; `S4_NO_SAMPLE_AGE` means none or an inconsistent future timestamp |
| `*_samples` | Complete acquisitions, including clipped values; compare these counters/timestamps to detect new data |
| `*_io_errors`, `accel_overruns` | Cumulative diagnostics; the cache is invalidated for a channel after its I/O error |
| `task_heartbeat`, `publish_errors` | Acquisition-loop progress and snapshot-publication failures |

`latest.*.fresh` means the cache has acquired a complete sample at least once;
it is not a promise of a new conversion for every getter call. Use the
`*_usable` flags, counters and original timestamps. `mg` is milli-g
acceleration; `mG` is milligauss magnetic field. They are different units.

Public sample `read_time_us` and `observed_time_us` use the kernel clock,
not the SDK hardware timer's unrelated boot origin. Their resolution is
**1 ms**, despite the us scale. For future fusion, convert encoder timestamps
to the same timebase and define sample alignment with Buddy 2. Do not
substitute commanded motor power for measured distance/speed.

The getter is a **latest-value interface**, not a lossless event queue.
Impact/hump event messaging to telemetry and speed requests to the Vehicle
Controller remain the next integration steps. S4 never writes motor outputs
and does not call WiFi, lwIP or MQTT APIs from either of its tasks.

## Wiring and the first physical test

Use GP2 / physical Pico pin 4 for SDA, GP3 / pin 5 for SCL, and common GND
(for example physical pin 3). The expected seven-bit addresses are **0x19**
for acceleration and **0x1E** for magnetic field.

Verify the actual GY-511 breakout's VIN/VCC/regulator and pull-ups before
connecting power. A chip datasheet does not define the breakout circuit.
SDA/SCL must use 3.3 V logic; do not pull them to 5 V. If the board accepts
a 3.3 V supply at its labelled power input, Pico 3V3(OUT) is physical pin 36.
Leave DRDY/interrupt outputs unconnected for this polling test. Wire with
power off, and keep motor/servo power off for bench tests.

1. First leave the sensor disconnected and flash the **no-WiFi `_s4_test`**
   image using BOOTSEL and the `RPI-RP2` drive.
2. Open the Pico USB serial port in **Tera Term**, 115200, 8N1, no flow control.
   Headers repeat every five seconds so opening the terminal after boot is fine.
3. Confirm `[S4]` time/heartbeat advance, `init=3` after an attempted read,
   `a_ok=m_ok=0`, and ages `-1`. This is the expected missing-sensor case.
4. The test image prints S4 unit checks; hardware check `S4-U06` is explicitly
   **SKIP** when the sensor has not configured. Do not record that as PASS.
5. Once power wiring is verified, connect with power off and repeat. Expect
   `init=0`, increasing sample counts, low ages and both usability flags set.
6. Save 30 seconds at rest, six-face tilt readings and a slow rotation log.
   At rest, the acceleration vector should be roughly 1000 mg in magnitude.
   These are response/scaling checks; calibration and accuracy tests come later.

Persistent I/O errors trigger reconfiguration after ten failed polls and a
one-second backoff. Invalid/stale data is never made usable just because a
task is still running. Hot-plug, stuck-bus, timing and cross-core runtime
behaviour still require physical testing; a successful build does not prove them.

## Tests without the Pico

From the project root in WSL/Linux/macOS:

```sh
make -C app_program/subsystem4/tests/host test
make -C app_program/subsystem4/tests/host sanitize
```

There are 12 host test groups, `S4-U10` through `S4-U21`, using strict warnings.
They cover the real driver, the real cache/quality logic, and the lifecycle/
snapshot code against simulated kernel calls. They do not emulate the RP2040
or prove the kernel's real SMP behaviour. In a container that blocks
LeakSanitizer's process inspection, use `ASAN_OPTIONS=detect_leaks=0` with the
sanitizer target; address and undefined-behaviour checks remain enabled.

Record physical tests in `testing/subsystem4/TEST_LOG.md` and follow the team's
log/commit conventions. Retired test `S4-U02` is not reused for a different test.

## Full-team integration blockers in the supplied ZIP

The unmodified upload does not currently complete a full application build:

1. `app_program/subsystem2/subsystem2.c` includes missing `../board_pins.h`.
   Only the host-test fake exists; it explicitly says it is not a board config.
2. `subsystem2/task_producer.c` and `subsystem3/task_consumer.c` still use the
   example `SAMPLE_MSG`, `sem_slots`, `mpf_samples` and `S2_PERIOD_MS` contract,
   which the updated motor-control `subsystem2.h` no longer declares.

S2/S3 owners need to reconcile those existing interfaces with the vehicle
controller and actual board mapping. Their files are preserved in this
adaptation. Do not copy the simulated host board header into the firmware.

`S4_ONLY=1` allows our own build/bench work now. `S4_ONLY=0` selects the normal
all-subsystem application, but its full build remains blocked by those
pre-existing S2/S3 gaps. Review the shared changes listed in
`BUDDY4_ADAPTATION.md` before merging them into the team repository.
