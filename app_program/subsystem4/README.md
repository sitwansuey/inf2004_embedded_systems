# Subsystem 4: GY-511 IMU, tilt, impacts and terrain

S4 owns the GY-511 (LSM303DLHC accelerometer and magnetometer) on I2C1. It
samples the sensor, publishes a compact acquisition record, and provides the
terrain algorithm that turns acceleration and wheel travel into pitch, roll,
hump state and hump height. The vehicle controller (VC) runs that algorithm
on S4's record each tick. See [TEAM.md](../../TEAM.md) for the owner, pins,
peripherals, task priorities, interfaces IF-04, IF-10 and IF-20, and the
kernel object budget.

## Hardware

| Pin | Use |
|---|---|
| GP2 | I2C1 SDA to the GY-511 (internal pull-up) |
| GP3 | I2C1 SCL to the GY-511 |

The bus runs at 100 kHz (`drv_pico_i2c.h`). The accelerometer (address
0x19) gives a sample every 20 ms at ±2 g; the magnetometer (0x1E) is
telemetry only. The board's mounting is mapped to the robot frame (X front,
Y left, Z up) by `BOARD_IMU_FRONT_AXIS`, `BOARD_IMU_LEFT_AXIS` and
`BOARD_IMU_UP_AXIS` in `../robot_config.h` (`imu_mount.h`). The pitch and
roll zero offsets and the terrain calibration are in the same file.

S4 uses the Pico SDK's I2C, GPIO, timer and tick code, which
`sdk.mk` adds to every build (TEAM.md section 5, shared-file changes).

## Tasks and timing

| Task or handler | Priority, core | Job |
|---|---|---|
| sensor cyclic handler | 10 ms, core 1 | Wakes `s4_sensor_task`; nothing else |
| `s4_sensor_task` (`task_sensor.c`) | 18, core 1 | The only user of I2C1: init, poll and recovery, then publishes the record; writes the IMU watchdog heartbeat (IF-20) |
| `s4_monitor_task` (`task_monitor.c`) | 19, any core | Prints `[S4]` CSV and `[S4TILT]` lines on the console every 200 ms |

`sensor_cycle.c` holds one acquisition cycle without kernel or hardware
calls. Ten failed accelerometer polls in a row, or ten polls without a fresh
sample, re-initialise the device after freeing a stuck bus
(`i2c_bus_clear.h`); failed set-ups are retried every second.

## Data

- `sensor_data.c` keeps the persistent state as a packed 71-byte record
  (`s4_sensor_cache_t`): latest accelerometer and magnetometer frames,
  usable and clipped flags, and wrapping counters for samples, I/O errors,
  overruns, impacts and faults. Readers compare counters with their last
  copy, so a short impact is not missed between reads. An accelerometer
  sample above 1.8 g, or clipped at ±2 g, counts as an impact.
- `s4_sensor_get_record()` copies that record under the priority-inheritance
  mutex `status_lock` (5 ms wait). `s4_sensor_get_status()` decodes it into
  `s4_sensor_status_t` with sample ages; check `configured` and each
  `*_usable` flag rather than the raw values.
- `terrain.c` classifies FLAT, ASCENT, CREST, DESCENT, SHOCK or INVALID and
  integrates hump height from pitch and wheel travel. `encoder_input.c`
  checks the S2 encoder snapshot (IF-04) for age, gaps and IMU skew before
  the travel is used. Up to 60 ms of unusable input is bridged.
- `result_data.c` packs the VC's terrain result (pitch, roll, height,
  terrain state, flags and faults) for telemetry.

## Interfaces

| ID | Direction | API |
|---|---|---|
| IF-04 | S2 → VC (→ S4 terrain) | `s2_encoder_snapshot()`, checked by `encoder_input.c` |
| IF-10 | S4 → VC, S1 | `s4_sensor_get_record()`: the 71-byte record under `status_lock` |
| IF-20 | S4 → watchdog | `HEALTH_BEAT(HEALTH_HB_IMU)` once per sensor loop |

## Build and run

The integrated build starts S4 with the other subsystems. For bench work on
the IMU alone:

```sh
make -C build_make S4_ONLY=1 -j8
```

This starts only S4 (no VC, other subsystems or watchdog) and prints the
monitor lines on the USB console. It needs `PICO_SDK_PATH` like every
other build.

## Verification

From the repository root with GCC, GNU Make and `python3`:

```sh
make -C app_program/subsystem4/tests/host analyze sizes
```

`test` runs the driver, data, API, recovery, terrain-input, result and
odometry-storage suites against a simulated kernel and bus, then checks the
result vectors with `result_vectors.py`; `analyze` rebuilds and runs the
same suites with `-fanalyzer`; `sizes` prints the packed record sizes.
`make -C app_program/subsystem4/tests/host sanitize` adds ASan and UBSan
where the host compiler supports them (the `Buddy 4 integration` workflow
runs it on Linux). The terrain algorithm with the VC is also covered by
`make -C testing/integration test`.

Test cases, results and saved logs are in
[testing/subsystem4/](../../testing/subsystem4/TEST_LOG.md).
