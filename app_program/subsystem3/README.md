# Subsystem 3: IR line sensing and Code 39 barcodes

S3 samples the two straddle line sensors and the barcode sensor, classifies
the line, captures and decodes Code 39 letters, and hands the results to the
vehicle controller (VC). It does not steer: the VC's line follower
(`../ir_line_trace.c`, `../vehicle_line_steer.h`) turns S3's line result into
S2 movement commands. See [TEAM.md](../../TEAM.md) for the owner, pins,
peripherals, task priorities, interfaces IF-05 and IF-07 to IF-09, and the
kernel object budget.

## Hardware

| Pin | Use |
|---|---|
| GP26 | Right straddle line sensor, ADC channel 0 |
| GP27 | Left straddle line sensor, ADC channel 1 |
| GP28 | Barcode sensor module DO, digital input (high = black) |

The mission profile is `BOARD_IR_MODE 3` in `../robot_config.h`: ADC line
sensors and a digital barcode sensor. The ADC endpoints are in
`../robot_ir_calibration.h`; `../robot_ir_config.h` applies the sensor roles,
the barcode settings and the default letter actions. The kernel ADC driver
`adca` only enables the analogue pads that the IR mode needs
(`device/adc/sysdepend/rp2040/adc_cnf_sysdep.h` reads `BOARD_IR_MODE`; see
TEAM.md section 5, shared-file changes). Modes 0 (disabled), 1 (all digital)
and 2 (all ADC) are build options for other wiring.

## Tasks and timing

| Task or handler | Priority, core | Job |
|---|---|---|
| sampler cyclic handler | 10 ms, core 0 | Wakes `consumer_task`; nothing else |
| `consumer_task` (`task_consumer.c`) | 16, core 0 | The only caller of the IR adapter: one scan, line and barcode update, status publish per release |
| `s3_adc_worker` (`drv_ir_adc.c`) | 17, core 0 | The only task that reads `adca`; started by `consumer_task` on its first pass |

`drv_ir.c` hides the sensor mode. In ADC modes the sampler sends a request
to the worker through a message buffer and waits for the copied reply with a
bounded timeout, so a blocked ADC read cannot block the sampler. A late scan
is dropped and requested again; a device or transport failure is latched and
reported as an S3 fault.

## Algorithms

`buddy3_line.c`, `buddy3_barcode.c` and `buddy3_core.c` are plain C with no
kernel or hardware calls, so the host tests run them directly.

- **Line.** Raw readings are normalised between the calibrated white and
  black endpoints (white 0, black 1000) with hysteresis. Both straddle
  sensors white means the robot is centred on the line.
- **Barcode.** GP28 edges become black and white elements. With encoders
  built in (`S2_ENCODER_ENABLED=1`), element widths are measured in wheel
  travel (half encoder counts from `s2_encoder_get_status()`, IF-05), so
  speed changes do not distort them; otherwise in time. A capture ends when
  the VC reports 30 mm of travel without a GP28 edge
  (`s3_controller_barcode_end()`), with a 4 s time backstop. The decoder
  accepts framed Code 39 letters A to Z in either direction.
- **Actions.** The default codebook is A U-turn, F left, Z right; B, C, D
  and every other letter are read without a turn. The dashboard's MAP
  command changes future decodes through `s3_controller_set_map()`.

## Interfaces

| ID | Direction | API |
|---|---|---|
| IF-05 | S2 → S3 | `s2_encoder_get_status()`: wheel travel for bar widths |
| IF-07 | VC → S3 | `s3_controller_bind()`, `s3_controller_control()` (PAUSED / TRACK / OVERRIDE with a lease), `s3_controller_renew()`, `s3_controller_set_map()`, `s3_controller_barcode_end()` |
| IF-08 | S3 → VC, S1 | `s3_status_get()`: `s3_status_t` copied under the priority-inheritance mutex `mtx_stats`, never blocking; a sample older than 30 ms is not used |
| IF-09 | S3 → VC | `s3_event_receive()`: navigation, stop, fault and recovery events from the `h_events` message buffer (at most 4, younger than 100 ms) |

Only the bound controller task may change the mode or take events. TRACK
needs a lease (the VC uses 100 ms and renews it every tick); a status read
after the lease has expired, or of a sample 30 ms old or more, reports
`steering_allowed = false`. After a decoded turn
S3 holds further markers until the VC acknowledges it with TRACK and a
marker clear.

## Verification

Host tests, from the repository root with GCC and GNU Make:

```sh
make -C testing/subsystem3 test analyze check-headers verify-failure
make -C testing/subsystem3 arm-check   # also needs arm-none-eabi-gcc
```

`test` builds and runs every S3 host suite (line, barcode, adapter, ADC
transport, calibration and layout). `analyze` repeats it with
`-fanalyzer`, `check-headers` parses the sources against the real kernel
headers, and `verify-failure` checks that a failing assertion fails the
run. The `Buddy 3 CI` workflow runs the same targets.

On the robot, `make -C build_make TEST=1 NET=lwip MQTT=1 -j8` builds the
passive on-target checks `S3-U01`, `S3-U66` and `S3-U67` (`subsystem3_test.c`); see
[testing/README.md](../../testing/README.md). Test cases, results and saved
logs are in [testing/subsystem3/](../../testing/subsystem3/TEST_LOG.md).
