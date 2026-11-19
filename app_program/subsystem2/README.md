# Buddy 2 Motion Control

Student ID: 2500613

PWM motor commands with dual-wheel x1 encoders and an optional calibrated
wheel-speed PID (`BOARD_PID_ENABLED`): `speed_pid.h` holds the per-wheel
regulator, `wheel_steering.h` the steering split and `turn_drive.h` the spin
limits, breakaway kick and stall allowance. The downloaded encoder update is
merged with the repository's command watchdog, validation, error handling and
motion-status API.

## Shared EDR16 integration

The shared [external data representation](../../docs/DATA_REPRESENTATION.md)
(EDR16) defines Buddy 2's two-byte records and shared ID/data layout, Python
decoding, field ranges and integration rules. B1's telemetry task builds the
motor and encoder records itself (`subsystem1/telemetry_edr.h`) from
`s2_motion_get_status()` and `s2_encoder_get_status()`. B2's own task-context
adapters in `s2_data.h` remain for the host tests: `s2_motion_encode` returns
IDs 11 and 12, `s2_encoder_encode` IDs 1 and 2, each as four bytes holding two
independent two-byte records. Full-width counters and timestamps stay in the
local snapshot APIs. No external variable-length EDR envelope remains. See
[verification evidence](../../testing/data_representation/TEST_LOG.md).

## Build and run

From WSL, with the Arm compiler on PATH and `PICO_SDK_PATH` set as described in
the root README:

```sh
cd build_make
make S2_ONLY=1 TEST=1 S2_ENCODER_ENABLED=1 -j8
```

This produces `mtk3pico_smp1_usb_cdc_wifi_s2_enc_test.uf2`. Raise the wheels before
flashing: the test automatically runs forward, both single-wheel pivots,
clockwise rotation, counterclockwise rotation and reverse at 30% for two seconds
each. Commands renew every 250 ms, with explicit STOPs between moves. The final
test deliberately withholds renewals to check watchdog expiry. Diagnostics
include both wheels' pulse counts and missing-pulse messages.

Use `TEST=0` to start the motor task and wait for commands without running tests.
`S2_ENCODER_ENABLED=1` is the make default (the PID needs both encoders);
`S2_ENCODER_ENABLED=0` avoids all encoder GPIO/IRQ access and requires
`BOARD_PID_ENABLED 0`. Build options participate in the profile stamp so
switching them rebuilds the objects.

`S2_ONLY=1` builds and starts only this subsystem. The default `S2_ONLY=0` selects
the full team application.

## Wiring and encoders

Defaults in `../board_pins.h` follow the downloaded hardware implementation:

| Connection | Left wheel (motor 1) | Right wheel (motor 2) |
|---|---|---|
| Motor PWM A/B | GP8 / GP9 | GP10 / GP11 |
| Encoder A/B | GP16 / GP17 | GP4 / GP5 |
| Forward encoder count | negative minus positive pulses | positive minus negative pulses |

This corrects the earlier README's reversed encoder assignment. Confirm actual
wiring and motor forward signs before testing. Encoder inputs use pullups and
must be 3.3 V compatible. This module owns PWM slices 4/5 and core-0 IO_BANK0
IRQ 13; another GPIO IRQ user must share a dispatcher rather than replace it.

Enable with `S2_ENCODER_ENABLED=1` in standalone or integrated builds. Each rising
edge of A counts one pulse, with B determining direction; both channels also
report edge diagnostics. Register banks are derived separately for every pin.

`s2_encoder_get_status()` returns a consistent snapshot of both wheels' cumulative
counts from task context on either core. Original unprefixed fields refer to
motor 2; added `motor1_*` fields refer to motor 1. Signed `motor1_forward_counts`
and `motor2_forward_counts` follow the wiring signs above. Counts wrap modulo
32 bits; use unsigned subtraction for interval deltas. GPIO levels are live
samples. Check `valid`: enabled startup returns `E_OK` with zero data and
`valid=0`, then `valid=1` after initialization. Disabled builds zero every field
and return `E_NOSPT`.

In PID builds these counts feed the wheel-speed regulator. The motor task never
drives to a distance or angle itself: `odometry.h` turns the counts into
distance and speed for the vehicle controller and S4, and turns to an angle
are the vehicle controller's job.
`S2_MOVE_RIGHT` rotates clockwise (left forward, right reverse); `S2_MOVE_LEFT`
rotates counterclockwise. These mappings already matched the download. Pivot
commands run only the outer wheel.

## Timeout and controller integration

Moving commands expire **1000 ms after sending**, including time spent queued.
Configure every translation unit consistently, for example:

```sh
make S2_ONLY=1 TEST=1 S2_ENCODER_ENABLED=1 S2_COMMAND_TIMEOUT_MS=750 -j8
```

A nondefault timeout appears in the image filename. Renew
`s2_movement_send(command, power_percent)` well before expiry (250 ms with the
default timeout) and check every result. Identical renewals leave PWM running.
The on-target test requires a timeout greater than its 250 ms renewal interval.
The downloaded test's single 1000 ms command conflicts with the 1000 ms watchdog;
its persistent-until-STOP motor loop was not imported.

Expiry switches all PWM outputs off and publishes `S2_MOTION_FAULT` with
`last_error == E_TMOUT`. Stale queued commands cannot restart motion; a fresh
valid command clears the fault. STOP or zero power switches PWM off. Invalid
commands or power above 100 return `E_PAR`. A full pool can reject STOP, so check
the send result. PWM off does not guarantee braking, and the watchdog requires
the motor task to keep running.

Check `subsystem2_init()` and `subsystem2_start()` results. In integrated builds,
the vehicle-controller task must call `s2_controller_bind()` before sending,
and must keep renewing moving commands before they expire.

`s2_motion_get_status()` provides a semaphore-protected snapshot of commanded
state, effective command/power, error and timestamp. It does not confirm actual
wheel motion.

The motor task never returns (a task return HardFaults core 0). A status copy
that misses the lock, which the VC and core-1 telemetry also take, is retried
on the next poll while PWM keeps following the current state (off in FAULT).
If motor or encoder setup fails, PWM stays off and the task keeps reporting
`S2_MOTION_FAULT` (VC fault 7 once armed) until the next power cycle.

## Verification

From the repository root, with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File .\app_program\subsystem2\tests\host\run.ps1
```

Nine host configurations exercise the actual motor task with simulated RTOS,
time and GPIO/PWM services, compiled with warnings as errors. Coverage includes
both encoders and directions, rollover, interrupted snapshots, shared/cross-bank
GPIOs, disabled encoders, integrated controller access, rotation renewals/expiry,
stale commands, deadtime expiry, stop/fault recovery, status-lock timeouts,
motor/encoder setup failure (the task must never return), a 750 ms override and,
with the robot PID settings, the spin/pivot PWM ceiling, breakaway kick and
turn stall time.
Standalone USB/WiFi firmware builds with encoders on (1000 ms timeout) and off
(750 ms timeout) also pass ARM compilation and linking. Physical motor/encoder
testing on the Pico remains required.
