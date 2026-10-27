# Buddy 2 Motion Control

Student ID: 2500613

The shared [external data representation](../../docs/DATA_REPRESENTATION.md)
defines Buddy 2's compact motor/encoder records, the five-buddy envelope,
Python decoding, field ranges and integration rules. Task-context byte adapters
are declared in `s2_data.h`; they preserve the existing controller and watchdog
boundaries. See [verification evidence](../../testing/data_representation/TEST_LOG.md).

Open-loop PWM motor commands with optional dual-wheel x1 encoder diagnostics.
The downloaded encoder update is merged with the repository's command watchdog,
validation, error handling and motion-status API.

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
`S2_ENCODER_ENABLED=0` (the default) avoids all encoder GPIO/IRQ access. Build
options participate in the profile stamp so switching them rebuilds the objects.

`S2_ONLY=1` builds and starts only this subsystem. The default `S2_ONLY=0` selects
the full team application, which still contains sample producer/consumer code
referencing removed `SAMPLE_MSG`, `sem_slots`, `mpf_samples` and `S2_PERIOD_MS`
interfaces. That build needs the team's controller integration. The downloaded
`task_motor.c` is another copy of the old sample producer and was not added.

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

These counts do not implement PID, calibrated distance or rotation to an angle.
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
the vehicle-controller task must call `s2_controller_bind()` before sending.
`app_vehicle_request()` remains an external controller API, not implemented here.
Controller request leases and motor command expiry are separate: an active
request still needs periodic motor renewals.

`s2_motion_get_status()` provides a semaphore-protected snapshot of commanded
state, effective command/power, error and timestamp. It does not confirm actual
wheel motion.

## Verification

From the repository root, with GCC on PATH:

```powershell
powershell -ExecutionPolicy Bypass -File .\app_program\subsystem2\tests\host\run.ps1
```

Eight host configurations exercise the actual motor task with simulated RTOS,
time and GPIO/PWM services, compiled with warnings as errors. Coverage includes
both encoders and directions, rollover, interrupted snapshots, shared/cross-bank
GPIOs, disabled encoders, integrated controller access, rotation renewals/expiry,
stale commands, deadtime expiry, stop/fault recovery and a 750 ms override.
Standalone USB/WiFi firmware builds with encoders on (1000 ms timeout) and off
(750 ms timeout) also pass ARM compilation and linking. Physical motor/encoder
testing on the Pico remains required.
