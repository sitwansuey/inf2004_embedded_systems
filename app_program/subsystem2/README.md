# Subsystem 2: motion control

S2 owns motor PWM, wheel-speed PID and both wheel encoders. The vehicle
controller requests motion; S2 applies it and reports commanded state.
See [TEAM.md](../../TEAM.md) for the owner, wiring, peripherals, task priority,
interfaces IF-01 to IF-06 and IF-20 to IF-22, and the kernel object budget.

`speed_pid.h` implements the per-wheel regulator, `wheel_steering.h` splits
steered demands, and `turn_drive.h` controls spin limits, breakaway kicks and
stall allowance. The mission settings are in `../robot_config.h`, including
`BOARD_PID_ENABLED`, gains, speed scale, PWM limits and wheel trims.

## Build and run

Use the [root build guide](../../README.md) for the MQTT mission image.
With the Arm compiler on `PATH` and `PICO_SDK_PATH` set, this standalone build
starts only the motor task:

```sh
make -C build_make S2_ONLY=1 S2_ENCODER_ENABLED=1 -j8
```

It has no vehicle controller, USB command bridge or startup motor
self-test, so nothing sends it commands and the motors stay off; the image
only shows that the motor task builds and starts on its own. `S2_ONLY=0`
selects the integrated application.
`S2_ENCODER_ENABLED=1` is the Make default. Disabling encoders requires
`BOARD_PID_ENABLED 0`; `S2_ENCODER_ENABLED=0` avoids encoder GPIO/IRQ access.

The command timeout defaults to 1000 ms. Set it consistently through Make,
for example:

```sh
make -C build_make NET=lwip MQTT=1 S2_COMMAND_TIMEOUT_MS=750 -j8
```

A nondefault timeout appears in the image filename. These Make options are
part of the build profile stamp, so changing them rebuilds objects.

## Commands and stopping

In the integrated build, the vehicle-controller task calls
`s2_controller_bind()` before sending. Other tasks receive `E_OACV` when
they try to send movement. The task-context APIs are:

| API | Purpose |
|---|---|
| `s2_movement_send(command, power_percent)` | STOP, forward, reverse, spin or pivot |
| `s2_follow_send(power_percent, steering)` | Forward motion with steering |
| `s2_reverse_follow_send(power_percent, steering)` | Reverse motion with steering |
| `s2_emergency_stop()` | Urgent STOP from the bound controller |
| `s2_motion_get_status()` | Semaphore-protected commanded state, power, error and command time |

Steering is −400 to 400. Nonzero steering requires the PID. Commands are
0–6 and power is 0–100%; invalid values return `E_PAR`. RIGHT spins clockwise;
LEFT spins counterclockwise. PIVOT_RIGHT drives only the left wheel forward;
PIVOT_LEFT drives only the right wheel forward.

Moving messages carry their issue time and STOP epoch. Renew commands well
before `S2_COMMAND_TIMEOUT_MS` (for example every 250 ms with the default
1000 ms timeout); queued time counts toward expiry. Identical renewals keep
PWM running. Expiry switches PWM off and publishes
`S2_MOTION_FAULT` with `E_TMOUT`. A fresh command can clear this command fault.
The status describes requested output, not proof that a wheel moved.

In the integrated build, STOP and zero power use `s2_emergency_stop()` and
bypass the mailbox and fixed pool. The motor task notices the changed epoch
on its next poll or before its next PWM write and discards older commands.
In `S2_ONLY=1`, STOP sent through `s2_movement_send()` is queued and can fail
when the pool is full. Check every API result. PWM off does not establish the
physical braking time. A moving command returns `E_OBJ` while an urgent STOP
is still pending.

The motor task never returns. Setup failure keeps PWM off and reports FAULT
until restart. A missed status-lock update is retried. The VC can reuse its
last S2 status through three missed copies; a fourth miss causes fault 7.
A STOP during a command switch ends in STOPPED rather than a motor fault. With PID enabled,
a driven wheel without progress faults after its stall allowance: 500 ms
for straight driving and pivots, or `BOARD_TURN_STALL_MS` (1200 ms by default)
for spins. Stall time carries across compatible command switches and short
stops; see `subsystem2.c` and the host tests for the reset conditions.

The motor loop writes its watchdog heartbeat (IF-20) and, with
`BOARD_TIMING_STATS 1`, timing counters (IF-22). The VC prints the motor
loop's longest run, longest gap and late count in the USB `[TIMING]` line
every 10 s; the run time excludes direction dead time.

## Encoder snapshots and odometry

Each rising edge on channel A counts one x1 pulse; channel B gives direction.
Both channels also collect edge diagnostics. Left forward counts are negative
pulses minus positive pulses; right forward counts are positive minus negative.
Use the assignments in [TEAM.md](../../TEAM.md) and `../board_pins.h`.

Encoder inputs use internal pull-ups and must be 3.3 V signals. S2 installs
the core-0 GPIO bank interrupt handler (IRQ 13); another GPIO interrupt
user must share that handler rather than replace it. S2 also owns PWM
slices 4 and 5.

`s2_encoder_get_status()` returns a sequence-checked copy of both wheels from
task context on either core. Unprefixed diagnostic fields refer to motor 2
(right); `motor1_*` fields refer to the left. Counts wrap modulo 32 bits;
use unsigned subtraction for interval deltas. GPIO levels are live samples.
Before initialization, the call returns `E_OK` with `valid=0`. Disabled
encoders return `E_NOSPT` and zero the output. A clock error also returns an
error and a zeroed snapshot.

`s2_encoder_snapshot()` in `s2_data.c` provides the checked 13-byte internal
handoff with full counts, sample time and validity. `odometry.c` converts
counts to travel, speed and yaw for the controller. S2 itself does not drive
to a distance or angle.

## EDR16 output

S1 reads `s2_motion_get_status()` and `s2_encoder_get_status()` for telemetry.
The direct adapters remain available in `s2_data.h`: `s2_motion_encode()`
returns IDs 11 and 12; `s2_encoder_encode()` returns IDs 1 and 2. Each writes
four bytes containing two independent two-byte records. Publish each record
separately. Full counts and timestamps remain in the local APIs.

See [DATA_REPRESENTATION.md](../../docs/DATA_REPRESENTATION.md) for bit layouts,
units, count wrap and limits.

## Verification

From the repository root with Python 3 and native GCC or Clang:

```sh
python -B app_program/subsystem2/tests/host/run.py
```

Use `--compiler PATH` if needed. The PowerShell wrapper
`app_program/subsystem2/tests/host/run.ps1` runs the same suite. Its nine
profiles cover encoder layouts, rollover, commands, urgent STOP, expiry,
controller access, setup/lock failures, PID output, kicks and stall handling.
They use simulated kernel, time, GPIO and PWM services.

Record results in [testing/subsystem2/TEST_LOG.md](../../testing/subsystem2/TEST_LOG.md)
and save output under `testing/subsystem2/logs/`. That record holds the
host cases S2-U01 to S2-U09 and the passive on-target `TEST=1` checks S2-U29
to S2-U33 (status and encoder reads, no motion); physical wheel direction, wheel speed and
stopping time have no hardware case there yet, so define one before
measuring them. See [testing/README.md](../../testing/README.md) for the
run-and-record rules.
