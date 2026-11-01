# Integrated mission setup

The default firmware intentionally boots DISARMED with motors locked. Software
changes cannot establish wheel dimensions, sensor mounting, PID tuning or track
performance. Do not copy synthetic test constants into the hardware profile.

## Local calibration

Use ignored `app_program/robot_config.local.h`. An autonomous mission requires:

- `BOARD_MOTORS_DISABLED=0`, actual `BOARD_IR_MODE` and calibrated IR endpoints.
- Build with `S2_ENCODER_ENABLED=1` (the Makefile default), after checking wiring.
- Measured `BOARD_TICKS_PER_REV` (x1 counts), `BOARD_WHEEL_CIRCUMFERENCE_MM`
  and `BOARD_TRACK_WIDTH_MM`.
- `BOARD_TERRAIN_CALIBRATED=1` only after verifying sensor mounting and
  `BOARD_PITCH_ZERO_CDEG` / `BOARD_ROLL_ZERO_CDEG` on level ground.
- `BOARD_PID_ENABLED=1`, measured `BOARD_PID_MAX_CPS`, and tuned
  `BOARD_PID_KP_MILLI`, `BOARD_PID_KI_MILLI`, `BOARD_PID_KD_MILLI`.
  `BOARD_PID_CALIBRATED=1` confirms the tuning run. Gains are milli-percent PWM
  per counts/second error; I uses seconds and D uses error change per second.
- `BOARD_BYPASS_VALIDATED=1` only after verifying scan geometry, clearance,
  encoder turns and recovery on the actual robot.

PID independently regulates both wheel speeds against equal targets for
straight travel, reducing wheel mismatch. Requested power selects a fraction
of the measured speed scale; feedback PWM is capped by `BOARD_PID_MAX_POWER`
(default 50%). This is requested demand, not a measured PWM duty. Updates run
in the motor owner at 10 ms; a >100 ms update gap, invalid encoder snapshot or
500 ms without forward progress of an active wheel faults/stops the motor.
Integral anti-windup prevents accumulation against output limits. Initial
example gains are not a tuning report. Measure speed overshoot, steady-state
error, straight drift, distance error and 90/180-degree turn error.

## Commands

USB `r` resets faults while stopped; `s` always stops. USB `a` retains the
supervised 500 ms ARM lease. USB `g` sends one autonomous START at demand 30:
once started, onboard sensing/control runs without remote lease renewal.
It still stops on local faults, STOP, lost safety data or motor watchdog expiry.
It does not restart automatically after a fault.

MQTT commands use `csc02/command` (separate from the single telemetry topic).
Frames are six raw bytes: `[0xC1, opcode, demand, lease_lo, lease_hi, sequence]`.

| Opcode | Behaviour |
|---|---|
| 0 | urgent STOP, survives a full command queue |
| 1 | supervised ARM, demand 1-50, lease 1-1000 ms |
| 2 | RESET while stopped, demand/lease zero |
| 3 | autonomous START, demand 1-50, lease zero |

USB examples at demand 30: START `c1 03 1e 00 00 00`, STOP `c1 00 00 00 00 00`.
Command publishers must use non-retained messages. Reconnect clears network
start authorization and requires RESET before accepting ARM/START. An already
running autonomous mission continues through a telemetry disconnect; timed ARM
still expires locally. Both modes retain controller-to-motor watchdog renewal.
The Python dashboard is read-only and subscribes only to
`Robopico/sensors/edr16/v1`; it does not send driving commands.

## Navigation and completion evidence

The Code39 decoder supports A, B, C, D and legacy Z in either travel direction.
Defaults: A left, B right, C straight, D U-turn; Z unmapped. `BOARD_MAP_*`
can change the course codebook without changing the decoder. Turns use encoder
yaw. After turning/bypassing or losing the line, recovery checks fresh forward
clearance and scans +30, -30, +60, -60 degrees around its starting heading, at
most 15% demand. It stops on missing safety data or after four seconds without
reacquiring the line. Track geometry and stopping clearance need physical tests.

Run `python -B testing/data_representation/run.py --compiler clang` and
`python -B testing/integration/run.py --compiler clang` (or GCC), plus the
Buddy 3/Buddy 4 test targets in README. `--compile-only` on the integration
runner is explicitly a build check, not a passed execution suite.

Before the demonstration, record line/barcode success rates (all four commands),
hump maximum versus measured height, obstacle profiles and bypass/reacquisition,
Wi-Fi reconnect/dashboard freshness, PID tuning, motion accuracy, task timing
and memory usage. Host tests cannot establish these physical measurements.
