# Integrated mission setup

The firmware boots DISARMED and drives only after RESET then START (dashboard
or USB `g`); STOP, watchdogs and faults always apply. The default profile
enables the motors. Software changes cannot establish wheel dimensions, sensor
mounting, PID tuning or track performance. Do not copy synthetic test
constants into the hardware profile.

## Calibration profile

The measured mission profile is the set of defaults in
`app_program/robot_config.h`. Per-checkout overrides go in the git-ignored
`app_program/robot_config.local.h` (template
`app_program/robot_config.local.example.h`), for example
`BOARD_MOTORS_DISABLED 1` for USB/sensor bench work. Re-measure and update the
defaults after changing the hardware. They encode what an autonomous mission
requires:

- `BOARD_MOTORS_DISABLED=0`, actual `BOARD_IR_MODE` (3: calibrated ADC line on
  GP26/27, digital GP28 barcode) and calibrated IR endpoints
  (`app_program/robot_ir_calibration.h`).
- Build with `S2_ENCODER_ENABLED=1` (the Makefile default), after checking wiring.
- Measured `BOARD_TICKS_PER_REV` (x1 counts), `BOARD_WHEEL_CIRCUMFERENCE_MM`
  and `BOARD_TRACK_WIDTH_MM`.
- `BOARD_TERRAIN_CALIBRATED=1` only after measuring the GY-511 mounting:
  `BOARD_IMU_FRONT_AXIS` / `_LEFT_AXIS` / `_UP_AXIS` name the signed sensor
  axis (1=X 2=Y 3=Z, negative for the minus axis) that points to the robot's
  front, left and up. Lifting the front must raise the front axis reading,
  lifting the left side the left one. Then set `BOARD_PITCH_ZERO_CDEG` /
  `BOARD_ROLL_ZERO_CDEG` to the robot-frame pitch (nose-up positive) and roll
  (left-up positive) read standing still on level ground.
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
in the motor owner every poll (it asks for 10 ms, but the kernel adds a 10 ms
tick to every relative wait, so about 20 ms, like the vehicle controller loop);
a >100 ms update gap, invalid encoder snapshot or
500 ms without forward progress of an active wheel faults/stops the motor.
The motor task never exits: a missed status copy is retried on the next poll,
and a motor/encoder setup failure keeps PWM off and S2 in FAULT, so ARM/START
gives fault 7 until a power cycle.
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
| 4 | MAP barcode letter: byte 2 letter 1-26 (A-Z), byte 3 action 0-4, byte 4 zero |

MAP actions use the navigation numbering: 0 ignore (read, no turn), 1 left,
2 right, 3 straight, 4 U-turn. Letter 0 with action 0 restores the built-in
codebook for every letter. MAP is accepted armed or not (no RESET needed) and
changes only future decodes; the table lives in RAM, so resend it after a
reboot. Example: F turns right, `c1 04 06 02 00 07`.

Replies use `csc02/command/ack`: `[0xC2, sequence, opcode, result, phase, fault]`,
result 0 accepted, 1 rejected (bad fields, queue full, or not allowed now),
2 RESET required (ARM/START after reconnect), 3 stale (queued 100 ms or more).
Network-side replies carry the last published phase and fault. Commands
superseded by STOP get result 1. Two STOPs offered before the VC takes one
are both answered 0; a full ACK buffer drops and counts its oldest reply.
Dashboard sequences are 0-127. USB uses bit 7, so its ACK cannot answer a
dashboard command. A timestamp just ahead of the VC clock is fresh.

USB examples at demand 30: START `c1 03 1e 00 00 00`, STOP `c1 00 00 00 00 00`.
Command publishers must use non-retained messages. Reconnect clears network
start authorization and requires RESET before accepting ARM/START. An already
running autonomous mission continues through a telemetry disconnect; timed ARM
still expires locally. Both modes retain controller-to-motor watchdog renewal.
Before the first Wi-Fi UP, a stalled join gets 20 s before leave/rejoin.
After a previous UP, an access-point loss gets 10 s. The owner waits 1 s
after leaving before joining again. Rejected join requests retry every 5 s.
A dead bus still parks the network task. Leaving drops the MQTT session; MQTT reconnects within about 3 s of the
link returning and the event log shows "reconnected after N s".
The Python dashboard subscribes to `Robopico/sensors/edr16/v1`,
`Robopico/events/edr16/v1` (the robot event log) and `csc02/command/ack`; it
publishes only button-pressed RESET/START/STOP and MAP frames (see
`app_program/external_systems/Dashboard/MQTT_TOPICS.md`).

## Navigation and completion evidence

The Code39 decoder reads any A-Z in either travel direction; A, B, C, D, F and
Z can be mapped to turns. Codebook: A U-turn, Z right, F left; B, C, D are
read and shown but unmapped (no turn). `BOARD_MAP_*`
can change the course codebook without changing the decoder. Turns use encoder
yaw. While tracking, the default mask-pivot follower pivots toward a straddle
IR on black and drives straight when both are white. Because that is also what
a lost line looks like, `BOARD_LINE_LOST_MM` (600 mm, 0 = off) of tracking
with neither IR on black starts the on-the-spot search below (not during a
barcode read or re-read, or while a decoded turn looks for its junction); a
line-data gap of 250 ms or more stops the run (fault 2, line, cause 1). Line
reacquisition is entered at START, after a navigation turn or a bypass, when
an acquired obstacle goes away, and after `BOARD_LINE_LOST_MM` of tracking
without a sensor on black; with fresh line data it hands straight back to
tracking. After a navigation turn
that ended without a line touch (a U-turn always; left/right with
`BOARD_NAV_SPOT_SCAN_ALL`), a bypass that restored its heading without
finding the line, or a line lost while tracking, only a sensor on black ends
it: recovery checks fresh
forward clearance and scans +30, -30, +60, -60 degrees around its starting
heading (toward the line's side first), at most 15% demand. It stops on
missing safety data or after four seconds without a sensor on black (fault 2,
cause 3). Track geometry and stopping clearance need physical tests.

A sampler hiccup can latch S3 STALE/SENSOR without an event, which stops
barcode capture until TRACK is sent again. While tracking on fresh line
samples the controller re-sends TRACK at most every 100 ms (`VC_S3_REARM_MS`),
keeping a decoded turn's marker; a latch still returning 1 s after it was
first seen (`VC_S3_LATCH_MS`) stops the run with fault 2 cause 2. A decoded
turn survives a later latch. S3 holding a marker with no pending turn gets
TRACK again after 100 ms. A late ADC job retries if no driver read overran;
a driver/device error still latches. `BOARD_BARCODE_FAULT_FAILS` defaults to
1: a sampler fault cutting a read of at least three elements short reports
a failed attempt and starts the usual re-read. Other 3-8 element partials
are counted and logged on USB without a re-read.

Run `python -B testing/data_representation/run.py --compiler clang` and
`python -B testing/integration/run.py --compiler clang` (or GCC), plus the
Buddy 3/Buddy 4 test targets in README. `--compile-only` on the integration
runner is explicitly a build check, not a passed execution suite.

Before the demonstration, record line/barcode success rates (all four commands),
hump maximum versus measured height, obstacle profiles and bypass/reacquisition,
Wi-Fi reconnect/dashboard freshness, PID tuning, motion accuracy, task timing
and memory usage. Host tests cannot establish these physical measurements.

## Checkpoint 8c checks

Integrated builds require IR mode 2 or 3. Modes 0 and 1 fail at compile time.
A controller bind/calibration failure prints STARTUP FAIL on USB.
Motor-status copy misses reuse the last status for three ticks; a fourth
miss or a real S2 FAULT stops. Missing odometry holds stopped for 200 ms
before fault 1 cause 3. A sonar-held spot search still ends at 4 s with
fault 2 cause 3. These changes do not change the tuned follower.

On the hump, plain descents get the 20% slope cap. Smoothed pitch keeps
that cap if height tracking is dropped. A flat top cannot finish the ramp
while S4 is in ASCENT/CREST. The level event uses the completed height for
that traversal, or zero. Short unusable runs bridge at most 60 ms; large
reversals and longer gaps still discard the traversal. The armed lift rule
never waives tilt. Impacts remain report-only (`BOARD_SHOCK_STOP 0`).

S4 records every fresh impact and accel fault, including ones between VC
reads. The VC counts counter deltas and its own IMU faults. Magnetometer
errors do not reset the accelerometer. Ten failed accel polls, or the
no-data watchdog, trigger a re-init on the next cycle. A failed re-init
backs off 1 s. A stuck SDA gets up to nine SCL clocks and a STOP.

Calibrate `BOARD_SERVO_US_PER_90` (750 us, allowed 500-1000) using measured
pulse changes at both +/-45 and +/-90 deg. Pulse limits still apply. S5
confirms an edge with two outward clear angles and checks the bypass side
corridor. The first braking restart is free; later motion restarts keep
the existing bound. Widths and side persist throughout the bypass.

BACKUP waits for a new APPROACH/0 sample, at most
`AVOID_BACKUP_SETTLE_MS` (1500 ms), then starts its full timed reverse.
No sample by the bound is fault 1 cause sonar. A shallower reverse replan
gets a stopped ray check for `AVOID_REPLAN_WAIT_MS` (1000 ms); blocked or
missing data keeps S5's checked heading. After every side pan the robot
waits for a clear new forward sample; `AVOID_FRONT_WAIT_MS` (3000 ms)
bounds that wait. DIAGONAL and ALIGN still have 10 s total limits.

Use the USB `[TIMING]` report every 10 s to record longest run, release
gap and late/run counts for VC, motor and network. `BOARD_TIMING_STATS 0`
disables it. Timing and the additional stopped pans need track checks.
Impact-triggered snapshots are limited to one per 200 ms. Newest queued
telemetry wins; dropped snapshots pass their flags on and are counted.
The dashboard holds impact/IMU flags for 2 s and logs their rising edges.
Broker login/ACL setup is optional and still needed to restrict command
publishers; see the dashboard README. Reconnect events go out before the
outage backlog. The six-byte commands and two-byte EDR16 format are unchanged.
