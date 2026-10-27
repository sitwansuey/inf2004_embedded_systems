# 20-second line-only bench — 24 October 2026

IRLineOnly v3 is installed on the operator-identified robot. Normal USB
confirmed version 3, the 20,000 ms cap and a fresh STOP acknowledgement.
Installation used motor power disconnected and wheels raised. The copy and
runtime identity were verified; no physical flash readback hash was measured.
The original sonar-enabled IRLineTrace v2 remains a separate build profile.

## Behaviour and limits

- Both WHITE: forward, assuming the operator initially centred the robot.
- Only GP27 LEFT BLACK: left wheel stopped, right forward.
- Only GP26 RIGHT BLACK: right wheel stopped, left forward.
- Both BLACK: STOP / LINE_AMBIGUOUS.
- WHITE/WHITE cannot prove that the line is present or detect every departure.

This profile disables Buddy 5 acquisition with APP_S5_ENABLED=0. It has no
ultrasonic obstacle protection and does not publish unavailable range as clear.
It does not stop automatically at the end of the line. GP28 remains barcode-only.
The ordinary IRLineTrace/IRMotorScan profiles retain sonar and their 6-second cap.
Core/peripheral ownership, micro T-Kernel 3 and Change Requirements 2/3 are unchanged.

The 20-second duration is a maximum. PWM is capped at 20%; the tested floor
powers were 8% and 10%. USB lease 250 ms, motor watchdog 150 ms, IR freshness
30 ms, explicit START, boot STOPPED and urgent ASCII s STOP remain active.
Stable WHITE on both line sensors and healthy encoders are required before START.

If a wheel commanded forward makes no positive encoder count change for
750 ms, the controller stops with NO_PROGRESS. The intentionally stopped pivot
wheel is excluded. A new explicit START is needed after a stop. This checks
encoder progress; it does not measure physical travel, speed, slip or the
mechanical cause of a missing count. The host finishes promptly on an early stop.

Disconnect USB and motor power before unplugging sensor wiring. A floor test
needs a clear supervised course away from edges, sufficient track and USB slack.
No powered run is queued after installation or after a completed test.

## Hardware evidence

Evidence and original logs: docs/evidence/2026-10-24/line_only_v3.
INSTALLATION_RESULT.json records the copy, runtime identity, STOP and run outcomes.

| Floor test | Requested cap | Observed active to NO_PROGRESS | Result |
|---|---:|---:|---|
| 8% PWM | 20,000 ms | 4,940 ms | Left correction; right count stopped at 273; final STOP acknowledged |
| 10% PWM | 20,000 ms | 2,700 ms | Several left corrections; right count stopped at 413; final STOP acknowledged |

Both tests started with both sensors WHITE. GP27 then detected BLACK and
triggered command 6 (left stopped / right forward). The right encoder stopped
advancing during that command. The user could not tell whether the right wheel
physically stopped, so a motor stall, caster cause or encoder failure is not
proven. Rough/sticking caster behaviour was reported earlier. No floor tracking
pass, speed/accuracy or braking-distance claim is made.

The subsequent 6-second raised-wheel run at 10% completed normally. During a
sustained 4-second left command, M1 stayed at 863 and M2 advanced from 840 to
1802 (41 status rows, continuous positive progress). The user confirmed LEFT
stopped / RIGHT turning forward. Final commanded STOP was acknowledged.
See RAISED_LEFT_VALIDATION.json and the original Steer log. The same power and
firmware therefore sustain this pivot without floor load. Load/friction or
supply under load is a candidate; this does not identify the exact cause or
exclude an intermittent fault. Before another floor trial, inspect the rough
caster, any wheel/chassis/sensor drag or cable snag, and motor power under load.
No further motion is queued.

Earlier v2 evidence remains under docs/evidence/2026-10-24/line_trace_v2:
2,418 stationary samples passed without gaps/mismatches; raised LEFT and RIGHT
responses were confirmed by the user and independent encoder changes. Its first
2-second floor test also showed missing right-encoder progress during a left turn.
Sensor endpoints have not changed.

## Verification and memory

- Native 20-second/progress policy: 70,432 assertions, zero failures, including
  every 16-bit duration, sequence renewal/wrap, fixed deadline, count/time wrap,
  pivot-wheel exclusion, invalid encoders and no-progress boundaries.
- ASan/UBSan passed the same 70,432 checks without reports.
- Existing integration suite passed, including 355 line-direction checks and
  the normal 6-second policy, urgent STOP, transport, sensors and binary codec.
- Host readiness/reporting/early-stop waiting: 19 checks, no serial port opened.
- Profile matrix: 12 expected accept/reject cases passed.
- Full Buddy 3 regression passed; original output retained in BUDDY3_REGRESSION.log.
- Forced full real-SDK ARM build and RP2040 UF2/ELF/source verification passed.
- Separate ARM stack-usage compile reported a 616-byte static frame for
  line_trace_task. This excludes callees and is not runtime stack high-water.

| Bytes | v2 sonar-enabled | v3 line-only |
|---|---:|---:|
| text | 313916 | 309920 |
| data | 160 | 96 |
| bss | 25436 | 25412 |
| linked static RAM | 25596 | 25508 |
| UF2 file | 628224 | 620032 |
| binary telemetry message | 50 | 50 |

The 88-byte static RAM reduction comes from the disabled subsystem/profile,
not an additional CR1 packing gain. Runtime heap/stack occupancy is unmeasured.
The progress watchdog adds local state without changing application message sizes.

Installed UF2 SHA256:
6b8fcd15cb9c9117259e439c24f5a5e9c5b021e4b29046edeba1c95384d038cc

## Local tools and changed files

INSTALL_LINE_ONLY.ps1 defaults to build-only; explicit -Flash copies the image.
RUN_LINE_ONLY.ps1 defaults to Inspect (STOP/query). Trace requires operator
setup flags, v3 identity, healthy encoders and stable WHITE. It records an early
NO_PROGRESS stop as an incomplete floor run. RUN_LINE_TRACE.ps1 retains v2 checks.

Changed: ir_line_progress.c/.h; guarded duration and reason in ir_motor_scan.h;
robot_config.h; vehicle_line_trace.c; app_main.c; build profile files;
integration test/Makefile; separate installer, host helper, verifier and tests.
No calibration endpoints, barcode algorithm or motor GPIO driver changed.
All work remains local on local/csc02-line-tracing-20261024. No push, PR or remote edit.
