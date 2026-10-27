# IR motor scan bench — 24 October 2026

Local branch: `local/csc02-ir-motor-scan-20261024`, base `b02e335`. Existing uncommitted edits preserved.
No push, PR, fetch, remote merge or remote ref update. Cached remote refs match the saved baseline.

## Installed and source state

Source version 3; installed version 3. Flash copy complete; runtime USB boot/STOP verified: **True**.
Installed UF2 SHA256: `f6b8cdc71ed544503f20e4810e63b1f61ad9d9ef75d12dc9a9cf68742f289076`.
The installed named profile **enables motor output** but boots STOPPED. The default,
raw, calibrated and passive capture profiles retain their compile-time motor lock.
This is a straight USB diagnostic, with no line steering, A/Z navigation, bypass or autonomous restart.
No physical A/Z decode has passed. Physical line following and GP26/27 after the borrowed-module change remain unvalidated.

## Behavior and ownership

- USB owner accepts a dedicated bounded binary frame, PREPARE then one START; ordinary MQTT/C1 ARM cannot start this profile.
- Maximum 20% PWM duty; raised-wheel pulse at most 250 ms; floor run at most 6000 ms.
- Host renews a 250 ms USB lease. Buddy 2 independently stops on command timeout at 150 ms.
- Same priority 5/core 0 Vehicle Controller uses Buddy 2's motor API. No direct motor GPIO, encoder, ADC, I2C or sonar ownership changes.
- Buddy 3's existing sampler feeds the production barcode decoder on every acquisition. Generation-tagged copied results reach the controller under the existing status lock. The bound scan controller uses the existing 5 ms requested priority-inheritance mutex handoff; its clock/freshness checks run after that handoff. General snapshot readers remain nonblocking. No new task or message buffer.
- Boot/explicit STOP, timeout, actual acquisition gaps, stale ADC, motor faults and controller scheduling gaps stop movement. Floor START additionally requires WHITE and a fresh centred front-sonar report above 250 mm; sonar accuracy remains unvalidated, so a clear floor is still required.
- Valid A/Z and decoder rejection both STOP. No decoder thresholds, timing tolerances, barcode geometry or navigation maps were relaxed.
- The host records a fresh post-run `reason=8` plus actual Buddy 2 STOP/zero-power snapshot. A pre-START stopped row cannot satisfy that check. It never retries START.

## Physical observations

1. v1 normal USB boot/STOP passed. Raised wheels, 15% PWM/200 ms: both encoder counters changed (-14/-11 to 25/31); operator confirmed forward rotation; final commanded STOP recorded. Distance/braking unmeasured.
2. v1 A-floor trial, 15%/maximum 3000 ms: commanded forward, then STOP after about 420 ms on `S3_BARCODE_NOISE` (status 3). Actual Buddy 2 STOP was logged by 241860 ms. GP28 raw 171..3144; 504 recordings, all 10 ms, no mask/drop discontinuity. Black runs in the full raw recording lasted 10 and 20 ms; production minimum remains 40 ms. PC replay independently rejected NOISE and decoded nothing. The old helper's original `MOTION_COMPLETE_NO_VALID_BARCODE` label was too broad; original evidence retained and this analysis corrects it.
3. v1 slower 8% trial was refused before START: latest-copy observation reported an apparent gap even though the sampler's raw queue was continuous. No motor activity recorded in that trial. A subsequent STOP-only inspection passed.
4. v2 fixes the latest-copy decoder defect. Its slower START was refused while a nonblocking status copy collided with publication: ADC0 at START, no motor activity, final STOP verified. Source v3 adds a controller-only bounded mutex handoff; it keeps the 30 ms age limit and all acquisition/timing errors. Hardware outcomes for the current version are in `evidence/2026-10-24/ir_motor_scan/CURRENT_HARDWARE_STATUS.json`.
5. v3 live A trial, 8%/3000 ms: START acknowledged, completed the full duration, fresh final STOP acknowledged, no decoder reject or valid decode. Encoders advanced by 370/258 counts; GP28 had repeated black/white transitions and remained BLACK at the stop. Operator then reported drift and confirmed the metal ball caster sticks/feels rough. The scan path is invalid/unverified; no further motor run was attempted. Stop powered scans until the caster rolls freely. See `HARDWARE/SCAN_A_V3_210006_ANALYSIS.json` and the retained raw recording.

The wide-bar replacement calibration remains GP28 WHITE197/BLACK2472; the earlier stationary 3 mm tests overlapped WHITE. The moving run demonstrates some black response, not reliable thin-bar resolution. PWM duty is not measured wheel speed. Do not infer a sensor fault or a physical decode pass from LEDs or sample availability.

## Validation and measurements

- Full Buddy 3 native suite PASS, including actual adapter/ADC-worker A/Z forward/reverse under sparse status reads, fault/cancel/ownership and bounded-handoff timeout/clock/age tests (25968 fixture assertions, zero failures).
- Native integration PASS: 339 bounded motor policy checks, actual USB framing/inbox/urgent STOP and all 10 existing INT-CR1 groups. Binary codec: 302 C/Python vectors and 74 bound rejections.
- Real kernel headers compile for SMP0/1. Incompatible motor/ADC/USB/watchdog combinations and accidental unlock of passive profiles are rejected.
- ARM v3 full rebuild and exact UF2/ELF flash comparison PASS. Old/wrong-family/mismatched-ELF artifact negative tests passed on v1; the verifier now additionally requires the v3 banner.
- Nine offline host-report tests PASS: post-run STOP freshness, early failure reasons, missing motor activity and matching decoder status. No USB opened by these tests.
- `git diff --check` PASS. Only explicitly reviewed source hashes refreshed; no unexpected board override.

| Measurement (bytes) | Locked capture baseline | Motor scan v1 | Motor scan v3 |
|---|---:|---:|---:|
| text | 331908 | 312820 | 313516 |
| data | 456 | 160 | 160 |
| bss | 25584 | 25260 | 25436 |
| linked static RAM | 26040 | 25420 | 25596 |
| UF2 file | 665088 | 626176 | 627712 |
| application binary telemetry | 50 | 50 | 50 |

These are profile comparisons: disabling MQTT also changes the link. They are not an isolated CR1 saving or peak runtime RAM measurement. Decoder state moved from the controller stack into sampler-owned storage; allocated task stacks are unchanged. Existing packed telemetry/event formats are unchanged (50-byte telemetry, 13-byte S3 events). Human USB diagnostics are deliberately verbose. CR2/3 were not implemented.

## Operator commands

From the repository, normal USB (no BOOTSEL):

```powershell
.\tools\robot_bench\RUN_IR_MOTOR_SCAN.ps1 -Mode Inspect
# Only with motor direction already observed, an aligned printed card on
# clear floor, no edges, cable slack and operator ready for a single run:
.\tools\robot_bench\RUN_IR_MOTOR_SCAN.ps1 -Mode Scan -ClearFloor -DirectionVerified -Power 8 -DurationMs 3000
```

Use the school 3/9 mm print, WHITE before/after and forward motion perpendicular across uninterrupted bars. Do not widen bars or lower thresholds to force a pass. A slower-duty test may stall; do not push it by hand while powered. Turn motor power off before repositioning or wiring. Firmware remains enabled for these deliberate commands until rebuilt/flashed to a locked profile.

Build-only: `tools/robot_bench/INSTALL_IR_MOTOR_SCAN.ps1` with SDK/Python paths; explicit `-Flash -DriveLetter D` only for the identified robot in BOOTSEL with motor power disconnected. No remote deployment occurs.

## Changed files in this motor work

- New `app_program/ir_motor_scan.[ch]`, `vehicle_ir_motor_scan.c`: bounded USB policy/controller integration.
- `subsystem3/subsystem3.[ch]`: profile-only sampler decoder and generation-tagged copied result; existing peripherals remain with Buddy 3.
- `robot_config.h`, `app_main.c`, `vehicle.c`, build tools/makefile: explicit profile/guards/startup.
- `subsystem1/control.h`, `telemetry.c`, `usb_control.c`, `task_telemetry.c`: existing USB owner/inbox/telemetry integration.
- `robot_ir_calibration.h`: provenance comment only in this motor work; numeric endpoints unchanged.
- Integration and sampler tests/makefiles; bench installer, runner, verifier, reporting tests/source manifest; this report and continuation notes.

Earlier subsystem fixes and user changes remain in the working tree; they were not introduced by this motor task. Snapshots and patches are in the workspace `outputs/IR_MOTOR_SCAN_20261024/BEFORE*`. Test/evidence copies accompany this report. Remaining: live scan validation, reliable thin-bar detection and complete A/Z passes; then broader robot integration/hardware validation. No full-project completion claim.
