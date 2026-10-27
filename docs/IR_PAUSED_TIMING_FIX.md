# Paused IR timing fault: 06:34 robot capture and fix

The first calibrated image was successfully built and copied to the robot at
06:33 +08:00, 22 October 2026. SHA256:
0184f5160a6c673d22dfa3d44ef167c36031fb3efc9a1944cff954a3388cf6c8.
Its USB warmup contains MOTOR LOCK ON and calibrated endpoint/role banners.
The eight-second WHITE test has41 distinct complete fresh IR acquisitions:
copied1/error0/acquisition0/mask7/configured1/fresh1/steering0. Every row
nevertheless reports algorithm_error3 (S3_ERROR_STALE). Eight controller rows
are stopped/disarmed, zero faults, zero commands and zero requested power.
The test correctly stopped instead of accepting the latched algorithm fault.

Warmup samples at180/380ms have algorithm_error0; it is3 by580ms. The exact
internal failed sample interval is not recorded by the 200ms console output,
so its duration/cause is not proven. Source inspection and native reproduction
show that a valid40ms forward interval while PAUSED latches the same fault
permanently, despite subsequent fresh20ms acquisitions. The expected kernel
guard-tick/scheduling jitter can cause such intervals; this is an inference,
not a measured worst-case execution time. Do not raise the30ms TRACK limit
or silently discard an error to pass a test.

## Code correction

s3_core_step now handles a forward interval exceeding max_sample_gap_ms
during PAUSED/OVERRIDE as a new observation: reset line temporal/hysteresis
history, abort partial barcode capture, and classify the new valid sample.
It grants no steering, no barcode run decoding and no navigation event.
It does NOT clear existing latched faults. TRACK still faults on the same
long interval and stays inhibited on subsequent fresh samples until explicit
controller rearm. Duplicate/backwards timestamps remain faults in all modes;
rollover with a small positive interval remains valid. Invalid sensor masks,
clock/transport failures, lease expiry and STOP behavior are unchanged.

Calibrated console now includes production right/left line_darkness values.
The test rejects ambiguous400..600 reflectance rather than accepting a zero
blackmask caused by hysteresis. This matters because GP26 changed from198
to1721 raw counts during WHITE (most samples near1700, normalized about500),
whereas its saved white endpoint is187. GP27 ranged178..249 and GP28 barcode
323..361. Target edge, motion, distance or lighting may explain GP26, but the
capture does not prove the cause. Leave calibration/wires/settings unchanged;
fully cover the sensing window with flat WHITE at the original working height
and keep the backing/robot still for the entire capture. A mixed/ambiguous
surface must fail separately after the timing defect is fixed.

## Verification

S3-U44:99 native production-core checks pass for paused forward20/30/40/1000ms
intervals, rollover, correct classification without authority, unchanged TRACK
fault/rearm, fault persistence, duplicate/backwards time and sensor failure.
The same test fails against the archived pre-fix production core. GCC analyzer
emits no diagnostic. S3-U45 runs the production sampler/copied ADC worker with
configured PAUSED jitter and rollover. ADC acquisition suite:6 groups/2061
checks. Original algorithm551, layout14, adapter20093, ADC1080, tick328, raw
authority/unlocked-rejection tests and real headers SMP0/1 x IR0/1/2 pass.
S3-U43 measured60/default2 and every362 calibration observation still pass.
Calibrated real headers SMP0/1 pass. Helper fixtures reject obsolete image
markers, stale/failed/truncated/duplicate/ambiguous observations and steering.
The real failed capture replays as41 acquisitions and41 STALE errors, never a
false interpretation pass. These native/helper checks are not robot retests.

The flasher now requires the paused-gap correction banner and line_darkness
format, preventing a rebuild/copy of the previous calibrated image from being
reported as this fix. All source hashes are refreshed after passing checks.
INSTALL_IR_CALIBRATED.ps1 still builds/verifies first, then prompts for the
robot BOOTSEL drive, then normal USB and four stationary target captures.
No helper sends ARM or movement commands. Drive lock remains1.

## Actual image measurements

ELF32 ARM allocated sections were parsed without a substitute compiler; every
flash PT_LOAD byte was compared with the UF2 payload (330836 matching bytes).
The UF2 family, block ordering, addresses/magic and image hash match the saved
build/flash result. Source hashes matched the72 tested files before edits.

| Image | text | data | bss | static RAM | UF2 bytes | telemetry bytes |
|---|---:|---:|---:|---:|---:|---:|
| Previous raw ADB655... | 330316 | 456 | 25576 | 26032 | 662016 | 50 |
| Installed calibrated0184F5... | 330380 | 456 | 25576 | 26032 | 662016 | 50 |
| Corrected3E84E2..., now installed | 330516 | 456 | 25576 | 26032 | 662016 | 50 |

The first calibrated build adds64 text bytes and0 static RAM versus raw.
Do not assign these sizes to the pending correction. This chat still lacks
ARM/USB access; operator build/flash and physical retest are outstanding.
No new firmware is copied by the code/native checks in this turn.

Preserved evidence: task-output IR_CAL_0634/ (original logs/warmup/results,
source manifest, UF2, ELF and map), IR_CAL_0634_ANALYSIS.json,
IR_PAUSED_TIMING_NATIVE_TESTS.log and IR_CALIBRATED_HELPER_CHECKS.log.
Measured endpoints remain white{187,189,213}/black{3214,3177,3326}; roles
GP26 right/GP27 left/GP28 barcode. Motor motion, physical line following and
barcode pattern decoding remain unvalidated. No push, PR or remote changes.

## Physical retest update, 06:45/06:46

The correction is now built/flashed and physically verified. WHITE41,
RIGHT40, LEFT40 and BARCODE40 all pass,161/161 fresh complete readings,
zero algorithm faults or mismatches, no steering/commands.32 controller rows
remain stopped/disarmed/power0/fault0. No active-tracking safety check changed.
See IR_BARCODE_CAPTURE.md for current measured image sizes and the next
separate optional capture profile. Earlier pending statements describe the
pre-retest state, not a present blocker for the paused correction.
