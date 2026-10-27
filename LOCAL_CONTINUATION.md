# Historical handoff notes

The sections below describe earlier sessions, not the current task or authorization.
Current implementation: docs/BUDDY4_IMPLEMENTATION.md. CR1 compliance remains
in progress. Do not flash or start motors from these historical notes.

<!-- CURRENT CONTINUOUS TEST -->
# Current user request: one continuous 20 s at 30%, observe IR response

Read docs/CONTINUOUS_30_PERCENT_TEST.md. v6 separate profile: no early STOP
for BLACK/WHITE patterns or missing encoder progress. Explicit STOP, USB,
ADC/clock/motor faults remain. Fixed 30%/20s local helper; no layout questions.
v6 INSTALLED, USB STOP verified. SHA 2deddabc20183951ddb8036240d8d1b50f5fdfe903ef4bf13a240956f018bf69.
6,661 native + sanitizers/integration, 15 host and 9 profile checks pass.
User completed 30%/20 s run: COMPLETE at exactly 20,000 ms, final STOP ack.
Operator confirms successful turns and straight through cross. User-operated
response test PASS; unassisted full-course performance remains unvalidated.
See docs/evidence/2026-10-25/continuous_v6/TURN_CROSS_VALIDATION.json.
No code/firmware change needed. User can repeat RUN_30PCT_20S.cmd independently.
v5 built, never installed, on hold.
No motion queued. Older sections below are historical.
<!-- END CURRENT CONTINUOUS TEST -->

<!-- CURRENT LINE COURSE -->
# Current request: turns and cross, straight across; v5 candidate

Read docs/LINE_COURSE_TURNS_CROSS.md. User confirms both BLACK at a turn,
and cross route STRAIGHT. Installed v4/50%: 6131fa182021d34b65dc1c5fbdcf8bb2322f84d21908deacadababcb50058eb3.
v4 USB STOP and 50% raised forward pulse confirmed; floor trial stopped at
both BLACK after 900 ms while remaining on sheet. No floor tracking pass.
v5 adds bounded temporal course handling; tests/build pending, not flashed.
Keep motor power off during code work. No motion queued. All work local.
Older sections below are historical and superseded by this block.
<!-- END CURRENT LINE COURSE -->

<!-- CURRENT LINE ONLY50 -->
# Current request: 50% line-only v4 candidate; installed remains v3/20% cap

Read docs/LINE_ONLY_50_PERCENT.md. User explicitly requested 50% PWM.
Guarded cap/state width/host handshake updated; tests and build pending.
v4 power uses six bits, USB frame/telemetry unchanged, max run 20 s.
STOP/watchdogs retained, ultrasonic disabled. No flash or motion queued.
Installed v3 is 6b8fcd15cb9c9117259e439c24f5a5e9c5b021e4b29046edeba1c95384d038cc.
Older sections below are historical and superseded by this block.
<!-- END CURRENT LINE ONLY50 -->

<!-- CURRENT LINE ONLY20S -->
# Current request: 20-second line-only v3 INSTALLED; floor trials stopped early

Read docs/LINE_ONLY_20S_BENCH.md and its line_only_v3 evidence directory.
Installed UF2: 6b8fcd15cb9c9117259e439c24f5a5e9c5b021e4b29046edeba1c95384d038cc.
Normal USB v3 identity / fresh STOP verified. No obstacle protection; max 20 s,
PWM cap 20%, USB lease 250 ms, motor watchdog 150 ms, encoder progress 750 ms.
Both WHITE is assumed-centre forward; GP27 LEFT / GP26 RIGHT. No automatic
line-loss detection or stop at line end. Endpoints and driver are unchanged.
Native 70,432 + sanitizers, integration, Buddy 3 regression, host/profile guards
and full ARM build pass. Static RAM 25,508 bytes; telemetry 50 bytes unchanged.
At 8% floor trial stopped after 4.94 s; at 10% after 2.70 s, both NO_PROGRESS
during left correction, right encoder stopped. Final STOP acknowledged for both.
User could not tell whether wheel stopped physically. Floor tracking NOT validated.
V3 raised LEFT at 10% completed 6 s: M1 fixed 863, M2 840->1802 over 4 s;
operator confirms LEFT stopped / RIGHT forward. Final STOP acknowledged.
Same-power pivot succeeds raised, fails on floor. Inspect rough caster, drag,
and motor supply under load; exact cause unproven. No START queued.
V2 raised forward/LEFT/RIGHT were physically confirmed; 2,418 stationary samples pass.
All local on local/csc02-line-tracing-20261024; no pushes, PRs or remote edits.
Older sections below are historical and superseded by this block.
<!-- END CURRENT LINE ONLY20S -->

<!-- CURRENT LINE PRIORITY -->
# Current priority: WHITE-centred line tracing, v2 INSTALLED/STOP verified

User correction: GP27 LEFT / GP26 RIGHT BOTH see WHITE when centred on the
18 mm line, which lies between them. Read docs/LINE_TRACING_BENCH.md.
Both WHITE -> assumed-centre forward in a supervised timed bench; one BLACK
-> pivot toward that side; both BLACK -> STOP/LINE_AMBIGUOUS. WHITE/WHITE
cannot prove line presence or detect all off-track departures. Never claim
automatic line-loss detection. Version1's both-BLACK-centre logic is withdrawn.
V2 native355 checks, sanitizers, integration suite and forced ARM build pass.
V2 installed: bbb9f0...; normal USB v2 and fresh STOP verified. One200 ms
8% raised-wheel pulse completed; user confirms both wheels forward, STOP ack.
No floor run or physical steering pass. Await raised-wheel LEFT target test.
Steer mode is host-only; no new flash needed. It retains fresh sonar clearance;
last pulse showed front0, so a flat60 cm target is requested before line RUN.
Latest stationary RIGHT798 / LEFT808 / WHITE812 all pass:2418 samples,
zero gaps/mismatches, unchanged endpoints. Earlier failures remain retained.
See docs/evidence/2026-10-24/line_trace_v2/LINE_INPUTS_VALIDATION.json.
Both-black physical placement is not required; host STOP checks cover it.
Caster remains rough; user expects feedback correction. Verify raised-wheel
response before interpreting a short supervised floor run. No START queued.
Branch local/csc02-line-tracing-20261024; all work remains local.
Older sections below are historical and superseded by this block.
<!-- END CURRENT LINE PRIORITY -->

<!-- BEGIN CURRENT IR MOTOR SCAN -->
# Current state - v5 installed, STOP verified; live A incomplete

Source/installed v5: 34d36e25406708316a6c4a1a2798108792486087cc52972ff6e09992b54d9ab3.
Normal USB v5 READY/STOP verified. Motor power disconnected; no START after installation.
Saved A replay passes; first independent recording has2502 clean samples but INCOMPLETE:
one40ms bar, then200ms WHITE ends capture while that gap lasts250ms and more bars follow.
See docs/IR_DECODER_EDGE_BIAS_FIX.md. Review partial-frame ending before more repeats.
Onboard decoding and physical A/Z remain unvalidated. No movement queued.
Older sections below are historical and superseded by this block.
<!-- END CURRENT IR MOTOR SCAN -->
# Latest IR status â€” thin-bar detection unresolved

The replacement passes WHITE versus wide BLACK calibration, but3mm bars remain
unreadable in current tests: stationary197..242, short pass160..248 and paper
rotated90degrees189..229. All overlap earlier WHITE177..229; no BLACK entries
in3426 clean10ms samples. The wide-bar candidate197/2472 remains unchanged,
motor locked and NOT flashed. Full A/Z testing is on hold. Next is a flat,
closer-target height diagnostic if there is clearance, with no endpoint refit.
See docs/IR_GP28_REPLACEMENT_CALIBRATION.md and its thin_bar evidence/report.

# Current update â€” GP28 replacement calibration, 24 October 2026

The old physical blocker described below is historical. The replacement GP28
module now separates the actual print at mounted height: WHITE177..229,
BLACK2449..3090 over four clean stationary recordings. First-pair means stage
GP28=197/2472, checked on an independent repeat pair. GP26/27 remain unchanged
and physically unvalidated after possible line-module borrowing. Motor lock
stays enabled. See `docs/IR_GP28_REPLACEMENT_CALIBRATION.md` and its evidence
for provenance, tests/build/flash status and remaining A/Z validation.

The earlier continuation report follows as a dated baseline.

# Active robot checkout - 24 October 2026

Continue in `C:\Users\godga\Downloads\ES\inf2004_embedded_systems` on local
branch `local/csc02-cr1-continue-20261024`, based on `b02e335`. This replaces
the earlier IdeaProjects checkout as the active location. The outer ES folder's
15 October handoff is historical. No commits, pushes, PRs, remote merges or
network fetches were performed during this continuation.

## Scope and preserved work

Complete INF2004 CSC02 locally, implementing only Change Requirement1.
Keep micro T-Kernel3 and TEAM.md peripheral/core ownership, fail-safe STOP,
and the software motor lock. Requirements2/3 are out of scope. All existing
uncommitted edits carried into this checkout:85 of86 previously verified
files remain byte-identical; the sole changed file is a calibration-header
provenance comment. Its macro definitions and numeric endpoints are unchanged.
Pre-change status, patch and hashes are saved in tools/robot_bench/artifacts.
The original IdeaProjects checkout was not edited.

## Changes in this continuation

- `tools/robot_bench/`: portable capture, plane check, retry, strict production
  replay, host builder, installer/build helpers, source verification and tests.
  Paths resolve from this checkout; source/output overrides are forwarded.
  Generated files go to Git-ignored `artifacts/`. No old source folder is needed.
- The flasher now defaults to build-only. USB copying requires explicit
  `-Flash` and all existing source/UF2/profile/motor-lock checks. Installer
  wrappers still prompt for the operator-selected robot before explicit flash.
- GP28 instructions now use the actual card's WHITE area and widest printed
  BLACK bar at fixed geometry. Table/tape substitutions are explicitly excluded.
  The table-derived reference is labelled, and changed calibration headers or
  untracked board overrides cannot silently bypass interpretation checks.
- Historical printed-target evidence is retained in `docs/evidence/2026-10-22`.
  Runtime firmware behavior, thresholds, timing, navigation maps and endpoints
  are unchanged. The physical failure is not converted into a software pass.

## Tests and build performed here

All commands below were run against this checkout. WSL/MSYS needed execution
outside the app sandbox; the installed compilers then ran successfully.

```powershell
make -B -C testing/subsystem3 test
make -B -C testing/integration test
powershell -NoProfile -ExecutionPolicy Bypass -File tools/robot_bench/TEST_HELPERS.ps1
```

Buddy3 native tests all pass, including algorithms, adapter, ADC acquisition,
raw authority, layout, original labelled calibration, paused timing,388 capture
checks and449 sampling/STOP/lease/freshness checks. These calibration tests
exercise the original labelled observations; they do not validate printed ink.
All10 INT-CR1 groups pass, including302 independent C/Python vectors,74 bound
rejections, queue ownership/urgent STOP, sonar status, terrain/odometry policy,
and fake-transport MQTT callbacks.16 helper groups pass, including A/Z synthetic
forward/reverse, moved paths, hash/override guards, mock captures and fault
exit. Archived actual Z still returns NOT_DECODED for1806 continuous scans.

A fresh ARM build used `MAKEFLAGS=-B` with `tools/build_firmware.sh` and the
installed Pico SDK, profile `IRBarcodeCapture`. The SDK is a read-only dependency;
the separate LoRaWAN project and board were not modified. The portable helper
then passed in its default build-only mode:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/robot_bench/FLASH_ROBOT_LOCKED.ps1 -PicoSdkPath 'C:\path\to\pico-sdk' -FirmwareProfile IRBarcodeCapture
```

| Measurement | Prior verified capture | Fresh build here |
|---|---:|---:|
| text | 331884 | 331884 |
| data | 456 | 456 |
| bss | 25584 | 25584 |
| linked static RAM | 26040 | 26040 |
| UF2 bytes | 665088 | 665088 |
| binary telemetry bytes | 50 | 50 |

The optional capture kernel buffer remains160 bytes; kernel object overhead
and full runtime heap occupancy are unmeasured. ELF flash load comparison
matches332340 bytes. UF2 SHA256 is byte-identical to the verified22 October
image:346dc09e75f348f36719e24ff6f88015ae4901b50e14397bea07f483bba26d2e.
Build result `flashed=false`. No new boot/sensor/motion test is claimed.

Logs and a machine-readable record are in `docs/evidence/2026-10-24`.
Generated build files/results are in `tools/robot_bench/artifacts/`.

## Current physical blocker and next action

Priority remains Buddy3 IR. GP27 is LEFT line(A0, D0 absent), GP26 RIGHT line,
GP28 dedicated barcode excluded from line mask3. Leave the wiring/trim screws
unchanged. Software motor lock stays1; physical electrical braking is unmeasured.
Use the robot Pico W, not the separate LoRaWAN board. Normal USB is identified
by MTK3PICO/ContainerID, not by guessing any available COM port.

The old GP28 black endpoint3326 came from the table. The latest printed bar
was783..1034 versus white147..198; all802 labelled BLACK observations read as
white, and the black range251 exceeded the179.25 stability limit. All1604
stationary acquisitions were complete10ms scans; that is not a calibration pass.
Actual sensor/card gap and alignment are unknown. Moving records reaching
about3900 do not resolve the geometry/material discrepancy. Decoder limits
must not be relaxed simply to accept those failed recordings.

Next: obtain a side-on photo showing GP28's optical face and the actual card
at the intended reading position, including the gap and selected printed bar
(ruler beside the gap if available). Establish fixed alignment and optical spot
coverage, then repeat actual-card white/black measurements. Apply new endpoints
only from adequate, reproducible measurements and rerun the existing tests.
Do not request more identical sweeps or another flash before this inspection.
No physical printed A/Z decode, onboard decode or moving line performance passes.

Motor/encoder left-right mapping, counts/revolution/circumference and motion
calibration remain outstanding. GY511 has accelerometer/magnetometer, no gyro;
terrain/hump accuracy is unvalidated. Sonar uses restored GP1TRIG/GP0ECHO at the
existing3V3 connection; geometry/obstacle motion and live MQTT remain unvalidated.
Preserve safe STOP and explain native fixtures separately from hardware results.
