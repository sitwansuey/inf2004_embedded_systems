# IR sampling correction: verified on the robot

## Current verification

The user built/flashed 346DC0 at 07:21 +08:00 on 22 October 2026.
The A/Z repeat has 5021 complete valid scans, all at 10ms intervals, with
zero drop/gap/error records. Both now reach BAD_PATTERN rather than failing
continuity/timeout. Physical symbol decoding remains unvalidated.
Measured text331884/data456/bss25584/staticRAM26040/UF2665088/telemetry50;
832 text bytes and zero static RAM bytes added versus the previous capture.
Use the existing firmware with TEST_BARCODE_RETRY.ps1; no flash needed.
See [physical evidence and next scan](IR_BARCODE_PHYSICAL_0723.md).

## Archived correction plan

The sections below record the pre-flash findings and plan from 07:18. Their
pending-build/retest statements describe that earlier point in time and are
superseded by the verification above.

## Actual robot evidence, 07:03/07:04 +08:00, 22 October 2026

The user built/flashed IRBarcodeCapture78ED21... . A has1115 complete distinct
ADC copies; Z has1132. All2247 records have mask7 and drop counter0, with
no malformed packet or capture API error.50 controller rows remain DISARMED,
STOP,power0,fault0. No robot command was sent. This is successful recording,
not successful barcode decoding. Both strict replays report NOT_PASSED.

| Card | 20ms intervals | 40ms intervals | GP28 range | Threshold edges | First-to-last edge |
|---|---:|---:|---|---:|---:|
| A | 979 | 135 | 193..3940 | 30 | 14060ms |
| Z | 1007 | 124 | 196..3930 | 30 | 16400ms |

GP28 clearly responds to black/white transitions, but this does not establish
the symbols, width ratios, accuracy or speed. Forty-ms gaps exceed the retained
30ms continuity limit. Both sweeps also exceed the unchanged10s decode timeout.
They begin before the recorded host SCAN NOW cue; the revised test explicitly
waits for a start beep and sets a finish beep six seconds later. GP26/27 both
remain dark in every recorded scan, blocking the previous full-controller host
replay. Barcode optics testing should not depend on artificial line patches.

## Software changes and safety

S3 sampling now uses a10ms micro T-Kernel cyclic handler when IR is enabled.
The interrupt handler only wakes the existing S3 task on processor1/core0;
it never reads ADC, takes a mutex or prints. The task retains the ADC worker
and all peripheral ownership. It drains old wake credits before waiting for
a new release, preventing catch-up bursts and duplicate timestamps. Disabled
IR retains the previous delayed idle path. No kernel or priority is changed.

Previously tk_dly_tsk(10) added the10ms kernel guard after each iteration,
giving20ms idle time plus execution/dispatch delays. Nonblocking state-lock
collisions could also skip an iteration or latch TRANSPORT at publication.
The cross-core capture consumer unnecessarily held that same mutex. The logs
cannot attribute every40ms gap to one cause; the correction addresses both
relative-delay drift and the avoidable contention. Real timing still requires
the next robot retest; a10ms requested cyclic period is not a hard timing proof.

Only sampler state acquisition now requests a bounded5ms mutex wait; the
kernel tick and scheduling can extend that bound. Public status/control reads
stay nonblocking. Monotonic time is refreshed after waits; STOP/generation,
lease expiry and30ms sample age are checked before publishing steering or
navigation. Late/failed publication cannot enqueue an old navigation event.
Clock failures invalidate output. Existing active tracking stale faults still
require explicit controller rearm. Motor lock remains1 at all existing layers.

Capture receive now uses only the kernel copied message buffer, with no
status mutex or shared counter read. Each producer record carries its own
drop snapshot. Records are16 internal bytes (existing12-byte ADC sample+4
drop count); capacity8 needs160 kernel buffer bytes, previously128. Kernel
object overhead/actual heap occupancy remain unmeasured. One cyclic handler
ID adds4 source-level static bytes; linked RAM must be measured after build.
The optional capture queue is absent in normal builds. Binary telemetry is
still50 bytes and CR2/CR3 are not introduced.

## Native verification

S3-U48/U49/U50:428 checks SMP0 and449 SMP1 pass for cyclic cadence, draining
old releases, a callback with no ADC/mutex work, failed cyclic/task creation
cleanup, and publication wait rechecks of STOP, lease, age and clock failure.
The archived pre-fix consumer fails the same cadence regression. GCC analyzer
emits no diagnostic. Updated capture tests pass382 SMP0/388 SMP1 checks,
including consuming copied records while the state mutex is unavailable.
Original algorithms/layout/adapter/ADC/reply/STOP/raw-authority regressions
and real kernel headers SMP0/1 x IR0/1/2 pass. Capture-profile real headers
and unsafe-build rejection pass.

The host replay tool now isolates GP28 using production s3_normalize,
400/600 hysteresis and s3_barcode_feed/decode. It retains actual timestamps,
30ms continuity,40ms minimum run and10000ms timeout. No line sensor input
is changed or fabricated. Gaps/invalid masks/duplicate time/drop changes
start a new capture; partial markers never cross discontinuities. Native
fixtures decode project A/Z in both directions even with both line sensors
dark, reject partial markers across gaps/invalid/drop, reject oversized time,
and retain the10s slow-sweep timeout. Fixtures are not physical results.

Existing real recordings still fail this barcode-only replay:136/125 separate
contiguous segments, no complete decoded symbol. No sample is interpolated,
rescaled or stitched to turn this into a pass. The robot's real controller
retains its original line/steering gates; host optics isolation does not alter
onboard integration or establish onboard character decoding.

## Installed and pending image measurements

| Image | text | data | bss | static RAM | UF2 | binary telemetry |
|---|---:|---:|---:|---:|---:|---:|
| Verified stationary3E84E2... | 330516 | 456 | 25576 | 26032 | 662016 | 50 |
| Installed capture78ED21... | 331052 | 456 | 25584 | 26040 | 663040 | 50 |
| Pending cyclic/mutex correction | Not built | Not built | Not built | Not measured | Not built | Contract50 |

Capture78ED21 adds536 text/8 static RAM versus stationary3E84E2; its kernel
capture buffer uses128 bytes, separate from static RAM accounting. ELF flash
PT_LOAD comparison matches331508 UF2 payload bytes and all block/family/hash
checks pass. Installed SHA256:
78ed21ebce546ef5285373bc333600e7af05134ce5664fc9e56341a0e33b0dcb.
All81 source hashes matched before edits. The correction is NOT ARM built or
flashed; this chat still lacks direct WSL/USB access. No new hardware action
was performed by the native tests. Electrical motor levels/braking unmeasured.

## Retest

Run task-output INSTALL_BARCODE_CAPTURE.ps1 in normal PowerShell. It builds
first, then prompts for the robot BOOTSEL drive and normal USB. The flasher
now requires the cyclic10/mutex-free receive marker and rejects the old image.
Only the robot is used; separate LoRaWAN COM4 remains untouched.

Keep motor power off, chassis still, existing wires/settings/height unchanged.
No line patches required. Present WHITE to GP28 and WAIT for SCAN NOW/start
beep. Move the WHOLE A card steadily through GP28 in about4-6 seconds; finish
on white before the second beep. Hold white for the remainder of capture.
Repeat for Z. Leading/trailing white margins and fixed sensor/card distance
matter. Do not slow a single sweep to14-16 seconds or alter thresholds to
force acceptance. New recordings must be replayed before any decoding claim.

Preserved evidence: BARCODE_CARDS_0704/ original logs/results/UF2/ELF/map/
manifest; BARCODE_CARDS_0704_ANALYSIS.json; BARCODE_CARDS_0704_BARCODE_ONLY_REPLAY.json;
IR_SAMPLE_SCHEDULE_NATIVE_TESTS.log; updated native/helper logs. Earlier161/161
stationary IR classification remains verified on3E84E2. Moving tracking,
physical decoded A/Z, onboard decoding, encoders and motion remain outstanding.
All edits stay uncommitted locally; no push, PR or remote modification.
