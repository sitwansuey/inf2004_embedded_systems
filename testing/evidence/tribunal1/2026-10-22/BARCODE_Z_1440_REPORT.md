# USB recovery and Z record, 14:39/14:40 +08:00, 22 October 2026

The robot USB connection recovered on COM5, matched by MTK3PICO serial
identity, USB ContainerID and live port. A new18s recording has1806 complete
distinct valid scans; every interval10ms; mask7/drop0 throughout, no malformed
record or capture error.18 controller records remain STOP/disarmed/power0/
fault0, no commands sent. This does not measure electrical motor shutdown.

Z remains NOT DECODED. Production feed returns BAD_PATTERN after29 runs;
the GP28 signal spans174..3913 and has30 threshold edges. The sweep spans
4770ms, within the requested4-6s and the existing10s timeout. Width variation
and structural mismatches remain. Continuous acquisition and a complete
sweep alone do not establish a correct symbol. The record does not prove
whether optical geometry or varying motion is the sole cause.

No firmware/repository source or decoder limit changed this turn. Last
verified flashed image346DC0 remains byte-identical: staticRAM26040 bytes,
telemetry50 bytes. Source/test/doc manifest85 files still match before this
operator-helper-only update. No ARM rebuild or reflash is needed.

## Next discriminator: stationary optical plane check

Use TEST_BARCODE_PLANE.ps1 with normal USB and motor power off/software lock
on. Keep the robot/GP28 mount fixed. Put a firm backing under the target at
the intended barcode card height and angle. Hold uniform WHITE at GP28,
then a broad matt BLACK patch at the same plane,8s per capture plus2s USB
warmup. Leave line targets, wires and sensor adjustment screws unchanged.
This is not a barcode sweep. No scan/finish beeps occur in stationary mode.

The helper preserves all unique raw/warmup/result logs, checks complete
fresh scans and zero controller drive/fault status, compares every labelled
GP28 value with current213/3326 endpoints and400/600 thresholds, and checks
each target's full observed range against one quarter of median contrast.
It reports stationary reflectance only; it never applies endpoints, changes
decoder limits or claims barcode/onboard/motion validation. Poor separation,
unstable targets or current-threshold mismatch require review before scanning.

TEST_ROBOT_USB.ps1 gained optional BarcodeHold WHITE/BLACK for this test,
with scan cues suppressed, and a specific no-serial-ports message. Existing
normal barcode timing and robot-only identification stay in place. Native
PowerShell5 helper tests cover stable/mismatched/weak/unstable/fault fixtures
and cue selection; existing capture parser and image guard checks pass.
All new helper fixtures are synthetic/mock and do not imply physical success.

Evidence:BARCODE_Z_1440/ preserves original recording/replay/CSV/trace,
capture/result records and pre-update metadata. The trace harness uses the
unchanged production normalization/feed/decode code and is host-only.
IR_BARCODE_PLANE_HELPER_TESTS.log records helper tests. No pushes, PRs,
remote updates or CR2/CR3; physical stationary plane test is pending.
