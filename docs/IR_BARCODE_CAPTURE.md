# Barcode capture and next test

Latest14:50: the actual printed-card stationary check fails. The user
clarifies the original BLACK reference came from the table. GP28's held
printed bar reads783..1034 versus reference3326 and is classified white in
all802 observations; its range is unstable relative to measured contrast.
Inspect the sensor/card gap and alignment before another sweep or endpoint
update. See [printed-target calibration mismatch](IR_PRINTED_BAR_CALIBRATION.md).
The retry procedure below is retained for later use after that inspection.

Sampling fix built/flashed: 346DC0, 07:21 +08:00, 22 October 2026.
A2512/Z2509 complete valid raw scans: all intervals10ms, zero drops/gaps/errors.
Both reach BAD_PATTERN under production width checks; neither is decoded.
No tolerances, timing limits, line gates or navigation maps were changed.
Fifty controller records remain stopped/disarmed/power0/fault0, no commands.

See [physical results, measurements and next step](IR_BARCODE_PHYSICAL_0723.md)
and [sampling implementation/native checks](IR_SCAN_SCHEDULE_FIX.md).
Prior stationary calibrated IR classification remains161/161 on3E84E2.

Use current firmware, normal USB and TEST_BARCODE_RETRY.ps1 -Symbol Z in
task outputs. No new flash/BOOTSEL. Keep motor power off and software lock on.
The user already uses a flat surface. Use a straight guide, one steady full
4-6s sweep across uninterrupted bars away from the printed letter. Wait for
SCAN NOW, finish on WHITE, and do not return the card across the sensor.
Keep the same calibrated sensor distance and adjustment settings.

The helper performs production decoding on the PC immediately after each
recording, preserves logs, permits at most3 prompted attempts, and stops early
on success or controller/capture error. Hash guards detect a stale host decoder.
Positive native/mock fixtures are synthetic; actual A/Z still NOT DECODED.
Onboard decoding remains PAUSED; motion/accuracy are unvalidated.
