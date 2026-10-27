# Confirmed IR layout and validation limits

User confirms GP26 RIGHT line, GP27 LEFT line, GP28 dedicated BARCODE.
Line mask3, barcode index2, logical positions{1000,-1000,0}; A/Z maps NONE.
GP27 A0 is connected and D0 absent; leave it disconnected. Full sensor
model/cable continuity remains unverified; carrier Grove5/6 share GP26.

Current integer endpoints in GP26/27/28 order: white{187,189,213},
black{3214,3177,3326}. GP26/27 are05:54 floor captures61/60 samples;
GP28 is06:12 labelledWHITE/06:20 labelledBLACK60/60. At14:50 the user
clarified the original BLACK target was the table. GP28's3326 is therefore
a table-derived reference, not a verified printed-ink endpoint. Sensor/card
gap and comparability of later card geometry are unmeasured. All362 original
labelled observations pass native normalization/calibration replay; they
were independent sessions, not fabricated simultaneous scans.

The robot06:45/06:46 stationary test PASSED161/161 fresh unique scans on its
original labelled targets: WHITE41, RIGHT40(+1000), LEFT40(-1000), BARCODE40.
No mismatches/errors/commands;32 controller rows stopped/disarmed/power0/
fault0. That result verifies interpretation for those surfaces and that setup;
it is not a physical printed-bar or barcode-symbol validation.

Sampling correction346DC0 was built/flashed07:21. Repeated A/Z and Z-only
captures have continuous10ms sampling without drops, but no decoded symbol.
The14:49/14:50 stationary printed-card check now fails: WHITE147..198,
printedBLACK783..1034. All802 BLACK samples normalize183..263 and are read
as white under current thresholds. BLACK range251 exceeds the179.25
stability limit for median contrast717. No endpoints were applied.

See [printed-target mismatch and inspection](IR_PRINTED_BAR_CALIBRATION.md),
[barcode capture](IR_BARCODE_CAPTURE.md), and
[sampling implementation/native tests](IR_SCAN_SCHEDULE_FIX.md).
Next step is inspection of GP28/card alignment and actual gap, followed by
reproducible actual-card calibration. Keep motor power off/software lock on.
Current capture image: text331884/data456/bss25584/staticRAM26040,
UF2665088/telemetry50 bytes. No firmware change is needed for inspection.

Moving line control, physical symbols/onboard decoding, speed and accuracy
remain unvalidated. Neither line sensor dark means SEARCHING/LOST; both
dark is a junction indication whose physical geometry still needs checking.
Actual sensor spacing is unmeasured. There are two line sensors plus a
dedicated barcode sensor; no three-line-sensor performance is claimed.
