# CSC02 Tribunal 1 supplementary bench evidence

Physical captures and operator inspection records: **22 October 2026, Singapore time**. Publication and log-count/hash review: **23 October 2026**. These are supplementary results after the 19 October discussion, not measurements available during that call.

This directory contains logs, recorded results, analysis and test documentation only. Firmware/source code, test programs, executable helpers, build scripts and firmware binaries are excluded. The sources used for these local bench images remain unpublished. These results therefore do not attest that the current `main` firmware reproduces the same behaviour.

## Compact evidence key for the report

| ID | Evidence supported | Supporting files | Limits |
|---|---|---|---|
| H1 | Operator-confirmed GP26 right line, GP27 left line, GP28 dedicated barcode; GP27 A0 connected, D0 disconnected | [IR role record](IR_CONFIRMED_WIRING.json), [wiring/design context](REPORT_WIRING_AUDIT.md) | Not a full cable-continuity or electrical-level survey. The original assembly photo remains in the tribunal document. |
| H2 | 30 successful fresh ultrasonic status copies; 26 valid distances (372-841 mm); same capture has 143 usable IMU rows and zero reported I/O errors | [Raw output](S5_STATUS_RETEST.log), [capture result](S5_STATUS_RETEST_RESULT.json), [analysis](S5_STATUS_RETEST_ANALYSIS.json), [report](S5_STATUS_RETEST_REPORT.md) | Basic acquisition only. Four samples were invalid during reported servo transitions. No calibrated range, terrain-height or stopping result. |
| H3 | 161 fresh unique stationary IR scans on the original labelled targets, with zero recorded classification mismatches | [Analysis](IR_CAL_0646_ANALYSIS.json), [operator steps](IR_CALIBRATED_STEPS.json), [WHITE](IR_CAL_WHITE.log), [RIGHT](IR_CAL_RIGHT.log), [LEFT](IR_CAL_LEFT.log), [BARCODE](IR_CAL_BARCODE.log) | Stationary target interpretation only. Moving line following and physical barcode-symbol decoding remain unvalidated. |
| H4 | Later actual printed-card check: 802 white readings (147-198) and 802 printed-black readings (783-1034); existing calibration misclassifies printed black | [Printed-target analysis](BARCODE_PLANE_1450_ANALYSIS.json), [WHITE raw](BARCODE_PLANE_WHITE_20261022_144931_760.log), [BLACK raw](BARCODE_PLANE_BLACK_20261022_144951_588.log), [Z PC-replay result](BARCODE_TRY_Z_20261022_143940_356_REPLAY_RESULT.json) | This failure is retained. Earlier table-derived black reference is not valid proof of printed-ink calibration. PC replay is not on-board decoding. |

The raw record counts and selected embedded capture hashes were rechecked without sending robot commands. See [rechecked results](RECHECKED_RESULTS.json) and [file hashes](SHA256SUMS.json). No new hardware test was run for this publication.

## Chronology and firmware identity

- The earlier reversed-pin test is retained in `REPORT_WIRING_TEST_20261022040523*`. It did not obtain valid ultrasonic ranges.
- The 04:33 pin-correction retest restored basic ranging with TRIG=GP1 / ECHO=GP0; unavailable status copies still occurred. See `ULTRASONIC_PIN_RETEST*`.
- The 04:46 status-copy retest produced the H2 counts. Recorded image SHA-256: `c0a4a67a31cf1227d7a35a3cc68fa9450f74f03748d55f2f82a1762f863de49d`.
- The 06:45-06:46 stationary IR test produced H3. Recorded image SHA-256: `3e84e2189a26e3c433126b3eea0bdf9733896cb171207496e375a609518346d3`.
- Later continuous barcode captures and printed-target results remain unsuccessful for physical symbols. The Z result is a host replay of physical raw data, explicitly labelled in its result file.

Different firmware images and sessions must not be presented as one simultaneous all-subsystem test. Original results/reports may contain their contemporary pending statuses; use the dated raw record and the later qualified summary together.

## What stays open

Measured supply/signal levels and full continuity; motor/servo power stability; physical encoder wheel mapping/polarity and distance calibration; electrical motor shutdown/braking; calibrated range/height; moving tracking; physical barcode decoding; full avoidance and live MQTT acceptance. Controller records showing STOP/disarmed/power 0 are software status, not an electrical stopping measurement.

## Using this in the tribunal document

Keep short results and H1-H4 references in the DOCX. Link this index or the individual evidence files; do not paste all raw logs into the report. Label these as 22 October post-discussion supplementary tests. Keep the original recorded dates and firmware identities. No team approval or PASS beyond the stated scope is implied.
