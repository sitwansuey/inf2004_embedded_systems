# Manual GP28 barcode continuation — 24 October 2026

Current mode: motor power physically disconnected, USB raw capture and production PC replay.
The operator selected manual movement after the guided motor trials. No new firmware was flashed.
Installed and active source remain IRMotorScan v3, SHA256 `f6b8cdc71ed544503f20e4810e63b1f61ad9d9ef75d12dc9a9cf68742f289076`.
Its compile-time motor lock is OFF, but STOP was verified before each manual recording; the
capture tool sends zero commands. Do not describe the firmware as motor locked.

## Movement time request and installed state

- One guided 6-second/8% run stopped at about460ms when the front-clear input became unavailable; STOP acknowledged. This does not establish an actual obstacle or sonar accuracy.
- A second guided 6-second/8% run completed its full duration and acknowledged STOP. No A decoded; operator confirmed GP28 was still crossing the barcode.
- A bounded 10-second v4 candidate was built and tested (1345 policy assertions and the native integration suite, plus ARM/ELF/UF2 verification). The operator then chose manual dragging instead of installing it. It is **NOT installed**. The six affected files were restored byte-for-byte to their saved v3 versions; the candidate and its source snapshot remain in workspace `BUILD_V4` / `UNFLASHED_10S_SOURCE` for provenance.
- Active powered scan maximum remains6000ms. For the manual tests, the 25-second recording window replaces the motor-time limit. Motor power stays disconnected.
- The manual scan cue window is now optionally10seconds (`-ScanWindowSeconds 10`); existing default6 is preserved. The recorded scan cue was5.16s and finish cue15.24s (10.08s apart). This only changes capture prompts; no serial movement commands or firmware changes are involved.

## Actual capture/replay results

| Recording | Valid scans | Gaps | Capture commands | PC result |
|---|---:|---:|---:|---|
| BARCODE_MANUAL_ROBOT_A_20261024_211354_236 | 2506 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_ROBOT_A_RETRY_20261024_211752_897 | 2502 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_CARD_A_20261024_211954_079 | 2506 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_CARD_A_10S_20261024_212443_973 | 2501 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_CARD_A_10S_REPEAT_20261024_213124_469 | 2504 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_DRAG_A_20261024_215119_251 | 2503 | 0 | 0 | NOT_DECODED |
| BARCODE_MANUAL_DRAG_A_REPEAT_20261024_215540_909 | 2510 | 0 | 0 | NOT_DECODED |

Detailed production-state traces (counts refer to the first latched capture attempt, not all transitions in the raw log):

- `BARCODE_MANUAL_ROBOT_A_20261024_211354_236`: TRACE status=2 t=852310 started=852010 captured_elements=1 required_elements=29 widths_ms=60
- `BARCODE_MANUAL_ROBOT_A_RETRY_20261024_211752_897`: TRACE status=3 t=1089690 started=1088320 captured_elements=1 required_elements=29 widths_ms=1340
- `BARCODE_MANUAL_CARD_A_20261024_211954_079`: TRACE status=3 t=1208330 started=1208300 captured_elements=0 required_elements=29 widths_ms=
- `BARCODE_MANUAL_CARD_A_10S_20261024_212443_973`: TRACE status=3 t=1499410 started=1497020 captured_elements=4 required_elements=29 widths_ms=1280,630,230,220
- `BARCODE_MANUAL_CARD_A_10S_REPEAT_20261024_213124_469`: TRACE status=3 t=1899250 started=1899000 captured_elements=2 required_elements=29 widths_ms=40,190
- `BARCODE_MANUAL_DRAG_A_20261024_215119_251`: TRACE status=2 t=3103320 started=3093710 captured_elements=13 required_elements=29 widths_ms=100,150,70,50,1650,200,70,50,170,70,250,110,70
- `BARCODE_MANUAL_DRAG_A_REPEAT_20261024_215540_909`: TRACE status=3 t=3355560 started=3355530 captured_elements=0 required_elements=29 widths_ms=

The first manual recording began and ended at BLACK-like readings. Its first attempt ended
INCOMPLETE after a60ms black element followed by a280ms white gap; later bars were retained
in the raw recording but the decoder correctly stayed latched after rejection. The robot-moving
retry passed a stopped-WHITE precheck and ended WHITE, then rejected NOISE: a1340ms black
element was followed by a30ms interval, below the unchanged40ms minimum. These results do
not prove a sensor fault; path, alignment, height and movement consistency still affect capture.
Earlier sheet-only trials kept the robot stationary, avoiding caster drag.
Its first6-second-window sheet pass still rejected an initial30ms black interval. The longer
first10-second-window sheet pass also rejected NOISE after4 captured elements (1280,630,230,220ms)
when the next interval was shorter than40ms. It contains2501 valid10ms samples, zero gaps/drops,
25 STOP controller reports and zero capture commands.
The operator requested another pass without changing the setup. That earlier repeat,
`BARCODE_MANUAL_CARD_A_10S_REPEAT_20261024_213124_469`, contains2504 valid scans, zero gaps/drops,
25 STOP reports and zero capture commands. Its first attempt captured40ms BLACK and190ms WHITE,
then rejected a20ms BLACK-like interval as NOISE (minimum40ms). The rejection occurred6.31seconds
into the sensor recording, after the5.24second scan cue. Later raw transitions remain preserved;
this trace describes the first latched attempt only. A has not decoded and Z remains untested
in this manual sequence. The recording alone does not establish whether the short interval
came from movement, alignment or optical contrast. The40ms minimum is an engineering default,
not an established school requirement; its relationship to minimum bar width, sample rate and
actual surface speed needs review. An offline cutoff sweep is retained separately in workspace
`LIMIT_REVIEW`; it changed no firmware or robot state.

The operator clarified that the21:37 commanded-motor recording also involved manual dragging.
That correction is recorded in `IR_POWERED_BARCODE_REPEAT.md`; it was not an isolated motor-only test.
The latest confirmed-unpowered manual drag, `BARCODE_MANUAL_DRAG_A_20261024_215119_251`,
contains2503 valid10ms samples with zero gaps/drops,25 STOP reports and zero capture commands.
Its first attempt returned INCOMPLETE after13 of29 required alternating intervals:
100,150,70,50,1650,200,70,50,170,70,250,110,70ms. Every captured interval exceeded40ms;
the minimum-run filter did not cause this rejection. One BLACK-like interval lasted1650ms,
and the subsequent quiet WHITE interval lasted6600ms before incomplete-frame rejection.
This identifies the observed waveform only; it does not establish a pause or missed/merged bars.
The record began WHITE-like(raw230) and ended BLACK-like(raw2559), rather than finishing WHITE.
Later transitions remain in the full log; the13 intervals refer only to the first latched attempt.
No A decoded in that recording.

Latest recording: `BARCODE_MANUAL_DRAG_A_REPEAT_20261024_215540_909`; 2510 valid samples, 0 gaps, 0 capture commands. GP28 began WHITE and ended WHITE. PC replay: NOT_DECODED. First latched attempt: NOISE, 0/29 captured intervals. PC-only cutoff experiments: min_ms=40 rows=2510 decoded=0 rejected=1 status=3 count=0; min_ms=30 rows=2510 decoded=0 rejected=1 status=6 count=29; min_ms=20 rows=2510 decoded=0 rejected=1 status=6 count=29; min_ms=10 rows=2510 decoded=0 rejected=1 status=6 count=29; min_ms=1 rows=2510 decoded=0 rejected=1 status=6 count=29.

The latest repeat's30ms first BLACK-like interval was rejected by the installed40ms setting.
PC-only experiments at30,20,10 and1ms reached29 captured intervals but rejected BAD_PATTERN;
lowering this one setting did not decode A from the same full recording. The record ended WHITE.
These experiments do not validate a changed firmware profile. No new movement is queued.

## Specific explanation from the last complete waveform

The21:55 recording contains exactly30 threshold edges, hence15 BLACK pulses and29 alternating
bar/space intervals over2570ms. For each of its three nine-interval characters, marking the
three largest widths as wide gives0x094,0x109,0x094: the expected framing/A/framing order.
This is an explicitly labelled offline ranking diagnostic, not a production decode or validated
alternative algorithm. It did not change any samples, firmware, capture timing or decoder settings.

The current classifier estimates one common narrow duration from BLACK and WHITE intervals
within each character, then requires each interval to be within25% of its nominal1x or3x width.
The observed narrow BLACK pulses are30..40ms, while narrow WHITE intervals are60..110ms.
For example the first character has a50ms estimated unit;30ms BLACK and70ms WHITE both fail
its37.5..62.5ms narrow band even though its three longest elements identify the expected frame.
The final frame has an estimated70ms unit and an even greater BLACK/WHITE asymmetry.

This establishes a concrete mismatch between recorded edge timings and the decoder's model;
it does not establish the physical cause. Threshold crossing/optical footprint and motion variation
can both affect the measured widths. It also shows why another identical hand pass is a weak
next step: the last record already contains the expected rank order, but the production width
checks reject it. A future classifier/sampling change needs independent A/Z, reverse, corrupted,
ambiguous and truncated-frame checks; ranking alone is not a sufficient acceptance rule.
The production result remains NOT_DECODED. No navigation or physical A/Z pass is claimed.

PC results validate only the recorded sensor data against the production decoder. They do not
validate onboard navigation, autonomous straight tracking, wheel calibration or electrical braking.
The mask7 value means three ADC channels were acquired, not that all physical line modules are present.

## Unchanged limits and validation

No threshold, calibration, barcode width ratio, navigation map or decoder timing was relaxed.
GP28 uses WHITE197/BLACK2472; hysteresis400/600; minimum run40ms; timeout10000ms.
Native replay sources and executable hashes were checked by the portable replay helper.
The active native integration suite passes (339 motor policy checks and all existing INT-CR1 groups).
A forced full ARM rebuild inside WSL matches the installed v3 UF2 SHA256 exactly: **True**.
The initial incremental check rejected a cached v4 banner before any flash; Windows MAKEFLAGS
did not reach WSL. Passing MAKEFLAGS=-B inside WSL produced the matching full rebuild.
Current firmware static RAM25596 bytes; telemetry50 bytes, unchanged by manual capture.
The uninstalled10-second candidate has the same RAM/telemetry and8 additional text bytes.
`tools/robot_bench/TEST_ROBOT_USB.ps1` now accepts an optional scan cue window, keeps the
existing6-second default, and allows either card movement or unpowered robot movement in its
instructions. It no longer incorrectly claims the installed firmware is motor locked. The script
parses and the new10-second cue timing was exercised on the robot with no movement commands.
Existing user changes remain preserved on `local/csc02-ir-motor-scan-20261024`. Cached remote refs match; no push,
PR, remote merge or fetch. Source guards and `git diff --check` pass.

## Repeat procedure

Keep motor power disconnected. Begin on real blank WHITE before the first bar, preserving the
mounted sensing height. After a stopped-WHITE check, wait for the scan beep and make one
continuous pass taking about5seconds from the FIRST barcode bar to the LAST, through uninterrupted
bars and away from printed text. Use `-ScanWindowSeconds 10` for extra lead-in/out time. End on WHITE
and keep it there until recording ends. Do not reverse back through the barcode. Keep the robot
fixed and slide the card flat if the caster makes robot movement uneven. Use A first, then Z
once A can be captured consistently. Evidence is in `evidence/2026-10-24/ir_manual_pass/`.
No new physical action is requested after the latest repeat. No physical A/Z pass is claimed.
