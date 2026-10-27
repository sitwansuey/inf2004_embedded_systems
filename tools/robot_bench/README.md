<!-- CURRENT CONTINUOUS TEST -->
Installed v6: fixed 30% / 20-second IR response test. Double-click
RUN_30PCT_20S.cmd to start one run after positioning/enabling motor power.
No BLACK/WHITE-pattern or encoder-progress early stop. Emergency STOP/USB
and actual hardware faults remain. Read ../../docs/CONTINUOUS_30_PERCENT_TEST.md.
Boot STOPPED; no automatic run/repetition. Older sections are historical.
<!-- END CURRENT CONTINUOUS TEST -->

<!-- CURRENT COURSE TOOLS -->
Current source: v5 turn/cross candidate; installed remains v4/50%.
Read [course behaviour and limits](../../docs/LINE_COURSE_TURNS_CROSS.md).
RUN_LINE_ONLY defaults to Inspect, supports 50% and requires v5/course identity
for Trace. Existing STOP/watchdogs remain. No automatic flashing or motion.
Older sections below are historical.
<!-- END CURRENT COURSE TOOLS -->

<!-- CURRENT LINE TOOLS -->
Current priority is line tracing. [Line bench behaviour, evidence and next
physical steps](../../docs/LINE_TRACING_BENCH.md). IRLineTrace v2 is installed, runtime STOP verified and forward wheel direction
confirmed by the operator after a200 ms8% raised-wheel pulse. CHECK_LINE_INPUTS.ps1 is
passive PC replay. RUN_LINE_TRACE.ps1 defaults to STOP/query Inspect and
rejects the barcode firmware identity; no automatic motion or flashing.
Earlier barcode sections below are historical.
<!-- END CURRENT LINE TOOLS -->

<!-- CURRENT DECODER V5 -->
V5 is installed and STOP verified; first independent live A was incomplete.
[Decoder change, tests and artifact](../../docs/IR_DECODER_EDGE_BIAS_FIX.md).
Updated motor helper requires v5. Raw USB capture sends zero movement commands.
The old v4 ten-second candidate is unrelated; v5 keeps the six-second cap.
Earlier manual-scan notes below are historical.

<!-- MANUAL SCAN WINDOW UPDATE -->
Current manual capture: motor power disconnected, stationary robot, slide the printed sheet.
Use `TEST_ROBOT_USB.ps1 -Procedure BarcodeCapture -Seconds 25 -ScanWindowSeconds 10`
for the longer cue window. [Latest captures and status](../../docs/IR_MANUAL_ROBOT_PASS.md).
Installed firmware remains v3; the10-second MOTOR candidate was not installed.

<!-- IR MOTOR SCAN TOOLS -->
# Motor scan profile â€” current update

The installed IRMotorScan profile enables motors and boots STOPPED. Its
`RUN_IR_MOTOR_SCAN.ps1` sends explicit STOP/query/PREPARE/START/heartbeat commands;
this differs from the passive capture tools described below. Inspect mode
sends only STOP/query. See [the current report](../../docs/IR_MOTOR_SCAN_BENCH.md)
for installation, bounds, checks and hardware evidence. Powered tests are
currently on hold: the operator reports a sticking ball caster and drift.
Repair/free the caster before retrying. No physical A/Z pass is claimed.

The earlier passive-tool instructions follow.

# Local robot bench tools

These scripts resolve the checkout from their location. They do not depend on
the previous IdeaProjects folder or the old chat-output directory. Generated
executables, captures and results go into `artifacts/`, which is Git ignored.
Optional `-RepoRoot` and `-OutputDirectory` select an explicit source/output
location and are forwarded consistently by the wrappers.

Keep motor power off and the compile-time drive lock on. Use the robot Pico W,
not the separate LoRaWAN board. The passive capture/replay scripts described below send no serial commands.

From the firmware repository in PowerShell:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/robot_bench/BUILD_HOST_REPLAY.ps1
powershell -NoProfile -ExecutionPolicy Bypass -File tools/robot_bench/TEST_BARCODE_PLANE.ps1
```

The second command captures stationary WHITE and the widest printed BLACK
bar on the actual barcode card at GP28. Do not substitute a table or tape.
The sensing spot must fit inside the selected bar at the intended card gap.
It checks
reflectance only and does not apply endpoints. Current physical blocker:
the old GP28 black reference came from the table; the printed-bar hold was
classified white and was insufficiently stable. Inspect the actual GP28/card
gap, alignment and printed bar coverage before another calibration or sweep.
See `../../docs/IR_PRINTED_BAR_CALIBRATION.md`. Earlier clean acquisition and
stationary labelled-target results are not printed-bar or symbol validation.

After that physical issue is resolved, use `TEST_BARCODE_RETRY.ps1 -Symbol Z
-Attempts 1`. Build the host replay first. Replay checks executable and source
hashes against the current checkout, preserving actual timestamps and the
production decoder's unchanged limits. It claims PC replay only, not onboard
decoding or motion. `-Port COMnumber` is optional; automatic detection matches
the robot identity. An empty port list means no capture started.

Firmware build, without copying to any USB drive:

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File tools/robot_bench/FLASH_ROBOT_LOCKED.ps1 -PicoSdkPath 'C:\path\to\pico-sdk' -FirmwareProfile IRBarcodeCapture -BuildOnly
```

The flash helper defaults to build-only. USB copying requires explicit
`-Flash`, a verified RP2040 BOOTSEL drive and all source/UF2/motor-lock checks.
BOOTSEL identity alone does not distinguish two RP2040 boards: the operator
must select the robot. `INSTALL_BARCODE_CAPTURE.ps1` and
`INSTALL_IR_CALIBRATED.ps1` build first, prompt for the robot drive, then call
explicit `-Flash`; do not run installers for ordinary captures. Both require
`-PicoSdkPath`. Source verification is in `source_verification.json`; source
changes require appropriate tests and an evidence-based manifest update.

All work remains local. These scripts do not push, create PRs or update remotes.
