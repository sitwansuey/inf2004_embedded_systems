# GP28 replacement calibration — 24 October 2026 (+08:00)

The replacement module now separates the actual printed BLACK bars from WHITE
at the user's reported final mounted height. A0 uses GP28; D0 was instructed
to remain disconnected, sensitivity unchanged, physical motor power off.
Exact gap and module model remain unmeasured. The user reports the corrected
school print dimensions are correct. No old-module or raised-target data is fitted.

| Split / target | GP28 range | Mean | Scans |
|---|---:|---:|---:|
| Training WHITE, 19:41:06 | 177–219 | 197.983871 | 806 |
| Training BLACK, 19:42:21 | 2449–2493 | 2472.320988 | 810 |
| Independent WHITE, 19:44:38 | 186–229 | 207.975309 | 810 |
| Independent BLACK, 19:45:15 | 3046–3090 | 3068.050617 | 810 |

Use integer means from the **first pair only**: GP28 WHITE=197, BLACK=2472.
The 1620 independent repeat observations are excluded from fitting. BLACK
strength changes between placements; this demonstrates classification margins
on these placements, not constant reflectance or a measured operating distance.
With unchanged 400/600 hysteresis, all 3236 observations classify correctly in
offline calculation. Production C normalization/core replay must also pass.
Raw BLACK entry is 1562 and WHITE exit is 1109 with integer normalization.
No decoder width, timing, polarity, channel ownership or STOP policy is relaxed.

All four recordings have complete mask7 ADC transport at 10ms intervals, zero
drop counter changes and no faults/armed/command/power in 32 controller reports.
Mask7 does not prove physical presence of the other modules. GP27 is low and
a line module may have been borrowed; its origin is not yet confirmed. GP26/27
endpoints remain historical and do not validate the current line setup.
Restore both line modules and revalidate them before any motion test.

Raw logs, capture summaries, setup and analysis are in
`docs/evidence/2026-10-24/gp28_replacement/`. `STATIONARY_ANALYSIS.json` deliberately
compares against the **installed old** 213/3326 endpoints at capture time. The
new candidate's production replay and build results are recorded separately.
Firmware boot diagnostics now print the configured arrays, avoiding stale
duplicated endpoint text. The plane helper likewise reads reviewed local
source endpoints, explicitly separate from installed-firmware identity.

At calibration staging, the robot still runs the old motor-locked capture
image. Building this change does not install it. A/Z physical pattern decoding,
onboard decoding, line following and motion remain unvalidated. All changes
are local; no remote branch is modified and no push or PR is created.

## Completed validation and build

All 3236 actual samples pass production C normalization and paused-core
classification, with no steering or navigation output. Normalized WHITE is
0..14; BLACK989..1000. All Buddy3 native regression executables pass, including
68 calibration checks. Real kernel headers pass SMP0/1; unsafe calibrated
profiles reject unlocked motors, raw mode and endpoint overrides. All16 helper
groups pass, including synthetic forward/reverse A/Z and stale-source guards.
The archived failed22Oct Z remains NOT_DECODED; it is not a current sensor pass.

Fresh ARM IRBarcodeCapture build (`MAKEFLAGS=-B`) and UF2/source/motor-lock
checks pass in **build-only** mode. No USB write or robot command was issued.

| Measurement | Before | After |
|---|---:|---:|
| text bytes | 331884 | 331908 |
| data bytes | 456 | 456 |
| bss bytes | 25584 | 25584 |
| linked static RAM bytes | 26040 | 26040 |
| telemetry message bytes | 50 | 50 |
| UF2 bytes | 665088 | 665088 |

Text increases24 bytes for accurate endpoint diagnostics. Runtime heap use is
not measured. ELF flash loads match332364 bytes of the generated UF2 payload.
The build hash and exact commands/results are retained in the evidence JSON
and the workspace calibration session. Motor lock remains enabled; building
does not validate electrical motor isolation or physical stopping distance.

## Later narrow-bar check: barcode readiness remains unresolved

At19:54 the user reported that the module indicator changed only on wide bars.
The stationary3mm BLACK reading was197..242 (814 scans), overlapping earlier
WHITE177..229; all814 classify WHITE with the new calibration. Production C
confirms the failed BLACK check (exit7). A short single-bar pass at19:57 stayed
160..248 (1810 scans), again with no BLACK entry. Both recordings were clean
10ms/drop0 acquisition with no controller fault or movement status.

The wide-bar calibration/build pass therefore **does not establish thin-bar
or full-barcode detection**. Keep the candidate endpoints and motor lock;
do not lower thresholds just to accept these overlapping readings. No flash
occurred. Paper-only orientation at the same height is the next diagnostic;
precise alignment/optical gap and the cause remain unmeasured. Raw evidence
and the complete failure report are in the `thin_bar/` evidence subfolder.

## Paper orientation check, 20:00:17

Turning only the sheet90degrees did not restore thin-bar contrast: GP28
189..229, mean207.823, normalized0..14; all802 samples classify WHITE. Clean
10ms/drop0 acquisition and8 zero-fault/nonzero-drive-free controller reports.
This does not prove which optical or alignment factor causes the weak signal.
Next: if clearance allows, raise the printed sheet flat about1–2mm on firm
backing without touching the sensing head, then compare the thin bar and WHITE
at that closer plane. This is a height diagnostic only; do not refit the
mounted calibration from raised-target data or flash the candidate yet.
