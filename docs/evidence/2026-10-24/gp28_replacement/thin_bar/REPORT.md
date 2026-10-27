# GP28 thin-bar diagnostic — 24 October 2026 (+08:00)

**Thin-bar detection is unresolved.** Earlier calibration passes apply to
WHITE versus widest9mm BLACK bars, not to complete barcode readability.

| Test | GP28 raw | New normalized darkness | Scans classified BLACK |
|---|---:|---:|---:|
| Stationary3mm BLACK, 19:54:53 | 197–242 | 0–19 | 0/814 |
| Short single-bar pass, 19:57:21 | 160–248 | 0–22 | 0/1810 |

The earlier independent WHITE range was177–229. The narrow-bar readings
overlap that range. Production C also rejects the stationary recording as
BLACK (diagnostic exit7). Its normal400/600 hysteresis and197/2472 calibration
were not relaxed or refitted to this failure. Lowering thresholds based on
these overlapping stationary labels would risk misclassifying WHITE.

Both recordings have complete10ms acquisition, unchanged zero drop counters,
and zero fault/armed/command/power status in8 and18 controller reports.
The raw signal is already weak before barcode decoding. These observations
do not establish a defective module, the exact optical spot position or a
measured maximum sensing distance. User positioned/moved the chosen bar;
movement distance and exact alignment were not independently measured.
GP26/27 values do not establish line health; a line module may be absent.

No firmware/source calibration or robot command changed during these tests.
The built image has not been flashed. Full A/Z pattern and onboard decoding,
line following and motion remain unvalidated. A paper-only90degree orientation
check at unchanged height is the next diagnostic; mounting and screw remain
unchanged until its result is reviewed. This is not a new barcode scan direction.

Stationary command: `TEST_ROBOT_USB.ps1 -Seconds 8 -Procedure BarcodeCapture
-BarcodeHold BLACK` with a unique THIN_BLACK log and separate output directory.
The short pass used a workspace-only copy with single-bar instructions and
5s/11s cues; acquisition/USB/status logic matches the original helper. It is
an alignment diagnostic, not a full barcode or positive decoder test.

## Paper orientation check, 20:00:17

Turning only the sheet90degrees did not restore thin-bar contrast: GP28
189..229, mean207.823, normalized0..14; all802 samples classify WHITE. Clean
10ms/drop0 acquisition and8 zero-fault/nonzero-drive-free controller reports.
This does not prove which optical or alignment factor causes the weak signal.
Next: if clearance allows, raise the printed sheet flat about1–2mm on firm
backing without touching the sensing head, then compare the thin bar and WHITE
at that closer plane. This is a height diagnostic only; do not refit the
mounted calibration from raised-target data or flash the candidate yet.
