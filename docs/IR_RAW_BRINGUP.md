# Buddy 3 IR bring-up before encoder/motion work

## Current physical roles and calibration

GP26 right line, GP27 left line, GP28 barcode. Measured endpoints and all four
stationary classifications now pass on the robot:161/161 fresh scans with
zero errors/mismatches/authority/commands. Corrected3E84E2... is installed,
RAM26032 and telemetry50 bytes. Keep GP27's D0 disconnected and all settings
unchanged. See IR_BARCODE_CAPTURE.md for verified evidence and the next A/Z
passive per-scan capture, whose new optional profile is not built/flashed yet.

## Earlier bring-up history

The user prioritized Buddy 3 on 22 October 2026. The Week 6 submission assigns
analogue inputs GP26/GP27/GP28, while the user confirms Grove 3V3 power and lit
modules but cannot identify AO/DO or the model. Do not infer functioning output
from an LED. Keep wiring and supply unchanged; input-only ADC reads can inspect
either a 3V3 analogue output or a 3V3 digital output without driving that signal.
Rail-valued readings alone do not prove that a module is digital.

New build profiles IRRawAdc (IR_RAW_MODE=2) and IRRawDigital (IR_RAW_MODE=1)
enable BOARD_IR_RAW_BENCH=1. Both require BOARD_MOTORS_DISABLED=1. The default
build retains BOARD_IR_MODE=0 and raw bench disabled. Each profile has a distinct
UF2 suffix and build stamp, so changing modes rebuilds dependent objects.

The startup hook calls s3_configure_raw before the existing S3 sampling task.
ADC acquisition uses the existing copied request/reply buffers, priority-17
worker and kernel adca device. The sampler submits the current scan and waits
on its copied reply with a requested 1 ms kernel timeout (up to 20 ms with the
guard tick, plus dispatch jitter). Completion normally wakes it sooner. Only
the worker enters the synchronous device driver. There is no prefetch of the
next scan before the sampling sleep. Only ADC0..2 are selected, and only the board's
existing ADC pad configuration applies. No SDK ADC owner or register reader is
added. Digital mode only configures inputs. Sensor failures remain latched for
the boot. Raw mode never runs line/barcode interpretation, never sends steering
events, reports configured=false, and refuses TRACK even from the controller.
The independent compile-time motor lock remains effective at controller and
driver levels. Binary telemetry remains 50 bytes and correctly marks IR line
authority unavailable; the bench-only S3RAW text contains uncalibrated scans.

S3RAW records copy result, kernel error, oldest-channel time, acquisition result,
valid mask, three raw channels, configured, steering and freshness. Use only
copied=1, error=0, acquisition=0, mask=7, fresh=1 records with distinct timestamps.
Never interpret failed, stale or pending scans'
zero-filled values as a white/black surface. Indices initially mean GP26, GP27,
GP28; their physical left/centre/right placement must be established separately.

The prepared FLASH_AND_TEST_IR.ps1 helper builds/flashes the locked IRRawAdc
profile, rejecting older/non-bench images, then opens two Enter-gated captures
through TEST_IR_SENSORS.ps1. Keep the robot stationary and the sensors at normal
working height. Put a uniform white surface under all sensing windows for the
first 12-second capture, then a uniform black surface for the second. Do not
adjust comparator screws or change lighting/height between surfaces. No motor
or serial commands are sent, and no calibration is automatically installed.
Logs are IR_WHITE.log / IR_BLACK.log and corresponding result JSON files.

After capture, review per-channel values, polarity, contrast, noise and label
placement before choosing AO/DO operation or applying endpoints. The existing
s3_calibration_* routines support labelled observations and reject low contrast
or unstable surfaces. No sensor performance, line following, barcode decoding
on hardware, or physical sensor placement is established by native tests.
GP26 sharing between Grove ports 5/6 and the GP28 VBAT solder jumper remain
physical checks if channels show coupling or unexpected readings.

The first IRRawAdc image was built and copied at 05:20 +08:00 on 22 October
2026 (SHA256 21d0c1651c0c7c157ecac7a20aec2cab7a1a7bb0673d0745d8c7af9826759878).
USB startup confirms motor lock and the raw GP26/27/28 profile. The white
capture contains 60 IR rows, all with acquisition=-50, mask=0; black capture
was correctly stopped. These zeros are invalid data, not surface readings.

Code inspection found a guaranteed polling mismatch: kernel timer.c adds its
10 ms guard tick to tk_dly_tsk(10), so the next poll runs at +20 ms or later.
The old client rejected every reply at age >=20 ms, even one already copied
by the worker. Conversion remains limited to <20 ms. The corrected client
allows delivery at 20 ms and rejects at >=30 ms, retaining the oldest-channel
timestamp and the existing 30 ms status freshness cutoff. No kernel timer,
task priority, ADC hardware ownership or motor setting is changed. This fixes
a reproducible software defect; it does not establish that physical ADC
conversions or sensor output work.

The timing image was copied at 05:30 +08:00 (SHA256
fd9b11d0220724d92566914818acef8a57ec5977971b77f0a06644623a846841).
Its saved startup contains two successful complete ADC scans, GP26/27/28 values
178/172/131 and 186/194/149. Both were stale when reported; a timeout is latched
by 580 ms. The subsequent white capture has 61/61 invalid scans. Startup
values are unlabelled and cannot be used as white/black endpoints. This proves
some ADC transfers completed, not that each connected sensor responds correctly.

The remaining defect is the prefetched pipeline: data is acquired before the
sampler's sleep, then delivered on the next wake; another delayed wake crosses
the 30 ms cutoff. The current correction submits after waking and publishes the
current reply without waiting for another sampling period. Existing nonblocking
polling remains available for compatibility. Post-wait clock, STOP/generation
and lease checks prevent publishing steering permission from an expired
directive. Conversion and freshness limits remain 20/30 ms. ADC failures still
latch; no forced cancellation or retry of a failed device occurs. Raw mode
adds one-time client/worker failure messages to diagnose any further failure.
No sensor endpoints, pin wiring, priorities or kernel timers are changed.

Native validation passes the original algorithm/adapter/ADC suites plus raw
digital/ADC acquisition, disabled-pad rejection, no-authority publication,
controller TRACK rejection, ADC timeout invalidation and motor-unlocked
rejection. Real kernel headers compile for SMP 0/1 with IR modes 0/1/2; a raw
bench with unlocked motors fails compilation. New S3-U34..36 tests exercise
the production ADC pipeline at the real 20 ms polling cadence, across rollover,
with rejection of stale delivery and late conversion. The original client
fails the cadence test. S3-U37..41 cover current-scan acquisition across delayed
20/30/40 ms wakeups and rollover, raw no-authority behavior, STOP during reply
wait, lease expiry, clock/transport failure and bounded absent reply. The actual
05:30 sampler fails the new delayed-wakeup test even when its worker is allowed
to execute during sleep. The current correction still needs an ARM rebuild and
physical black/white tests using the user's normal PowerShell helper.
