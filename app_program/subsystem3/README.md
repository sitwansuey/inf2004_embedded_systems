# Buddy 3: IR acquisition, line processing and barcode decoding

This branch contains only Buddy 3-owned production sources, native tests,
IR layout/calibration helpers and documentation. It does not include the
local Vehicle Controller, motor/encoder, USB/MQTT, IMU, ultrasonic, kernel
ADC or global build-profile changes used for robot integration testing.

Current sensor roles: GP27 LEFT line, GP26 RIGHT line, GP28 barcode. The
sampler owns acquisition at a nominal 10 ms and publishes copied status;
only the existing ADC worker accesses ADC. Configure kernel ADC pad ownership
separately with its owner. Defaults leave IR disabled and never start motion.

Recent changes: cyclic sampling without catch-up bursts; bounded ADC reply
wait followed by STOP/generation/lease/freshness checks; copied raw/capture
status; paused-mode timing; compact 13-byte events; barcode edge-bias model
and native regressions. Physical barcode decoding remains unfinished.

Buddy 3 board/layout/calibration helpers are private to this directory, so
native tests do not depend on unpublished shared controller configuration.
Without a coordinator-declared drive lock, raw bring-up refuses E_OACV;
its native positive tests explicitly declare a test-only lock.
If a team robot_config.h exists, its settings remain authoritative. Measured
endpoints are opt-in and robot-specific. The optional motor-scan API only
feeds a diagnostic decoder in the IR sampler; it never commands a motor.

Validation: see testing/subsystem3/PUBLISH_VALIDATION.md, HARDWARE_VALIDATION.md
and TEST_LOG.md. Native/analyzer and real ARM checks pass. The whole branch
still has an unchanged Buddy 2 producer build failure; its owners must fix it.
The user-operated 30%/20-second line-response test turned successfully and
went straight at a cross, then acknowledged STOP. Its external motor/test
controller is intentionally excluded here. This does not establish autonomous
full-course tracking, calibrated distance/error accuracy or A/Z hardware reliability.

Run native checks with make -C testing/subsystem3 test verify-failure check-headers.
Historical design and API documentation follows; dated claims of installed
firmware below describe earlier integration experiments, not this branch's image.

---

# Buddy 3 IR line following and barcode processing

Student ID: 2500383. This module produces tracking results and navigation
requests for the Vehicle Controller. It never executes movement. Motor PWM,
encoders, motion leases and actual turns remain Buddy 2/controller responsibilities.

The native algorithms, digital GPIO adapter and optional kernel ADC worker are
implemented and host tested. Hardware acquisition remains disabled at boot. ADC
activation requires the board/kernel owner to verify the wiring and configure
all three analogue pads; the checked-in board flags still produce E_NOSPT.
A/Z navigation mappings remain NONE until the team supplies its map.
This is an integration candidate; no robot or sensor performance is claimed.

## Repository audit and ownership

Audited main commit: 82114e223badc3d0eda2560663353eb9fbf045a1.
The repository is a micro T-Kernel 3.0 SMP port for Pico W/RP2040, not FreeRTOS
or a Pico SDK CMake application. build_make/makefile includes pico_rp2040.mk;
firmware uses gnu11, Cortex-M0+, Thumb, soft float and freestanding flags.
The default build needs an external Pico SDK (README specifies 2.2.0; also
reports 2.3.0 tested), TinyUSB and cyw43 submodules, Arm GCC, Make and host C++.
The UART/no-radio profile avoids the SDK. No AGENTS.md, CODEOWNERS, previous
Actions workflow or native Buddy 3 runner was present.

app_main.c initializes all subsystems, then starts all of them. The application
Make fragment picks up every .c directly in app_program and one subsystem level.
All Buddy 3 production sources stay at that level; host tests live under testing.
No shared configuration, other buddy subsystem, motor or kernel code is changed.
Phase 2 claims the previously empty ADC peripheral ownership row in TEAM.md;
that claim does not establish sensor wiring or change pad configuration.

TEAM.md still documents the sample application. Its 16-17 priority band is used:
consumer_task is now the Buddy 3 sampling task, priority 16, core 0 under SMP.
The design write-up's priority numbers 0-4 cannot be copied into this kernel.
Existing mbx_samples, mtx_stats and APP_STATS symbols are retained for S4 source
compatibility. They are legacy placeholders and are not IR telemetry. The old
sample consumer and statistics smoke test have been replaced.

Main already contains independent integration failures: S2 includes a missing
app_program/board_pins.h; S2 task_producer.c still uses SAMPLE_MSG, sem_slots,
mpf_samples and S2_PERIOD_MS removed from its header; S4's monitor still uses
the removed pool/semaphore declarations. app_vehicle_request is declared but
has no implementation, and there is no Vehicle Controller task implementation.
Their owners must reconcile these files. Buddy 3 does not invent their APIs.

Direct network cloning was unavailable in the development environment. All 649
tracked files (642 distinct blobs) were imported through the GitHub connector,
verified against their Git object hashes, and reconstructed as an exact shallow
checkout of the original signed commit and tree. See testing/subsystem3/AUDIT.md.

## Architecture and interfaces

| File | Responsibility |
|---|---|
| buddy3_line.c/h | Labelled calibration, polarity, reflectance normalization, hysteresis, weighted error, loss and junction indications |
| buddy3_barcode.c/h | Bounded transition capture, observed A/Z profile validation, reverse decoding and marker hold/rearm |
| buddy3_core.c/h | Sample/time validation, mode control, navigation mapping and bounded recovery requests |
| drv_ir.c/h | Digital GPIO sampling and bounded ADC client, explicit disabled/error states |
| drv_ir_adc.c/h | Priority-17 worker owns exclusive kernel ADC descriptor; one-entry copied request/reply buffers |
| subsystem3.c/h | Lifecycle, controller binding/lease, copied event message buffer and status snapshots |
| task_consumer.c | Periodic kernel task; no motor calls, busy waits or blocking mutex waits |
| subsystem3_test.c | Two pure algorithm smoke checks for TEST=1 images; not physical verification |

The pure API accepts raw sensor values, validity bits and uint32 millisecond
timestamps. Configure each sensor's logical position and line_mask. The selected
barcode sensor must not also be in line_mask. Three sensors may all track a line
with barcode_sensor=S3_NO_BARCODE; simultaneous barcode reading needs a physically
separate sensing location, potentially leaving only one or two sensors on the
line. A single line sensor cannot estimate lateral displacement or prove a
junction: its configured position is the only reported position. Multiple dark
line sensors indicate a possible junction, not a classified intersection.

Normalized darkness is 0 (white) to 1000 (black). Either electrical polarity is
supported. Error is a weighted sensor position in configured logical units,
negative left, positive right; it is not a measurement in millimetres. Default
hysteresis is 400/600. All-white immediately revokes steering; continuous loss
for 100 ms produces one RECOVERY_REQUEST. After another 500 ms without a line,
one STOP_REQUEST is emitted. These are controller requests with no PWM action.
Reacquisition remains held until an explicit controller resume. While an accepted
navigation request is pending, line loss cannot start a competing recovery.

Calibration uses 4-64 explicitly labelled observations of each surface, all three
sensors. s3_calibration_finish requires minimum contrast and rejects surface
variation exceeding one quarter of contrast. A failed finish leaves the config
unchanged. Collect while stationary, validate polarity and lighting, and call
s3_configure with the resulting endpoints before starting the task. Calibration
storage/persistence and the operator procedure are team integration work. Digital
modules need their comparator adjusted physically; software cannot recover the
analogue reflectance discarded by a digital comparator.

## Sensor activation and bounded ADC acquisition

No pin assignment is selected at boot. Verify sensor supply/output voltage,
AO versus DO, sensor-to-pin permutation, polarity and physical placement before
claiming pins. Digital mode uses the repository BSP without changing ADC/PWM.
Configure raw_max=1 for measured digital comparator levels.

ADC mode uses the existing kernel device "adca" through s3_adc_worker, priority
17 on core 0 (priority 16 sampling task). The worker is the sole application
ADC owner and uses TD_READ | TD_EXCL. The kernel retains register/IRQ ownership.
No SDK ADC initialization, register writes, DMA or motor calls are introduced.
The previously empty ADC row in TEAM.md records this optional ownership.

The kernel does not honour TMO_POL at read submission. It calls the driver
synchronously under FastLock; a conversion can wait 1000 ms and READY loops
are unbounded. Therefore the sampling task never calls a device function. It
exchanges copied, fixed-size messages with a worker through two one-entry
micro T-Kernel message buffers. Every client send/receive uses TMO_POL. The
worker may block in the legacy driver; it holds no Buddy 3 status mutex.

A request expires after 20 ms. The client checks this before accepting even a
buffered response. Results carry a sequence and the oldest channel timestamp;
partial/out-of-range, wrong-sequence, future and stale results are rejected.
Status snapshots expose complete raw scans for low-priority bench logging and
clear the raw values/validity mask when acquisition is pending or failed.
s3_status_get exposes the complete raw scan for low-priority bench logging;
pending/error snapshots clear the raw values and validity mask.
Successful receipt pipelines the next scan. While waiting, steering is revoked.
A timeout is observed on the next sampling iteration (nominally every 10 ms,
plus scheduler jitter); it is not a measured hardware WCET guarantee. A terminal
failure reports the acquisition error, latches a sensor fault and prevents
further requests on that boot. Controller resume cannot restart a failed ADC.
The worker is never forcibly cancelled while the kernel uses its local buffer.

ADC activation requires DEVCNF_USE_ADC and DEVCONF_ADC_PIN_INIT_0/1/2 enabled
by the board/kernel owner after verified AO wiring and pin claims. They remain
unchanged and FALSE for the pads, so checked-in firmware fails safely with
E_NOSPT. Channel selection follows explicit pin[] order; only ADC0..2/GP26..28
are accepted. ADC3/GP29 is reserved for the Pico W radio.

Worker isolation does not repair the kernel's unbounded READY loop during
boot or the loss of hardware conversion-error information in its ISR. Those
remain shared-driver review and hardware readiness blockers. See
[Phase 2 analysis and integration contracts](../../testing/subsystem3/FIRMWARE_READINESS.md)
and [hardware procedure](../../testing/subsystem3/HARDWARE_VALIDATION.md).

## Barcode evidence and limitations

The supplied Barcode Sample (1).pdf has A on pages 1/2 and Z on pages 3/4. Pages
2/4 scale the same barcode forms horizontally by approximately one half.
Forms 7/15 contain exact vector rectangles: 29 alternating runs, starting black,
three nine-element characters separated by narrow white gaps, three wide elements
per character, nominal narrow:wide ratio 1:3. The framing pattern is 0x094;
A is 0x109 and Z is 0x0D0. These match the familiar single-character framing
pattern, but this implementation supports only the supplied A/Z sample profile.
It does not assert that the complete project alphabet or checksum is specified.

Decoder tests independently transcribe the PDF runs. Validation requires the
exact run count, start/stop patterns, both separator widths, three wide elements
per character, and a known symbol. Forward and reverse passes inspect only the
saved buffer, at most twice. Local narrow-width medians allow different scales
and moderate changes in speed between characters. Each element must remain
within 25% of its local nominal width; adjacent character units cannot differ
by more than 2:1. Abrupt motion within a character may cause rejection. No
odometry is read: Buddy 2 currently supplies no measured travel-distance API.

Capture stores at most 29 widths. Default minimum run is 40 ms; quiet approach
is 200 ms; the trailing quiet must exceed both 200 ms and four times the widest
observed run. A frame times out at 10,000 ms. Bad timing, partial data, overflow,
invalid framing and unknown symbols generate no navigation. In the active core,
a failed frame causes a fault/hold requiring controller intervention. No retry
pretends the moving robot can revisit a marker. A checksum cannot be invented;
pattern validation cannot detect every corruption that happens to form another
valid symbol. Keep mappings disabled until the encoding contract is agreed.

A completed or interrupted marker latches HOLD. Time alone, a repeat waveform,
pause/resume or a renewed steering lease cannot rearm it. Only a new controller
TRACK directive with marker_clear=true, followed by a quiet white surface,
allows a new capture. The controller must establish physical clearance (for
example verified travel since the marker), not infer it from a noisy sensor.
The default map_a/map_z are NONE; test-only LEFT mappings do not enter firmware.

The minimum measurable element sets a speed limit: v <= narrow_width / 0.040 s
before adding scheduler and optical margins. The approximately 141 mm extent in
the supplied dimension figure does not establish actual narrow widths on every
print. Measure both scales, speed and worst-case sample jitter. Trailing quiet
also delays the navigation event; marker-to-turn spacing and marker separation
must be validated. These are engineering limits, not measured performance.

## Vehicle Controller integration contract

1. After subsystem3_init and before subsystem3_start, the team must arrange a
   call to s3_configure with verified sensor pins, geometry and calibration.
   app_main.c currently starts all subsystems immediately; this configuration
   hook requires the shared-file owner's integration. Default startup is inert.
2. The single controller task calls s3_controller_bind. Other callers cannot
   issue directives, renew leases or consume events. Public APIs are task-only.
3. Explicit s3_controller_control(S3_MODE_TRACK, marker_clear, lease_ms) resumes
   tracking or acknowledges a completed manoeuvre/fault. Lease range is 1-1000 ms.
   Use s3_controller_renew to refresh an unexpired lease without resetting capture
   or clearing a navigation hold. An expired lease requires explicit resume.
4. Read s3_status_get frequently, check E_OK and steering_allowed, and enforce
   central safety arbitration before sending anything to Buddy 2. Reject stale
   snapshots. Never refresh a movement lease using an invalid/held tracking result.
   False steering authority requires the controller to stop or use an explicitly
   approved alternate authority. Buddy 3 cannot stop motors by itself.
5. Drain s3_event_receive with TMO_POL semantics. Only this API consumes events;
   status snapshots clear the embedded event field to prevent repeated delivery.
   Events include sequence, timestamp and controller generation. Unknown mappings
   produce an UNMAPPED diagnostic without a turn. A mapped event suspends Buddy 3
   steering as soon as queued; a full event buffer causes a transport fault/hold.
6. Set S3_MODE_OVERRIDE for safety/obstacle manoeuvres, or S3_MODE_PAUSED for idle.
   Both immediately invalidate published steering and earlier-generation events,
   including a publication race across cores. No stale turn is replayed after resume.
   Discarded events are not automatically retried. The controller owns movement
   acceptance, execution, cancellation and completion acknowledgements.
7. On recovery requests, the controller may implement a bounded search with
   Buddy 2, respecting the 500 ms budget, or stop immediately. STOP/FAULT requests
   preempt normal tracking. Resume only after the unsafe condition is resolved.

Sampling requests a 10 ms delay after each bounded iteration; the actual period
includes execution and preemption on the kernel's 10 ms tick. Pure core gaps
above 30 ms, duplicate/backward timestamps, invalid channels and bad calibration
revoke steering. Status age 30 ms and event age 100 ms are checked at consumption.
All timer intervals fit the unsigned half-range; ordinary uint32 rollover works.
They are provisional software budgets and need on-device measurements.

Objects: one priority-16 sampling task, one priority-inheritance status mutex,
one four-record event message buffer and one retained legacy mailbox. Optional
ADC mode adds a priority-17 worker and two one-entry request/reply message buffers.
Kernel resources allocate once during startup. No new semaphore/pool/ISR is
created by S3. Status mutexes and all client message-buffer accesses poll; the
worker waits for requests without holding the status mutex.
No unbounded drain loop, sleep inside decoding, dynamic algorithm allocation,
network service call, encoder access or motor/PWM API is used.

## Verification and delivery

From the repository root, with GCC and GNU Make on PATH:

    make -C testing/subsystem3 test
    make -C testing/subsystem3 analyze
    make -C testing/subsystem3 verify-failure
    make -C testing/subsystem3 check-headers

The test command builds and executes algorithms, the actual adapter and ADC
worker against native RTOS/device fakes. analyze rebuilds/runs all of them with
GCC -fanalyzer. verify-failure deliberately fails a real assertion through the
same test target and requires the failure to propagate. check-headers parses S3
translation units against real kernel headers in both SMP profiles; this is a
host syntax check, not an ARM build. No tests are silently skipped.

CI is .github/workflows/buddy3-ci.yml: Ubuntu 24.04, checkout@v4, push to
main, pull request and manual triggers, contents:read only. Native tests retain
the original 19 groups and add nine ADC regression groups (28 total), plus GCC
static analysis, real-header checks and failure propagation. A separate job
installs Arm GCC/newlib and Pico SDK 2.2.0 with TinyUSB/cyw43, compiles Buddy 3
ARM objects for SMP=0/1 and attempts the complete default firmware build.
Actual build commands:
make -C build_make -j2
    make -C build_make CONSOLE=uart WIFI=none -j2

Both stop at the missing arm-none-eabi-gcc prerequisite in this environment.
See testing/subsystem3/TEST_LOG.md and logs for actual execution evidence,
HARDWARE_VALIDATION.md for the unexecuted bench protocol, and AUDIT.md for source
findings. No flash, movement test, GitHub Actions run or PR success is implied.
