# Buddy 3 IR line following and barcode processing

Student ID: 2500383. This module produces tracking results and navigation
requests for the Vehicle Controller. It never executes movement. Motor PWM,
encoders, motion leases and actual turns remain Buddy 2/controller responsibilities.

The native algorithms and digital GPIO adapter are implemented and tested.
Hardware acquisition is disabled at boot. Analogue acquisition explicitly returns
E_NOSPT: the existing kernel ADC driver cannot currently meet the sampling
contract. A/Z navigation mappings remain NONE until the team supplies its map.
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
No shared configuration, other buddy subsystem, motor or kernel file is changed.

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
| drv_ir.c/h | Sole-task digital GPIO sampling; explicit disabled/unsupported ADC states |
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

## Sensor activation and ADC blocker

No GP26/27/28 assignment is silently selected. The supplied write-up proposes
those pins but labels the signals DO/SIG and analogue inconsistently; TEAM.md
contains no IR pin or ADC ownership claim. Verify voltage, output type and
placement, then claim pins before configuring. Reserved radio pins and the
existing GP19 LED are rejected. GP0/1 are rejected when UART console is enabled.
The remaining proposed assignments in the write-up also need team reconciliation.

Digital acquisition uses gpio_set_pin(..., GPIO_MODE_IN) and gpio_get_val from
the repository BSP. It does not choose pulls, enable outputs or touch ADC/PWM.
Set raw_max=1 and measured white/black levels (0/1, either polarity).

Analogue calibration and processing accept 12-bit values, but S3_IR_ADC is
rejected by both s3_configure and s3_ir_open with E_NOSPT, without pin access.
This is deliberate, not a skipped test or a successful analogue hardware read:

- config/config_device.h enables the kernel ADC; all ADC pad init flags are FALSE.
- device/common/drvif/msdrvif.c: msdi_execfn calls the read function synchronously
  under its lock and does not use its timeout argument.
- device/adc/sysdepend/rp2040/adc_rp2040.c: adc_convert waits up to 1000 ms and
  contains READY busy waits. tk_rea_dev(..., TMO_POL) therefore is not a bounded
  asynchronous solution in this repository.
- The kernel/ADC owner must supply bounded, timestamped acquisition and configure
  the verified analogue pads. Do not add SDK adc_init/adc_read beside this driver.
  Connect the resulting samples to s3_core_step or replace the explicit ADC stub
  in drv_ir after that contract is tested. Do not enable the radio's ADC3/GP29.

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

Objects: one priority-16 task, one priority-inheritance mutex shared with legacy
stats, one copied message buffer holding four records (header/rounding budget
included), and one retained legacy mailbox. No new semaphore/pool/ISR is created.
Every S3 mutex and message-buffer access polls; failure is visible to callers.
No unbounded drain loop, sleep inside decoding, dynamic algorithm allocation,
network service call, encoder access or motor/PWM API is used.

## Verification and delivery

From the repository root, with GCC and GNU Make on PATH:

    make -C testing/subsystem3 test
    make -C testing/subsystem3 verify-failure
    make -C testing/subsystem3 check-headers

The first command builds and executes algorithm and actual adapter tests against
native RTOS/GPIO fakes. The second deliberately fails a real assertion through
the same test target and requires the failure to propagate. The third checks
S3 translation units against real kernel headers in both SMP profiles; it is a
host syntax check, not an Arm build. No tests are silently skipped.

CI is .github/workflows/buddy3-ci.yml: Ubuntu 24.04, checkout@v4, GCC, push to
main, pull request and manual triggers, contents:read only. A firmware CI job is
deferred because main's integration interfaces are already broken and its external
Arm/SDK dependencies are not provisioned here. Actual build commands remain:

    make -C build_make -j2
    make -C build_make CONSOLE=uart WIFI=none -j2

Both stop at the missing arm-none-eabi-gcc prerequisite in this environment.
See testing/subsystem3/TEST_LOG.md and logs for actual execution evidence,
HARDWARE_VALIDATION.md for the unexecuted bench protocol, and AUDIT.md for source
findings. No flash, movement test, GitHub Actions run or PR success is implied.
