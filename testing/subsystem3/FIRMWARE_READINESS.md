# Buddy 3 Phase 2: firmware and integration readiness

Date: 2026-10-15. Base: 82114e223badc3d0eda2560663353eb9fbf045a1.
Branch: feat/buddy3-line-barcode-ci. No physical hardware was connected or tested.

## ADC root cause, established from repository source

1. The previous Buddy 3 E_NOSPT was an intentional safety gate, not evidence of
   a measured ADC hardware failure. The kernel ADC is enabled, but the board
   enables none of DEVCONF_ADC_PIN_INIT_0/1/2. Wiring and AO versus DO are unknown.
2. device/common/drvif/msdrvif.c, msdi_execfn: tmout is ignored; the selected
   readfn executes synchronously while holding the device FastLock. The waitfn
   subsequently returns immediately. TMO_POL does not make submission asynchronous.
   lib/libtk/fastlock.c uses a semaphore when contended; the read is not executed
   under a global dispatch-disable region, so the sampler may preempt the worker.
3. device/adc/adc.c, read_data: dev_adc_llctl calls the low-level conversion
   directly. The RP2040 driver checks READY with an unbounded busy loop, then
   waits on done_flgid with DEVCNF_ADC_TMOSCAN (1000 ms). Three sequential reads
   can consequently exceed the 10 ms sampling period by orders of magnitude.
4. device/adc/sysdepend/rp2040/adc_rp2040.c, dev_adc_llinit also busy-waits for
   READY at kernel boot. A worker cannot protect boot against that wait.
5. The ISR reads ADC_FIFO & 0x0FFF without reporting ADC_CS_ERR. FIFO error
   recording is not enabled. Software range checking cannot recover error bits
   the kernel has discarded. Timeout cleanup also does not drain/reset a late
   conversion FIFO before another read. No automatic retry is safe to assume.

## Implemented bounded acquisition boundary

Only s3_adc_worker opens adca with TD_READ | TD_EXCL and calls tk_srea_dev.
It uses priority 17/core 0, below the priority-16 sampler. Register and interrupt
ownership stays in the existing kernel driver; no SDK ADC APIs are used.
The optional ADC peripheral claim occupies the formerly empty TEAM.md row.
No GPIO wiring claim or shared-driver/configuration code was changed.

The sampler and worker exchange copied requests/replies through two fixed-capacity
one-entry micro T-Kernel message buffers. There is one outstanding scan. The
sampler uses TMO_POL exclusively and holds no status mutex across acquisition.
A scan specifies a sequence, request timestamp and explicit sensor/channel order.
Only ADC0..2 are accepted; GP29/ADC3 remains reserved for the radio. Each kernel
read requests one UW conversion with asize == 1, matching the repository driver.

A response must arrive before the 20 ms request deadline, carry the matching
sequence, contain all three 12-bit readings and a plausible oldest-channel
acquisition timestamp. Receipt starts the next request without reusing its
sample. Pending scans revoke steering. Deadline expiry, unavailable device,
short/invalid reading or malformed transport data invalidates the whole scan.
s3_status_get also returns the latest complete raw scan for a bounded external
logger/calibration observer; pending/error snapshots clear raw values and mask.
The original acquisition ER is reported; the algorithm publishes a sensor fault
and holds steering. A terminal ADC failure cannot be cleared by TRACK/resume.
No more requests are sent on that boot and the worker exits after a returned
error. It is never forcibly terminated inside a kernel read. Its descriptor
remains held until system restart; another task must not steal/reset the ADC.

This bounds the sampling task's I/O work, not the legacy driver's execution time.
The 20 ms deadline is observed at the next sampler service, nominally 10 ms
apart plus scheduler jitter. No measured hard real-time or sensor performance
claim is made. A hung READY loop may leave the worker stuck until restart.
Kernel boot hangs and hidden hardware conversion errors remain blockers.

ADC mode is available only when DEVCNF_USE_ADC and all three kernel pad-init
flags are enabled by the board/kernel owner following verified wiring/pin claims.
Current flags are unchanged/FALSE; ADC configuration therefore returns E_NOSPT.
Native ADC tests inject pad-enabled build flags only in their test fixture.

## Actual subsystem contracts and responsible owners

| Dependency | Repository evidence / contract | Responsible owner / remaining action |
|---|---|---|
| Board pins | app_program/board_pins.h is absent; Buddy 2 includes it. The file under subsystem2/tests/host is a host fixture only. | Board/team integration owner: provide reviewed production motor/encoder/IR assignments and electrical levels; resolve TEAM.md claims. |
| Vehicle Controller | app_vehicle_request(UINT subsystem, s2_move_command_t, UINT power, UW lease_ms) is declared in subsystem2.h but unimplemented. No controller task/arbitrator exists in the checked-in source. | Vehicle Controller owner: implement arbitration, safety priorities, periodic leases and command acknowledgements. |
| Buddy 2 | s2_controller_bind, s2_movement_send, s2_motion_get_status; moving commands expire after 1000 ms. Status is commanded output, not measured motion. | Buddy 2/controller: bind only the controller; renew leases, handle failed STOP/queue exhaustion, supply measured odometry if needed. |
| S2 producer | task_producer.c uses removed SAMPLE_MSG, sem_slots, mpf_samples, S2_PERIOD_MS. | Buddy 2/controller owner: reconcile the obsolete producer with the intended controller task. Buddy 3 does not recreate the deleted producer protocol. |
| Buddy 4 | subsystem4.h currently declares a sample-reporting MBF/event flag, not an IMU contract. task_monitor.c still references S2's removed sem_slots/mpf_samples. | Buddy 4 owner: reconcile stale monitor declarations and publish the actual IMU interface if required. |
| Buddy 3 events | s3_controller_bind binds one task; s3_controller_control/renew provide modes/leases. s3_status_get copies tracking/error/fault state. s3_event_receive consumes copied navigation, recovery, STOP and fault messages, TMO_POL. | Vehicle Controller: bind and service these APIs, validate configured/fresh/no-error/steering_allowed before motion, explicitly acknowledge recovery/turn completion. No consumer is implemented elsewhere yet. |
| ADC kernel | Synchronous driver, unbounded READY waits, discarded conversion-error status and late FIFO hazards. | Kernel/board owner: review shared changes listed below, confirm boot and peripheral behaviour on hardware. |
| Sensor geometry / barcode | Wiring, AO/DO, left/centre/right ordering, barcode offset and A/Z-to-navigation mapping remain unverified. | Buddy 3/team hardware owner: measure, document and agree configuration; mappings stay NONE. |
| Team approval | No team approval has yet been recorded for the new acquisition contract/shared-driver work. | Maintainers and affected subsystem owners: review before merge. |

The legacy S3 APP_STATS/mbx_samples symbols remain compatibility placeholders,
not tracking telemetry. Buddy 3 does not drive PWM, read encoders, profile
obstacles, or bypass Vehicle Controller arbitration. Controllers must stop
renewing movement on invalid/stale acquisition and actively request STOP, while
checking Buddy 2's send result. An accepted barcode event holds Buddy 3 steering;
physical marker clearance plus an explicit TRACK(marker_clear=true) rearms it.

## Shared-driver change request (isolated for review, not applied)

The implementation deliberately changes no device/, kernel/, lib/ or shared
configuration source. The kernel owner should review these specific changes:

- Bound READY polling both at boot and before conversions using an appropriate
  already-initialized monotonic hardware clock, returning initialization/I/O
  errors without global spinning. Review the boot clock/reset sequencing.
- Preserve conversion-error information (ADC_CS_ERR and/or FIFO error flag),
  reject failed samples, and verify FIFO emptiness/IRQ ownership before reads.
- Make timeout/late IRQ cancellation safe under the existing SMP ADC lock;
  reset/drain stale conversions before any subsequent request is permitted.
- If a genuinely asynchronous ADC device endpoint is introduced, define its
  submission, cancellation, buffer-lifetime and timeout contracts explicitly.
  Changing common msdrvif timeout semantics affects serial and other devices
  and must be reviewed separately, not silently done for Buddy 3.
- Enable only hardware-verified analogue pads after recording pin claims. Never
  configure ADC3/GP29 or initialize Pico SDK ADC alongside the kernel driver.

## Verification commands and current results

Run from repository root in a POSIX shell (Windows: MSYS2 with native GCC or WSL):

~~~sh
make -B -C testing/subsystem3 test
make -C testing/subsystem3 analyze
make -C testing/subsystem3 check-headers
make -C testing/subsystem3 verify-failure
make -C testing/subsystem3 arm-check
make -C build_make -j2
make -C build_make CONSOLE=uart WIFI=none -j2
~~~

Native baseline: all original 19 groups retained. Added nine ADC groups U22..U30.
Current native total: 28 groups, 529 algorithm + 15261 legacy adapter + 1012 ADC
checks, zero failures/warnings. GCC -fanalyzer and SMP=0/1 real-header parsing pass.
The forced false assertion returns nonzero through the actual test target.
These are deterministic software tests, not sensor/robot measurements.

Local Arm GCC, binutils/newlib and Pico SDK are absent. WSL enumeration returns
E_ACCESSDENIED. Direct GitHub and official Arm downloads are blocked by this
execution environment. Both local firmware commands fail at the Makefile's
arm-none-eabi-gcc-not-found guard (exit 2). No local ARM link or UF2 was produced.

The GitHub workflow now retains native tests/static analysis and adds a genuine
firmware-build job: install Ubuntu's gcc-arm-none-eabi/newlib, obtain Pico SDK
2.2.0 plus TinyUSB/cyw43 submodules, compile Buddy 3 objects for SMP=0/1 and run
make -k -C build_make -j2. No missing source or test is skipped and no subsystem
is replaced with a stub. The integration rejected Git data tree upload and draft-PR creation with HTTP
403 Resource not accessible by integration, despite account metadata showing
Admin/Write. No reconnect action is exposed in this session. Remote execution
results can be recorded after successful publication and PR creation. A successful Buddy 3 object check alone is not a firmware link.

See TEST_LOG.md, logs/ and HARDWARE_VALIDATION.md for evidence and bench steps.
Do not merge while the complete firmware build, critical integration or required
team approvals remain unresolved. No hardware validation has passed.
