# CSC02 local integration report

The robot code is integrated locally for CR1 and builds for seven profiles.
It is not a completed physical acceptance run. Wiring, calibration, closed-loop
speed PID, absolute heading and audio work remain; see the explicit limits below.

## Repository and branch audit

The starting tree was clean on feat/buddy3-line-barcode-ci at bac31d8. Created
`local/csc02-cr1-integration-20261021`, fetched origin read-only, and fast-forwarded
only this new local branch to origin/main 210a407. Those two commits have the
same file tree. No user edits were overwritten. No push, PR, remote merge or
flash was performed.

| Audited branch | Head | Treatment |
|---|---|---|
| main | 210a407 | Latest integrated B2/B3/B4 foundation |
| feat/buddy3-line-barcode-ci | bac31d8 | Existing tested sampler/ADC-worker/line/barcode interfaces retained |
| system2commit | 25589b3 | Already in main; retained motor ownership and consistent dual encoders |
| buddy4/gy511 | edee296 | Already in main; retained I2C owner and independent channel validity |
| buddy1/MQTT | 176587a | Reused copied nonblocking inbox/dashboard approach, replaced scalar/text transport with binary; did not import its stale copies of other buddies |
| subsystem5 | 5d8cf50 | Selected ultrasonic/servo, scan, width and path policy; replaced auto-driving main and timed travel with a disarmed controller and calibrated encoder feedback |

## Changes and behavior

Added a sole Vehicle Controller at priority 5/core 0. It checks fresh line,
terrain, odometry and obstacle inputs; consumes bounded events; renews motor
leases; and stops at every handoff. Missing sensors, stale data, shocks,
transport errors and expired commands inhibit motion or latch a fault. A
separate urgent STOP epoch invalidates queued movement even when the motor pool
is full. The motor task alone changes PWM and keeps its independent watchdog.

Line/barcode events use a 13-byte copied format. State/flag containers and S5
status are compact. Timestamps/counters retain their range. Added gated
odometry, ordered hump estimation and run maximum without claiming a gyro or
physical accuracy. S5 rejects failed echoes as unknown space, movement during
scan acquisition, inconsistent edge samples and unverified planned headings.
The controller stops while the servo changes direction and waits for matching
fresh forward/side observations. Bypass remains disabled until validated.

MQTT now publishes one length-aware binary snapshot topic, with bounded owner
polling, stale-frame rejection, retry guarding and copied fragmented commands.
Reconnect offers STOP and requires RESET before ARM. A read-only Python/browser
dashboard shares the decoder. USB command reception remains inside the USB
owner. Fixed the obsolete sample-producer build failure, the SDK 2.3.1 UART
include gap and two unbounded ADC readiness loops; ADC conversions have a 5 ms
kernel timeout and a 20 ms sampler deadline.

## Verification

- Buddy 2: eight native Windows GCC configurations pass, including command
  validation, renewal/expiry/wrap, encoder banks, pool/clock errors, urgent STOP
  saturation and controller ownership.
- Buddy 3: algorithms, adapter and ADC tests pass; GCC analyzer, real headers
  and Cortex-M0+ object checks pass for SMP 0 and 1. The intentionally forced
  failed assertion is detected by verify-failure, not counted as a regression.
- Buddy 4: existing driver/data/API groups pass with ASan/UBSan.
- Integration: controller, odometry, terrain, scan, copied queues and actual
  MQTT-port callback tests pass. The latter use fake RTOS/network boundaries,
  not a live broker. ASan/UBSan passes. Independent C/Python agreement covers
  302 vectors and 74 bound rejections, plus invalid length/version/padding.
- ARM firmware profiles: default, MQTT, passive test, SMP=0, UART/no-radio,
  S2-only test and S4-only build. Toolchain: ARM GCC 14.2.1, native GCC 15.2.0,
  Pico SDK 2.3.1. MQTT compiled with a dummy config in the isolated snapshot;
  no real credentials were read or changed.

Commands and exact logs accompany this report. Final installation verification
is recorded separately after applying to the real checkout.

## RAM and message measurements

ARM object symbol sizes from the actual headers, not host sizeof assumptions:

| Representation | Before bytes | After bytes | Saving bytes |
|---|---:|---:|---:|
| line_event | 12 | 12 | 0 |
| line_event_record | 16 | 16 | 0 |
| motor_message | 16 | 16 | 0 |
| motor_status | 16 | 12 | 4 |
| line_output | 32 | 28 | 4 |
| line_status | 60 | 56 | 4 |
| obstacle_status | 136 | 64 | 72 |

S5 before is its audited branch layout; other before sizes are the starting
main-equivalent tree. ARM already uses small enum types, so the event's decoded
C layout does not shrink; its copied wire record shrinks from 16 to 13 bytes.
The motor message stays 16 bytes because the safety STOP epoch uses the saved
space. Bounded fields use explicit packing; pointers/32-bit arithmetic stay aligned.

The unified payload is 50 bytes/394 meaningful bits plus 6 zero pad bits. An
equivalent synthetic JSON reference with the same 37 field names is 469
bytes; this is an illustrative same-fields comparison, not a captured old robot
packet. The old application did not produce a functioning unified payload.
Buddy 1's original scalar text inbox uses a 136-byte record and 2240-byte
16-entry kernel buffer. The new two-snapshot buffer uses 112 bytes. That queue
saving includes a capacity/semantics change; it is not solely a codec effect.

The exact starting firmware fails to build because the retired producer still
references deleted interfaces (saved failure log). For a baseline ELF, only
that file was excluded in the measurement copy. With encoders enabled the
repaired baseline is text/data/bss 294544/96/25076 bytes, static RAM 25172 bytes.
Final integrated profile sizes are in MEASUREMENTS.json. Added integration
increases total static RAM despite smaller records. Runtime peak stacks, heap,
CPU load, end-to-end latency and electrical STOP response have not been measured.

## Outstanding hardware and feature limits

No USB/serial identification, flashing, wheel movement, sensor calibration,
physical line/barcode run, echo/width test, hump test or live MQTT test was
performed. The user postponed robot connection. Motion stays gated until
verified IR configuration, encoder scale/chassis geometry and IMU mounting are
provided. Barcode A/Z symbols are implemented but navigation mappings are
unset; one sensor cannot be both line-facing and independently barcode-facing.

Motor actuation remains open-loop PWM; PID speed regulation has not been
implemented or tuned. Relative encoder-based turns are implemented, not an
accuracy guarantee. Absolute compass heading and audio playback remain absent.
The height calculation is a guarded geometric estimate, not validated hump
metrology. Width bounds cover angle quantization only and require finite valid
background echoes. No-echo readings stop operation. Physical validation must
establish response time, braking distance, slip, motor signs and supply margins.
Follow HARDWARE_ACCEPTANCE.md before enabling bypass or calling the project done.
