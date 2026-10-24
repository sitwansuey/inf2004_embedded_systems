# Buddy 3 validation scope — 25 October 2026

## Sensor evidence

GP27 is LEFT, GP26 RIGHT, GP28 the dedicated barcode input. The user confirmed
GP28 A0 wiring and disconnected its D0 during analogue tests. Three separate
modules use 3V3. Sensor model/optical gap are not established. Endpoints are
robot-specific and must be measured again after mounting/module changes.

Independent stationary line checks passed 2,418 samples: 798 RIGHT BLACK /
LEFT WHITE, 808 LEFT BLACK / RIGHT WHITE, and 812 both WHITE. No acquisition
gaps or classification mismatches. GP26/27 endpoints remained unchanged.
GP28 replacement separates wide printed BLACK and WHITE in stationary checks;
thin-bar and complete independent A/Z decode reliability remain unresolved.

## User-operated line response test

On 25 October, external integrated IRContinuous firmware v6 ran 30% PWM for
20,000 ms and acknowledged final STOP. The user reports successful turning
and straight travel through the cross. The recorded masks include 0/1/2/3,
and commanded directions include forward, left pivot and right pivot.
The user planned to move/reposition the robot during this test.

This passes the reported user-operated turn/cross response check. It does
not certify unassisted full-course tracking, junction classification,
stopping distance or lateral error in millimetres. The external controller
and motor/USB integration are not part of this Buddy 3-only push.

Integration test firmware SHA256:
2deddabc20183951ddb8036240d8d1b50f5fdfe903ef4bf13a240956f018bf69
Recorded run log SHA256:
703d6d7380a30da02fa382e0be03220b82878ae0d41e7b4596c9d790494344a6

## Remaining team integration

Controller owner selects line/junction behaviour, motion limits, STOP policy
and navigation maps. Buddy 2 owns PWM, encoder and motion feedback. Kernel
owner enables ADC pads/device. This branch changes none of those components.
Buddy 3 continues publishing junction/validity indications without inventing
an autonomous route, physical scale, unsupported sensor or hardware accuracy.

Native and ARM-object validation of the isolated branch are recorded separately.

---

## Historical validation plan (retained)

# Buddy 3 bench and robot validation protocol

Status: NOT RUN. Native tests exercise C algorithms and fake RTOS/GPIO boundaries;
they are not sensor, board, optical, timing or robot verification. Keep navigation
mapping NONE and acquisition disabled until the relevant checks below are complete.
Record real results and waveforms using testing/README.md. Do not overwrite old
results or convert this checklist to PASS without evidence.

| ID | Test | Required observations and pass criteria |
|---|---|---|
| S3-M01 | Electrical identification | Record module part number, power supply, DO/AO/SIG connector and output voltage. Confirm 3.3 V safe logic/ADC, black/white polarity, common ground and comparator behavior. Resolve the write-up's contradictory output labels. |
| S3-M02 | Pin/peripheral ownership | Reconcile actual wiring with TEAM.md and board_pins.h. Claim verified IR pins and ADC owner where applicable. Exclude radio GP23/24/25/29 and occupied GP19; reconcile GP0/1 with UART crash reporting. Verify actual firmware never initializes unclaimed pads. |
| S3-M03 | Geometry | Measure the 18 mm line, 20 mm separation to the barcode and both printed sample extents. Log each sensor's lateral/longitudinal position and optical footprint. Show which sensors actually track the line while another crosses the right-side barcode. Demonstrate that a close-packed three-sensor line array is not assumed to observe both. |
| S3-M04 | Calibration and lighting | Stationary vehicle: collect 4-64 labelled white observations and 4-64 black observations per sensor at working height, repeat in expected lighting. Record raw distributions, selected contrast and polarity. For DO modules adjust the hardware comparator, validate stable levels and noise rejection. Verify low contrast/noisy calibration is rejected. |
| S3-M05 | Analogue worker and deadline | After verified AO wiring and owner-reviewed kernel pad configuration, exercise the optional ADC worker. Verify sampler deadline observation, blocked/unavailable ADC, late completion rejection and terminal hold. The kernel read may block for 1000 ms and READY waits remain unbounded; review those shared-driver defects before production deployment. No parallel SDK ADC initialization. |
| S3-M06 | Captured A/Z waveforms | With motor execution disabled or externally controlled safely, capture timestamped raw levels and transitions across every A/Z scale in both travel directions. Compare all 29 runs and framing to supplied vector fixtures; record any encoding/checksum differences. Obtain the team's signed-off LEFT/RIGHT/STRAIGHT/U-TURN map; do not infer it from A/Z. |
| S3-M07 | Sample budget and speed | Measure worst-case period/jitter and task stack high-water mark with WiFi/USB/other buddies active. Minimum observed run must exceed the configured 40 ms budget with margin. Check max sample gap 30 ms, status age 30 ms and event age 100 ms. Record real velocity/narrow-width relation; do not treat open-loop PWM percent as measured speed. Check near-threshold/corrupted/partial waveforms produce no turn. |
| S3-M08 | Controller and motor handoff | Wheels raised first. Controller binds once, consumes events and refreshes its lease. Verify one request per marker and steering disabled during a queued/accepted turn. Buddy 2 alone executes motor output. Exercise queue full, controller crash/lease expiry, stale event, missed iteration and sensor unplug. Controller must command STOP or another approved authority when S3 steering is false. |
| S3-M09 | Line loss and bounded recovery | Start at low measured speed after raised-wheel checks. Cover line sensors: steering invalid immediately, one recovery request after 100 ms continuous loss, stop request after another 500 ms. No autonomous PWM from S3. Recover line during and after the search: controller acknowledgement is necessary to resume. Verify all-black/split patterns are treated as junction ambiguity. |
| S3-M10 | Override and marker clearance | Trigger emergency/obstacle override before, during and just after capture. No stale navigation event may execute after override/resume. Reverse or oscillate over the same marker: no duplicate until controller confirms physical clearance and the sensor sees quiet white. Validate the source of clearance evidence; a timer or quiet alone is insufficient. |
| S3-M11 | Marker spacing and turn timing | Measure approach/trailing blank surfaces. Default trailing quiet is max(200 ms, 4 x widest run), which adds travel before event delivery. Confirm marker-to-junction lead distance and adjacent-marker spacing at both scales and maximum tested speed. Reject operating conditions that cannot preserve framing/quiet zones. |

Use s3_controller_control(S3_MODE_OVERRIDE, false, 0) for software preemption,
check its return value, and retain an independent physical means of stopping the
vehicle. No hardware flashing or movement was performed by this change.


## Phase 2 reproducible three-sensor bench sequence

All steps below are NOT RUN. A hardware operator records actual observations;
expected behaviours below are software acceptance criteria, not measurements.

1. Obtain an actual complete ARM firmware build, reviewed board_pins.h and the
   controller/S2/S4 integration fixes before integrated operation. Record the
   compiler/SDK versions, image hash, board revision and commit. Keep propulsion
   physically disabled while identifying the sensors. No host fixture board
   header may be used as production wiring evidence.
2. Identify all three modules separately (sensor 0/1/2). Record part number,
   supply, connector labels, output voltage on white/black, AO or DO and polarity.
   Confirm the output is safe for the actual RP2040 input reference. Map measured
   wiring to pin[] explicitly; no GP26/27/28 assignment is assumed by this code.
3. If AO wiring is confirmed, obtain board/kernel owner review for pad flags
   DEVCONF_ADC_PIN_INIT_0/1/2 and TEAM.md pin claims. Check no other application
   opens the ADC; s3_adc_worker uses an exclusive read descriptor. Verify GP29 is
   untouched with USB/WiFi enabled. If DO, keep ADC mode off, select measured
   digital pins and raw_max=1, and adjust physical comparators as needed.
4. Before enabling acquisition, verify default-disabled firmware reports no
   valid scan and no steering authority. With ADC pad flags still FALSE,
   s3_configure(ADC) must return E_NOSPT without GPIO/ADC initialization.
5. With verified activation and a stationary chassis, have the controller or
   a separate low-priority logger copy s3_status_get snapshots into a bounded
   log. Log sample.raw[0..2], sample.valid_mask, sample_time_ms,
   acquisition_error, output.line state/error and steering_allowed. Only E_OK
   acquisition with a complete mask and fresh timestamp is a valid scan. Raw
   values are cleared on a pending/failed scan; do not calibrate from those.
   Do not print from the timing-critical worker/sampler or call the ADC from
   the logger. Record labelled white/black sets (4-64 per surface per sensor),
   contrast/noise, working height and illumination; verify calibration rejection
   for poor contrast/noisy surfaces and correct polarity in all three channels.
6. Move a black strip beneath each optical footprint independently. Confirm the
   logged channel permutation and signed tracking error. Test all-white loss,
   all-black/split junction ambiguity and invalid/unplugged outputs. An unplugged
   AO lead can float to an in-range voltage: software range checking alone cannot
   prove disconnection. Establish the module's real electrical failure behaviour.
7. Measure request-to-reply age, oldest-channel skew, sampler cadence/jitter and
   task stack margin on SMP=0 and SMP=1 under WiFi/USB/controller load. Confirm
   normal scans arrive before the 20 ms deadline and remain within the 30 ms
   core freshness budget. Log the actual maximums; do not infer them from tests.
8. In an owner-reviewed fault-injection image, suppress/delay ADC completion
   without disabling the scheduler. The sampler must continue running, revoke
   steering and report E_TMOUT/S3_ERROR_SENSOR at the first service after the
   deadline. A late reply must not restore authority or start another scan.
   Test unavailable/exclusively-held device, short reads and conversion errors.
   Since the current kernel masks conversion-error information, validating those
   errors requires its owner-reviewed fix. Test boot READY failure separately:
   worker isolation cannot protect initialization before the scheduler starts.
   Restart after a terminal acquisition fault; never kill a worker inside a read.
9. Pass every supplied A/Z sample at both printed scales, forwards and backwards,
   first with motors off and measured manual transport. Log raw timestamped
   transitions and compare the 29-run framing/widths. Measure the smallest printed
   element and actual transport speed; verify minimum run >= 40 ms with optical
   and scheduler margin. Include corrupted, obscured, partial and unknown codes.
   Keep map_a/map_z=NONE until the team approves the navigation codebook.
10. With a functioning Vehicle Controller and wheels raised, verify one copied
    navigation event per physically cleared marker, controller leases, pause,
    override, recovery/STOP requests, event expiry and queue-full behaviour. Only
    Buddy 2 may execute movement. Repeat line reacquisition and both-scale barcode
    approaches at progressively measured speeds, recording marker-to-junction
    lead distance and trailing quiet requirements before any floor test.

Record each step against S3-M01..M11 with dated logs, image/commit, raw captures,
measurements, pass/fail and responsible operator. No row is currently PASS.
