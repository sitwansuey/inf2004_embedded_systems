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
| S3-M05 | Analogue driver deadline | If sensors are AO, first obtain a reviewed bounded acquisition implementation from the kernel/ADC owner. Existing msdi_execfn ignores TMO_POL and may block for 1000 ms. Validate missed-conversion/error/timeout behavior and ownership on both cores before replacing the S3_IR_ADC E_NOSPT stub. No parallel SDK ADC initialization. |
| S3-M06 | Captured A/Z waveforms | With motor execution disabled or externally controlled safely, capture timestamped raw levels and transitions across every A/Z scale in both travel directions. Compare all 29 runs and framing to supplied vector fixtures; record any encoding/checksum differences. Obtain the team's signed-off LEFT/RIGHT/STRAIGHT/U-TURN map; do not infer it from A/Z. |
| S3-M07 | Sample budget and speed | Measure worst-case period/jitter and task stack high-water mark with WiFi/USB/other buddies active. Minimum observed run must exceed the configured 40 ms budget with margin. Check max sample gap 30 ms, status age 30 ms and event age 100 ms. Record real velocity/narrow-width relation; do not treat open-loop PWM percent as measured speed. Check near-threshold/corrupted/partial waveforms produce no turn. |
| S3-M08 | Controller and motor handoff | Wheels raised first. Controller binds once, consumes events and refreshes its lease. Verify one request per marker and steering disabled during a queued/accepted turn. Buddy 2 alone executes motor output. Exercise queue full, controller crash/lease expiry, stale event, missed iteration and sensor unplug. Controller must command STOP or another approved authority when S3 steering is false. |
| S3-M09 | Line loss and bounded recovery | Start at low measured speed after raised-wheel checks. Cover line sensors: steering invalid immediately, one recovery request after 100 ms continuous loss, stop request after another 500 ms. No autonomous PWM from S3. Recover line during and after the search: controller acknowledgement is necessary to resume. Verify all-black/split patterns are treated as junction ambiguity. |
| S3-M10 | Override and marker clearance | Trigger emergency/obstacle override before, during and just after capture. No stale navigation event may execute after override/resume. Reverse or oscillate over the same marker: no duplicate until controller confirms physical clearance and the sensor sees quiet white. Validate the source of clearance evidence; a timer or quiet alone is insufficient. |
| S3-M11 | Marker spacing and turn timing | Measure approach/trailing blank surfaces. Default trailing quiet is max(200 ms, 4 x widest run), which adds travel before event delivery. Confirm marker-to-junction lead distance and adjacent-marker spacing at both scales and maximum tested speed. Reject operating conditions that cannot preserve framing/quiet zones. |

Use s3_controller_control(S3_MODE_OVERRIDE, false, 0) for software preemption,
check its return value, and retain an independent physical means of stopping the
vehicle. No hardware flashing or movement was performed by this change.
