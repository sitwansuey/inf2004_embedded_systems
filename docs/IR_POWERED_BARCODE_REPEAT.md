# A barcode repeat — manual dragging during commanded motor run, 24 October 2026, 21:37

The operator subsequently clarified that they manually dragged the robot during this
recording. The host had sent a 12% PWM START command with a 6000 ms maximum to the
already-installed IRMotorScan v3. This was NOT a controlled motor-only traverse.
Whether motor battery power was physically enabled was not separately confirmed.
The original host result and raw log remain unchanged. No firmware,
calibration, threshold, decoder guard or source logic was changed. All work remains local.

## Result

- The controller reported a forward command and nonzero requested motor power, and
  encoder counts changed. Those observations do not distinguish manual movement from
  powered wheel movement. The controller became inactive after 420 ms because the
  onboard decoder returned NOISE (status 3). This was not a completed six-second traverse.
- GP28 crossed the BLACK hysteresis threshold at t=2267110 ms (raw 1712), then returned
  to WHITE at t=2267120 ms (raw 256). The first BLACK-like interval was one 10 ms sample,
  below the existing 40 ms minimum. No barcode element was accepted before rejection.
- All 806 recorded raw samples were 10 ms apart, valid mask 7, with zero dropped records.
  Mask 7 establishes ADC acquisition validity only, not physical line-sensor presence.
- Production PC replay reproduced the onboard NOISE at exactly the same timestamp.
  The final explicit STOP was acknowledged with motor state/command/power all zero.
- No A decoded. Physical stopping distance, braking and a full path across the card
  were not validated. No automatic retry was sent.

The earlier statement that this reproduced the pulse independently of manual dragging
was incorrect and is withdrawn. Manual speed was not controlled. The cause remains
unresolved: this recording does not distinguish a real
bar crossed briefly from optical contrast, geometry, alignment or vibration effects.
PWM percentage is not calibrated surface speed; wheel circumference remains unspecified.
The operator's previously reported sticking caster and drift remain physical limitations.

The installed motor maximum remains six seconds. The older ten-second motor candidate
is still NOT installed. Motor power availability after this run was not separately confirmed;
only the commanded STOP state is verified. No further movement is queued.

## Evidence and checks

Evidence: `evidence/2026-10-24/ir_motor_repeat_213738/`, including full USB log,
host result, original-timestamp CSV, native decoder trace and analysis JSON.
Earlier manual results are preserved in `IR_MANUAL_ROBOT_PASS.md`.
No executable source changed, so native/ARM builds were not repeated for this capture.
Source hashes and cached remote refs are checked against the existing local records;
`git diff --check` is run after this report update. No push, fetch, PR or remote changes.
