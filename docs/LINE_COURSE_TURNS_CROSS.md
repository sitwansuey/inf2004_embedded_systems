# Turn and cross handling — local v5 candidate, 25 October 2026

The user confirmed both BLACK occurred at a turn and that the track contains
a cross; the route at the cross is straight. v4/50% is installed and STOP
verified. The v5 candidate is not installed and no motion is queued.

The old immediate both-BLACK STOP was incomplete for this course. In v5 the
line-only controller adds bounded temporal handling, without new sensors:

- WHITE/WHITE: assumed-centre forward, as before.
- One BLACK: pivot toward that side, as before, while collecting side history.
- Both BLACK after at least 60 ms continuously on one side: hold that turn.
- Both BLACK after centred, brief or alternating readings: continue straight
  through the crossing-like band, ignoring transient side edges within it.
- A phase ends after 30 ms of fresh, strong WHITE on both sensors. Repeated
  use of one old sample cannot satisfy this condition.
- A crossing-like band lasts at most 500 ms; a held turn at most 800 ms.
  Unresolved phases stop with COURSE_TIMEOUT. WHITE flicker does not renew
  these bounds. Explicit STOP/disarm resets manoeuvre history.

This uses history as a hypothesis: two sensors cannot uniquely distinguish
every corner, skewed cross, broad dark patch, or off-track WHITE region.
Straight is the default crossing route, not a claim of certified junction
classification. Thresholds are bench time bounds, not measured distances or
angles. Real turns/crosses must be tested at the current height and power.

50% PWM and 20-second run maximum remain; USB lease 250 ms, motor watchdog
150 ms, encoder no-progress 750 ms, ADC freshness 30 ms and urgent STOP are
retained. Ultrasonic remains disabled. Barcode/GP28 is excluded. Motion still
passes through the existing Vehicle Controller and Buddy 2 motor owner.
No normal-profile behaviour, calibration, GPIO ownership or kernel changes.
Application message sizes are unchanged; new phase/history are controller-local.

v5 identity/course=1/cross=straight is required by the host before a line run.
Legacy v2 and v4 helpers/images are not accepted by this v5 helper.
Native, sanitizer, integration and ARM checks are pending.

Installed baseline evidence: docs/evidence/2026-10-25/line_only_v4_50pct.
At 50% the raised 200 ms pulse passed (operator confirms both forward and
stopping). Its first floor run made left/right corrections and both encoders
advanced, then stopped after 900 ms solely on both BLACK. The user reports
it remained on the sheet. This is not a floor-tracking pass or caster diagnosis.
The school's supplied dimensional sheet defines 18 mm line, 20 mm gap and
141 mm barcode; it does not prescribe a junction route. Straight at the cross
is the explicit user instruction. No school corner radius/turn speed invented.

All changes remain local; no push, PR or remote branch change.
