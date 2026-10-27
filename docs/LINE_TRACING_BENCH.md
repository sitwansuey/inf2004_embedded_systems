# Line tracing bench - corrected WHITE-centred geometry

On24 October2026 the user corrected the geometry: GP27 is the LEFT sensor,
GP26 the RIGHT sensor, and BOTH see WHITE when the robot is centred, with
the18 mm black line between them. The earlier both-BLACK candidate v1 was
never installed and is superseded. Do not use its behaviour table or artifact.
IRLineTrace v2 is now installed on the operator-selected robot RPI-RP2 D:.
UF2/ELF/source checks passed before copy. Normal USB identifies v2 and gives
a fresh STOP acknowledgement. Installed artifact SHA256:
bbb9f0bdb891703f9b1fe7d870b219d1abde52a256ab3e78ac42b7b93d4342a5.
This is verified copy plus runtime identity, not a physical flash-readback hash.

## Corrected supervised bench behaviour

| GP26 RIGHT | GP27 LEFT | Output during an explicitly started timed run |
|---|---|---|
| WHITE | WHITE | Forward, assuming the operator positioned it over the line |
| BLACK | WHITE | Pivot right: left wheel forward, right stopped |
| WHITE | BLACK | Pivot left: right wheel forward, left stopped |
| BLACK | BLACK | STOP: ambiguous wide-black/junction observation |

WHITE/WHITE also occurs with the robot completely off the line on white
ground. These two measurements cannot distinguish those positions. Thus
there is NO reliable automatic line-loss detection in this bench profile;
WHITE/WHITE must not be published as proof of line presence. Both-black
STOP is labelled LINE_AMBIGUOUS, not a detected lost line or confirmed junction.
GP28 remains barcode-only and does not provide an extra line-presence channel.
Automatic junction/navigation/terrain/obstacle bypass is not enabled here.

Production Buddy3 normalization/hysteresis remains unchanged. The ADC sampler,
motor/encoder driver and peripheral owners remain unchanged. The replacement
Vehicle Controller uses the existing priority5/core0 task. The integrated
autonomous controller and generic Buddy3 junction classifier are unchanged.
No sensor endpoints, barcode algorithm or calibration were changed.

Boot STOPPED; PREPARE alone cannot move. RUN requires100 ms of fresh,
strong WHITE on both line sensors, healthy motor status and a fresh forward
ultrasonic reading greater than250 mm. This WHITE check is electrical/optical
readiness only. The host additionally requires explicit CentredOnTrack,
LineInputsVerified, DirectionVerified and ClearFloor setup flags for Trace.
They record operator setup, not an independent measurement of line position.

PWM remains capped at20%, host default8%; default timed run1000 ms,
maximum6000 ms. PWM is not measured wheel speed. USB lease250 ms, independent
motor watchdog150 ms, ASCII s urgent STOP and30 ms sample freshness remain.
Missing/stale inputs, motor/clock/USB failure, blocked/stale forward range,
both-black ambiguity or run expiry stop the bench, with no automatic restart.
WHITE-only departure from the line may remain undetected until a supervisory
stop/expiry. Powered trials must be short and supervised on a clear floor,
away from edges. The rough caster affects interpretation. No physical trace yet.

## Checks and measurements

- Corrected native line policy:355 assertions, zero failures.
- AddressSanitizer/UndefinedBehaviorSanitizer:355 checks, no reports.
- Existing integration suite passed, including339 motor policy checks,
  STOP/USB/inbox ownership, sensor snapshots, telemetry and MQTT fixtures.
- Corrected operator reporting:10 checks pass. Previously validated passive
  input tool24 checks and profile guards10 checks remain applicable; these
  tools/config guards did not change. Existing Buddy3 suite passed before
  this correction; no Buddy3 algorithms/calibration changed in this correction.
- Forced full ARM build and RP2040 UF2/ELF verification pass. No movement during those build checks; installation is recorded below.

| Bytes | Previous V5 baseline | Corrected v2 candidate |
|---|---:|---:|
| text |313900|313916|
| data |160|160|
| bss |25436|25436|
| linked static RAM |25596|25596|
| UF2 file |628224|628224|
| binary telemetry message |50|50|

Runtime heap and stack occupancy are unmeasured. The candidate's flash
text is16 bytes larger than installed V5 and40 bytes larger than the
superseded v1 line candidate. Static RAM/message size remain unchanged.
UF2 SHA256: bbb9f0bdb891703f9b1fe7d870b219d1abde52a256ab3e78ac42b7b93d4342a5

## Latest stationary inputs: PASS,2418 samples

Using the wide18 mm track and fixed flat backing, three complete labelled
stationary recordings pass with unchanged production calibration:

| Setup | Samples | GP26 right raw | GP27 left raw | Mismatches/gaps |
|---|---:|---|---|---|
| RIGHT BLACK / LEFT WHITE |798|3420..3446|177..228|0/0|
| LEFT BLACK / RIGHT WHITE |808|206..233|2941..2985|0/0|
| Both WHITE |812|206..232|176..228|0/0|

The aggregate and complete recordings are retained in
docs/evidence/2026-10-24/line_trace_v2/LINE_INPUTS_VALIDATION.json. No endpoints
were changed and no motor command was sent. Earlier failed recordings remain
retained: they included changing readings and do not count as passed holds.
The user cannot place both sensors over the same black line; no simultaneous
BLACK physical check is required for these three input states. Both-black
ambiguity/STOP remains covered by host policy tests, not a physical claim.

Installation and normal USB Inspect succeeded. One operator-confirmed
raised-wheel200 ms pulse at8% PWM completed with final STOP acknowledged;
the user observed both wheels turning forward. This verifies forward direction
for that pulse, not physical line tracking or steering direction.
Next: raised-wheel closed-loop steering in host Steer mode, which requires
WheelsRaised, LineInputsVerified and DirectionVerified. It starts only after
v2/healthy input/fresh forward clearance/WHITE readiness checks and a START
acknowledgement, then beeps for a single target slide. Time/PWM/heartbeat limits
are unchanged. It does not assert that the wheels stayed on a floor course.
The pulse's forward-clearance flag was0; provide a flat target about60 cm
ahead for a fresh valid sonar observation before a line RUN. No override was made.

The caster remains rough. The user expects feedback to correct the drift;
steering can compensate only within the available traction/control response.
Any short supervised floor test must record caster-induced drift/stall as an
unresolved mechanical factor rather than claiming calibrated tracking accuracy.
No floor movement is queued. Keep it away from edges with cable slack.

Build-only: INSTALL_LINE_TRACE.ps1 -PicoSdkPath <SDK> -PythonPath <Python>.
Explicit -Flash is required to copy. RUN_LINE_TRACE.ps1 defaults to Inspect;
v2 identity is mandatory so the old v1/profile is refused before START.

Local branch local/csc02-line-tracing-20261024. Prior user edits are preserved.
No commits, pushes, PRs, fetches, remote edits or CR2/CR3 changes were made.
