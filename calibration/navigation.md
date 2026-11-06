# Monitored avoidance and heading assistance — 2026-11-06

The full course build starts disarmed. Dashboard/USB Start requests30;
actual PWM is capped at30%. The supplied groupmate calibration uses fractional
PI, trim74/78, feedforward offsets3/4 and gains20/10/0. A separate active
trial overrides the right forward target factor from970 to965 for slight
left drift. Nominal targets are500 left /482 right counts/s at command30.
The source snapshot header is preserved unchanged.

## Avoidance sequence

1. A measured forward obstacle at or below200mm stops line tracking.
2. Reverse100mm at a command no higher than15; stop if this is not completed
   within2500ms. There is no rear ranger: provide clear space behind the robot.
3. Explicitly start a fresh scan after the retreat. Its maximum acquisition
   distance is320mm so backing away does not create a reverse/approach loop.
   The ordinary forward obstacle trigger remains200mm.
4. Check30/60/90 degrees on each side, refine the edges and verify a feasible
   heading. Three settled no-return readings count as clear, per user policy.
5. Bend outward and drive for three seconds of commanded motion, with periodic
   obstacle-side observations. Observation pauses do not consume driving time.
6. Align parallel to the original approach heading. Continue past the obstacle
   while alternating front checks and side observations. The servo pans while
   stopped: one ranger cannot observe forward and side directions simultaneously.
7. Require three distinct consecutive clear side observations beyond the
   original front-face distance plus100mm before turning back. A measured
   side obstacle resets the clear count. Side range above600mm or confirmed
   no-return indicates clear space. Phase time/distance limits remain active.
8. Counter-steer toward the saved path using encoder displacement and the
   assisted heading estimate. Allow for the final restoring arc. Begin IR
   search only within30mm estimated lateral offset and3degrees estimated
   heading, with at most two corrective approaches. Invalid pose stops motion.

## IMU calibration and limits

The LSM303 supplies acceleration and magnetic field, without a gyroscope.
Encoder travel and heading are integrated into local XY position; acceleration
is not integrated twice to obtain displacement. The saved path reference is
captured before reversing.

The assembled robot's223-sample magnetic sweep fitted offsets365,-100,-155mG
and diagonal scales959,953,1102 per mille. Upright manual quarter-turn checks
measured relative headings-97.21,174.50,89.15degrees; return-to-zero differed
by3.59degrees. The largest intended-angle discrepancy was7.21degrees. These
are manual static checks, not precision or moving-course acceptance.

`imu_heading_calibrated_20261106.h` enables guarded compass assistance. Fresh
magnetic heading is tilt compensated using gravity and blended gently with
encoder heading. Stale data, unusual acceleration, field changes over20% or
heading disagreement over15degrees reject magnetic correction. Encoders still
provide distance. Wheel slip and motor magnetic interference can affect the
estimate, so moving-course validation remains required. Terrain/height and
accelerometer bias/scale calibration are separate and remain unchanged.

Console records: `[POSE]`, `[S5SCAN]`, `[S5MOTION]`, `[S2PI]`, `[DEMO SENSOR]`.
The dashboard also names the new Backing up phase. MQTT sensor records remain
the existing two-byte EDR16 format. Audio Pi remains disabled.
