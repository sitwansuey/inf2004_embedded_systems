# Navigation and avoidance settings

This guide describes the current mission code in `app_program/vehicle.c`,
`vehicle_policy.c`, `vehicle_approach.h` and `shared/avoidance_config.h`.
It replaces the earlier heading-assistance design note. See
[TEAM.md](../TEAM.md) for hardware ownership and interfaces.

The robot boots `DISARMED`. USB `r` then `g` requests a mission at 30% demand;
the dashboard's Reset robot then Start test uses 20%. With PID enabled,
demand sets wheel-speed targets. PWM limits, trims and breakaway kicks are
separate settings in `app_program/robot_config.h`.

## Avoidance sequence

1. During line tracking, two fresh forward echoes within 300 mm confirm an
   early approach. `vehicle_approach.h` uses odometry to drive on to the
   200 mm manoeuvre distance, stopping 20 mm early for sample age and
   braking, then holds still for up to `VC_APPROACH_HOLD_MS` (1500 ms) while
   S5 starts its scan; if no scan starts, it follows the line on. S5 begins
   acquisition after two forward echoes at or below 200 mm. These are
   control thresholds; the actual stopping distance must be measured.
2. The approach heading is saved when acquisition begins, before reversing.
   The robot stops while S5 scans. The coarse search uses 20°, 40° and 60°,
   then refines an edge to 1°. It checks the mirrored side and verifies a
   feasible path. Three settled no-return samples can count as open space;
   the range itself remains unavailable. A one-sided result is marked in
   S5's status.
3. If S5 finds no way round, or the obstacle is wider than
   `AVOID_MAX_WIDTH_MM` (500 mm), the controller can request two rescans
   from the same place (`AVOID_RESCAN_RETRIES`). Each new scan must start
   within `AVOID_RESCAN_WAIT_MS` (3000 ms). Exhausted or unusable scans stop
   the run with a sonar fault.
4. Once S5 has a valid plan, the controller waits for a fresh forward sample
   and reverses. The reverse ends after `AVOID_BACKUP_MS` (1000 ms) or
   `AVOID_BACKUP_MAX_MM` (300 mm), whichever comes first. It ramps power and
   waits for the robot to settle before turning out. There is no rear ranger;
   the test course needs clear space behind the robot.
5. A proposed shallower turn-out heading is checked with the robot stopped.
   A blocked or timed-out check keeps S5's verified heading. The robot follows
   the out diagonal, then stops to look toward the obstacle side. If needed,
   it aligns and passes in steps of `AVOID_PASS_STEP_MM` (100 mm), stopping
   for further side looks.
6. A side look is clear after three new samples in a row beyond
   `AVOID_SIDE_CLEAR_MM` (200 mm) or with no echo, and blocked after three
   nearer ones; no answer within `AVOID_SIDE_CHECK_TIMEOUT_MS` (4000 ms)
   stops the run. Before driving again, the servo returns forward and a
   clear forward sample is required. A clear side result then leads to a
   straight run of at least `AVOID_CLEAR_RUN_MM` (20 mm) and
   `AVOID_CLEAR_RUN_MS` (500 ms) of driving.
7. The robot turns back toward the line at an angle of at least
   `AVOID_RETURN_MIN_DEG` (45°). Line detection waits for
   `AVOID_TURN_IN_LINE_DELAY_MS` (1000 ms) of turn-in driving; for the next
   `AVOID_TURN_IN_FAR_MS` (500 ms), the far sensor alone can also count.
   It then drives straight toward the line until a sensor finds it, for
   at most the expected distance plus `AVOID_RETURN_EXTRA_MM` (250 mm) or
   `AVOID_RETURN_MAX_MM` (1000 mm), whichever is longer. It turns back to
   the approach heading and resumes tracking, or, if the line was not
   found, searches on the spot (±30°, then ±60°, within `VC_SEARCH_MS`,
   4000 ms).

The policy also bounds phase time, travel and sensor waits. TRACK has no
phase deadline; ACQUIRE and PASS have 30 s, TURN_OUT through RESTORE 10 s,
and the other timed phases 5 s. See the constants
and checks in `vehicle_policy.h`, `vehicle_policy.c` and
`shared/avoidance_config.h` before changing a limit.

## Width, heading and height

S5 keeps measured edge bounds for route planning. The displayed width uses
`s5_width_reported()`: each edge angle is reduced by `S5_BEAM_HALF_DEG`
(20°), floored at zero, before the tangent calculation. This corrects the
reported width for the sonar beam; it does not reduce the plan's clearance.
Telemetry and the obstacle-scanned event use that reported width.

`subsystem2/odometry.c` derives travel and yaw from wheel counts. The firmware
does not use a magnetometer heading. The saved magnetic calibration files
in this folder are historical measurements, not active heading settings.
Wheel slip therefore affects the estimated path.

S4's terrain code, which the controller runs on S4's sensor record,
estimates terrain state and the completed hump height from tilt and wheel
travel. `vehicle_hump.h` also estimates height from wheel distance and
smoothed pitch, with a correction for smoothing delay. Hump events identify
which estimate was used; the terrain telemetry height fields always come
from S4's terrain code. Neither path double-integrates acceleration to
obtain position.

On the USB console, `[APPROACH]` lines trace the early approach,
`[S5SCAN]` and `[S5MOTION]` lines the scan, and `[VC] no way round` each
rescan.

See [DATA_REPRESENTATION.md](../docs/DATA_REPRESENTATION.md) for event kinds and
units. Software checks are in `testing/integration/`; hardware procedures,
results and saved evidence are in [testing/](../testing/README.md).
