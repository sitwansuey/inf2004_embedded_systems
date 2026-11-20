# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | IF-01, IF-02 | auto | `subsystem3_test.c` | `[TEST] INT-01 PASS` |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | IF-03 | auto | `subsystem4_test.c` | `[TEST] INT-02 PASS` |
| INT-CR1-08 | Original Buddy 5 signal roles and production sonar driver | S5 GPIO/timer and sensor status | native | `s5_hardware_test.c` via integration Makefile | correct roles pass; reversed roles fail |
| INT-CR1-09 | Preempted status publisher and failure propagation | S5 snapshot/control API | native | `s5_status_test.c` | bounded wait yields, poll fails; no stale-copy success |
| INT-CR1-10 | Snapshot timestamps during bounded reads | telemetry freshness and codec | native | `snapshot_time_test.c` | age reference follows read; stale/error data unavailable |
| INT-CR1-01 | Controller policy (`vehicle_policy.c`): command priority, lease expiry, stale data, bounded bypass, STOP handoffs; sensor waits and phase deadlines; avoidance power ramps at 5/10/20 ms ticks | VC -> S2, network/USB -> VC; FR1, FR5, NFR1 | auto (host) | `python -B testing/integration/run.py`, recipe `policy` (`policy_test.c`) | `INT-CR1-01 controller: ... PASS`, `Sensor waits: ... PASS`, `Avoidance ramps: ... PASS` |
| INT-CR1-02 | Odometry from S2 encoder snapshots (`odometry.c`): count wrap, scale, speed, yaw, gaps, discontinuities | S2 -> VC/S1, S2 -> S4 in VC; FR4, FR10 | auto (host) | `python -B testing/integration/run.py`, recipe `policy` (`policy_test.c`) | `INT-CR1-02 odometry: ... PASS` |
| INT-CR1-03 | Terrain algorithm on IMU samples (`terrain.c`): calibration gate, ordered hump, maximum, impact, invalid samples; IMU mount axis remap, saturation, pitch and roll | S4 -> VC/S1; FR2, FR3, FR10 | auto (host) | `python -B testing/integration/run.py`, recipe `policy` (`policy_test.c`) | `INT-CR1-03 terrain: ... PASS`, `IMU mount: ... PASS` |
| INT-CR1-04 | S5 scan (`s5_scan.c`, `s5_policy.c`, `s5_width.c`): 3 near echoes start it, angle association, failed echo, moving scan, geometric bounds; width, side choice, verify fallback, edge flicker and edge check, side corridor, braking, servo slope | S5 -> VC/S1; FR5, FR6, NFR6 | auto (host) | `python -B testing/integration/run.py`, recipe `scan` (`scan_test.c`) | `INT-CR1-04 scan: ... PASS` and nine `S5 ...: ... PASS` lines (width, side choice, verify fallback, edge flicker, scan geometry, edge check, side corridor, braking, servo slope) |
| INT-CR1-05 | Telemetry wire format: the C codec (`telemetry_codec.c`) and the Python decoder agree on the minimum, maximum and 300 random snapshots; out-of-range values and a bad length, version or fault are rejected | VC -> S1, S1 -> network; FR12, NFR7 | auto (host) | `python -B testing/integration/run.py`, recipe `protocol` (`protocol_cli.c`, then `protocol_test.py` with `snapshot_decoder.py`) | `INT-CR1-05 protocol: 302 C/Python vectors, 90 bound rejections, ... PASS`; `git status` shows no change to `protocol_golden.json` |
| INT-CR1-06 | S1 queues in the real `telemetry.c`: command copy, full queue, urgent STOP, stale telemetry, validation, 14-byte S3 events; robot event queue (16 deep, drops counted); drop counter wrap; newest-first snapshots; command replies | network/USB -> VC, VC -> S1; FR12, NFR7, NFR8 | auto (host) | `python -B testing/integration/run.py`, recipe `queue` (`telemetry_queue_test.c`) | `INT-CR1-06 actual queues: ... PASS`, then `Robot event queue`, `Drop counter`, `Snapshots` and `Command replies` lines ending in PASS |
| INT-CR1-07 | MQTT port (`mqtt_utk.c` with `mocks/`): binary length and NULs, fragments, malformed input, backpressure, driving does not depend on the link, retry guard; command replies; abandoned snapshot; robot events held through a link drop; Wi-Fi re-join timing | S1 -> network, network/USB -> VC; FR12, NFR8 | auto (host) | `python -B testing/integration/run.py`, recipe `mqtt` (`mqtt_test.c`) | `INT-CR1-07 actual MQTT port: ... PASS`, then `MQTT replies`, `Abandoned snapshot`, `MQTT robot events` and `Wi-Fi re-join` lines ending in PASS |
| INT-S4-11 | Real controller task (`vehicle.c`) with S2 and S4 inputs: encoded getter errors, future samples, IMU skew, reset, motor STOP | S2 -> S4 in VC, S4 -> VC/S1; FR3, NFR2, NFR3 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | `INT-S4-11 actual vehicle path: ... PASS` in all five recipes |
| INT-12 | S5 width to the VC bypass plan and the EDR16 width record: width-based legs, bounded reverse, side look, pass in steps, mirrored return, restore and spot search, 3 fresh side readings, sonar gap and pan stops | S5 -> VC/S1, VC -> S1; FR5, FR6, FR7, NFR6 | auto (host) | `python -B testing/integration/run.py`, recipe `width_avoidance` (`width_avoidance_test.c`) | `PASS: estimated width -> EDR16; ...` |
| INT-13 | S2 speed PI used for VC demands: fractional corrections kept, rounding, anti-windup at saturation, steered demand capped | VC -> S2; FR4 | auto (host) | `python -B testing/integration/run.py`, recipe `smooth_motor` (`smooth_motor_test.c`) | `PASS: fractional PI corrections retained; ...` |
| INT-14 | VC demand at 30% through S2 steering: straight demand, feedforward, mirrored 30/18 bend targets, every steering target and PID output at most 30% | VC -> S2; FR4, FR8 | auto (host) | `python -B testing/integration/run.py`, recipe `speed30` (`speed30_test.c`) | `PASS: 30-percent straight demand; ...` |
| INT-15 | Obstacle met on a bend (one line sensor on black): keeps the last side, bounded straight reverse, mirrored left/right bypass, side look, return, restore, spot search; STOP, sonar gap and disabled bypass stop | S5 -> VC/S1, VC -> S2, VC <-> S3; FR5 | auto (host) | `python -B testing/integration/run.py`, recipe `obstacle_bend` (`obstacle_bend_test.c`) | `PASS: both-black keeps the last side; ...` |
| INT-16 | Bypass return (`vehicle_policy.c`): diagonal side look, clear run, direct return, align and pass, return until the line is under the robot, restore the detection heading, TRACK or spot search; line-data and sonar gap holds | S5 -> VC/S1, VC <-> S3; FR1, FR5 | auto (host) | `python -B testing/integration/run.py`, recipe `bypass_return` (`bypass_return_test.c`) | `Bypass return: ... PASS` and no `FAIL` line |
| INT-17 | Reverse barcode re-read steering (`vehicle_reverse_track.h` with S2 steering): heading and line terms, heading wrap, closed-loop drift | VC <-> S3, VC -> S2; FR9 | auto (host) | `python -B testing/integration/run.py`, recipe `reverse_track` (`reverse_track_test.c`) | `reverse_track: ok (worst sensor offset ... mm, worst heading ... deg)` and exit code 0 |
| INT-18 | Line and barcode-read steering (`vehicle_line_steer.h`): first correction, ramp, cap, pivot; the `robot_config.h` defaults and the S2 slew rate | VC <-> S3, VC -> S2; FR8, FR9 | auto (host) | `python -B testing/integration/run.py`, recipe `line_steer` (`line_steer_test.c`) | `line_steer: ok` and exit code 0 |
| INT-19 | Obstacle confirmation from S5 samples (`vehicle_obstacle_confirm.h`): 3 new near samples in a row, a re-read counts once, any other sample resets | S5 -> VC/S1; FR5, NFR1 | auto (host) | `python -B testing/integration/run.py`, recipe `obstacle_confirm` (`obstacle_confirm_test.c`) | `Obstacle confirmation: ... PASS` |
| INT-20 | Barcode reverse re-read (`vehicle_barcode_retry.h`): the reverse runs past the first bar plus the run-up; an early noise count does not stop it | VC <-> S3; FR9 | auto (host) | `python -B testing/integration/run.py`, recipe `barcode_retry` (`barcode_retry_test.c`) | `barcode_retry: ... PASS` |
| INT-21 | Snapshot to EDR16 records to the dashboard: 33 two-byte records from `telemetry_codec.c` are decoded by the dashboard bridge (`dashboard.py`) into the expected values; the browser card check also runs when Node.js is on PATH | VC -> S1, S1 -> network; FR12, FR13, NFR7 | auto (host) | `python -B testing/integration/run.py`, recipe `edr_stream` (`edr_stream_test.c`, then `edr_stream_test.py`) | `EDR16 production snapshot through Python/dashboard PASS` |
| INT-22 | Mission handoffs (`vehicle_policy.c`, `buddy3_barcode.c`): obstacle priority over a deferred turn, turn at the junction, TRACK acknowledgement, timeout and wrap, STOP; autonomous START, bounded reacquisition, PID limits, barcodes A-D forward and reverse | VC <-> S3, network/USB -> VC; FR1, FR9, NFR1 | auto (host) | `python -B testing/integration/run.py`, recipe `mission` (`mission_test.c`) | `Mission handoffs: ... PASS` and `Mission: autonomous start, ... PASS` |
| INT-23 | Barcode turns and line search (`vehicle_policy.c`): turn at the next junction after 90 mm, 600 mm search limit, U-turn, line-guided turns, line lost while tracking, spot search limits | VC <-> S3; FR1, FR9 | auto (host) | `python -B testing/integration/run.py`, recipe `nav_junction` (`nav_junction_test.c`) | nine lines ending in PASS, from `Right turn waits through tracking ...` to `Spot search held by the sonar ...` |
| INT-24 | Straddle line observer (`ir_line_trace.c` with the S3 configuration) set up as `vehicle.c` does: GP26 right, GP27 left, measured endpoints, black/white hysteresis; IR mode 3 and IR mode 2 | VC <-> S3; FR8, NFR4 | auto (host) | `python -B testing/integration/run.py`, recipes `line_trace` and `line_trace_mode2` (`line_trace_test.c`) | `Straddle line observer (IR mode 3): <n> checks, 0 failures`, and the same line for IR mode 2 |
| INT-25 | Watchdog supervisor decision (`shared/watchdog_policy.c`) on the five heartbeats (VC, motor, network, telemetry, IMU): no arming before the 20 s grace, the first network join and every loop running; feeds while all are fresh; a loop stale for 1 s trips once and is named; crash record codec; early-crash count and hold | Watchdog heartbeat path (`app_watchdog.c`); NFR2 | auto (host) | `python -B testing/integration/run.py`, recipe `watchdog` (`watchdog_test.c`) | `Watchdog supervisor: ... PASS`, `Crash record: ... PASS`, `Early crashes: ... PASS` |
| INT-26 | S5 scan rejects robot motion (encoder counts in the S5 status): 1-3 counts tolerated, a larger move recentres and retries twice, retry budget kept, encoder wrap | S5 -> VC/S1, S2 -> VC/S1; FR5, FR6 | auto (host) | `python -B testing/integration/run.py`, recipe `obstacle_motion` (`obstacle_motion_test.c`) | `PASS: 1-3 encoder counts tolerated; ...` |
| INT-27 | S5 scan with open space beyond the object: clear after 3 settled samples, the open side planned with one edge mirrored, no plan when neither side is open, READY check, forward bypass guidance | S5 -> VC/S1; FR5, FR6 | auto (host) | `python -B testing/integration/run.py`, recipe `obstacle_open_space` (`obstacle_open_space_test.c`) | `PASS: no-rise/no-fall clear after 3 settled samples; ...` |
| INT-28 | S5 edge search: 20/40/60 deg coarse search, bisection, mirror check, outward search, one-sided width flagged, hard echo faults count as the object, edge flicker retreat | S5 -> VC/S1; FR6, NFR6 | auto (host) | `python -B testing/integration/run.py`, recipe `obstacle_scan` (`obstacle_scan_test.c`) | `PASS: 20/40/60 deg coarse search, ...` and `PASS: an edge flicker while averaging ...` |
| INT-29 | Real `vehicle.c` with S3 snapshots and events: straddle barcode-read steering, mask-pivot follower (edge tracing in `buddy4_vehicle_edge_trace`), barcode first pass and reverse re-read, silent S3 latch and TRACK re-send, held marker, line lost while tracking, lifted robot (GP28) | VC <-> S3; FR1, FR8, FR9 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | in each recipe these lines end in PASS: `Straddle barcode read`, `Mask-pivot follower` (or `Edge tracing`), `Barcode retry`, `Barcode first pass`, `Silent S3 latch`, `Marker cut short`, `S3 holding a decoded marker`, `Line lost ...`, `Lifted ...` |
| INT-30 | Real `vehicle.c` with S5 status: obstacle while tracking (3 new near echoes), obstacle found through S5's real scan step, sonar gap hold, S5 lock busy across a phase change, avoidance ended by STOP | VC -> S5, S5 -> VC/S1; FR5, NFR1 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | in each recipe these lines end in PASS: `Obstacle while tracking`, `Obstacle detected through S5's real step`, `Sonar gap`, `S5 lock busy`, `Avoidance ended by a STOP` |
| INT-31 | Commands into the real `vehicle.c` (the same records as USB `a`/`g` and MQTT): MAP in any phase, ARM lease renewal, renewal limits, STOP during renewals, RESET | network/USB -> VC; FR1, FR12 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | in each recipe these lines end in PASS: `MAP command`, `ARM lease renewal`, `ARM renewal limits`, `STOP during renewals` |
| INT-32 | Real `vehicle.c` motion requests to S2 and S2 status back: line follow/mask/steer match the S2 request every tick; short timing faults (odometry gap, failed motor status copy and fault 7, busy S3 lock, command stamped a tick ahead); loop timing on the hardware timer | VC -> S2, S2 -> VC/S1; FR1, FR4, NFR5 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | in each recipe these lines end in PASS: `Straddle app_state line_follow/line_mask/line_steer`, `Short timing faults ride through`, `Loop timing` |
| INT-33 | Real `vehicle.c` with S4 records, and its events to S1: hump detected after 400 ms, crest power cap, level restore (no events and no cap with `BOARD_RAMP_SPEED 0`); shock published and counted (stops the run with `BOARD_SHOCK_STOP 1`); 8c counter-wrap regressions; EV_BARCODE before EV_INSTRUCTION | S4 -> VC/S1, VC -> S1; FR2, FR3, FR7, FR10 | auto (host) | `python -B testing/integration/run.py`, recipes `buddy4_vehicle`, `buddy4_vehicle_no_ramp_lift`, `buddy4_vehicle_shock_stop`, `buddy4_vehicle_edge_trace`, `buddy4_vehicle_read150` (`buddy4_vehicle_test.c`, includes `vehicle.c`) | these lines end in PASS: `Hump: ...` (`Hump with BOARD_RAMP_SPEED 0` in `buddy4_vehicle_no_ramp_lift`), `Shock while tracking`, `Event log`, `8c: ...` (not in `buddy4_vehicle_no_ramp_lift`) |
| INT-34 | VC and S4 state in the S1 snapshot (`task_telemetry.c`): line state, mask and error; fault cause (VC_SENSOR, VC_LINE, fault 8); a new shock sent at once; IMU fault held for the next snapshot; at most one shock extra per 200 ms; IMU fault count | VC -> S1, S4 -> VC/S1; FR3, FR7, FR11, NFR7 | auto (host) | `python -B testing/integration/run.py`, recipe `snapshot_time` (`snapshot_time_test.c`) | six lines end in PASS before the `INT-CR1-10` line: `Straddle line telemetry`, `Fault cause`, `Shock count`, `IMU fault`, `Shock extras`, `IMU fault count` |
| INT-35 | Watchdog heartbeat path on the robot: all five loops beat and the supervisor arms the watchdog | Watchdog heartbeat path (`app_watchdog.c`); NFR2 | manual | 1. Build the normal image (`make -j8` in `build_make`) and flash it. 2. Keep the Wi-Fi access point in range. Open the USB console and reset the Pico. 3. Watch the console for 60 s. | `[WDT] armed: every monitored loop is running` appears once, 20-60 s after reset. No `[WDT] not armed yet` line. |
| INT-36 | USB console commands reach the VC (`usb_control.c` -> S1 command inbox -> VC) | network/USB -> VC; FR1 | manual | 1. Flash the normal image. Put the robot on a straight part of the line. Open the USB console. 2. Press `a` once. 3. Press `g`, then press `s` about 1 s later. | Step 2: the robot drives for about 0.5 s and stops by itself (500 ms ARM lease). Step 3: the robot starts on `g` and stops at once on `s`. |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

**auto (host)** = a host program in `testing/integration`. Run `python -B testing/integration/run.py` from the
repository root (needs a native `gcc` on PATH, or `--compiler <path to gcc>`).
On Linux, macOS or CI, `make -C testing/integration test` runs the same recipes.
The older rows INT-CR1-08 to INT-CR1-10 say **native**: that is the same host run.

## How to run the host integration tests

1. From the repository root, run `python -B testing/integration/run.py`.
2. Save the whole console output in `logs/`, for example `logs/YYYY-MM-DD_host_integration.txt`.
3. PASS needs all of these: exit code 0, the last line `Integration recipes PASS`,
   every line named in *Expected result*, and no `FAIL` or `Assertion` text in the log.
4. `protocol_test.py` rewrites `protocol_golden.json`. After the run, `git status` must show no change to it.
5. Optional (Linux or CI): `make -C testing/integration sanitize` runs the same recipes with ASan and UBSan.

## Host recipes and their IDs

The `test` target in `testing/integration/Makefile` has 33 recipes. `run.py` reads the same list.
Only the IDs marked *printed* appear in the output. For the others, the PASS line in *Expected result* identifies the check.

| Recipe (`.build/...`) | Program | Test IDs |
|---|---|---|
| `width_avoidance` | `width_avoidance_test.c` | INT-12 |
| `smooth_motor` | `smooth_motor_test.c` | INT-13 |
| `speed30` | `speed30_test.c` | INT-14 |
| `obstacle_bend` | `obstacle_bend_test.c` | INT-15 |
| `bypass_return` | `bypass_return_test.c` | INT-16 |
| `reverse_track` | `reverse_track_test.c` | INT-17 |
| `line_steer` | `line_steer_test.c` | INT-18 |
| `obstacle_confirm` | `obstacle_confirm_test.c` | INT-19 |
| `barcode_retry` | `barcode_retry_test.c` | INT-20 |
| `edr_stream` | `edr_stream_test.c`, `edr_stream_test.py` | INT-21 |
| `mission` | `mission_test.c` | INT-22 |
| `nav_junction` | `nav_junction_test.c` | INT-23 |
| `line_trace`, `line_trace_mode2` (`-DBOARD_IR_MODE=2`) | `line_trace_test.c` | INT-24 |
| `policy` | `policy_test.c` | INT-CR1-01, INT-CR1-02, INT-CR1-03 (printed) |
| `watchdog` | `watchdog_test.c` | INT-25 |
| `obstacle_motion` | `obstacle_motion_test.c` | INT-26 |
| `obstacle_open_space` | `obstacle_open_space_test.c` | INT-27 |
| `obstacle_scan` | `obstacle_scan_test.c` | INT-28 |
| `scan` | `scan_test.c` | INT-CR1-04 (printed) |
| `sonar`, `sonar_low_centre` (`-DBOARD_SERVO_CENTER_US=1300`), `sonar_slope` (`-DBOARD_SERVO_US_PER_90=1000`) | `s5_hardware_test.c` | INT-CR1-08 (printed by all three) |
| `sonar_status` | `s5_status_test.c` | INT-CR1-09 (printed) |
| `snapshot_time` | `snapshot_time_test.c` | INT-CR1-10 (printed), INT-34 |
| `protocol` | `protocol_cli.c`, `protocol_test.py` | INT-CR1-05 (printed) |
| `queue` | `telemetry_queue_test.c` | INT-CR1-06 (printed) |
| `mqtt` | `mqtt_test.c` | INT-CR1-07 (printed) |
| `buddy4_vehicle_no_ramp_lift` (`-DBOARD_RAMP_SPEED=0 -DBOARD_LIFT_FAULT_MS=0U`), `buddy4_vehicle`, `buddy4_vehicle_shock_stop` (`-DBOARD_SHOCK_STOP=1`), `buddy4_vehicle_edge_trace` (`-DBOARD_LINE_MASK_PIVOT=0 -DBOARD_LINE_EDGE_TRACE=1`), `buddy4_vehicle_read150` (`-DBOARD_BARCODE_READ_LINE_STEER=150`) | `buddy4_vehicle_test.c` | INT-S4-11 (printed by all five), INT-29 to INT-33 |
| not in the `test` target | `groupmate_motor_test.c` | none (not run by `run.py` or `make test`) |

## Interface -> test IDs

One row per interface in TEAM.md, plus the watchdog heartbeat path. The owners on both sides sign off the result.

| Interface | Owners | Test IDs |
|---|---|---|
| VC -> S2 | VC (team), S2 2500613 | INT-CR1-01, INT-13, INT-14, INT-15, INT-17, INT-18, INT-32 |
| S2 -> VC/S1 | S2 2500613, VC (team), S1 2500238 | INT-CR1-02, INT-26, INT-32 |
| S2 -> S4 in VC | S2 2500613, S4 2500243, VC (team) | INT-CR1-02, INT-S4-11 |
| VC <-> S3 | VC (team), S3 2500383 | INT-15, INT-16, INT-17, INT-18, INT-20, INT-22, INT-23, INT-24, INT-29 |
| S4 -> VC/S1 | S4 2500243, VC (team), S1 2500238 | INT-CR1-03, INT-S4-11, INT-33, INT-34 |
| VC -> S5 | VC (team), S5 2502324 | INT-CR1-09, INT-30 |
| S5 -> VC/S1 | S5 2502324, VC (team), S1 2500238 | INT-CR1-04, INT-CR1-08, INT-CR1-09, INT-12, INT-15, INT-16, INT-19, INT-26, INT-27, INT-28, INT-30 |
| VC -> S1 | VC (team), S1 2500238 | INT-CR1-05, INT-CR1-06, INT-CR1-10, INT-12, INT-21, INT-33, INT-34 |
| S1 -> network | S1 2500238 | INT-CR1-05, INT-CR1-07, INT-21 |
| network/USB -> VC | S1 2500238, VC (team) | INT-CR1-01, INT-CR1-06, INT-CR1-07, INT-22, INT-31, INT-36 |
| Watchdog heartbeats (VC, S2, network, S1, S4 -> supervisor) | team (supervisor), all owners | INT-25, INT-35 |

## Retired IDs

- INT-01 and INT-02 (the first two rows above): retired - do not reuse. They checked the template's S2 -> S3 sample flow and S3 -> S4 summary lines. Those interfaces no longer exist, and the checks were removed from `subsystem3_test.c` (commit `93f71ae`) and `subsystem4_test.c` (commit `ac7d949`).

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-22 | INT-CR1-08 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_sonar_pin_fix.txt` | Production driver, old-role negative control, scan policy, real headers SMP 0/1 and lock 0/1; ARM/physical retest pending |
| 2026-10-22 | INT-CR1-09, INT-CR1-10 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_s5_status_fix.txt` | Production S5 API/telemetry task, old-poll negative control, real headers SMP 0/1 and lock 0/1; ARM/physical validation pending |
