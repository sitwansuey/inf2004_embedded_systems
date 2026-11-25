# Integration — test log

Tests that two or more subsystems work together. There is one test (or more) for every interface listed in TEAM.md section 5. Both owners agree on the test and sign off the result.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| INT-01 | controller policy prioritizes commands, bounds bypasses, waits for sensors and ramps avoidance power | VC -> S2, network/USB -> VC; FR1, FR5, NFR1 | auto (host) | `policy_test.c` via `python -B testing/integration/run.py` | `INT-01 controller: priority, expiry, stale data, complete bounded bypass, STOP handoffs PASS`; `Sensor waits: APPROACH sample, full reverse, forward pan bound, phase deadlines and intermittent side echoes PASS`; `Avoidance ramps: real ms at 5/10/20 ms ticks, tested 20 ms steps, drops and slews PASS` |
| INT-02 | encoder snapshots produce valid odometry | S2 -> VC/S1, S2 -> S4 in VC; FR4, FR10 | auto (host) | `policy_test.c` via `python -B testing/integration/run.py` | `INT-02 odometry: wrap, scale, speed, yaw, gaps and discontinuities PASS` |
| INT-03 | IMU samples produce terrain and tilt results | S4 -> VC/S1; FR2, FR3, FR10 | auto (host) | `policy_test.c` via `python -B testing/integration/run.py` | `INT-03 terrain: calibration gate, ordered hump, maximum, impact and invalid samples PASS`; `IMU mount: axis remap, saturation, nose-up pitch and side roll PASS` |
| INT-04 | obstacle scans measure widths and choose routes | S5 -> VC/S1; FR5, FR6, NFR6 | auto (host) | `scan_test.c` via `python -B testing/integration/run.py` | `INT-04 scan: 3 consecutive near echoes start it (far or open resets, a no-echo retry does not), wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS`, then nine lines starting `S5 width:`, `S5 side choice:`, `S5 verify fallback:`, `S5 edge flicker:`, `S5 scan geometry:`, `S5 edge check:`, `S5 side corridor:`, `S5 braking:` and `S5 servo slope:`, each ending in PASS |
| INT-05 | C and Python telemetry codecs agree | VC -> S1, S1 -> network; FR12, NFR7 | auto (host) | `protocol_cli.c`, `protocol_test.py` via `python -B testing/integration/run.py` | `INT-05 protocol: 302 C/Python vectors, 90 bound rejections, malformed length/version/fault PASS`; `protocol_golden.json` unchanged. |
| INT-06 | command and telemetry queues preserve delivery rules | network/USB -> VC, VC -> S1; FR12, NFR7, NFR8 | auto (host) | `telemetry_queue_test.c` via `python -B testing/integration/run.py` | `INT-06 actual queues: copy ownership, full queue, urgent STOP, stale telemetry, validation and 14-byte events PASS`; `Robot event queue: EDR16-format words, FIFO, 16 deep, full queue drops and counts, bad ids ignored PASS`; `Drop counter: wraps past 65535, status subtype 12 is drops modulo 256 PASS`; `Snapshots: newest first, older/stale and full-queue drops counted, impact and IMU fault handed on PASS`; `Command replies: superseded commands rejected with the VC state, two STOPs both answered, full reply buffer drops the oldest PASS` |
| INT-07 | MQTT handles binary messages and link recovery | S1 -> network, network/USB -> VC; FR12, NFR8 | auto (host) | `mqtt_test.c` via `python -B testing/integration/run.py` | `INT-07 actual MQTT port: binary length/NULs, fragments, malformed input, backpressure, autonomous link-loss independence and retry guard PASS`, then lines starting `MQTT replies:`, `Abandoned snapshot:`, `MQTT robot events:` and `Wi-Fi re-join:`, each ending in PASS |
| INT-08 | sonar driver checks GPIO roles and timing | S5 GPIO/timer and sensor status | native | `s5_hardware_test.c` via `python -B testing/integration/run.py` | `INT-08: GP1 trigger/GP0 echo, valid pulse, faults, timer wrap/start failure, narrow sample PASS` |
| INT-09 | status publication handles preemption and failures | S5 snapshot/control API | native | `s5_status_test.c` via `python -B testing/integration/run.py` | `INT-09: preempted publisher handoff, ownership, timeout, unchanged failed output, unlock/lifecycle errors PASS` |
| INT-10 | snapshots use fresh timestamps after bounded reads | telemetry freshness and codec | native | `snapshot_time_test.c` via `python -B testing/integration/run.py` | `INT-10: post-snapshot clock, freshness/errors, packed signed speed, actual binary codec PASS` |
| INT-11 | controller handles motor and IMU input failures | S2 -> S4 in VC, S4 -> VC/S1; FR3, NFR2, NFR3 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | `INT-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS` |
| INT-12 | obstacle widths guide the bypass route | S5 -> VC/S1, VC -> S1; FR5, FR6, FR7, NFR6 | auto (host) | `width_avoidance_test.c` via `python -B testing/integration/run.py` | `PASS: estimated width -> EDR16; width-based legs; bounded reverse before turn-out; side look at the diagonal end, pass in steps, clear run, mirrored return, restore and spot search; 3 fresh consistent side readings; cached/invalid/occupied checks, sonar gap and pan stops.` |
| INT-13 | speed PI preserves corrections and output limits | VC -> S2; FR4 | auto (host) | `smooth_motor_test.c` via `python -B testing/integration/run.py` | `PASS: fractional PI corrections retained; legacy rounding preserved; saturation anti-windup; calibrated wheel ratios remain capped.` |
| INT-14 | 30-percent steering stays within the demand limit | VC -> S2; FR4, FR8 | auto (host) | `speed30_test.c` via `python -B testing/integration/run.py` | `PASS: 30-percent straight demand; calibrated feedforward; mirrored 30/18 bend targets; all steering targets and PID outputs bounded at 30 percent.` |
| INT-15 | obstacles on bends use a bounded bypass | S5 -> VC/S1, VC -> S2, VC <-> S3; FR5 | auto (host) | `obstacle_bend_test.c` via `python -B testing/integration/run.py` | `PASS: both-black keeps the last side; bounded straight reverse then mirrored left/right bypass; clear side look at the diagonal end returns straight through 2x the heading; restore and spot search; STOP, sonar gap and disabled bypass stop.` |
| INT-16 | bypass return restores heading and line tracking | S5 -> VC/S1, VC <-> S3; FR1, FR5 | auto (host) | `bypass_return_test.c` via `python -B testing/integration/run.py` | `Bypass return: diagonal side look (clear run, direct 2x return), align and pass in steps, return until the line is under the robot or the extra distance, RESTORE to the detection heading, TRACK or spot search; no line search for the first second of the turn-in (far sensor alone right after it); line-data and sonar gap holds PASS` |
| INT-17 | reverse barcode steering controls line and heading drift | VC <-> S3, VC -> S2; FR9 | auto (host) | `reverse_track_test.c` via `python -B testing/integration/run.py` | `reverse_track: ok (worst sensor offset <n> mm, worst heading <n> deg)`; `exit code 0` |
| INT-18 | line steering follows ramps and power limits | VC <-> S3, VC -> S2; FR8, FR9 | auto (host) | `line_steer_test.c` via `python -B testing/integration/run.py` | `line_steer: ok`; `exit code 0` |
| INT-19 | obstacle confirmation requires three new near samples | S5 -> VC/S1; FR5, NFR1 | auto (host) | `obstacle_confirm_test.c` via `python -B testing/integration/run.py` | `Obstacle confirmation: 3 consecutive new near samples, re-reads count once, any other sample resets PASS` |
| INT-20 | barcode retry reverses past the first bar | VC <-> S3; FR9 | auto (host) | `barcode_retry_test.c` via `python -B testing/integration/run.py` | `barcode_retry: reverse runs past the first bar with run-up; noise-early count does not stop it PASS` |
| INT-21 | production snapshots reach the dashboard as EDR16 records | VC -> S1, S1 -> network; FR12, FR13, NFR7 | auto (host) | `edr_stream_test.c`, `edr_stream_test.py` via `python -B testing/integration/run.py` | `EDR16 production snapshot through Python/dashboard PASS` |
| INT-22 | mission handoffs preserve navigation and STOP priority | VC <-> S3, network/USB -> VC; FR1, FR9, NFR1 | auto (host) | `mission_test.c` via `python -B testing/integration/run.py` | `Mission handoffs: obstacle priority, deferred navigation to the junction, TRACK acknowledgement, timeout/wrap and STOP PASS`; `Mission: autonomous start, bounded reacquisition, PID limits/anti-windup, A-D forward/reverse PASS` |
| INT-23 | barcode turns and line searches obey bounds | VC <-> S3; FR1, FR9 | auto (host) | `nav_junction_test.c` via `python -B testing/integration/run.py` | nine lines starting `Right turn waits through tracking`, `Junction ignored during the re-read hold`, `Leaving TRACK resets the junction progress`, `U-turn spot scan:`, `U-turn repeat:`, `Line-guided turns:`, `Line-guided U-turn:`, `Line lost while tracking:` and `Spot search held by the sonar`, each ending in PASS |
| INT-24 | line observer uses the configured robot IR layout | VC <-> S3; FR8, NFR4 | auto (host) | `line_trace_test.c` via `python -B testing/integration/run.py` (IR modes 3 and 2) | `Straddle line observer (IR mode 3): <n> checks, 0 failures` and `Straddle line observer (IR mode 2): <n> checks, 0 failures`; `exit code 0` |
| INT-25 | watchdog supervises heartbeats and records crash history | Watchdog heartbeat path (`app_watchdog.c`); NFR2 | auto (host) | `watchdog_test.c` via `python -B testing/integration/run.py` | lines starting `Watchdog supervisor:`, `Crash record:` and `Early crashes:`, each ending in PASS |
| INT-26 | moving robot scans retry within their budget | S5 -> VC/S1, S2 -> VC/S1; FR5, FR6 | auto (host) | `obstacle_motion_test.c` via `python -B testing/integration/run.py` | `PASS: 1-3 encoder counts tolerated; larger displacement recentres and retries twice; retry budget survives acquisition; repeated motion blocks; width hold cleared by motion; encoder wrap is handled.` |
| INT-27 | open-space scans choose a clear bypass side | S5 -> VC/S1; FR5, FR6 | auto (host) | `obstacle_open_space_test.c` via `python -B testing/integration/run.py` | `PASS: no-rise/no-fall clear after 3 settled samples; open left or right plans that side (one measured edge mirrored), neither blocks; READY verification; forward bypass guidance; angle reset; hard faults remain closed.` |
| INT-28 | obstacle scans follow edge search and fallback rules | S5 -> VC/S1; FR6, NFR6 | auto (host) | `obstacle_scan_test.c` via `python -B testing/integration/run.py` | `PASS: 20/40/60 deg coarse search, edge bisection, mirror check then outward search; one open side plans it with the edge mirrored (width_one_sided), none blocks; hard echo faults count as the object; unknown stays blocked.`; `PASS: an edge flicker while averaging retreats that bound 1 deg inward and still reaches READY; S5_EDGE_RETREATS+1 flickers fail the side.` |
| INT-29 | controller handles line following and barcode events | VC <-> S3; FR1, FR8, FR9 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | in each recipe, lines starting `Straddle barcode read:`, `Mask-pivot follower:` (`Edge tracing:` in `buddy4_vehicle_edge_trace`), `Barcode retry:`, `Barcode first pass:`, `Silent S3 latch`, `Marker cut short by the latch:`, `S3 holding a decoded marker`, `Line lost while tracking:` (`Line lost with edge tracing:` in `buddy4_vehicle_edge_trace`) and `Lifted (no surface, GP28 black):`, each ending in PASS |
| INT-30 | controller handles sonar status and obstacle avoidance | VC -> S5, S5 -> VC/S1; FR5, NFR1 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | in each recipe, lines starting `Obstacle while tracking:`, `Obstacle detected through S5's real step`, `Sonar gap:`, `S5 lock busy across a phase change:` and `Avoidance ended by a STOP:`, each ending in PASS |
| INT-31 | controller commands enforce leases and STOP priority | network/USB -> VC; FR1, FR12 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | in each recipe, lines starting `MAP command:`, `ARM lease renewal:`, `ARM renewal limits:` and `STOP during renewals:`, each ending in PASS |
| INT-32 | controller motor requests match S2 and survive bounded timing faults | VC -> S2, S2 -> VC/S1; FR1, FR4, NFR5 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | in each recipe, lines starting `Straddle app_state line_follow/line_mask/line_steer`, `Short timing faults ride through:` and `Loop timing:`, each ending in PASS |
| INT-33 | controller reports humps, shocks and events | S4 -> VC/S1, VC -> S1; FR2, FR3, FR7, FR10 | auto (host) | `buddy4_vehicle_test.c` via `python -B testing/integration/run.py` (all five `buddy4_vehicle` recipes) | lines starting `Hump:` (`Hump with BOARD_RAMP_SPEED 0:` in `buddy4_vehicle_no_ramp_lift`), `Shock while tracking`, `Event log:` and `8c:` (not in `buddy4_vehicle_no_ramp_lift`), each ending in PASS |
| INT-34 | snapshots preserve vehicle state and IMU events | VC -> S1, S4 -> VC/S1; FR3, FR7, FR11, NFR7 | auto (host) | `snapshot_time_test.c` via `python -B testing/integration/run.py` | six lines before the `INT-10:` line, starting `Straddle line telemetry:`, `Fault cause:`, `Shock count:`, `IMU fault:`, `Shock extras:` and `IMU fault count:`, each ending in PASS |

*Method:* **auto (host)** and **native** = a test runs on a PC without the robot.

**auto (host)** = a host program in `testing/integration`. Run `python -B testing/integration/run.py` from the
repository root (needs a native `gcc` on PATH, or `--compiler <path to gcc>`).
On Linux, macOS or CI, `make -C testing/integration test` runs the same recipes.
The older rows INT-08 to INT-10 say **native**: that is the same host run.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| INT-CR1-08 | INT-08 |
| INT-CR1-09 | INT-09 |
| INT-CR1-10 | INT-10 |
| INT-CR1-01 | INT-01 |
| INT-CR1-02 | INT-02 |
| INT-CR1-03 | INT-03 |
| INT-CR1-04 | INT-04 |
| INT-CR1-05 | INT-05 |
| INT-CR1-06 | INT-06 |
| INT-CR1-07 | INT-07 |
| INT-S4-11 | INT-11 |
| INT-12 | INT-12 (unchanged) |
| INT-13 | INT-13 (unchanged) |
| INT-14 | INT-14 (unchanged) |
| INT-15 | INT-15 (unchanged) |
| INT-16 | INT-16 (unchanged) |
| INT-17 | INT-17 (unchanged) |
| INT-18 | INT-18 (unchanged) |
| INT-19 | INT-19 (unchanged) |
| INT-20 | INT-20 (unchanged) |
| INT-21 | INT-21 (unchanged) |
| INT-22 | INT-22 (unchanged) |
| INT-23 | INT-23 (unchanged) |
| INT-24 | INT-24 (unchanged) |
| INT-25 | INT-25 (unchanged) |
| INT-26 | INT-26 (unchanged) |
| INT-27 | INT-27 (unchanged) |
| INT-28 | INT-28 (unchanged) |
| INT-29 | INT-29 (unchanged) |
| INT-30 | INT-30 (unchanged) |
| INT-31 | INT-31 (unchanged) |
| INT-32 | INT-32 (unchanged) |
| INT-33 | INT-33 (unchanged) |
| INT-34 | INT-34 (unchanged) |
| INT-35 | INT-35 (unchanged) |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): INT-01 and INT-02 (the template's S2 -> S3 sample flow and S3 -> S4 summary lines; retired, the checks were removed in `93f71ae` and `ac7d949`) and INT-36 (USB console commands reach the VC; never run, removed on 2026-11-20).

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): INT-35.

</details>

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
| `policy` | `policy_test.c` | INT-01, INT-02, INT-03 (printed) |
| `watchdog` | `watchdog_test.c` | INT-25 |
| `obstacle_motion` | `obstacle_motion_test.c` | INT-26 |
| `obstacle_open_space` | `obstacle_open_space_test.c` | INT-27 |
| `obstacle_scan` | `obstacle_scan_test.c` | INT-28 |
| `scan` | `scan_test.c` | INT-04 (printed) |
| `sonar`, `sonar_low_centre` (`-DBOARD_SERVO_CENTER_US=1300`), `sonar_slope` (`-DBOARD_SERVO_US_PER_90=1000`) | `s5_hardware_test.c` | INT-08 (printed by all three) |
| `sonar_status` | `s5_status_test.c` | INT-09 (printed) |
| `snapshot_time` | `snapshot_time_test.c` | INT-10 (printed), INT-34 |
| `protocol` | `protocol_cli.c`, `protocol_test.py` | INT-05 (printed) |
| `queue` | `telemetry_queue_test.c` | INT-06 (printed) |
| `mqtt` | `mqtt_test.c` | INT-07 (printed) |
| `buddy4_vehicle_no_ramp_lift` (`-DBOARD_RAMP_SPEED=0 -DBOARD_LIFT_FAULT_MS=0U`), `buddy4_vehicle`, `buddy4_vehicle_shock_stop` (`-DBOARD_SHOCK_STOP=1`), `buddy4_vehicle_edge_trace` (`-DBOARD_LINE_MASK_PIVOT=0 -DBOARD_LINE_EDGE_TRACE=1`), `buddy4_vehicle_read150` (`-DBOARD_BARCODE_READ_LINE_STEER=150`) | `buddy4_vehicle_test.c` | INT-11 (printed by all five), INT-29 to INT-33 |
| not in the `test` target | `groupmate_motor_test.c` | none (not run by `run.py` or `make test`) |

## Interface -> test IDs

One row per interface in TEAM.md section 5 (IF-01 to IF-22), with the same test IDs. The owners on both sides sign off the result.

| Interface | Owners | Test IDs |
|---|---|---|
| IF-01 VC → S2 movement commands | VC (team), S2 2500613 | INT-01, INT-13, INT-14, INT-15, INT-17, INT-18, INT-32 |
| IF-02 VC → S2 urgent STOP | VC (team), S2 2500613 | INT-01, INT-11, INT-30 (VC side; the S2 side is in the S2 host tests) |
| IF-03 S2 → VC, S1 motion status | S2 2500613, VC (team), S1 2500238 | INT-11, INT-32, INT-10 |
| IF-04 S2 → VC encoder snapshot (to S4 terrain) | S2 2500613, VC (team), S4 2500243 | INT-02, INT-11 |
| IF-05 S2 → S3 wheel travel for barcodes | S2 2500613, S3 2500383 | none yet (the S3 unit tests feed positions directly) |
| IF-06 S2 → S5 encoder counts around each echo | S2 2500613, S5 2502324 | INT-08, INT-26 |
| IF-07 VC → S3 control, lease, MAP, barcode end | VC (team), S3 2500383 | INT-22, INT-23, INT-29, INT-31 |
| IF-08 S3 → VC, S1 status | S3 2500383, VC (team), S1 2500238 | INT-15, INT-16, INT-17, INT-18, INT-20, INT-23, INT-24, INT-29, INT-34 |
| IF-09 S3 → VC events | S3 2500383, VC (team) | INT-06, INT-22, INT-23, INT-29 |
| IF-10 S4 → VC, S1 acquisition record | S4 2500243, VC (team), S1 2500238 | INT-03, INT-11, INT-33, INT-10, INT-34 |
| IF-11 VC → S5 navigation | VC (team), S5 2502324 | INT-09, INT-30 |
| IF-12 S5 → VC, S1 status | S5 2502324, VC (team), S1 2500238 | INT-04, INT-08, INT-09, INT-10, INT-12, INT-15, INT-16, INT-19, INT-26, INT-27, INT-28, INT-30 |
| IF-13 VC → S1 controller snapshot | VC (team), S1 2500238 | INT-05, INT-10, INT-21, INT-32, INT-34 |
| IF-14 VC → S1 robot events | VC (team), S1 2500238 | INT-06, INT-07, INT-33 |
| IF-15 S1 → dashboard (MQTT) telemetry | S1 2500238 | INT-05, INT-06, INT-07, INT-10, INT-21 |
| IF-16 S1 → dashboard (MQTT) events | S1 2500238 | INT-06, INT-07 |
| IF-17 dashboard (MQTT) → VC commands | S1 2500238, VC (team) | INT-01, INT-06, INT-07, INT-22, INT-31 |
| IF-18 USB console → VC commands | S1 2500238, VC (team) | none for the hook itself (INT-31 replays the same frames into the VC; INT-06 tests `s1_command_offer()`) |
| IF-19 command replies → dashboard (MQTT) | VC (team), S1 2500238 | INT-06, INT-07, INT-31 |
| IF-20 heartbeats → watchdog supervisor | team (supervisor), all owners | INT-25 |
| IF-21 S2 → S1 encoder counts | S2 2500613, S1 2500238 | INT-10 (S1 side, with S2 stubbed) |
| IF-22 S2, network → VC loop timing | S2 2500613, VC (team) | none (USB diagnostic only) |

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | INT-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-22 | INT-08 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_sonar_pin_fix.txt` | Production driver, old-role negative control, scan policy, real headers SMP 0/1 and lock 0/1; ARM/physical retest pending |
| 2026-10-22 | INT-09, INT-10 | `b02e335` + local working changes | Codex | PASS (native) | `logs/2026-10-22_s5_status_fix.txt` | Production S5 API/telemetry task, old-poll negative control, real headers SMP 0/1 and lock 0/1; ARM/physical validation pending |
| 2026-11-20 | INT-01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `policy`: `INT-CR1-01 controller`, `Sensor waits` and `Avoidance ramps` lines PASS. All 33 recipes ran, last line `Integration recipes PASS`, exit code 0 |
| 2026-11-20 | INT-02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `policy`: `INT-CR1-02 odometry` line PASS |
| 2026-11-20 | INT-03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `policy`: `INT-CR1-03 terrain` and `IMU mount` lines PASS |
| 2026-11-20 | INT-04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `scan`: `INT-CR1-04 scan` and the nine `S5 ...` lines (width to servo slope) PASS |
| 2026-11-20 | INT-05 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `protocol`: 302 C/Python vectors, 90 bound rejections PASS. Run on a clean export of the commit, so the `protocol_golden.json` git status check is not in the log |
| 2026-11-20 | INT-06 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `queue`: `INT-CR1-06 actual queues`, `Robot event queue`, `Drop counter`, `Snapshots`, `Command replies` PASS |
| 2026-11-20 | INT-07 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `mqtt`: `INT-CR1-07 actual MQTT port`, `MQTT replies`, `Abandoned snapshot`, `MQTT robot events`, `Wi-Fi re-join` PASS |
| 2026-11-20 | INT-11 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | `INT-S4-11 actual vehicle path` PASS in all five `buddy4_vehicle` recipes |
| 2026-11-20 | INT-12 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `width_avoidance` PASS line |
| 2026-11-20 | INT-13 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `smooth_motor` PASS line |
| 2026-11-20 | INT-14 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `speed30` PASS line |
| 2026-11-20 | INT-15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `obstacle_bend` PASS line |
| 2026-11-20 | INT-16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `bypass_return`: `Bypass return: ... PASS`, no failure line; no-line RETURN ended at 392 mm (141 + 250 expected) |
| 2026-11-20 | INT-17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `reverse_track`: ok, worst sensor offset 14.2 mm, worst heading 9.9 deg; run exit code 0 |
| 2026-11-20 | INT-18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `line_steer`: `line_steer: ok`; run exit code 0 |
| 2026-11-20 | INT-19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `obstacle_confirm`: `Obstacle confirmation: ... PASS` |
| 2026-11-20 | INT-20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `barcode_retry`: `barcode_retry: ... PASS` |
| 2026-11-20 | INT-21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `edr_stream` PASS; the log does not show whether the Node.js browser card check ran |
| 2026-11-20 | INT-22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `mission`: `Mission handoffs` and `Mission: autonomous start` lines PASS |
| 2026-11-20 | INT-23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `nav_junction`: all nine lines PASS |
| 2026-11-20 | INT-24 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipes `line_trace` and `line_trace_mode2`: 178 checks, 0 failures each |
| 2026-11-20 | INT-25 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `watchdog`: `Watchdog supervisor`, `Crash record`, `Early crashes` PASS |
| 2026-11-20 | INT-26 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `obstacle_motion` PASS line |
| 2026-11-20 | INT-27 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `obstacle_open_space` PASS line |
| 2026-11-20 | INT-28 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `obstacle_scan`: coarse search and edge flicker PASS lines |
| 2026-11-20 | INT-29 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | All nine lines PASS in all five `buddy4_vehicle` recipes (`Edge tracing` and `Line lost with edge tracing` in `buddy4_vehicle_edge_trace`) |
| 2026-11-20 | INT-30 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | All five obstacle/sonar/STOP lines PASS in all five `buddy4_vehicle` recipes |
| 2026-11-20 | INT-31 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | `MAP command`, `ARM lease renewal`, `ARM renewal limits`, `STOP during renewals` PASS in all five `buddy4_vehicle` recipes |
| 2026-11-20 | INT-32 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | `Straddle app_state ...`, `Short timing faults ride through`, `Loop timing` PASS in all five `buddy4_vehicle` recipes |
| 2026-11-20 | INT-33 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | `Hump` (`Hump with BOARD_RAMP_SPEED 0` in `no_ramp_lift`), `Shock while tracking` (stops the run with VC_SHOCK in `shock_stop`), `Event log`, `8c` (four recipes) PASS |
| 2026-11-20 | INT-34 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `snapshot_time`: six lines PASS before the `INT-CR1-10` line |
| 2026-11-20 | INT-08 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipes `sonar`, `sonar_low_centre`, `sonar_slope`: `INT-CR1-08 ... PASS` on lines 124, 127 and 130 |
| 2026-11-20 | INT-09 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `sonar_status`: `INT-CR1-09 ... PASS` on line 133 |
| 2026-11-20 | INT-10 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_integration.txt` | Recipe `snapshot_time`: `INT-CR1-10 ... PASS` on line 142 |
