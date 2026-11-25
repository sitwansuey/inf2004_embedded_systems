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
| INT-04 | obstacle scans measure widths and choose routes | S5 -> VC/S1; FR5, FR6, NFR6 | auto (host) | `scan_test.c` via `python -B testing/integration/run.py` | `INT-04 scan: 2 consecutive near echoes (S5_ACQUISITION_SAMPLES) start it (far or open resets, a no-echo retry does not), wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS`, then nine lines starting `S5 width:`, `S5 side choice:`, `S5 verify fallback:`, `S5 edge flicker:`, `S5 scan geometry:`, `S5 edge check:`, `S5 side corridor:`, `S5 braking:` and `S5 servo slope:`, each ending in PASS |
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
| 2026-10-22 | INT-08 | `b02e335` + local working changes [^int-c1] | Codex | PASS (native) | `logs/2026-10-22_sonar_pin_fix.txt` | Production driver, old-role negative control, scan policy, real headers SMP 0/1 and lock 0/1; ARM/physical retest pending |
| 2026-10-22 | INT-09, INT-10 | `b02e335` + local working changes [^int-c1] | Codex | PASS (native) | `logs/2026-10-22_s5_status_fix.txt` | Production S5 API/telemetry task, old-poll negative control, real headers SMP 0/1 and lock 0/1; ARM/physical validation pending |
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

**Audit note (2026-11-25); this is not a result row.** The `ed197b3` PASS rows above stay as recorded. The same suite fails at `0bac48c` (the head of `main` on 2026-11-25): an audit re-run on 2026-11-25 of `mingw32-make -C testing/integration test` and of `python -B testing/integration/run.py --compiler gcc` (Windows, MSYS2 GCC) stopped in recipe `buddy4_vehicle_no_ramp_lift` with `Assertion failed: ... file buddy4_vehicle_test.c, line 1277` (the mode-15 obstacle check expects the event at `OC_ACQUIRE_AT`). Commit `d3d4bbf` changed obstacle acquisition (2 echoes instead of 3), and its message says the integration vehicle test mode 15 "still expects the 3-echo acquisition timing" (known failing). The run stops at the first failure, so the other four `buddy4_vehicle` recipes were not reached and `Integration recipes PASS` was not printed; the recipes before it printed no failure. The affected IDs are INT-11 and INT-29 to INT-33 (all five `buddy4_vehicle` recipes). The audit also reported further stale expectations at lines 1337 and 1484 of `buddy4_vehicle_test.c` (not re-checked here). The re-run output was not saved in `logs/`, so no result row is added for `0bac48c`; a FAIL row with a saved log, and later a fix, still need to be recorded.

**Audit note (2026-11-25), branch `audit-fixes`; this is not a result row.** The stale mode-15 expectations in `buddy4_vehicle_test.c` (2-echo acquisition from `d3d4bbf`, `OC_ACQUIRE_AT` and the sample table) were updated on branch `audit-fixes` in commit `6dc2fd8`, which is a test-only change. The same commit also updated the mode-17 event kinds and the published follow-code check. No result row is added for that branch: the team must re-run `mingw32-make -C testing/integration test` and `python -B testing/integration/run.py --compiler gcc` on the merged commit, save the output in `logs/`, and record the result rows for INT-11 and INT-29 to INT-33 here.

### Restored result rows (audit, 2026-11-25)

These result rows were deleted from the table above in `5f93cb6` (2026-11-25). They are copied back word for word from `d18dd38`, the last commit that had them; the first two columns are the only additions. Their *Test ID* cells use the IDs in force at `d18dd38`, not today's IDs, and some of those numbers now name different tests (second column, and *ID history (git record)* at the end of this file). Nothing was re-run. The INT-03 row is older: it was deleted in `373ad6d` (2026-10-27) together with its test case, and is copied back from `fc1dd8d`.

| Restored from | Test ID meaning (then → now) | Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|---|
| `d18dd38` (deleted in `5f93cb6`) | INT-35 at `d18dd38` = Watchdog heartbeat path on the robot: all five loops beat and the …; test removed in `5f93cb6`; no current test uses INT-35 | 2026-11-20 | INT-35 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_flash_8c_boot.txt` | Console from the reset after the UF2 flash: `[WDT] supervisor started` (line 16), then `[WDT] armed: every monitored loop is running` once (line 262), just after the t=20000 status line (line 257), so about 20 s after reset; no `[WDT] not armed yet` line. Uptime kept counting to t=45000 (the `[RESET]` line repeats every 10 s with `[TIMING]`; not a reboot). Not the full procedure: the access point was not in range (all 8 joins ended "SSID not found", lines 58 to 536; no IP, no MQTT), and the console was watched for 45 s (13:48:27 to 13:49:12), not 60 s. |
| `fc1dd8d` (deleted in `373ad6d`); the hash `78f6171` in its Commit cell is not found in repository history | INT-03 at `fc1dd8d` = Complete PR #6 firmware compiles with current S2/S3/S4 interfaces; test removed in `373ad6d` (2026-10-27); today's INT-03 is a different test (IMU samples produce terrain and tilt results) | 2026-10-27 | INT-03 | `78f6171` plus local producer-removal patch | Codex | PASS | [Build and regression summary](logs/2026-10-27_pr6_build_fix.txt) | Tested the PR merge containing current Buddy 3; local verification, no hardware or remote CI rerun claimed |

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

`5f93cb6` also changed the *Test ID* cells of the two 2026-10-22 rows (INT-CR1-08 and INT-CR1-09, INT-CR1-10 became INT-08 and INT-09, INT-10). The numbers INT-01 to INT-03 were already used before: INT-01 and INT-02 were the template sample-flow tests, and INT-03 was the October PR #6 build test (last row of the table below).

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| INT-01 | samples flow from S2 to S3 (≥ 5 in 2 s) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: controller policy prioritizes commands, bounds bypasses, waits for sensors and ramps avoidance power |
| INT-02 | summary lines flow from S3 to S4 (≥ 2 in 3.5 s) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: encoder snapshots produce valid odometry |
| INT-CR1-08 | Original Buddy 5 signal roles and production sonar driver | INT-08 | yes | not used |
| INT-CR1-09 | Preempted status publisher and failure propagation | INT-09 | yes | not used |
| INT-CR1-10 | Snapshot timestamps during bounded reads | INT-10 | yes | not used |
| INT-CR1-01 | Controller policy (`vehicle_policy.c`): command priority, lease expiry, stale data, bounded bypass, STOP handoffs; sensor waits and phase deadlines; avoidance power ramps at 5/10/20 ms ticks | INT-01 | yes | not used |
| INT-CR1-02 | Odometry from S2 encoder snapshots (`odometry.c`): count wrap, scale, speed, yaw, gaps, discontinuities | INT-02 | yes | not used |
| INT-CR1-03 | Terrain algorithm on IMU samples (`terrain.c`): calibration gate, ordered hump, maximum, impact, invalid samples; IMU mount axis remap, saturation, pitch and roll | INT-03 | yes | not used |
| INT-CR1-04 | S5 scan (`s5_scan.c`, `s5_policy.c`, `s5_width.c`): 3 near echoes start it, angle association, failed echo, moving scan, geometric bounds; width, side choice, verify fallback, edge flicker and edge check, side corridor, braking, servo slope | INT-04 | yes | not used |
| INT-CR1-05 | Telemetry wire format: the C codec (`telemetry_codec.c`) and the Python decoder agree on the minimum, maximum and 300 random snapshots; out-of-range values and a bad length, version or fault are rejected | INT-05 | yes | not used |
| INT-CR1-06 | S1 queues in the real `telemetry.c`: command copy, full queue, urgent STOP, stale telemetry, validation, 14-byte S3 events; robot event queue (16 deep, drops counted); drop counter wrap; newest-first snapshots; command replies | INT-06 | yes | not used |
| INT-CR1-07 | MQTT port (`mqtt_utk.c` with `mocks/`): binary length and NULs, fragments, malformed input, backpressure, driving does not depend on the link, retry guard; command replies; abandoned snapshot; robot events held through a link drop; Wi-Fi re-join timing | INT-07 | yes | not used |
| INT-S4-11 | Real controller task (`vehicle.c`) with S2 and S4 inputs: encoded getter errors, future samples, IMU skew, reset, motor STOP | INT-11 | yes | not used |
| INT-12 | S5 width to the VC bypass plan and the EDR16 width record: width-based legs, bounded reverse, side look, pass in steps, mirrored return, restore and spot search, 3 fresh side readings, sonar gap and pan stops | INT-12 | yes | same test |
| INT-13 | S2 speed PI used for VC demands: fractional corrections kept, rounding, anti-windup at saturation, steered demand capped | INT-13 | yes | same test |
| INT-14 | VC demand at 30% through S2 steering: straight demand, feedforward, mirrored 30/18 bend targets, every steering target and PID output at most 30% | INT-14 | yes | same test |
| INT-15 | Obstacle met on a bend (one line sensor on black): keeps the last side, bounded straight reverse, mirrored left/right bypass, side look, return, restore, spot search; STOP, sonar gap and disabled bypass stop | INT-15 | yes | same test |
| INT-16 | Bypass return (`vehicle_policy.c`): diagonal side look, clear run, direct return, align and pass, return until the line is under the robot, restore the detection heading, TRACK or spot search; line-data and sonar gap holds | INT-16 | yes | same test |
| INT-17 | Reverse barcode re-read steering (`vehicle_reverse_track.h` with S2 steering): heading and line terms, heading wrap, closed-loop drift | INT-17 | yes | same test |
| INT-18 | Line and barcode-read steering (`vehicle_line_steer.h`): first correction, ramp, cap, pivot; the `robot_config.h` defaults and the S2 slew rate | INT-18 | yes | same test |
| INT-19 | Obstacle confirmation from S5 samples (`vehicle_obstacle_confirm.h`): 3 new near samples in a row, a re-read counts once, any other sample resets | INT-19 | yes | same test |
| INT-20 | Barcode reverse re-read (`vehicle_barcode_retry.h`): the reverse runs past the first bar plus the run-up; an early noise count does not stop it | INT-20 | yes | same test |
| INT-21 | Snapshot to EDR16 records to the dashboard: 33 two-byte records from `telemetry_codec.c` are decoded by the dashboard bridge (`dashboard.py`) into the expected values; the browser card check also runs when Node.js is on PATH | INT-21 | yes | same test |
| INT-22 | Mission handoffs (`vehicle_policy.c`, `buddy3_barcode.c`): obstacle priority over a deferred turn, turn at the junction, TRACK acknowledgement, timeout and wrap, STOP; autonomous START, bounded reacquisition, PID limits, barcodes A-D forward and reverse | INT-22 | yes | same test |
| INT-23 | Barcode turns and line search (`vehicle_policy.c`): turn at the next junction after 90 mm, 600 mm search limit, U-turn, line-guided turns, line lost while tracking, spot search limits | INT-23 | yes | same test |
| INT-24 | Straddle line observer (`ir_line_trace.c` with the S3 configuration) set up as `vehicle.c` does: GP26 right, GP27 left, measured endpoints, black/white hysteresis; IR mode 3 and IR mode 2 | INT-24 | yes | same test |
| INT-25 | Watchdog supervisor decision (`shared/watchdog_policy.c`) on the five heartbeats (VC, motor, network, telemetry, IMU): no arming before the 20 s grace, the first network join and every loop running; feeds while all are fresh; a loop stale for 1 s trips once and is named; crash record codec; early-crash count and hold | INT-25 | yes | same test |
| INT-26 | S5 scan rejects robot motion (encoder counts in the S5 status): 1-3 counts tolerated, a larger move recentres and retries twice, retry budget kept, encoder wrap | INT-26 | yes | same test |
| INT-27 | S5 scan with open space beyond the object: clear after 3 settled samples, the open side planned with one edge mirrored, no plan when neither side is open, READY check, forward bypass guidance | INT-27 | yes | same test |
| INT-28 | S5 edge search: 20/40/60 deg coarse search, bisection, mirror check, outward search, one-sided width flagged, hard echo faults count as the object, edge flicker retreat | INT-28 | yes | same test |
| INT-29 | Real `vehicle.c` with S3 snapshots and events: straddle barcode-read steering, mask-pivot follower (edge tracing in `buddy4_vehicle_edge_trace`), barcode first pass and reverse re-read, silent S3 latch and TRACK re-send, held marker, line lost while tracking, lifted robot (GP28) | INT-29 | yes | same test |
| INT-30 | Real `vehicle.c` with S5 status: obstacle while tracking (3 new near echoes), obstacle found through S5's real scan step, sonar gap hold, S5 lock busy across a phase change, avoidance ended by STOP | INT-30 | yes | same test |
| INT-31 | Commands into the real `vehicle.c` (the same records as USB `a`/`g` and MQTT): MAP in any phase, ARM lease renewal, renewal limits, STOP during renewals, RESET | INT-31 | yes | same test |
| INT-32 | Real `vehicle.c` motion requests to S2 and S2 status back: line follow/mask/steer match the S2 request every tick; short timing faults (odometry gap, failed motor status copy and fault 7, busy S3 lock, command stamped a tick ahead); loop timing on the hardware timer | INT-32 | yes | same test |
| INT-33 | Real `vehicle.c` with S4 records, and its events to S1: hump detected after 400 ms, crest power cap, level restore (no events and no cap with `BOARD_RAMP_SPEED 0`); shock published and counted (stops the run with `BOARD_SHOCK_STOP 1`); 8c counter-wrap regressions; EV_BARCODE before EV_INSTRUCTION | INT-33 | yes | same test |
| INT-34 | VC and S4 state in the S1 snapshot (`task_telemetry.c`): line state, mask and error; fault cause (VC_SENSOR, VC_LINE, fault 8); a new shock sent at once; IMU fault held for the next snapshot; at most one shock extra per 200 ms; IMU fault count | INT-34 | yes | same test |
| INT-35 | Watchdog heartbeat path on the robot: all five loops beat and the supervisor arms the watchdog | removed from the table in `5f93cb6` (INT-35 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| INT-36 | USB console commands reach the VC (`usb_control.c` -> S1 command inbox -> VC) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| INT-03 (at `fc1dd8d`, 2026-10-27) | Complete PR #6 firmware compiles with current S2/S3/S4 interfaces | removed in `373ad6d` (2026-10-27) | — | different test: IMU samples produce terrain and tilt results |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. A cell that names exactly one existing commit is not marked. Any other cell keeps its original text and carries a footnote; the footnotes are at the end of this file. The restored October INT-03 row names `78f6171`, which is not found in repository history; its cell is left as written and the note is in its first column.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened.

[^int-c1]: Hash `b02e335` not found in repository history (`git cat-file`, all branches, 2026-11-25), and the entry adds uncommitted local changes, so the code these rows ran on is not in the repository.
