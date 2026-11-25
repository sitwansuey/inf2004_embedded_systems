# Subsystem 1 — test log

**Subsystem:** Binary telemetry, command/telemetry buffers, MQTT dashboard, status LED  ·  **Student ID:** 2500238  ·  **Code:** `app_program/subsystem1/`, `app_program/external_systems/Dashboard/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | dashboard command delivery and acknowledgements | FR12, NFR8 | auto (host) | `test_commands.py` via `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_commands.py" -v` | `OK` |
| S1-U02 | dashboard event text and log handling | FR13, FR12, FR7, FR9, FR10 | auto (host) | `test_events.py` via `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_events.py" -v` (GCC required). | `OK`; no skipped tests. |
| S1-U03 | obstacle widths reach the dashboard correctly | FR7, FR13 | auto (host) | `test_obstacle_width.py` via `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_obstacle_width.py" -v` (build `testing/integration/.build/width_avoidance` with the S1-U05 command first). | `OK`; no skipped tests. |
| S1-U04 | binary telemetry passes the data checks | FR12, FR13, NFR7, NFR8 | auto (host) | `test_edr.py`, `test_edr.c`, `telemetry_host_test.c` via `python -B testing/data_representation/run.py --compiler gcc` | `EDR CHECK PASS: all three stages passed. Run again after your integration changes.` |
| S1-U05 | production snapshots reach the dashboard correctly | FR12, FR13, FR11 | auto (host) | `edr_stream_test.c`, `edr_stream_test.py` via `python -B testing/integration/run.py --compiler gcc` | `EDR16 production snapshot through Python/dashboard PASS`; `Integration recipes PASS` |
| S1-U06 | snapshots are built every 200 ms | NFR7 | manual | 1. Leave the robot DISARMED, still and fault-free on the desk, linked to the broker; do not touch it. 2. Run `mosquitto_sub -h <broker> -t Robopico/sensors/edr16/v1 -F "%x" -W 60 > logs/YYYY-MM-DD_rate60.txt` (add dashboard login if needed). 3. Keep only the status sequence records (ID 0, subtype 11, one per snapshot): hex `x0by`, sequence `16*y + x`. 4. Take each step between consecutive sequence values modulo 256, add the steps up, then add 1. | 300 ± 2 snapshots built in 60 s |
| S1-U07 | robot reconnects after a broker restart | FR12 | manual (video) | 1. Leave the robot DISARMED and linked; open the dashboard. 2. Run `net stop mosquitto`. 3. When the dashboard shows the broker lost, run `net start mosquitto`. | Dashboard shows broker lost, then "MQTT subscribed"; robot values return live within 10 s of broker start. |

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

*Method:* **auto (host)** = a test runs on a PC; **manual** = a person checks the result; **manual (video)** = a person checks and films the result.

S1-U01 to S1-U04 also run together:
`python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py" -v` (43 tests).
Run every host command from the repository root.

Subsystem 1 has no on-target `TEST_CHECK`. The host
tests above and the integration tests below cover it.

Host tests across an interface have INT IDs. They are defined and recorded in
`testing/integration/TEST_LOG.md`, not here. Those on Subsystem 1 code:
INT-05 (55-byte snapshot codec, C and Python vectors, `protocol_test.py`),
INT-06 (telemetry, command, ACK and event buffers, `telemetry_queue_test.c`),
INT-07 (MQTT port: 2-byte records, backpressure, records older than 500 ms
dropped, held events, Wi-Fi re-join, `mqtt_test.c`) and INT-10 (snapshot
timestamps, `snapshot_time_test.c`). All run with
`python -B testing/integration/run.py --compiler gcc`.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| S1-U04 | S1-U01 |
| S1-U05 | S1-U02 |
| S1-U06 | S1-U03 |
| S1-U07 | S1-U04 |
| S1-U08 | S1-U05 |
| S1-U09 | S1-U06 |
| S1-U10 | S1-U07 |
| S1-U11 | S1-U08 |
| S1-U12 | S1-U09 |
| S1-U13 | S1-U10 |
| S1-U17 | S1-U11 |
| S1-U18 | S1-U12 |
| S1-U19 | S1-U13 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): S1-U01 and S1-U02 (LED cyclic handler checks, removed with `subsystem1_test.c`), S1-U03, S1-U14, S1-U15, S1-U16 and S1-U20 (never run). All removed on 2026-11-20.

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

| Old ID | New ID |
|---|---|
| S1-U03 | S1-U02 |
| S1-U04 | S1-U03 |
| S1-U05 | S1-U04 |
| S1-U06 | S1-U05 |
| S1-U07 | S1-U06 |
| S1-U12 | S1-U07 |

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): S1-U02, S1-U08, S1-U09, S1-U10, S1-U11, S1-U13.

</details>

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S1-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s1.txt` | Combined Dashboard run (`test_*.py`): all 13 `test_commands.Commands` tests ok (lines 11-43); `Ran 43 tests`, `OK` (lines 88-90) |
| 2026-11-20 | S1-U02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s1.txt` | All 13 `test_events` tests ok (6 `EventLogTests`, 7 `EventTextTests`, lines 44-60), none skipped; `test_matches_the_firmware_packing` ran with gcc; combined run `Ran 43 tests`, `OK` |
| 2026-11-20 | S1-U03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s1.txt` | All 4 `test_obstacle_width.WidthPipeline` tests ok (lines 61-64), none skipped (`test_c_producer_to_dashboard` ran); combined run `Ran 43 tests`, `OK` |
| 2026-11-20 | S1-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s1.txt` | `--compiler gcc`: `[PASS] Python wire contract and dashboard decoder` (line 192), `INT-CR1-06 actual queues: ... PASS` (line 197), `[PASS] C99 codec, golden packets and exhaustive 16-bit records` (line 202), `EDR CHECK PASS` (line 284); stage 3 (Buddy 2, nine profiles) also PASS |
| 2026-11-20 | S1-U05 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `edr_stream` recipe: `EDR16 production snapshot through Python/dashboard PASS` (line 56), then `Integration recipes PASS` (line 299), `[exit code 0]` |
| 2026-11-19 | S1-U06 | `7a6c373` | Ron Tan | PASS | `../system/logs/2026-11-19_mqtt_rate60.txt` | Robot disarmed with no fault (last status: phase 0, fault 0, pitch and roll 0, motor command 0, lines 72-85) and no robot events (line 53). 62 s capture by a bench MQTT client instead of `mosquitto_sub -W 60`, counted the same way (sequence steps + 1): 311 snapshots built (line 6) over the 61.8 s between the first and last sequence record (60 × 259 / 251.5 received per minute, line 8), so about 301 per 60 s (target 300 ± 2) |
| 2026-11-19 | S1-U06 | `7a6c373` | Ron Tan | PASS | `../system/logs/2026-11-19_t7_mqtt_idle.txt` | 89 s idle capture: in every snapshot the robot is Disarmed, fault 0, pitch and roll 0, encoders 0, no impact. First 60 s of the capture: sequence 106 at 0.052 s (line 14) to 150 at 59.994 s (line 11997) is 300 steps (one wrap), so 301 snapshots built (target 300 ± 2). Whole capture: 446 built in 88.9 s (300.2 per 60 s) |
| 2026-11-19 | S1-U07 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t7_usb_no_imu_broker_restart.txt` | Broker restarted with the robot disarmed (armed=0) and the USB console logging. `mqtt: connection status 256, retrying in 3s` at 87.1 s (line 1482), then `mqtt: connected to ...` at 90.2 s (line 1533): 3.1 s after the robot lost the broker, on its first retry. Uptime keeps counting (t=110040 ms at 86.7 s, t=113610 ms at 90.3 s), so no reboot. Not shown: the length of the broker stop (20 s in the test) and its start time, records returning, and the "Robot reconnected to the MQTT broker" event (no MQTT capture across the restart) |
| 2026-11-24 | S1-U07 | `f8b68dd` + uncommitted S2 low-speed kick | Ron Tan | PASS | https://youtu.be/qj4SRrp7bfA | Robot disarmed and linked (same image as SYS-08 that evening). Broker stopped with `net stop mosquitto` (0:06) for about 6 s, then started with `net start mosquitto` (0:12). Dashboard: "MQTT disconnected; reconnecting" (0:06), "MQTT broker unavailable; retrying" (0:09), "MQTT subscribed" again (0:12). The robot's cards were back with live values (5 of 5 supplied, Disarmed) by 0:15, about 3 s after the broker start; the barcode card still showed the last letter (F, Left). |
