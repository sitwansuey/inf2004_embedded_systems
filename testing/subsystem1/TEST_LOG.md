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
| 2026-11-24 | S1-U07 | `f8b68dd` + uncommitted S2 low-speed kick [^s1-c1] | Ron Tan | PASS | https://youtu.be/qj4SRrp7bfA | Robot disarmed and linked (same image as SYS-08 that evening). Broker stopped with `net stop mosquitto` (0:06) for about 6 s, then started with `net start mosquitto` (0:12). Dashboard: "MQTT disconnected; reconnecting" (0:06), "MQTT broker unavailable; retrying" (0:09), "MQTT subscribed" again (0:12). The robot's cards were back with live values (5 of 5 supplied, Disarmed) by 0:15, about 3 s after the broker start; the barcode card still showed the last letter (F, Left). |

### Restored result rows (audit, 2026-11-25)

These result rows were deleted from the table above in `5f93cb6` (2026-11-25). They are copied back word for word from `d18dd38`, the last commit that had them; the first two columns are the only additions. Their *Test ID* cells use the IDs in force at `d18dd38`, not today's IDs, and some of those numbers now name different tests (second column, and *ID history (git record)* at the end of this file). Nothing was re-run.

| Restored from | Test ID meaning (then → now) | Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|---|
| `d18dd38` (deleted in `5f93cb6`) | S1-U05 at `d18dd38` = Dashboard telemetry receiver and display: exactly the three topics …; test removed in `5f93cb6`; today's S1-U05 is a different test (production snapshots reach the dashboard correctly) | 2026-11-20 | S1-U05 | `ed197b3` | Ron Tan | INCOMPLETE | `logs/2026-11-20_host_s1.txt` | All 13 `test_packed16.DashboardTests` tests ok (lines 65-85; combined run `Ran 43 tests`, `OK`), but `Browser display test not run: Node.js is not on PATH` (line 78): the browser card check did not run (Node.js is not installed on this PC), so the test is not complete. Run again with Node.js on PATH |
| `d18dd38` (deleted in `5f93cb6`) | S1-U11 at `d18dd38` = Snapshot delivery over MQTT; test removed in `5f93cb6`; no current test uses S1-U11 | 2026-11-19 | S1-U11 | `7a6c373` | Ron Tan | FAIL | `../system/logs/2026-11-19_mqtt_rate60.txt` | Same capture as the S1-U10 row above: 260 of 311 snapshots received (83.6 %, lines 5-7), target ≥ 95 %. 51 snapshots lost in 7 bursts of 2-22 (gaps at 10.4, 15.5, 20.0, 22.7, 24.9, 44.5 and 55.1 s, lines 9-52); no undecodable records (line 4) |
| `d18dd38` (deleted in `5f93cb6`) | S1-U11 at `d18dd38` = Snapshot delivery over MQTT; test removed in `5f93cb6`; no current test uses S1-U11 | 2026-11-19 | S1-U11 | `7a6c373` | Ron Tan | PASS | `../system/logs/2026-11-19_t7_mqtt_idle.txt` | Same capture and 60 s window as the S1-U10 row above: 293 of 301 snapshots received (97.3 %, target ≥ 95 %). Whole 88.9 s capture: 438 of 446 (98.2 %); largest loss 5 in a row (45.2-45.6 s); no gap over 3 s |
| `d18dd38` (deleted in `5f93cb6`) | S1-U12 at `d18dd38` = Command ACKs for 20 dashboard commands; test removed in `5f93cb6`; no current test uses S1-U12 | 2026-11-19 | S1-U12 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_mqtt_acks20.txt` | Different procedure: 20 commands (sequences 10-29) published straight to `csc02/command` by a bench MQTT client and timed to their ACK at that client, not 20 STOP presses on the dashboard timed in the browser; the command type and spacing are not in the log. 20 of 20 ACKed, all with result 0 (accepted) (lines 2-3, 11-32), but only 13 within 1 s (line 4); min 92.5 ms, median 640.2 ms, max 2470.3 ms (lines 6-9). The 7 slow replies would miss the ≤ 1 s limit |
| `d18dd38` (deleted in `5f93cb6`) | S1-U19 at `d18dd38` = Robot leaves and rejoins Wi-Fi after an access-point loss, without a …; test removed in `5f93cb6`; no current test uses S1-U19 | 2026-11-19 | S1-U19 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t6_usb_sonar_wifi_off_on.txt` | Robot disarmed, USB console logging; the access point off and on times are not logged. Link lost at 191.2 s (`authentication rejected`, line 3171); the robot tried to re-join every 5-6 s (196.2 to 230.6 s, lines 3254-3909) and printed `wifi: link up` at 233.5 s (line 3956). The expected `no link for ... s, leaving to re-join` line never appears. MQTT did not reconnect after that link up: 128 `mqtt: publish failed, err=-3` lines (209.4-264.4 s, lines 3480-4519). The link dropped again at 271.3 s (`SSID not found`, line 4632), came back at 279.2 s, and `mqtt: connected to ...` followed at 280.6 s (lines 4772-4793). Uptime kept counting (t=960840 ms at 190.0 s, t=1050840 ms at 280.1 s; no reboot), but the USB output stops at 332.3 s (line 5611) with no end marker. Records returning not captured |
| `d18dd38` (deleted in `5f93cb6`) | S1-U11 at `d18dd38` = Snapshot delivery over MQTT; test removed in `5f93cb6`; no current test uses S1-U11 | 2026-11-20 | S1-U11 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run with the robot driving, not the idle desk set-up of S1-U10. 3641 sequence records received; counting the sequence steps inside each stretch with no gap over 3 s (14 gaps of 3.1-10.1 s, e.g. 206.9-217.0 s) gives 4500 built: 80.9 % (target ≥ 95 %) |
| `d18dd38` (deleted in `5f93cb6`) | S1-U11 at `d18dd38` = Snapshot delivery over MQTT; test removed in `5f93cb6`; no current test uses S1-U11 | 2026-11-20 | S1-U11 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run with the robot driving; the capture spans 5 robot boots ("connected ... after boot" events at 36.3, 282.1, 1720.9, 1825.9 and 2353.9 s). 4412 sequence records received; counted the same way inside the 51 stretches with no gap over 3 s, 6178 built: 71.4 % (target ≥ 95 %). Old records that arrive just after an outage (2818.5 s, lines 122462-122587) make this count approximate |
| `d18dd38` (deleted in `5f93cb6`) | S1-U13 at `d18dd38` = Command round trips for 50 dashboard commands; test removed in `5f93cb6`; no current test uses S1-U13 | 2026-11-20 | S1-U13 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run, not the desk set-up, and 31 commands, not 50 (5 STOP, 13 RESET, 10 START, 3 MAP; sequences 0-30, lines 77-77029), timed from each command to its ACK as seen by the capture client, not in the browser. 31 of 31 ACKed (28 accepted, 3 rejected); 25 within 1 s (81 %, target ≥ 48 of 50); min 80 ms, median 160 ms, max 2409 ms (RESET at 179.1 s, lines 26972-27291) |
| `d18dd38` (deleted in `5f93cb6`) | S1-U13 at `d18dd38` = Command round trips for 50 dashboard commands; test removed in `5f93cb6`; no current test uses S1-U13 | 2026-11-20 | S1-U13 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run, not the desk set-up; 47 commands (11 STOP, 22 RESET, 14 START; sequences 34-80, lines 2983-122258), timed by the capture client. 42 ACKed (37 accepted, 5 rejected); the 5 without an ACK were sent while the robot was offline (24.8-33.4 s, before "Robot connected ... (20 s after boot)" at 36.3 s; 2335.7 and 2350.9 s, before the connect at 2353.9 s). 22 of 42 within 1 s (52 %, target ≥ 48 of 50); median 837 ms, max 18.6 s (RESET sent at 2562.8 s, ACK at 2581.5 s, lines 106942-107123) |
| `d18dd38` (deleted in `5f93cb6`) | S1-U17 at `d18dd38` = Event log shows the robot's route events; test removed in `5f93cb6`; no current test uses S1-U17 | 2026-11-20 | S1-U17 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Event rows read from the MQTT capture, not the Events tab; the run was not filmed, so "none missing" cannot be checked. Barcode: attempt rows with attempt number and bars read (17/29 at 123.5 s; 13/29, then reverse scan 18/29 at 406.8-411.2 s), and "decoded A, 29/29" with "A -> Straight" at 568.9 s (lines 76420-76425). Obstacle: detected 18 cm, scanned 23 cm left, bypass started 32° left, back on the line after 27 s (spot search), in order at 221.2-253.0 s (lines 31093-33143). Hump: 5 detections, each with detected and crest rows (height 1.6 cm at 253.0 s, otherwise "not measured"), but no "level" row after any of them |
| `d18dd38` (deleted in `5f93cb6`) | S1-U17 at `d18dd38` = Event log shows the robot's route events; test removed in `5f93cb6`; no current test uses S1-U17 | 2026-11-20 | S1-U17 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Event rows read from the MQTT capture; the run was not filmed. No barcode rows in this capture. Obstacle, twice and in order: detected 18 cm, scanned 29 cm right, bypass started 32° right, back on the line after 22 s (line found on the way back, heading restored) at 50.0-72.3 s (lines 5690-9472); detected 17 cm, scanned 25 cm left, bypass started 57° left, back after 19 s at 2721.8-2749.7 s (lines 120459-122164). Hump: detected, crest and level rows in order at 2593.6-2595.0 s and 2665.5 s (lines 108576-108664, 114053-114068), but the height is "not measured" both times; the 13.9° hump at 2038.4 s has only detected and crest (1.8 cm) rows (lines 69472-69483) |

**Note on the restored FAIL row (audit, 2026-11-25).** S1-U11 FAIL, 2026-11-19, `7a6c373`: S1-U11 is the `d18dd38` test *Snapshot delivery over MQTT* (expected ≥ 95 % of snapshots received). The row's own note says what went wrong (260 of 311 received, 83.6 %, lost in 7 bursts). The next restored row is a PASS of the same test on the same commit `7a6c373` from a different capture (idle capture t7, 97.3 %); both runs are on the same code, so that PASS does not show a fix. The two later runs (`89f1bdc`, 80.9 %; `662c1ef`, 71.4 %) are INCOMPLETE and below the target. No fixing commit is recorded. The test was removed in `5f93cb6`. The same capture is the step 1 failure in the restored SYS-14 FAIL row of `testing/system/TEST_LOG.md`.

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

The 2026-11-19 INCOMPLETE row now labelled S1-U07 was recorded as S1-U18 against the `d18dd38` definition (USB console shows `mqtt: connected` within 10 s of the broker start, records return, a reconnect event is logged, no reboot). The 2026-11-24 PASS row is recorded against the narrower S1-U07 definition that first appears in `5f93cb6` (dashboard shows the broker lost, then "MQTT subscribed"; values return within 10 s).

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| S1-U01 | LED cyclic handler exists and is running | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: dashboard command delivery and acknowledgements |
| S1-U02 | LED task sleeps between ticks, not busy-looping (waiting in ≥ 9 of 10 samples over about 1 s) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: dashboard event text and log handling |
| S1-U03 | Status LED on GP19 toggles every 250 ms | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: obstacle widths reach the dashboard correctly |
| S1-U04 | Dashboard command bridge: STOP/RESET/START/MAP frames on `csc02/command` (QoS 0, not retained); sequences 0-127, so a USB command's ACK (bit 7 set) never matches; ACK matched by sequence and opcode; retained or wrong ACKs ignored; reject and no-ACK messages; MAP table changes only on accept; send-all A-Z; nothing published while the broker is down; command API accepts only the local page | S1-U01 | yes | different test: binary telemetry passes the data checks |
| S1-U05 | Dashboard telemetry receiver and display: exactly the three topics (no wildcards), resubscribe on reconnect; all 16 record IDs and units; malformed packets and other topics rejected; status records (phase, fault and cause, quality, barcode, hump height and run maximum, widths, bypass side); IMU fault bit; impact and IMU fault held 2 s; drops in the last 60 s; fault 8 (lifted); barcode read history; `/api/telemetry` and the browser cards | removed from the table in `5f93cb6` (S1-U02 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | different test: production snapshots reach the dashboard correctly |
| S1-U06 | Dashboard event log: text for barcode attempts, decoded turns, humps, obstacle route and broker link events; bad framing rejected; same packing as firmware `shared/events.h`; retained or invalid records ignored; keeps the last 200 rows; "Robot offline" once after 3 s, then "Robot back online"; each new fault logged once; no false "Robot offline" while the dashboard itself lost the broker | S1-U02 | yes | different test: snapshots are built every 200 ms |
| S1-U07 | Obstacle width on the dashboard: width in 5 mm units, left/right halves, kept until the quality bit clears or the broker reconnects, out-of-range values rejected; C width producer to dashboard | S1-U03 | yes | different test: robot reconnects after a broker restart |
| S1-U08 | EDR16 data gate, stages 1-2: Python codec (all 65,536 words, byte vectors, units, faults 0-8) and every Dashboard test; C codec `shared/edr.c` with golden bytes; S1 binary inbox host test `app_program/subsystem1/tests/host/telemetry_host_test.c` (copied queues, full-queue drop counting, urgent STOP, telemetry older than 500 ms dropped, 2-byte robot events) | S1-U04 | yes | not used |
| S1-U09 | Production snapshot to dashboard: a `telemetry_codec.c` snapshot becomes 33 two-byte records; the Python receiver accepts all 33, rejects none and shows the expected values (encoders, IR, barcode, hump height, width and halves, servo, pitch/roll, speed, fault and cause, IMU fault, drop count) | S1-U05 | yes | not used |
| S1-U10 | Snapshot rate: one full snapshot every 200 ms | S1-U06 | yes | not used |
| S1-U11 | Snapshot delivery over MQTT | removed from the table in `5f93cb6` (S1-U08 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S1-U12 | Command ACKs for 20 dashboard commands | removed from the table in `5f93cb6` (S1-U09 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S1-U13 | Command round trips for 50 dashboard commands | removed from the table in `5f93cb6` (S1-U10 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S1-U14 | Dashboard shows a change within 1 s; every field is shown | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S1-U15 | A reading not updated for 5 s is marked stale | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S1-U16 | "Robot offline" is logged 3 s after the last record | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S1-U17 | Event log shows the robot's route events | removed from the table in `5f93cb6` (S1-U11 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S1-U18 | Robot reconnects after a broker restart | S1-U07 | yes | not used |
| S1-U19 | Robot leaves and rejoins Wi-Fi after an access-point loss, without a reboot | removed from the table in `5f93cb6` (S1-U13 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S1-U20 | Events raised during a link outage arrive after reconnection; driving does not depend on the network | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. A cell that names exactly one existing commit is not marked. Any other cell keeps its original text and carries a footnote; the footnotes are at the end of this file.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened.

[^s1-c1]: Commit not recorded precisely: per the original entry the robot ran `f8b68dd` plus uncommitted changes ("S2 low-speed kick"), so the exact tested code is not in the repository. `f8b68dd` itself exists in the repository history.
