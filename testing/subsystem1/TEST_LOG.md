# Subsystem 1 — test log

**Subsystem:** Binary telemetry, command/telemetry buffers, MQTT dashboard, status LED  ·  **Student ID:** 2500238  ·  **Code:** `app_program/subsystem1/`, `app_program/external_systems/Dashboard/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S1-U01 | LED cyclic handler exists and is running | TEAM.md Buddy 1 LED GP19; IF-05 (`tid_led`) | auto | Integrated `make TEST=1` image, `subsystem1_test.c`. Note: `usermain()` in `app_main.c` does not call `subsystem1_test()` yet, so this line is not printed until that call is added. | `[TEST] S1-U01 PASS` |
| S1-U02 | LED task sleeps between ticks, not busy-looping (waiting in ≥ 9 of 10 samples over about 1 s) | TEAM.md Buddy 1 LED GP19; IF-05 (`tid_led`) | auto | Integrated `make TEST=1` image, `subsystem1_test.c`. Same note as S1-U01. | `[TEST] S1-U02 PASS` |
| S1-U03 | Status LED on GP19 toggles every 250 ms | TEAM.md Buddy 1 LED GP19 | manual | 1. Flash the normal image (`make`) and power the robot. 2. Put a scope probe on GP19, or film the LED for 10 s. 3. Measure the period and the on time. | 2 Hz blink: period 500 ± 10 ms, on time 250 ± 10 ms (50 % duty); 20 ± 1 blinks in 10 s |
| S1-U04 | Dashboard command bridge: STOP/RESET/START/MAP frames on `csc02/command` (QoS 0, not retained); sequences 0-127, so a USB command's ACK (bit 7 set) never matches; ACK matched by sequence and opcode; retained or wrong ACKs ignored; reject and no-ACK messages; MAP table changes only on accept; send-all A-Z; nothing published while the broker is down; command API accepts only the local page | FR12, NFR8 | auto (host) | `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_commands.py" -v` (class `Commands`, 13 tests) | `Ran 13 tests`, `OK` |
| S1-U05 | Dashboard telemetry receiver and display: exactly the three topics (no wildcards), resubscribe on reconnect; all 16 record IDs and units; malformed packets and other topics rejected; status records (phase, fault and cause, quality, barcode, hump height and run maximum, widths, bypass side); IMU fault bit; impact and IMU fault held 2 s; drops in the last 60 s; fault 8 (lifted); barcode read history; `/api/telemetry` and the browser cards | FR13, FR11, FR12, FR3 | auto (host) | `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_packed16.py" -v` (class `DashboardTests`, 13 tests). The browser card check (`test_dashboard_display.cjs`) runs only when Node.js is on PATH. | `Ran 13 tests`, `OK`, and no `Browser display test not run` line |
| S1-U06 | Dashboard event log: text for barcode attempts, decoded turns, humps, obstacle route and broker link events; bad framing rejected; same packing as firmware `shared/events.h`; retained or invalid records ignored; keeps the last 200 rows; "Robot offline" once after 3 s, then "Robot back online"; each new fault logged once; no false "Robot offline" while the dashboard itself lost the broker | FR13, FR12, FR7, FR9, FR10 | auto (host) | `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_events.py" -v` (classes `EventTextTests`, `EventLogTests`, 13 tests). The firmware packing test needs `gcc` on PATH; without it, it is skipped. | `Ran 13 tests`, `OK`, no skipped test |
| S1-U07 | Obstacle width on the dashboard: width in 5 mm units, left/right halves, kept until the quality bit clears or the broker reconnects, out-of-range values rejected; C width producer to dashboard | FR7, FR13 | auto (host) | `python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_obstacle_width.py" -v` (class `WidthPipeline`, 4 tests). `test_c_producer_to_dashboard` is skipped unless `testing/integration/.build/width_avoidance` was built first (run the S1-U09 command). | `Ran 4 tests`, `OK`, no skipped test |
| S1-U08 | EDR16 data gate, stages 1-2: Python codec (all 65,536 words, byte vectors, units, faults 0-8) and every Dashboard test; C codec `shared/edr.c` with golden bytes; S1 binary inbox host test `app_program/subsystem1/tests/host/telemetry_host_test.c` (copied queues, full-queue drop counting, urgent STOP, telemetry older than 500 ms dropped, 2-byte robot events) | FR12, FR13, NFR7, NFR8 | auto (host) | `python -B testing/data_representation/run.py --compiler gcc` (or `--compiler clang`). Needs a native C compiler. Stage 3 is Buddy 2's motor tests. | `[PASS] Python wire contract and dashboard decoder`; `[PASS] C99 codec, golden packets and exhaustive 16-bit records` (with the `INT-CR1-06 actual queues: ... PASS` line from the S1 host test); `EDR CHECK PASS` |
| S1-U09 | Production snapshot to dashboard: a `telemetry_codec.c` snapshot becomes 33 two-byte records; the Python receiver accepts all 33, rejects none and shows the expected values (encoders, IR, barcode, hump height, width and halves, servo, pitch/roll, speed, fault and cause, IMU fault, drop count) | FR12, FR13, FR11 | auto (host) | `python -B testing/integration/run.py --compiler gcc`. Recipe in `testing/integration/Makefile`: `edr_stream_test.c` + `telemetry_codec.c` -> `.build/edr_stream`, then `edr_stream_test.py`. | `EDR16 production snapshot through Python/dashboard PASS`, then `Integration recipes PASS` |
| S1-U10 | Snapshot rate: one full snapshot every 200 ms | NFR7 | manual | 1. Robot DISARMED and still on the desk, no fault, linked to the broker. Do not touch it. 2. On the PC run `mosquitto_sub -h <broker> -t Robopico/sensors/edr16/v1 -F "%x" -W 60 > logs/YYYY-MM-DD_rate60.txt` (add the dashboard login if the broker needs one). 3. Keep only the status sequence records (ID 0, subtype 11, one per snapshot). Their hex is `x0by`; sequence = 16·y + x. 4. Add up the steps between consecutive sequence values (mod 256), then add 1. | 300 ± 2 snapshots built in 60 s |
| S1-U11 | Snapshot delivery over MQTT | FR12 | manual | 1. Use the capture from S1-U10, or repeat its steps 1-2. 2. Count the sequence records received. 3. Divide by the snapshots built (S1-U10 step 4). | ≥ 95 % received (≥ 285 of 300) |
| S1-U12 | Command ACKs for 20 dashboard commands | FR12 | manual | 1. Robot DISARMED on the desk; dashboard open at http://127.0.0.1:8001. 2. Open the browser developer tools, Network tab. 3. Press STOP 20 times, about 2 s apart. 4. For each press, read the reply message and the time of the `POST /api/command` request. | 20 of 20 replies "RoboPico accepted stop"; every request takes ≤ 1 s |
| S1-U13 | Command round trips for 50 dashboard commands | NFR8 | manual | 1. Robot, broker and dashboard PC on one 2.4 GHz network; robot within 5 m of the access point; no other heavy traffic. 2. Do S1-U12 steps 2-4 with 50 presses. | All 50 acknowledged; ≥ 48 of 50 (95 %) within 1 s |
| S1-U14 | Dashboard shows a change within 1 s; every field is shown | FR13 | manual | 1. Robot DISARMED on the desk, linked; dashboard open. 2. Film the robot and the screen together (30 fps or more). 3. Tilt the robot about 10° nose up; slide black tape under the left line IR; turn the left wheel by hand. Do each 5 times. 4. In the video, count the frames from each change to the new value on screen. 5. Check that every FR13 field shows a value: motor command and power, speed, encoder counts, line and barcode IR, distance, width, servo angle, acceleration, pitch/roll, hump height and run maximum, phase, fault and cause. | Each change shown within 1 s (5 of 5 tries each); every field the robot sends shows a value, not "Awaiting" |
| S1-U15 | A reading not updated for 5 s is marked stale | FR13 | manual | 1. Dashboard showing live values. 2. Switch the robot off and start a stopwatch (or film the screen). 3. Note when the cards show "stale". | Readings marked stale 5-6 s after power-off (the page repaints every 1 s); the last values stay on screen |
| S1-U16 | "Robot offline" is logged 3 s after the last record | FR13 | manual | 1. Do S1-U15 steps 1-2 with the Events tab open. 2. Note when the "Robot offline" row appears. 3. Switch the robot on again. | One "Robot offline: no data for 3 s" (WARN) row, 3-4 s after power-off; a "Robot back online" row when records return |
| S1-U17 | Event log shows the robot's route events | FR13, FR7, FR9, FR10 | manual | 1. Dashboard open on the Events tab. 2. Run one mission on the track that passes a barcode (`*A*`, `*F*` or `*Z*`), the hump and the obstacle. Film the run. 3. Compare the rows with the video. | Rows in order, none missing: each barcode attempt (attempt number, bars read) and the decoded turn; hump detected, crest and level with height; obstacle detected (range), scanned (width, side), bypass started, back on the line (how, seconds). The 200-row limit is checked by S1-U06. |
| S1-U18 | Robot reconnects after a broker restart | FR12 | manual | 1. Robot DISARMED on the desk, linked; dashboard open; USB console logging. 2. Stop the broker for 20 s. 3. Start it and start a stopwatch. 4. Note when the USB console shows `mqtt: connected to ...`. | Reconnected within 10 s of the broker start; robot records return; a "Robot reconnected to the MQTT broker after N s" event is logged; no `[RESET]` line (no reboot) |
| S1-U19 | Robot leaves and rejoins Wi-Fi after an access-point loss, without a reboot | FR12 | manual | 1. Robot DISARMED on the desk, linked; USB console logging. 2. Switch the access point off for 30 s, then on. 3. Watch the USB console and the dashboard. | About 10 s after the loss the console shows `wifi: no link for ... s, leaving to re-join`; after the access point returns it shows `wifi: link up` and `mqtt: connected to ...`, and records return; no `[RESET]` line (no reboot) |
| S1-U20 | Events raised during a link outage arrive after reconnection; driving does not depend on the network | NFR8, FR12 | manual | 1. Robot running a mission on the track; dashboard open on the Events tab. 2. Switch the access point off just before the robot reaches a barcode. Switch it on 30 s later. 3. Film the run. | The robot keeps driving and reads the barcode during the outage; after the rejoin, a "Robot reconnected to the MQTT broker" row comes first, then the barcode attempt rows in order |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**auto (host)** = a test run on a PC with no Pico (exact command in the row);
**manual** = a person observes the result (LED, scope, meter, phone app…).

S1-U04 to S1-U07 also run together:
`python -B -m unittest discover -s app_program/external_systems/Dashboard -p "test_*.py" -v` (43 tests).
Run every host command from the repository root.

Subsystem 1 has no on-target `TEST_CHECK` for the telemetry path yet. The host
tests above and the integration tests below cover it.

Host tests across an interface have INT IDs. They are defined and recorded in
`testing/integration/TEST_LOG.md`, not here. Those on Subsystem 1 code:
INT-CR1-05 (55-byte snapshot codec, C and Python vectors, `protocol_test.py`),
INT-CR1-06 (telemetry, command, ACK and event buffers, `telemetry_queue_test.c`),
INT-CR1-07 (MQTT port: 2-byte records, backpressure, records older than 500 ms
dropped, held events, Wi-Fi re-join, `mqtt_test.c`) and INT-CR1-10 (snapshot
timestamps, `snapshot_time_test.c`). All run with
`python -B testing/integration/run.py --compiler gcc`.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
