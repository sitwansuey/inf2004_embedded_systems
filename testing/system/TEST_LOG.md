# System — test log

Tests of the complete product against its requirements, run on the final hardware set-up. Run the full set before every demo and before hand-in.

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| SYS-01 | obstacle status and route events match the bypass | FR7 | manual | 1. Capture MQTT or open the dashboard. 2. Run an obstacle bypass. 3. Read status during the bypass and events afterwards. | Status shows controller phase, bypass side and widths (total in 5 mm steps). Events in order: detected (range), scanned (width, side), bypass started (side, heading), back on the line or ended (how, seconds). |
| SYS-02 | snapshot rate stays within the required limit | NFR7 | manual | 1. Leave the robot disarmed, still and broker-connected. 2. Capture 60 s of telemetry. 3. Count received and missing snapshots from sequence numbers. | 300 ± 2 snapshots in 60 s (5 Hz). |
| SYS-03 | robot follows straight lines and bends | FR8, FR4 | manual (video) | 1. Flash the mission image. 2. Place the robot on a straight line leading into bends. 3. Start tracking and film the straight and bends. | Follows the line along the straight and through bends, stays on it and keeps driving to the end of the clip. |
| SYS-04 | robot stops and bypasses obstacles on both sides | FR5, FR6, FR7 | manual (video) | 1. Flash the mission image. 2. Place a box on a straight line. 3. Start tracking; film the stop, scan, bypass and return. 4. Repeat with the box placed for the other bypass side. | Each run: stops before the box without touching it, stays still during the sonar scan, passes on the chosen side and returns to the line. One left run and one right run. |
| SYS-05 | robot crosses humps and reports height events | FR2, FR10 | manual (video) | 1. Flash the mission image; open the dashboard. 2. Place a low hump under a wheel path. 3. Start tracking. 4. Film the crossing, then Events. | Crosses without stalling and keeps following the line. Events tab shows "Hump detected: climbing, tilt N deg" and "Hump crest: going down slowly, height N cm". |
| SYS-06 | turnover stops the robot with a vehicle fault | FR3, FR1 | manual (video) | 1. Flash the mission image; open the dashboard. 2. Start tracking. 3. Turn the driving robot over by hand. 4. Film the robot and dashboard. | Wheels stop within about 1 s of turnover. Dashboard: "Robot stopped: fault 1 (sensor)", Mission state Fault, Motion STOPPED. Robot stays stopped to the end of the clip. |
| SYS-07 | barcode reads trigger the mapped junction turns | FR9, FR4 | manual (video) | 1. Flash the mission image. 2. Place a Code 39 card before a junction; map its letter to Left on the dashboard. 3. Start tracking; film the read and turn. 4. Repeat mapped to Right, then with a card mapped to U-turn. | Each run: reads the card (one reverse re-read allowed), then acts at the junction: Left/Right takes that branch; U-turn turns round and follows the line back. |
| SYS-08 | dashboard recovers automatically after a network drop | FR12, FR13 | manual (video) | 1. Leave the robot disarmed and linked; open the dashboard. 2. Film the dashboard. 3. Switch the access point off; switch it on when the dashboard shows the robot offline. 4. Film until live values return. | Within about 4 s of loss: a "Robot offline: no data for 3 s" row appears; cards marked stale. After access point returns, robot rejoins automatically: a "Robot back online after N s without data" row appears; live cards return within 30 s. |

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

SYS-03 to SYS-08 are the **core system tests**: the set run before a demo. Each is a filmed run, and its video link goes in *Log / evidence*. SYS-01 and SYS-02 check the status fields and route events, and the snapshot rate.

*Method:* **manual** = a person checks the result; **manual (video)** = a person checks and films the result.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| SYS-03 | SYS-01 |
| SYS-04 | SYS-02 |
| SYS-05 | SYS-03 |
| SYS-07 | SYS-04 |
| SYS-08 | SYS-05 |
| SYS-09 | SYS-06 |
| SYS-11 | SYS-07 |
| SYS-13 | SYS-08 |
| SYS-14 | SYS-09 |
| SYS-17 | SYS-10 |
| SYS-18 | SYS-11 |
| SYS-21 | SYS-12 |
| SYS-22 | SYS-13 |
| SYS-23 | SYS-14 |
| SYS-24 | SYS-15 |
| SYS-25 | SYS-16 |
| SYS-26 | SYS-17 |
| SYS-27 | SYS-18 |
| SYS-28 | SYS-19 |
| SYS-29 | SYS-20 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): SYS-01 and SYS-02 (template examples), SYS-06, SYS-10, SYS-12, SYS-15, SYS-16, SYS-19 and SYS-20 (never run). Removed on 2026-11-20.

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

| Old ID | New ID |
|---|---|
| SYS-06 | SYS-01 |
| SYS-13 | SYS-02 |
| SYS-15 | SYS-03 |
| SYS-16 | SYS-04 |
| SYS-17 | SYS-05 |
| SYS-18 | SYS-06 |
| SYS-19 | SYS-07 |
| SYS-20 | SYS-08 |

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): SYS-01, SYS-02, SYS-03, SYS-04, SYS-05, SYS-07, SYS-08, SYS-09, SYS-10, SYS-11, SYS-12, SYS-14.

</details>

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | SYS-01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-19 | SYS-02 | `7a6c373` | Ron Tan | PASS | `logs/2026-11-19_mqtt_rate60.txt`, `logs/2026-11-19_t7_mqtt_idle.txt`, `logs/2026-11-19_flash_8b_boot.txt` | Robot disarmed and still. Rate test: 311 sequence numbers in 62 s (about 301 per 60 s). Idle capture t7: 446 sequence numbers in 88.9 s (300.3 per 60 s), with no impacts or fault changes. |
| 2026-11-20 | SYS-01 | `662c1ef` | Ron Tan | PASS | `logs/2026-11-20_track_run3_mqtt.txt` | 2 bypasses. Status showed the phase (Acquisition to Restore heading), side and widths (27.5 cm, L 16.0 / R 11.2; 24.0 cm, L 12.0 / R 12.8). Events in order: detected (18 and 17 cm), scanned (width, side), bypass started (side, heading), back on the line (how, 22 and 19 s). The event width is 1-1.5 cm above the status width (29 vs 27.5; 25 vs 24.0). |
| 2026-11-20 | SYS-03 | `89f1bdc` / `662c1ef` | Ron Tan | PASS | https://youtu.be/jEHOhwIJ0F8 | Track day; the clip has no clock. That day the robot ran `89f1bdc` (from 13:48), a test build (from 15:26) and `662c1ef` (from 15:35); all three have the same line following. The robot drives down the straight past the blue tape (0:00-0:04), round the U-bend (0:04-0:10), through the S-bend (0:10-0:16) and into the next corner, still turning when the clip ends (0:21); it stays on the line throughout. Nobody touches it (at 0:18 a hand slides away a phone lying on the mat beside it). The dashboard is not in shot. The clip starts 16.5 s into the original recording, cutting the part where the robot stands still. |
| 2026-11-11 | SYS-04 | `af11df6` or `1d088b2` | Ron Tan | PASS | https://youtu.be/KhjVVtTxKeM | Right-side run (0:21-0:49 of the video), filmed on 11 Nov in another room with a clear box (source clip dated 19:48). The robot ran checkpoint 4 (`1d088b2`, from 19:08) or checkpoint 3 (`af11df6`, reflashed at about 19:32); which one is not known. It stops short of the box (0:22), stands about 11 s while the sonar scans, reverses, goes round on the right, stops once to look, turns back in and reaches the line about 23.5 s after the stop (0:46), then moves along it. Nobody touches it. |
| 2026-11-20 | SYS-04 | `662c1ef` | Ron Tan | PASS | https://youtu.be/KhjVVtTxKeM, `logs/2026-11-20_track_run3_mqtt.txt` | Left-side run (0:00-0:21 of the video): the 25 cm box in track run 3 at 16:20:54. The robot stops short of the box without touching it (0:03), stands about 8 s while the sonar scans (half-widths L 12.0 / R 12.8 cm in the log), backs up a little (0:11), turns out to the left (0:13), stops once to look (0:15-0:17), turns back in and reaches the line beyond the box at about 0:21, with no on-the-spot search. Log: out at 57°, "Back on the line after 19 s (line found on the way back, heading restored)". The video cuts at 0:21 as the robot reaches the line. Nobody touches it. |
| 2026-11-25 | SYS-05 | `ac5485d` | Ron Tan | PASS | https://youtu.be/qSe3YlFr-ys, `logs/2026-11-25_hump_height_mqtt.txt` | Checkpoint 8g image. The low red strip lay beside the line, so the right-side wheel rode along it (0:03-0:05.6); the robot slowed but did not stall, and kept following the line after it (0:06; in the log until 10:16:31.7). Events: "Hump detected: climbing, tilt 4.8 deg" (10:16:24) and "Hump crest: going down slowly, height 0.4 cm" (10:16:25). Power 20 % climbing, 10 % at 10:16:23.3-24.0, then the 15 % descent cap from 10:16:25.2. At 10:16:31.8, off camera, the log shows the robot lifted and turned over (both line sensors off the floor, accel_z −10.7 m/s²), which stopped the run with fault 1 (sensor, cause Tilt) at 10:16:32.1. The strip's real height was not measured. |
| 2026-11-20 | SYS-06 | `662c1ef` | Ron Tan | PASS | https://youtu.be/gjNppsurhxw, `logs/2026-11-20_track_run3_mqtt.txt` | Track run 3. The first frame is held 3 s so the clip reaches 15 s. The robot is line tracking (log). A hand takes it at 0:07.5 and turns it wheels-up at about 0:09.7; both wheels are still from 0:10.2, about 0.5 s later, and do not turn while it is in shot (to 0:13.4). Dashboard (0:14-0:15.7): Mission state Fault, Motion STOPPED and "Robot stopped: fault 1 (sensor)" at 16:17:02, after a 3 s data gap ("Robot offline: no data for 3 s" and "Robot back online after 3 s without data" are stamped the same second). Log: fault 1, cause Tilt, motors STOP at 0 % and accel_z −9.5 m/s² (upside down) at 2489.2 s; it stayed in Fault until the reset at 2581.5 s. |
| 2026-11-11 | SYS-07 | `af11df6` or `1d088b2` | Ron Tan | PASS | https://youtu.be/b28la7W1juI | Filmed on 11 Nov in another room (source clips dated 19:41-19:46), not on the 20 Nov track. The robot ran checkpoint 4 (`1d088b2`, from 19:08) or checkpoint 3 (`af11df6`, reflashed at about 19:32); which one is not known. Letters read from the bars, actions from the captions (the dashboard is not in shot). Card Z mapped to Left (0:00-0:16): the robot drives onto the card, backs off it (0:06-0:10), crosses it again (0:11.5-0:14) and turns left at the crossing (0:14-0:15.5). Card Z mapped to Right (0:16-0:32): the same back-up and second pass, then a right turn (0:29.5-0:31). Card A mapped to U-turn (0:32-0:50): one pass, then it turns round at the crossing (0:43-0:45) and follows the line back to the card (0:47-0:50). No hands in shot. |
| 2026-11-24 | SYS-08 | `f8b68dd` + uncommitted S2 low-speed kick | Ron Tan | PASS | https://youtu.be/9uLjh_gtLUs | Robot disarmed and linked (checkpoint 8c with the uncommitted S2 low-speed kick, broker 172.21.87.92, flashed 18:48). The phone hotspot (the access point) is switched off at 0:02; switching it back on is not in shot, and the phone shows it on again at 0:10. Dashboard: "Robot offline: no data for 3 s (Wi-Fi or MQTT broker link lost)" at 19:10:27, about 3-4 s after the loss; the cards are marked stale by 0:08; "Robot back online after 8 s without data" at 19:10:31; live values again by 0:10. An earlier drop is also listed (offline 19:10:03, back after 6 s at 19:10:06). |

How the November numbers were counted:

- Snapshot delivery = snapshots received ÷ sequence numbers expected, counted inside each connected stretch (a gap over 3 s counts as a link outage).
- ACK time = time from a command to its ACK, both seen by the MQTT capture client.
- Times such as 391 s are seconds from the start of that capture or console log; t=... values are robot uptime.
- Logs are unchanged copies of the bench captures. MQTT captures (.tsv) and JSON results are saved as .txt.

## 21 October 2026 CR1 local integration

Software checks PASS in the staged integration snapshot; see `docs/INTEGRATION_REPORT.md` and the `logs/2026-10-21_cr1_*` evidence. Hardware acceptance NOT RUN: robot connection postponed by the user. Exact tested source content is recorded in the delivered manifest; final checkout verification records the local commit.
