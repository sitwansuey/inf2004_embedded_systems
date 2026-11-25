# Subsystem 5 — test log

**Subsystem:** HC-SR04 sonar + servo, stationary edge scan  ·  **Student ID:** 2502324  ·  **Code:** `app_program/subsystem5/`

## Test cases

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | status codec and copied API preserve valid records | S5 → VC/S1 copied status (TEAM.md); FR7 (widths in 1 mm) | auto (host) | `test_data.c`, `test_s5.py` via `python -B -m unittest discover -s testing/data_representation -p test_s5.py -v` (`APP_TEST=0`, `APP_TEST=1`) | `OK` |
| S5-U02 | scan logic measures widths and selects routes | FR5, FR6, FR7, NFR6 (geometry only) | auto (host) | `scan_test.c` via `make -C testing/integration test` | `INT-04 scan: 2 consecutive near echoes (S5_ACQUISITION_SAMPLES) start it (far or open resets, a no-echo retry does not), wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS`; `exit code 0` |
| S5-U03 | edge search follows the coarse and fine sequence | FR6 | auto (host) | `obstacle_scan_test.c` via `make -C testing/integration test` | `PASS: 20/40/60 deg coarse search, edge bisection, mirror check then outward search; one open side plans it with the edge mirrored (width_one_sided), none blocks; hard echo faults count as the object; unknown stays blocked.`; `PASS: an edge flicker while averaging retreats that bound 1 deg inward and still reaches READY; S5_EDGE_RETREATS+1 flickers fail the side.` |
| S5-U04 | settled open-space samples allow a bypass plan | FR6 (no echo = clear) | auto (host) | `obstacle_open_space_test.c` via `make -C testing/integration test` | `PASS: no-rise/no-fall clear after 3 settled samples; open left or right plans that side (one measured edge mirrored), neither blocks; READY verification; forward bypass guidance; angle reset; hard faults remain closed.` |
| S5-U05 | robot motion stops and retries obstacle scans | FR5 (scan only while stopped) | auto (host) | `obstacle_motion_test.c` via `make -C testing/integration test` | `PASS: 1-3 encoder counts tolerated; larger displacement recentres and retries twice; retry budget survives acquisition; repeated motion blocks; width hold cleared by motion; encoder wrap is handled.` |
| S5-U06 | sonar and servo drivers handle pulses and faults | S5 hardware (TEAM.md: TRIG GP1, ECHO GP0, servo GP12 on PWM slice 6) | auto (host) | `s5_hardware_test.c` via `make -C testing/integration test` (`sonar`, `sonar_low_centre`, `sonar_slope`) | `INT-08: GP1 trigger/GP0 echo, valid pulse, faults, timer wrap/start failure, narrow sample PASS` in all three builds. |
| S5-U07 | status API handles preemption and lock failures | S5 → VC/S1 copied status (TEAM.md) | auto (host) | `s5_status_test.c` via `make -C testing/integration test` | `INT-09: preempted publisher handoff, ownership, timeout, unchanged failed output, unlock/lifecycle errors PASS` |
| S5-U16 | path check: a fresh MONITOR reading of 201 mm allows forward | FR5 (`s5_assess_path()`) | auto (integrated TEST=1) | `subsystem5_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S5-U16 PASS  fresh clear path allows forward` |
| S5-U17 | path check: 200 mm stops (inclusive threshold) | FR5 (`s5_assess_path()`) | auto (integrated TEST=1) | `subsystem5_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S5-U17 PASS  inclusive stop threshold` |
| S5-U18 | path check: a reading older than `S5_STALE_MS` is unavailable | FR5 (`s5_assess_path()`) | auto (integrated TEST=1) | `subsystem5_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S5-U18 PASS  stale reading is unavailable` |
| S5-U19 | scan task publishes a new sample after start-up centring and is not in FAULT | IF-12 (`s5_get_status()`) | auto (integrated TEST=1) | `subsystem5_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S5-U19 PASS  scan task publishes fresh samples, no fault`; `S5-U19 SKIP` in profiles without S5 or with an S5 diagnostic |
| S5-U20 | live status copy round-trips through the 92-byte status codec on the target compiler | IF-12 (92-byte record) | auto (integrated TEST=1) | `subsystem5_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S5-U20 PASS  live status round-trips through the status codec`; `S5-U20 SKIP` as for S5-U19 |

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

*Method:* **auto (host)** = a test runs on a PC without the robot; **auto (integrated TEST=1)** = a `TEST_CHECK` that runs on the robot in the integrated test image.

S5-U16 to S5-U20 (added 2026-11-25) are passive on-target checks in `app_program/subsystem5/subsystem5_test.c`, restored after `5f93cb6` deleted the old self-test file. They take new numbers because every lower number has already named another test (see the ID history at the end of this log). They read status only: no motor, servo, sonar or MQTT command. The project member reported these checks complete on 2026-11-25. [Latest completion status](../README.md#completion-of-pico-w-and-sanitizer-checks). S5-U16 to S5-U18 are the old S5-U01 to S5-U03 path checks under new IDs.

Notes:

- **Host tests (S5-U01 to S5-U07)** need a native GCC and Python.
  `make -C testing/integration test` runs every integration recipe, S5-U02 to S5-U07
  among them; `python -B testing/integration/run.py` does the same without make. Check
  that each recipe prints its PASS line, not only the last line of the run.
- S5-U02, S5-U06 and S5-U07 print integration IDs (INT-04, INT-08, INT-09).
  The integration log keeps those IDs. Record S5 runs of these programs here, under
  the S5 IDs.
- Host tests that join S5 with the Vehicle Controller (`width_avoidance_test.c`,
  `buddy4_vehicle_test.c`) belong in `testing/integration/TEST_LOG.md`.
- The 2026-10-27 notes under Results describe earlier runs of S5-U01 (the status was
  84 bytes then; it is 92 bytes now).
- **Known limit for S5-U09.** On the 2026-11-19 bench run (checkpoint 8b), the HC-SR04
  lost the echo from a flat face 1–2° off the face normal. The edges were found near
  ±1°, far inside the real edges, and scans ended BLOCKED or with a wrong width. If this
  happens again, record the scan as FAIL with this note.
- The one-sided width flag (`width_one_sided`) is not sent to the dashboard. Use the
  `[S5SCAN]` lines: if a side is still blocked at 60°, or its edge search failed, that
  scan is one-sided.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| S5-U04 | S5-U01 |
| S5-U05 | S5-U02 |
| S5-U06 | S5-U03 |
| S5-U07 | S5-U04 |
| S5-U08 | S5-U05 |
| S5-U09 | S5-U06 |
| S5-U10 | S5-U07 |
| S5-U14 | S5-U08 |
| S5-U15 | S5-U09 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): S5-U01 to S5-U03 (path check self-test, removed with `subsystem5_test.c`) and S5-U11 to S5-U13 (never run). All removed on 2026-11-20.

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): S5-U08, S5-U09.

</details>

## Results

### 2026-10-27: packed application status (host)

Command: `python -B testing/data_representation/run.py --report edr-check-report.json`

Result: **EDR CHECK PASS**, all three gate stages. S5 tests compile the actual
codec, shared-status API, scan, width and policy in normal and `APP_TEST=1`
configurations using native GCC with warnings as errors. Both profiles pass.
Observed native status size: 136 bytes; authoritative packed status: 84 bytes;
EDR frame: 86 bytes. Coverage includes reserved bits, angles, all 32-bit field
positions, signed extrema, C/Python agreement, mutex failure, navigation
invalidation, fault fallback and rollover freshness. The existing eight
Buddy 2 host profiles also pass. See `docs/SUBSYSTEM5_DATA.md` for the contract.

No board was flashed. Firmware timing, stack high-water marks and physical
sensor/navigation behavior were not measured by these host tests.

### 2026-10-27: pre-push verification and full ARM build

Repeated the complete EDR gate: PASS (10 Python test methods, 268,307 C
checks with zero failures, both S5 configurations and all eight S2 profiles).
The full ARM build initially exposed a missing `app_program/board_pins.h`:
the S5 branch removed it while the merged motor module still includes it.
Restored the guarded pin definitions from `origin/main`, preserving the
existing pin values.

Built the complete default firmware using WSL Ubuntu, ARM GCC and the local
Pico SDK: `PICO_SDK_PATH=/home/ncht/pico/pico-sdk make -C build_make -j2`.
The initial run used `-B` to force recompilation; after restoring the header,
the resumed build passed and generated both ELF and UF2. Image:
`mtk3pico_smp1_usb_cdc_wifi`; text=290420, data=172, bss=24900 bytes.
No board was flashed and no hardware behavior is claimed.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S5-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s5.txt`, `../subsystem1/logs/2026-11-20_host_s1.txt` | `S5 PASS: native=148 packed=92 bytes` twice, unittest `OK` (1 test), exit code 0. The same two lines and `OK` in stage 1 of `testing/data_representation/run.py` (S1 log). `OK` is printed before the two S5 lines (output order only). Native status is now 148 bytes. |
| 2026-11-20 | S5-U02 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/scan`: `INT-CR1-04 scan: … PASS`; S5 width, side choice, verify fallback, edge flicker, scan geometry, edge check, side corridor, braking and servo slope all end in PASS. Run ends `Integration recipes PASS`, exit code 0. |
| 2026-11-20 | S5-U03 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_scan`: both `PASS:` lines (20/40/60° search, bisection, mirror check, outward search; edge flicker retreat). Exit code 0. |
| 2026-11-20 | S5-U04 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_open_space`: `PASS: no-rise/no-fall clear after 3 settled samples; …`. Exit code 0. |
| 2026-11-20 | S5-U05 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_motion`: `PASS: 1-3 encoder counts tolerated; …`. Exit code 0. |
| 2026-11-20 | S5-U06 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `INT-CR1-08: GP1 trigger/GP0 echo, … PASS` from all 3 builds (`.build/sonar`, `.build/sonar_low_centre`, `.build/sonar_slope`). Exit code 0. |
| 2026-11-20 | S5-U07 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/sonar_status`: `INT-CR1-09: preempted publisher handoff, … PASS`. Exit code 0. |

### Restored result rows (audit, 2026-11-25)

These result rows were deleted from the table above in `5f93cb6` (2026-11-25). They are copied back word for word from `d18dd38`, the last commit that had them; the first two columns are the only additions. Their *Test ID* cells use the IDs in force at `d18dd38`, not today's IDs, and some of those numbers now name different tests (second column, and *ID history (git record)* at the end of this file). Nothing was re-run.

| Restored from | Test ID meaning (then → now) | Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|---|
| `d18dd38` (deleted in `5f93cb6`) | S5-U15 at `d18dd38` = Width of three flat-faced boxes, scanned from rest; test removed in `5f93cb6`; no current test uses S5-U15 | 2026-11-19 | S5-U15 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t5c_usb_sonar.txt`, `../system/logs/2026-11-19_t6_usb_sonar_wifi_off_on.txt`, `logs/2026-11-19_t5c_mqtt_sonar.txt` | Bench, robot disarmed, scans from rest. Box widths, box offset and the S5-U13 servo check were not logged; 4 scans, not 10 per box. t5c scans 3, 4 and 5 (face 151, 121 and 170 mm at 0°; USB 103.6, 303.0 and 453.6 s): the readings at 1–10° were 194–912 mm, past the face, so the left edge was bisected to 1° and the mirror at −1° read clear (301, 188, 217 mm); READY came at +21°, +25° and +19°. The MQTT width stayed 0.0 cm in all 166 width messages, also during READY (145 messages at 402.4–437.8 s, 9 at 549.4–552.0 s). Each READY went BLOCKED when an echo of 155, 70 or 189 mm came at the planned heading. t6 scan 6 (185 mm at 0°) found the left edge at 33–34° and was still searching the right side when 37.8 s of console output went missing (after line 66); when it resumed the scan was BLOCKED (`7->0` at 2.1 s). No scan gave a usable width (see the known limit for S5-U15). |
| `d18dd38` (deleted in `5f93cb6`) | S5-U14 at `d18dd38` = Obstacle confirmed within 200 mm; the robot stops at least 120 mm …; test removed in `5f93cb6`; no current test uses S5-U14 | 2026-11-20 | S5-U14 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run 1 at 20% power, MQTT only: no USB log, so no `[S5SCAN]` value M, and the front-to-face gap was not measured with a ruler. 5 "Obstacle detected" events: 19 cm (174.3 s), 16 cm (175.4 s, re-detected with the wheels still after "the obstacle went away"; ended by STOP at 176.7 s), 18 cm (221.2 s, scanned and bypassed), 17 cm (434.6 s, creeping at 6%; ended by fault 1, cause Sonar, at 440.0 s) and 6 cm (570.2 s). N ≤ 20 in 5 of 5. Phase Acquisition with power 0 within 0.3 s of each event. Forward sonar (servo at 0°) at the stop: 20 then 17 cm (174.4–175.0 s), 20 cm (221.2 s). At 570.2 s the reading fell from 58 cm (570.11 s) to 7 cm (570.25 s) at 0.12 m/s and the robot stopped with the sonar at 7 cm, under 120 mm; the log does not show what the object was. Contact not logged. |
| `d18dd38` (deleted in `5f93cb6`) | S5-U15 at `d18dd38` = Width of three flat-faced boxes, scanned from rest; test removed in `5f93cb6`; no current test uses S5-U15 | 2026-11-20 | S5-U15 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run, robot armed: 1 scan from rest after the obstacle stop (221.2–231.1 s, forward sonar 20 cm), not the bench set-up. Event "Obstacle scanned: 23 cm wide, bypass on the left"; status width 22.0 cm (left 11.2, right 10.4). The box's ruler width was not logged, so the accuracy is not shown, and without a USB log the one-sided check cannot be made. 1 scan, not 10 per box; S5-U13 not done. |
| `d18dd38` (deleted in `5f93cb6`) | S5-U14 at `d18dd38` = Obstacle confirmed within 200 mm; the robot stops at least 120 mm …; test removed in `5f93cb6`; no current test uses S5-U14 | 2026-11-20 | S5-U14 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run 3 at 20% power, MQTT only: no `[S5SCAN]` value M, gap not measured with a ruler. 2 approaches on the line: "Obstacle detected 18 cm ahead" at 49.99 s (sonar 56 → 24 cm over the 2.7 s before) and "17 cm ahead" at 2721.78 s. N ≤ 20 in 2 of 2. Phase Acquisition with power 0 by 50.21 s and 2721.78 s. Forward sonar (servo at 0°) at the stop: 19 then 17 cm (50.2–50.8 s), and 17 cm (2722.0 s); both were then scanned and bypassed. Contact not logged. 2 trials, not 10. |
| `d18dd38` (deleted in `5f93cb6`) | S5-U15 at `d18dd38` = Width of three flat-faced boxes, scanned from rest; test removed in `5f93cb6`; no current test uses S5-U15 | 2026-11-20 | S5-U15 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run, robot armed: 2 scans from rest after obstacle stops, not the bench set-up. Scan 1 (50.0–59.0 s, forward sonar 17–19 cm): status width 27.5 cm (left 16.0, right 11.2), event 29 cm, bypass right. Scan 2 (2721.8–2735.6 s, forward sonar 17 cm): status 24.0 cm (left 12.0, right 12.8), event 25 cm, bypass left. Ruler widths not logged, so the accuracy is not shown; no USB log for the one-sided check. 2 scans, not 10 per box; S5-U13 not done. |

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| S5-U01 | Clear path: a fresh MONITOR reading of 201 mm at 0° gives FORWARD | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: status codec and copied API preserve valid records |
| S5-U02 | Stop threshold is inclusive: a reading of 200 mm gives STOP | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: scan logic measures widths and selects routes |
| S5-U03 | Stale reading: 501 ms old (over `S5_STALE_MS` = 500 ms) gives UNAVAILABLE, not STOP or FORWARD | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: edge search follows the coarse and fine sequence |
| S5-U04 | Status codec and status API: every control and flag byte, angles 0–180, every bit of the 32-bit fields, round trip, bad input rejected, a lock timeout leaves the output unchanged, the READY plan is kept when the VC asks for APPROACH | S5-U01 | yes | different test: settled open-space samples allow a bypass plan |
| S5-U05 | Scan logic and width geometry: 3 echoes in a row ≤ 200 mm start a scan (a far echo resets the count, a no-echo retry does not); 20/40/60° steps, then 1° bisection; mirror check; 3-ping edge mean and flicker retreat; Pythagoras/tangent width; one-sided width flagged; side with the smaller half-width chosen; side corridor; servo pulses kept in 750–2250 µs; braking restart | S5-U02 | yes | different test: robot motion stops and retries obstacle scans |
| S5-U06 | Edge search order: coarse 20/40/60°, bisection, mirror check, then outward search; one open side is planned with its edge mirrored; hard echo faults count as the object | S5-U03 | yes | different test: sonar and servo drivers handle pulses and faults |
| S5-U07 | Open space: no echo (no-rise/no-fall) on 3 settled samples counts as clear; an open side is planned; no open side blocks | S5-U04 | yes | different test: status API handles preemption and lock failures |
| S5-U08 | Scan only while stopped: 1–3 encoder counts are tolerated; more re-centres and retries twice, then blocks; encoder wrap is handled | S5-U05 | yes | not used |
| S5-U09 | Real sonar and servo driver on mocked registers: TRIG GP1 and ECHO GP0; a 580 µs echo reads 98–102 mm; stuck-high, no-rise, no-fall and too-short faults; timer wrap and start failure; servo pulse at 0/±30/±90° with the 750–2250 µs clamp | S5-U06 | yes | not used |
| S5-U10 | Status API under preemption: a bounded wait (≤ 5 ms) hands over, a poll fails, a lock timeout leaves the caller's copy unchanged, unlock and start errors are returned | S5-U07 | yes | not used |
| S5-U11 | Sonar distance against a ruler at 10, 20 and 40 cm | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S5-U12 | Servo travel at 0° and ±90° against a protractor | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S5-U13 | Servo calibration at ±45° against a protractor (run before S5-U15) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S5-U14 | Obstacle confirmed within 200 mm; the robot stops at least 120 mm from the face, with no contact | removed from the table in `5f93cb6` (S5-U08 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S5-U15 | Width of three flat-faced boxes, scanned from rest | removed from the table in `5f93cb6` (S5-U09 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. Each cell names exactly one existing commit; none needed a note.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened.

`docs/SUBSYSTEM5_DATA.md`, named in the 2026-10-27 section, is not in the repository at `0bac48c`: it was deleted in `54a371f` (2026-10-27); its last version is in `efedeb1`.
