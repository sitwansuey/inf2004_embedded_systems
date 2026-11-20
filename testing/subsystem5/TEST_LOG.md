# Subsystem 5 — test log

**Subsystem:** HC-SR04 sonar + servo, stationary edge scan  ·  **Student ID:** 2502324  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | Clear path: a fresh MONITOR reading of 201 mm at 0° gives FORWARD | FR5 (obstacle at ≤ 200 mm); S5 → VC path check `s5_assess_path()` | auto (integrated TEST=1) | `subsystem5_test.c`. In `build_make`: `make TEST=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2`, save the USB console to `logs/`. | `[TEST] S5-U01 PASS  fresh clear path` |
| S5-U02 | Stop threshold is inclusive: a reading of 200 mm gives STOP | FR5, NFR1 (echoes ≤ 200 mm) | auto (integrated TEST=1) | `subsystem5_test.c`; same image and log as S5-U01. | `[TEST] S5-U02 PASS  inclusive stop threshold` |
| S5-U03 | Stale reading: 501 ms old (over `S5_STALE_MS` = 500 ms) gives UNAVAILABLE, not STOP or FORWARD | S5 → VC status freshness (TEAM.md) | auto (integrated TEST=1) | `subsystem5_test.c`; same image and log as S5-U01. | `[TEST] S5-U03 PASS  stale stop`, then `[TEST] subsystem5 SUMMARY 3 passed, 0 failed` |
| S5-U04 | Status codec and status API: every control and flag byte, angles 0–180, every bit of the 32-bit fields, round trip, bad input rejected, a lock timeout leaves the output unchanged, the READY plan is kept when the VC asks for APPROACH | S5 → VC/S1 copied status (TEAM.md); FR7 (widths in 1 mm) | auto (host) | `python -B -m unittest discover -s testing/data_representation -p test_s5.py -v` (also stage 1 of `python -B testing/data_representation/run.py`). It builds `testing/subsystem5/test_data.c` with the real `s5_data.c`, `s5_policy.c`, `s5_scan.c`, `s5_width.c` and `shared/edr.c` (GCC, `-Werror`), once with `APP_TEST=0` and once with `APP_TEST=1`. Set `CC` if GCC is not on PATH. | `S5 PASS: native=<n> packed=92 bytes` twice, then unittest `OK` |
| S5-U05 | Scan logic and width geometry: 3 echoes in a row ≤ 200 mm start a scan (a far echo resets the count, a no-echo retry does not); 20/40/60° steps, then 1° bisection; mirror check; 3-ping edge mean and flicker retreat; Pythagoras/tangent width; one-sided width flagged; side with the smaller half-width chosen; side corridor; servo pulses kept in 750–2250 µs; braking restart | FR5, FR6, FR7, NFR6 (geometry only) | auto (host) | `make -C testing/integration test` (or `python -B testing/integration/run.py`). Recipe: `scan_test.c` with `s5_scan.c s5_policy.c s5_width.c` → `.build/scan` | `INT-CR1-04 scan: … PASS` and every `S5 …` line ends in `PASS` (width, side choice, verify fallback, edge flicker, scan geometry, edge check, side corridor, braking, servo slope); exit code 0 |
| S5-U06 | Edge search order: coarse 20/40/60°, bisection, mirror check, then outward search; one open side is planned with its edge mirrored; hard echo faults count as the object | FR6 | auto (host) | Same command as S5-U05. Recipe: `obstacle_scan_test.c` → `.build/obstacle_scan` | Two lines starting `PASS:`; exit code 0 |
| S5-U07 | Open space: no echo (no-rise/no-fall) on 3 settled samples counts as clear; an open side is planned; no open side blocks | FR6 (no echo = clear) | auto (host) | Same command as S5-U05. Recipe: `obstacle_open_space_test.c` → `.build/obstacle_open_space` | `PASS: no-rise/no-fall clear after 3 settled samples; …`; exit code 0 |
| S5-U08 | Scan only while stopped: 1–3 encoder counts are tolerated; more re-centres and retries twice, then blocks; encoder wrap is handled | FR5 (scan only while stopped) | auto (host) | Same command as S5-U05. Recipe: `obstacle_motion_test.c` → `.build/obstacle_motion` | `PASS: 1-3 encoder counts tolerated; …`; exit code 0 |
| S5-U09 | Real sonar and servo driver on mocked registers: TRIG GP1 and ECHO GP0; a 580 µs echo reads 98–102 mm; stuck-high, no-rise, no-fall and too-short faults; timer wrap and start failure; servo pulse at 0/±30/±90° with the 750–2250 µs clamp | S5 hardware (TEAM.md: TRIG GP1, ECHO GP0, servo GP12 on PWM slice 6) | auto (host) | Same command as S5-U05. Recipes: `s5_hardware_test.c` with `s5_hardware.c` → `.build/sonar`, `.build/sonar_low_centre` (`-DBOARD_SERVO_CENTER_US=1300`), `.build/sonar_slope` (`-DBOARD_SERVO_US_PER_90=1000`) | Each of the 3 prints `INT-CR1-08: GP1 trigger/GP0 echo, … PASS`; exit code 0 |
| S5-U10 | Status API under preemption: a bounded wait (≤ 5 ms) hands over, a poll fails, a lock timeout leaves the caller's copy unchanged, unlock and start errors are returned | S5 → VC/S1 copied status (TEAM.md) | auto (host) | Same command as S5-U05. Recipe: `s5_status_test.c` with `s5_data.c` → `.build/sonar_status` | `INT-CR1-09: preempted publisher handoff, … PASS`; exit code 0 |
| S5-U11 | Sonar distance against a ruler at 10, 20 and 40 cm | FR5 (200 mm threshold); FR13 (ultrasonic distance) | manual | 1. Set `#define BOARD_MOTORS_DISABLED 1` in `app_program/robot_config.local.h`. Build the mission image (`make NET=lwip MQTT=1` in `build_make`) and flash it. Keep the robot DISARMED, servo at 0°. 2. Stand a flat board square to the sensor, 100 mm from the HC-SR04 front face (ruler). 3. Start the USB log, then power on. The scan starts by itself; `[S5SCAN] scan=1 state=0->1 angle=0 … mm=N` gives the forward reading N. 4. Power-cycle for 3 readings. 5. Repeat at 200 mm (if no scan starts, read the dashboard). 6. At 400 mm no scan starts: read the dashboard "Obstacle distance" (whole cm) 3 times. | All 9 readings within ±10 mm of the ruler (dashboard: 9–11, 19–21 and 39–41 cm) |
| S5-U12 | Servo travel at 0° and ±90° against a protractor | FR6 (scan angles); TEAM.md (servo GP12) | manual | 1. In `app_program/robot_config.local.h` set `#define APP_S5_SERVO_TEST 1` and `#define BOARD_MOTORS_DISABLED 1`. 2. In `build_make`: `make TEST=1`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2`. 3. Fix a protractor centred on the servo shaft, 0° on the robot's forward axis. 4. Start the USB log, then power on. The sweep holds 0°, −90° (right), +90° (left) and 0° for 1 s each (`[S5 TEST] move=…` lines). 5. Read where the sensor faces at each stop. Film it. | 0° within ±1°; −90° within ±3°; +90° about +84° ± 3°, because the pulse is capped at 2250 µs: (2250 − 1554) / 750 × 90 = 83.5° |
| S5-U13 | Servo calibration at ±45° against a protractor (run before S5-U15) | NFR6 (servo ±45° within ±1°) | manual | 1. The sweep at `15aebfc` does not stop at ±45°. For this run, add +45° and −45° stops to `s5_servo_sweep_test.c` (pulses 1929 µs and 1179 µs with centre 1554 µs and 750 µs per 90°) and record that commit. 2. Build, flash and set up as in S5-U12. 3. Read the protractor at 0°, +45° and −45°. 4. If one is off by more than 1°, change `BOARD_SERVO_US_PER_90` (500–1000) or `BOARD_SERVO_CENTER_US` (1300–1700), rebuild and repeat. | 0°, +45° and −45° each within ±1° |
| S5-U14 | Obstacle confirmed within 200 mm; the robot stops at least 120 mm from the face, with no contact | FR5, NFR1 | manual | 1. Mission image (motors on), dashboard and USB log running. 2. Straight line; a flat-faced box 100–200 mm wide, centred on the line, about 600 mm ahead. 3. USB `r`, then `g` (START at 30% demand). 4. The phase goes TRACK → Acquisition and power goes to 0. 5. Note the event "Obstacle detected N cm ahead" and the `[S5SCAN] … state=0->1 angle=0 … mm=M` line. 6. Measure the gap from the robot's front to the box face with a ruler. 7. Run 10 trials. | In 10 of 10 trials: M ≤ 200 and N ≤ 20; the robot stops with a gap ≥ 120 mm; no contact |
| S5-U15 | Width of three flat-faced boxes, scanned from rest | NFR6; FR6; FR7 | manual | 1. S5-U13 passed. Mission image with `BOARD_MOTORS_DISABLED 1`; dashboard and USB log running; robot DISARMED. 2. Measure three flat-faced boxes with a ruler, for example about 80, 150 and 250 mm wide. 3. Nothing within 200 mm, servo at 0°. Put box 1 square to the robot, face 150 mm from the sensor, centre within ±50 mm of the robot's centre line. 4. The scan starts by itself. When `[S5SCAN] … ->6` (READY) appears, read the dashboard "Estimated width" (cm). 5. Before the next scan: take the box away, hold a card within 5 cm of the sensor until `->7` (BLOCKED), remove the card and wait for `->0` (MONITOR). 6. Do 10 scans per box. | For each box, at least 9 of 10 scans within ±10% or ±10 mm (whichever is larger) of the ruler width. Only scans with both edges measured count; a one-sided scan is left out (NFR6) |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
**auto (host)** = a test program built with a native GCC and run on a PC; it prints
`PASS` lines and exits with code 0, and an assertion stops it on a failure.

Notes:

- **On-target image (S5-U01 to S5-U03).** `make TEST=1 -j8` in `build_make` (or
  `tools/build_windows.ps1 … -Profile PassiveTest`) gives
  `mtk3pico_smp1_usb_cdc_wifi_enc_test.uf2`. It uses the USB console, so GP0/GP1 stay
  free for the sonar. This integrated image runs only `subsystem5_test()`, then prints
  `[TEST] passive integrated checks completed; no motor test requested`.
- **Servo sweep image (S5-U12, S5-U13).** A separate `TEST=1` build, chosen with
  `#define APP_S5_SERVO_TEST 1` in the git-ignored `app_program/robot_config.local.h`.
  It does not start the vehicle controller. Its file name is the same as the integrated
  test image, so name the log after the build.
- **Host tests (S5-U04 to S5-U10)** need a native GCC and Python.
  `make -C testing/integration test` runs every integration recipe, S5-U05 to S5-U10
  among them; `python -B testing/integration/run.py` does the same without make. Check
  that each recipe prints its PASS line, not only the last line of the run.
- S5-U05, S5-U09 and S5-U10 print older labels (INT-CR1-04, INT-CR1-08, INT-CR1-09).
  The integration log keeps those IDs. Record S5 runs of these programs here, under
  the S5 IDs.
- Host tests that join S5 with the Vehicle Controller (`width_avoidance_test.c`,
  `buddy4_vehicle_test.c`) belong in `testing/integration/TEST_LOG.md`.
- The 2026-10-27 notes under Results describe earlier runs of S5-U04 (the status was
  84 bytes then; it is 92 bytes now).
- **Known limit for S5-U15.** On the 2026-11-19 bench run (checkpoint 8b), the HC-SR04
  lost the echo from a flat face 1–2° off the face normal. The edges were found near
  ±1°, far inside the real edges, and scans ended BLOCKED or with a wrong width. If this
  happens again, record the scan as FAIL with this note.
- The one-sided width flag (`width_one_sided`) is not sent to the dashboard. Use the
  `[S5SCAN]` lines: if a side is still blocked at 60°, or its edge search failed, that
  scan is one-sided.

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

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S5-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s5.txt`, `../subsystem1/logs/2026-11-20_host_s1.txt` | `S5 PASS: native=148 packed=92 bytes` twice, unittest `OK` (1 test), exit code 0. The same two lines and `OK` in stage 1 of `testing/data_representation/run.py` (S1 log). `OK` is printed before the two S5 lines (output order only). Native status is now 148 bytes. |
| 2026-11-20 | S5-U05 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/scan`: `INT-CR1-04 scan: … PASS`; S5 width, side choice, verify fallback, edge flicker, scan geometry, edge check, side corridor, braking and servo slope all end in PASS. Run ends `Integration recipes PASS`, exit code 0. |
| 2026-11-20 | S5-U06 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_scan`: both `PASS:` lines (20/40/60° search, bisection, mirror check, outward search; edge flicker retreat). Exit code 0. |
| 2026-11-20 | S5-U07 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_open_space`: `PASS: no-rise/no-fall clear after 3 settled samples; …`. Exit code 0. |
| 2026-11-20 | S5-U08 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_motion`: `PASS: 1-3 encoder counts tolerated; …`. Exit code 0. |
| 2026-11-20 | S5-U09 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `INT-CR1-08: GP1 trigger/GP0 echo, … PASS` from all 3 builds (`.build/sonar`, `.build/sonar_low_centre`, `.build/sonar_slope`). Exit code 0. |
| 2026-11-20 | S5-U10 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/sonar_status`: `INT-CR1-09: preempted publisher handoff, … PASS`. Exit code 0. |
| 2026-11-19 | S5-U15 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t5c_usb_sonar.txt`, `../system/logs/2026-11-19_t6_usb_sonar_wifi_off_on.txt`, `logs/2026-11-19_t5c_mqtt_sonar.txt` | Bench, robot disarmed, scans from rest. Box widths, box offset and the S5-U13 servo check were not logged; 4 scans, not 10 per box. t5c scans 3, 4 and 5 (face 151, 121 and 170 mm at 0°; USB 103.6, 303.0 and 453.6 s): the readings at 1–10° were 194–912 mm, past the face, so the left edge was bisected to 1° and the mirror at −1° read clear (301, 188, 217 mm); READY came at +21°, +25° and +19°. The MQTT width stayed 0.0 cm in all 166 width messages, also during READY (145 messages at 402.4–437.8 s, 9 at 549.4–552.0 s). Each READY went BLOCKED when an echo of 155, 70 or 189 mm came at the planned heading. t6 scan 6 (185 mm at 0°) found the left edge at 33–34° and was still searching the right side when 37.8 s of console output went missing (after line 66); when it resumed the scan was BLOCKED (`7->0` at 2.1 s). No scan gave a usable width (see the known limit for S5-U15). |
| 2026-11-20 | S5-U14 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run 1 at 20% power, MQTT only: no USB log, so no `[S5SCAN]` value M, and the front-to-face gap was not measured with a ruler. 5 "Obstacle detected" events: 19 cm (174.3 s), 16 cm (175.4 s, re-detected with the wheels still after "the obstacle went away"; ended by STOP at 176.7 s), 18 cm (221.2 s, scanned and bypassed), 17 cm (434.6 s, creeping at 6%; ended by fault 1, cause Sonar, at 440.0 s) and 6 cm (570.2 s). N ≤ 20 in 5 of 5. Phase Acquisition with power 0 within 0.3 s of each event. Forward sonar (servo at 0°) at the stop: 20 then 17 cm (174.4–175.0 s), 20 cm (221.2 s). At 570.2 s the reading fell from 58 cm (570.11 s) to 7 cm (570.25 s) at 0.12 m/s and the robot stopped with the sonar at 7 cm, under 120 mm; the log does not show what the object was. Contact not logged. |
| 2026-11-20 | S5-U15 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run, robot armed: 1 scan from rest after the obstacle stop (221.2–231.1 s, forward sonar 20 cm), not the bench set-up. Event "Obstacle scanned: 23 cm wide, bypass on the left"; status width 22.0 cm (left 11.2, right 10.4). The box's ruler width was not logged, so the accuracy is not shown, and without a USB log the one-sided check cannot be made. 1 scan, not 10 per box; S5-U13 not done. |
| 2026-11-20 | S5-U14 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run 3 at 20% power, MQTT only: no `[S5SCAN]` value M, gap not measured with a ruler. 2 approaches on the line: "Obstacle detected 18 cm ahead" at 49.99 s (sonar 56 → 24 cm over the 2.7 s before) and "17 cm ahead" at 2721.78 s. N ≤ 20 in 2 of 2. Phase Acquisition with power 0 by 50.21 s and 2721.78 s. Forward sonar (servo at 0°) at the stop: 19 then 17 cm (50.2–50.8 s), and 17 cm (2722.0 s); both were then scanned and bypassed. Contact not logged. 2 trials, not 10. |
| 2026-11-20 | S5-U15 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run, robot armed: 2 scans from rest after obstacle stops, not the bench set-up. Scan 1 (50.0–59.0 s, forward sonar 17–19 cm): status width 27.5 cm (left 16.0, right 11.2), event 29 cm, bypass right. Scan 2 (2721.8–2735.6 s, forward sonar 17 cm): status 24.0 cm (left 12.0, right 12.8), event 25 cm, bypass left. Ruler widths not logged, so the accuracy is not shown; no USB log for the one-sided check. 2 scans, not 10 per box; S5-U13 not done. |
