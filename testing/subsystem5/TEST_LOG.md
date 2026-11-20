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
