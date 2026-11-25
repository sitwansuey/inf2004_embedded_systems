# Subsystem 5 — test log

**Subsystem:** HC-SR04 sonar + servo, stationary edge scan  ·  **Student ID:** 2502324  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | status codec and copied API preserve valid records | S5 → VC/S1 copied status (TEAM.md); FR7 (widths in 1 mm) | auto (host) | `test_data.c`, `test_s5.py` via `python -B -m unittest discover -s testing/data_representation -p test_s5.py -v` (`APP_TEST=0`, `APP_TEST=1`) | `OK` |
| S5-U02 | scan logic measures widths and selects routes | FR5, FR6, FR7, NFR6 (geometry only) | auto (host) | `scan_test.c` via `make -C testing/integration test` | `INT-04 scan: 3 consecutive near echoes start it (far or open resets, a no-echo retry does not), wrap, angle association, inclusive stop, failed echo, moving scan, geometric bounds PASS`; `exit code 0` |
| S5-U03 | edge search follows the coarse and fine sequence | FR6 | auto (host) | `obstacle_scan_test.c` via `make -C testing/integration test` | `PASS: 20/40/60 deg coarse search, edge bisection, mirror check then outward search; one open side plans it with the edge mirrored (width_one_sided), none blocks; hard echo faults count as the object; unknown stays blocked.`; `PASS: an edge flicker while averaging retreats that bound 1 deg inward and still reaches READY; S5_EDGE_RETREATS+1 flickers fail the side.` |
| S5-U04 | settled open-space samples allow a bypass plan | FR6 (no echo = clear) | auto (host) | `obstacle_open_space_test.c` via `make -C testing/integration test` | `PASS: no-rise/no-fall clear after 3 settled samples; open left or right plans that side (one measured edge mirrored), neither blocks; READY verification; forward bypass guidance; angle reset; hard faults remain closed.` |
| S5-U05 | robot motion stops and retries obstacle scans | FR5 (scan only while stopped) | auto (host) | `obstacle_motion_test.c` via `make -C testing/integration test` | `PASS: 1-3 encoder counts tolerated; larger displacement recentres and retries twice; retry budget survives acquisition; repeated motion blocks; width hold cleared by motion; encoder wrap is handled.` |
| S5-U06 | sonar and servo drivers handle pulses and faults | S5 hardware (TEAM.md: TRIG GP1, ECHO GP0, servo GP12 on PWM slice 6) | auto (host) | `s5_hardware_test.c` via `make -C testing/integration test` (`sonar`, `sonar_low_centre`, `sonar_slope`) | `INT-08: GP1 trigger/GP0 echo, valid pulse, faults, timer wrap/start failure, narrow sample PASS` in all three builds. |
| S5-U07 | status API handles preemption and lock failures | S5 → VC/S1 copied status (TEAM.md) | auto (host) | `s5_status_test.c` via `make -C testing/integration test` | `INT-09: preempted publisher handoff, ownership, timeout, unchanged failed output, unlock/lifecycle errors PASS` |

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

*Method:* **auto (host)** = a test runs on a PC without the robot.

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

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S5-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s5.txt`, `../subsystem1/logs/2026-11-20_host_s1.txt` | `S5 PASS: native=148 packed=92 bytes` twice, unittest `OK` (1 test), exit code 0. The same two lines and `OK` in stage 1 of `testing/data_representation/run.py` (S1 log). `OK` is printed before the two S5 lines (output order only). Native status is now 148 bytes. |
| 2026-11-20 | S5-U02 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/scan`: `INT-CR1-04 scan: … PASS`; S5 width, side choice, verify fallback, edge flicker, scan geometry, edge check, side corridor, braking and servo slope all end in PASS. Run ends `Integration recipes PASS`, exit code 0. |
| 2026-11-20 | S5-U03 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_scan`: both `PASS:` lines (20/40/60° search, bisection, mirror check, outward search; edge flicker retreat). Exit code 0. |
| 2026-11-20 | S5-U04 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_open_space`: `PASS: no-rise/no-fall clear after 3 settled samples; …`. Exit code 0. |
| 2026-11-20 | S5-U05 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/obstacle_motion`: `PASS: 1-3 encoder counts tolerated; …`. Exit code 0. |
| 2026-11-20 | S5-U06 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `INT-CR1-08: GP1 trigger/GP0 echo, … PASS` from all 3 builds (`.build/sonar`, `.build/sonar_low_centre`, `.build/sonar_slope`). Exit code 0. |
| 2026-11-20 | S5-U07 | `ed197b3` | Ron Tan | PASS | `../integration/logs/2026-11-20_host_integration.txt` | `.build/sonar_status`: `INT-CR1-09: preempted publisher handoff, … PASS`. Exit code 0. |
