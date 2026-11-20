# Subsystem 2 — test log

**Subsystem:** Buddy 2 Motion Control (motor PWM/PID, wheel encoders, command watchdog)  ·  **Student ID:** 2500613  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | Straight (FORWARD) at 30% for 2 s: send and 250 ms renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U01 PASS` |
| S2-U02 | Straight: left and right encoders both count pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U02 PASS` |
| S2-U03 | Right pivot (PIVOT_RIGHT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U03 PASS` |
| S2-U04 | Right pivot: the left (driven) encoder counts pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U04 PASS` |
| S2-U05 | Left pivot (PIVOT_LEFT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U05 PASS` |
| S2-U06 | Left pivot: the right (driven) encoder counts pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U06 PASS` |
| S2-U07 | Clockwise spin (RIGHT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U07 PASS` |
| S2-U08 | Clockwise spin: left and right encoders both count pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U08 PASS` |
| S2-U09 | Counterclockwise spin (LEFT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U09 PASS` |
| S2-U10 | Counterclockwise spin: left and right encoders both count pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U10 PASS` |
| S2-U11 | Reverse at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | FR4; VC -> S2, S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U11 PASS` |
| S2-U12 | Reverse: left and right encoders both count pulses | FR4; S2 -> VC/S1 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U12 PASS` |
| S2-U13 | Command watchdog: FORWARD 20% with no renewal for the timeout + 100 ms gives FAULT, E_TMOUT, command STOP, 0% power | FR4 (no setpoint for 1 s), NFR5; VC -> S2 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U13 PASS` |
| S2-U14 | Final STOP after the watchdog FAULT reads STOPPED (PWM off) | FR4, NFR5; VC -> S2 | auto | `subsystem2_test.c`; S2_ONLY=1 TEST=1 image, wheels raised | `[TEST] S2-U14 PASS`, then `[TEST] subsystem2 SUMMARY 14 passed, 0 failed` |
| S2-U15 | Host profile `standalone-no-encoder` (`-DAPP_S2_ONLY=1`). Checks: lifecycle and argument errors (E_OBJ, E_PAR), init cleanup, 1 s command watchdog and clock rollover, stale or malformed messages, renewal and recovery, all 7 move directions, mailbox pool and transport errors, clock errors and 0% power, status-lock timeouts, init failure (FAULT, PWM off), turn-drive helpers, STOP in a command change's dead time, EDR motion adapter; encoder API gives E_NOSPT and never touches encoder registers. Also compiles `subsystem2_test.c` | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | `python app_program/subsystem2/tests/host/run.py --compiler gcc` (profile 1 of 9) | `PASS Buddy 2 standalone-no-encoder` |
| S2-U16 | Host profile `standalone-dual-encoder` (`-DAPP_S2_ONLY=1 -DS2_ENCODER_ENABLED=1`). Checks: as S2-U15, with encoders on: x1 counts on both wheels, forward/reverse sign, falling A edges as diagnostics only, snapshot retried across an interrupt, counter rollover, EDR encoder adapter | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | same command (profile 2 of 9) | `PASS Buddy 2 standalone-dual-encoder` |
| S2-U17 | Host profile `standalone-dual-encoder-swapped` (motor 1 encoder GP4/5, motor 2 GP16/17). Checks: as S2-U16 with the wheel pins swapped | FR4; S2 -> VC/S1 | auto (host) | same command (profile 3 of 9) | `PASS Buddy 2 standalone-dual-encoder-swapped` |
| S2-U18 | Host profile `standalone-dual-encoder-shared-bank` (motor 1 encoder GP2/3). Checks: as S2-U16 with both encoders in one GPIO interrupt register | FR4; S2 -> VC/S1 | auto (host) | same command (profile 4 of 9) | `PASS Buddy 2 standalone-dual-encoder-shared-bank` |
| S2-U19 | Host profile `standalone-dual-encoder-cross-bank` (GP7/12 and GP14/16). Checks: as S2-U16 with encoder pins split across interrupt registers | FR4; S2 -> VC/S1 | auto (host) | same command (profile 5 of 9) | `PASS Buddy 2 standalone-dual-encoder-cross-bank` |
| S2-U20 | Host profile `integrated-no-encoder` (`-DAPP_S2_ONLY=0`). Checks: as S2-U15, plus: only the VC may send (E_OACV), STOP epoch (STOP passes a full pool, old commands dropped, motor off within 11 ms), a missed STOP status copy is retried, an init FAULT stays FAULT after STOP (VC fault 7) | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | same command (profile 6 of 9) | `PASS Buddy 2 integrated-no-encoder` |
| S2-U21 | Host profile `integrated-dual-encoder` (`-DAPP_S2_ONLY=0 -DS2_ENCODER_ENABLED=1`). Checks: S2-U20 plus the S2-U16 encoder checks | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | same command (profile 7 of 9) | `PASS Buddy 2 integrated-dual-encoder` |
| S2-U22 | Host profile `integrated-custom-timeout` (as S2-U21 with `-DS2_COMMAND_TIMEOUT_MS=750U`). Checks: as S2-U21; the watchdog fires at 750 ms | FR4; VC -> S2 | auto (host) | same command (profile 8 of 9) | `PASS Buddy 2 integrated-custom-timeout` |
| S2-U23 | Host profile `integrated-pid-turn` (robot PID: `BOARD_PID_ENABLED=1`, MAX_CPS 1950, Kp/Ki 30/40 milli, PWM 35%, spins 55%, kick 150 ms at 45%, spin stall 1200 ms, trims 86/93). Checks: turn-drive helpers; PID spins (kick until real motion, re-kick, raised ceiling, longer stall; straight and pivots unchanged); STOP in dead time; stall time kept across FORWARD/PIVOT switches and short STOPs; a held wheel during follower switching faults within 0.6 s, free wheels never fault | FR4, NFR1, NFR5; VC -> S2 | auto (host) | same command (profile 9 of 9) | `PASS Buddy 2 integrated-pid-turn`, then run.py exits 0 |
| S2-U24 | Held wheel: stall check gives fault 7 and PWM off; RESET clears it | FR4, NFR1, NFR5 | manual | 1. Flash the mission image; log the USB console. 2. Robot on the floor on the line. Film both wheels at 60 fps. 3. Hold the right wheel still by hand. 4. Press `g` (START 30%). 5. Count frames from the left wheel starting to it stopping. 6. Press `r` (RESET). 7. Repeat 3 times, then 3 times holding the left wheel | Fault 7 every time within 36 frames (0.6 s); console shows `[S2 FAULT]` and `[STATE] ... fault=7`; the free wheel stays off; after `r` the `[STATE]` line shows `fault=0` |
| S2-U25 | Left/right encoder counts agree over 2 s at 35% demand | FR4 | manual | 1. Flash the mission image; open the dashboard. 2. Robot centred on a straight line at least 1.5 m long. 3. Note both wheel encoder counts (robot stopped). 4. Send the USB frame `C1 03 23 00 00 00` (hex; START at 35%). 5. Press `s` after 2 s (stopwatch). 6. Note both counts again; ΔL and ΔR (add 4096 if a count wrapped). 7. Repeat 5 times | ΔL and ΔR differ by ≤ 1% of their mean in every run; `[STATE]` shows `command=1` (FORWARD) only while driving |
| S2-U26 | 1 m straight: encoder distance matches the tape-measured distance | FR4 | manual | 1. Flash the mission image; open the dashboard. 2. Robot on a straight line; tape mark under the axle centre. 3. Note both encoder counts. 4. Press `g`, then `s` after about 1 m. 5. Mark the axle again; tape-measure the distance (mm). 6. Encoder distance = (ΔL + ΔR) / 2 × 0.349 mm. 7. Repeat 5 times | Every run is 1.00 ± 0.05 m and the two distances differ by ≤ 20 mm |
| S2-U27 | 90° spins: mapped navigation turn (spin ends on the new line) | FR4, FR9 | manual | 1. Flash the mission image (codebook `Z` = right, `F` = left). 2. Track: a `*Z*` marker before a 90° right junction, a `*F*` marker before a 90° left junction. 3. Film from above at 60 fps over a printed protractor centred on the junction. 4. Press `g`. 5. On the frame where the spin ends, measure the chassis axis against its heading before the turn. 6. 5 turns each way | Every turn is 90° ± 5° |
| S2-U28 | 180° spins: mapped U-turn | FR4, FR9 | manual | 1. Flash the mission image (codebook `A` = U-turn). 2. Track: a `*A*` marker on a straight. 3. Film from above at 60 fps over a printed protractor. 4. Press `g`. 5. On the frame where the spin ends, measure the chassis axis against its heading before the turn. 6. 5 U-turns | Every U-turn is 180° ± 5° |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).
**auto** here (S2-U01..S2-U14) = the raised-wheel motor test image: in `build_make` run `make S2_ONLY=1 TEST=1 -j8` (`mtk3pico_smp1_usb_cdc_wifi_s2_enc_test.uf2`). Raise the robot so both wheels turn freely, start the console log, then flash. The checks start 2 s after boot, take about 21 s and end with `[TEST] subsystem2 SUMMARY`.
**auto (host)** (S2-U15..S2-U23) = a host program, no robot. From the repository root run `python app_program/subsystem2/tests/host/run.py --compiler gcc` (any native GCC or Clang; `run.ps1` on Windows). It builds each profile with `-Wall -Wextra -Werror -Wpedantic`, runs it and prints `PASS Buddy 2 <profile>`. It stops at the first failure with a non-zero exit code.
**manual** rows (S2-U24..S2-U28) use the mission image (`make`, default profile). USB console keys: `g` START at 30%, `s` STOP, `r` RESET. Save the console log and video links in `logs/`.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S2-U15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 1/9: `PASS Buddy 2 standalone-no-encoder` (log line 17); its 4 checks PASS, `Motor-control host checks passed` |
| 2026-11-20 | S2-U16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 2/9: `PASS Buddy 2 standalone-dual-encoder` (line 25); 4 checks PASS |
| 2026-11-20 | S2-U17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 3/9: `PASS Buddy 2 standalone-dual-encoder-swapped` (line 33); 4 checks PASS |
| 2026-11-20 | S2-U18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 4/9: `PASS Buddy 2 standalone-dual-encoder-shared-bank` (line 41); 4 checks PASS |
| 2026-11-20 | S2-U19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 5/9: `PASS Buddy 2 standalone-dual-encoder-cross-bank` (line 49); 4 checks PASS |
| 2026-11-20 | S2-U20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 6/9: `PASS Buddy 2 integrated-no-encoder` (line 58); 5 checks PASS incl. integrated STOP epoch |
| 2026-11-20 | S2-U21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 7/9: `PASS Buddy 2 integrated-dual-encoder` (line 67); 5 checks PASS incl. integrated STOP epoch |
| 2026-11-20 | S2-U22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 8/9: `PASS Buddy 2 integrated-custom-timeout` (line 76); 5 checks PASS |
| 2026-11-20 | S2-U23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 9/9: `PASS Buddy 2 integrated-pid-turn` (line 85); 5 checks PASS incl. PID spins and held-wheel fault within 0.6 s; run.py `[exit code 0]` |
