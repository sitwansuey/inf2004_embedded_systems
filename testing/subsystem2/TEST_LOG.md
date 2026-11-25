# Subsystem 2 — test log

**Subsystem:** Buddy 2 Motion Control (motor PWM/PID, wheel encoders, command watchdog)  ·  **Student ID:** 2500613  ·  **Code:** `app_program/subsystem2/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S2-U01 | standalone motor control without wheel encoders | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`standalone-no-encoder`) | `PASS Buddy 2 standalone-no-encoder` |
| S2-U02 | standalone motor control with both wheel encoders | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`standalone-dual-encoder`) | `PASS Buddy 2 standalone-dual-encoder` |
| S2-U03 | wheel encoders work with swapped pins | FR4; S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`standalone-dual-encoder-swapped`) | `PASS Buddy 2 standalone-dual-encoder-swapped` |
| S2-U04 | wheel encoders share one interrupt register | FR4; S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`standalone-dual-encoder-shared-bank`) | `PASS Buddy 2 standalone-dual-encoder-shared-bank` |
| S2-U05 | wheel encoder pins span interrupt registers | FR4; S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`standalone-dual-encoder-cross-bank`) | `PASS Buddy 2 standalone-dual-encoder-cross-bank` |
| S2-U06 | integrated motor control without wheel encoders | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`integrated-no-encoder`) | `PASS Buddy 2 integrated-no-encoder` |
| S2-U07 | integrated motor control with both wheel encoders | FR4, NFR5; VC -> S2, S2 -> VC/S1 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`integrated-dual-encoder`) | `PASS Buddy 2 integrated-dual-encoder` |
| S2-U08 | command watchdog uses the custom timeout | FR4; VC -> S2 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`integrated-custom-timeout`) | `PASS Buddy 2 integrated-custom-timeout` |
| S2-U09 | PID turns handle kicks and wheel stalls | FR4, NFR1, NFR5; VC -> S2 | auto (host) | `subsystem2_host_test.c` via `python app_program/subsystem2/tests/host/run.py --compiler gcc` (`integrated-pid-turn`) | `PASS Buddy 2 integrated-pid-turn` |

The IDs were renumbered on 2026-11-21 so they run in order with no gaps. The old IDs are listed under *ID changes on 2026-11-21* below.

*Method:* **auto (host)** = a test runs on a PC without the robot.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| S2-U15 | S2-U01 |
| S2-U16 | S2-U02 |
| S2-U17 | S2-U03 |
| S2-U18 | S2-U04 |
| S2-U19 | S2-U05 |
| S2-U20 | S2-U06 |
| S2-U21 | S2-U07 |
| S2-U22 | S2-U08 |
| S2-U23 | S2-U09 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): S2-U01 to S2-U14 (raised-wheel motor self-test, removed with `subsystem2_test.c`) and S2-U24 to S2-U28 (never run). All removed on 2026-11-20.

</details>

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-11-20 | S2-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 1/9: `PASS Buddy 2 standalone-no-encoder` (log line 17); its 4 checks PASS, `Motor-control host checks passed` |
| 2026-11-20 | S2-U02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 2/9: `PASS Buddy 2 standalone-dual-encoder` (line 25); 4 checks PASS |
| 2026-11-20 | S2-U03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 3/9: `PASS Buddy 2 standalone-dual-encoder-swapped` (line 33); 4 checks PASS |
| 2026-11-20 | S2-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 4/9: `PASS Buddy 2 standalone-dual-encoder-shared-bank` (line 41); 4 checks PASS |
| 2026-11-20 | S2-U05 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 5/9: `PASS Buddy 2 standalone-dual-encoder-cross-bank` (line 49); 4 checks PASS |
| 2026-11-20 | S2-U06 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 6/9: `PASS Buddy 2 integrated-no-encoder` (line 58); 5 checks PASS incl. integrated STOP epoch |
| 2026-11-20 | S2-U07 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 7/9: `PASS Buddy 2 integrated-dual-encoder` (line 67); 5 checks PASS incl. integrated STOP epoch |
| 2026-11-20 | S2-U08 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 8/9: `PASS Buddy 2 integrated-custom-timeout` (line 76); 5 checks PASS |
| 2026-11-20 | S2-U09 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s2.txt` | Profile 9/9: `PASS Buddy 2 integrated-pid-turn` (line 85); 5 checks PASS incl. PID spins and held-wheel fault within 0.6 s; run.py `[exit code 0]` |
