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

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

No result rows were deleted from this log; `5f93cb6` changed only the *Test ID* cells of the nine 2026-11-20 rows (S2-U15 to S2-U23 became S2-U01 to S2-U09). The numbers S2-U01 to S2-U09 named the raised-wheel on-target motor checks at `d18dd38`; no result rows for those were ever committed.

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| S2-U01 | Straight (FORWARD) at 30% for 2 s: send and 250 ms renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: standalone motor control without wheel encoders |
| S2-U02 | Straight: left and right encoders both count pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: standalone motor control with both wheel encoders |
| S2-U03 | Right pivot (PIVOT_RIGHT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: wheel encoders work with swapped pins |
| S2-U04 | Right pivot: the left (driven) encoder counts pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: wheel encoders share one interrupt register |
| S2-U05 | Left pivot (PIVOT_LEFT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: wheel encoder pins span interrupt registers |
| S2-U06 | Left pivot: the right (driven) encoder counts pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: integrated motor control without wheel encoders |
| S2-U07 | Clockwise spin (RIGHT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: integrated motor control with both wheel encoders |
| S2-U08 | Clockwise spin: left and right encoders both count pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: command watchdog uses the custom timeout |
| S2-U09 | Counterclockwise spin (LEFT) at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: PID turns handle kicks and wheel stalls |
| S2-U10 | Counterclockwise spin: left and right encoders both count pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U11 | Reverse at 30% for 2 s: send and renewals accepted, then STOP reads STOPPED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U12 | Reverse: left and right encoders both count pulses | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U13 | Command watchdog: FORWARD 20% with no renewal for the timeout + 100 ms gives FAULT, E_TMOUT, command STOP, 0% power | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U14 | Final STOP after the watchdog FAULT reads STOPPED (PWM off) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U15 | Host profile `standalone-no-encoder` (`-DAPP_S2_ONLY=1`). Checks: lifecycle and argument errors (E_OBJ, E_PAR), init cleanup, 1 s command watchdog and clock rollover, stale or malformed messages, renewal and recovery, all 7 move directions, mailbox pool and transport errors, clock errors and 0% power, status-lock timeouts, init failure (FAULT, PWM off), turn-drive helpers, STOP in a command change's dead time, EDR motion adapter; encoder API gives E_NOSPT and never touches encoder registers. Also compiles `subsystem2_test.c` | S2-U01 | yes | not used |
| S2-U16 | Host profile `standalone-dual-encoder` (`-DAPP_S2_ONLY=1 -DS2_ENCODER_ENABLED=1`). Checks: as S2-U15, with encoders on: x1 counts on both wheels, forward/reverse sign, falling A edges as diagnostics only, snapshot retried across an interrupt, counter rollover, EDR encoder adapter | S2-U02 | yes | not used |
| S2-U17 | Host profile `standalone-dual-encoder-swapped` (motor 1 encoder GP4/5, motor 2 GP16/17). Checks: as S2-U16 with the wheel pins swapped | S2-U03 | yes | not used |
| S2-U18 | Host profile `standalone-dual-encoder-shared-bank` (motor 1 encoder GP2/3). Checks: as S2-U16 with both encoders in one GPIO interrupt register | S2-U04 | yes | not used |
| S2-U19 | Host profile `standalone-dual-encoder-cross-bank` (GP7/12 and GP14/16). Checks: as S2-U16 with encoder pins split across interrupt registers | S2-U05 | yes | not used |
| S2-U20 | Host profile `integrated-no-encoder` (`-DAPP_S2_ONLY=0`). Checks: as S2-U15, plus: only the VC may send (E_OACV), STOP epoch (STOP passes a full pool, old commands dropped, motor off within 11 ms), a missed STOP status copy is retried, an init FAULT stays FAULT after STOP (VC fault 7) | S2-U06 | yes | not used |
| S2-U21 | Host profile `integrated-dual-encoder` (`-DAPP_S2_ONLY=0 -DS2_ENCODER_ENABLED=1`). Checks: S2-U20 plus the S2-U16 encoder checks | S2-U07 | yes | not used |
| S2-U22 | Host profile `integrated-custom-timeout` (as S2-U21 with `-DS2_COMMAND_TIMEOUT_MS=750U`). Checks: as S2-U21; the watchdog fires at 750 ms | S2-U08 | yes | not used |
| S2-U23 | Host profile `integrated-pid-turn` (robot PID: `BOARD_PID_ENABLED=1`, MAX_CPS 1950, Kp/Ki 30/40 milli, PWM 35%, spins 55%, kick 150 ms at 45%, spin stall 1200 ms, trims 86/93). Checks: turn-drive helpers; PID spins (kick until real motion, re-kick, raised ceiling, longer stall; straight and pivots unchanged); STOP in dead time; stall time kept across FORWARD/PIVOT switches and short STOPs; a held wheel during follower switching faults within 0.6 s, free wheels never fault | S2-U09 | yes | not used |
| S2-U24 | Held wheel: stall check gives fault 7 and PWM off; RESET clears it | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U25 | Left/right encoder counts agree over 2 s at 35% demand | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U26 | 1 m straight: encoder distance matches the tape-measured distance | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U27 | 90° spins: mapped navigation turn (spin ends on the new line) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S2-U28 | 180° spins: mapped U-turn | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. Each cell names exactly one existing commit; none needed a note.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened.
