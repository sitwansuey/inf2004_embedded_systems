# Subsystem 4 — test log

**Subsystem:** Buddy 4, GY-511 / LSM303DLHC acquisition and snapshots
**Student ID:** 2500243 · **Code:** `app_program/subsystem4/`

See `testing/README.md` for evidence/commit conventions and
`app_program/subsystem4/README.md` for the bench/API guide.
Hardware and actual SMP runtime tests remain pending.

## Test cases

Define a test here before running it. Test IDs never change.

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | sensor identity and initialization faults are checked | driver | host auto | `driver_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U01 PASS  sensor identity/configuration and initialization faults` |
| S4-U02 | sensor readings preserve signs, axes and units | driver | host auto | `driver_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U02 PASS  signed decoding, XYZ ordering, units and freshness` |
| S4-U03 | clipped and invalid sensor samples are rejected | quality | host auto | `driver_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U03 PASS  clipped/out-of-range samples are invalid` |
| S4-U04 | channel failures preserve the healthy sensor | driver | host auto | `driver_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U04 PASS  partial reads and independent channel failures` |
| S4-U05 | driver locks recover and arguments are validated | driver | host auto | `driver_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U05 PASS  magnetic lock recovery and invalid API arguments` |
| S4-U06 | missing sensors publish no usable data | snapshot | host auto | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U06 PASS  absent/uninitialized state cannot publish usable data` |
| S4-U07 | cached samples expire without false timestamp updates | snapshot | host auto | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U07 PASS  no-data polls preserve timestamps; stale data expires` |
| S4-U08 | sensor channels recover independently after faults | snapshot | host auto | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U08 PASS  per-channel faults, recovery and reinitialization` |
| S4-U09 | invalid data and timestamps stay unusable | snapshot | host auto | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U09 PASS  clipping, future timestamps, 64-bit time and age saturation` |
| S4-U10 | lifecycle failures propagate and clean up | lifecycle/API | host auto, simulated kernel | `api_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U10 PASS  two-phase lifecycle, mutex failure retry and argument checks` |
| S4-U11 | snapshot failures preserve the caller output | IF-06 | host auto, simulated kernel | `api_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U11 PASS  protected snapshot, timeout propagation and kernel-clock wrap` |
| S4-U12 | tasks use correct scheduling and reject duplicate starts | lifecycle | host auto, simulated kernel | `api_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U12 PASS  task priorities/affinity and duplicate-start rejection` |
| S4-U13 | wall clock changes leave operating time unchanged | NFR3 (sample age); S4 -> VC/S1 | auto (host) | `api_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U13 PASS  unset/changed wall clock cannot alter shared operating time` |
| S4-U14 | encoder records are validated before use | S2 -> S4 in VC; FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U14 PASS exact lengths, reserved bits, invalid records, failed API and retained maximum` |
| S4-U15 | encoder freshness and timing gaps are checked | S2 -> S4 in VC; NFR3 (encoder data over 60 ms old) | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U15 PASS duplicates, zero intervals, stale/future input, skew, gaps and reboot history` |
| S4-U16 | encoder counts handle boundaries and discontinuities | S2 -> S4 in VC; FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U16 PASS unsigned/signed count boundaries, timer wrap, half-range and discontinuity` |
| S4-U17 | hump height handles interruptions and run resets | FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U17 PASS completed humps, partial recovery, reversal, shock, pause, calibration and run reset` |
| S4-U18 | clock errors preserve published and caller data | NFR3; S4 -> VC/S1 | auto (host) | `api_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U18 PASS  clock errors cannot publish a fresh sample or replace caller output` |
| S4-U19 | completed humps only increase the run maximum | FR10 (run maximum) | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U19 PASS smaller/larger completed humps update maximum only upward` |
| S4-U20 | compact cache preserves raw values and counters | S4 -> VC/S1 | auto (host) | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U20 compact cache: 564 bits/71 bytes, raw/time boundaries, exact units, counter isolation/wrap PASS` |
| S4-U21 | terrain result codec validates records and errors | S4 -> VC/S1; FR11 | auto (host) | `result_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U21 result codec: exact size, boundaries, padding, fault mapping, semantic checks and unchanged errors PASS` |
| S4-U22 | C and Python result receivers agree | S4 -> VC/S1; FR13 | auto (host) | `result_vectors.py` via `make -C app_program/subsystem4/tests/host test` | `S4-U22 result receivers: 57 valid and 202 rejected C/Python vectors PASS` |
| S4-U23 | packed odometry preserves motion and calibration values (same test as the October S4-U23 line below this table) | S2 -> S4 in VC; FR10 | auto (host) | `odometry_storage_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U23 PASS 4001 speeds, signed delta/yaw bounds and wraps, discontinuity/overflow, checked calibration; odometry=25 calibration=8 bytes` (byte sizes from Windows MSYS2 GCC; other compilers can print other sizes) |
| S4-U24 | clipped acceleration counts as shock without height | FR3; FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U24 PASS a clipped (+/-2 g) sample is a shock on every axis, never tilt or height` |
| S4-U25 | clipped acceleration is reported but never usable | FR3; NFR3 | auto (host) | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U25 PASS  a current clipped accel sample is reported clipped, never usable` |
| S4-U26 | plain and deep descents report no height | FR2; FR11 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U26 PASS a plain descent is DESCENT without a height, a deep far side stays DESCENT` |
| S4-U27 | brief sample faults preserve hump tracking | FR10; NFR3 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U27 PASS one shock/clipped/skewed/missing sample keeps a hump; more than 60 ms abandons it` |
| S4-U28 | hump completion latches valid results and counts | FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U28 PASS hump end latched with a count; invalid end reports 0 and clears the peak` |
| S4-U29 | shock flag clears on the next sample | FR3 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U29 PASS the shock flag clears on the next sample, usable or not` |
| S4-U30 | pivots preserve hump tracking until reversal | FR10 | auto (host) | `terrain_input_test.c` via `make -C app_program/subsystem4/tests/host test` | `S4-U30 PASS one wheel back a count during a pivot keeps the hump; a reversal ends it` |
| S4-U31 | impact and accelerometer faults are counted correctly | FR3; NFR3 | auto (host) | `data_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U31 PASS  impacts and accel faults are counted per sample, mag faults never` |
| S4-U32 | magnetometer faults leave the accelerometer running | NFR3 | auto (host) | `recovery_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U32 PASS  a dead or failing magnetometer never takes the accelerometer down` |
| S4-U33 | sensor reinitialization follows retry and back-off rules | NFR3 | auto (host) | `recovery_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U33 PASS  re-init on the next cycle, back-off only after a failed attempt, bus cleared first` |
| S4-U34 | unresponsive sensor data triggers reinitialization | NFR3 | auto (host) | `recovery_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U34 PASS  a sensor that answers without data is re-initialised` |
| S4-U35 | held data lines recover through clocks and STOP | NFR3 (I2C reset) | auto (host) | `recovery_test.c` via `make -C app_program/subsystem4/tests/host test` | `[TEST] S4-U35 PASS  a held SDA is clocked free (up to 9 clocks) and ends with a STOP` |
| S4-U36 | pitch and roll follow the stationary robot tilt | A4 (IMU mount); FR11 | manual (video) | 1. Flash the normal image; open Sensors on the dashboard. 2. Leave the DISARMED robot flat beside the hump; read Pitch and Roll. 3. Rest one side on the ramp so the robot leans forward/back and sideways; keep it still. 4. Film the robot and dashboard; read both again. | Flat: Pitch and Roll 0° ±1°. On ramp: both hold a steady tilt of at least 2° while still; terrain card stays supplied, no IMU fault or reboot. |

*Method:* **host auto** and **auto (host)** = a test runs on a PC; **host auto, simulated kernel** = a PC test with a simulated kernel; **manual (video)** = a person checks and films the result.

Notes on the current test set (added 2026-11-20, checkpoint 8c code):

- S4-U14 to S4-U17, S4-U19 to S4-U24 and S4-U26 to S4-U30 print without the `[TEST]` prefix.
  Search the log for `S4-U`.
- `make -C app_program/subsystem4/tests/host sanitize` runs the same host IDs with ASan/UBSan.
- Manual tests use the normal image (`cd build_make && make`) unless the steps say otherwise.
- S4-U39 replugs the IMU with the power off. The hot-plug times in NFR3 (fault within 1 s, back
  within 2 s) are not tested on the robot; S4-U33 and S4-U34 check the re-init timing in software.
- Next free ID: S4-U40.

`S4-U01` through `S4-U12` run with
`make -C app_program/subsystem4/tests/host test`.
They test software against mock bus/kernel calls, not real bus timing or
physical cross-core execution.

| S4-U23 | Packed odometry/calibration, all speed values, signed interval/yaw wraps and invalid inputs | CR1 / B2-to-B4 | host auto | `odometry_storage_test.c` | Values preserved; invalid input rejected before narrowing |

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| S4-U10 | S4-U01 |
| S4-U11 | S4-U02 |
| S4-U12 | S4-U03 |
| S4-U13 | S4-U04 |
| S4-U14 | S4-U05 |
| S4-U15 | S4-U06 |
| S4-U16 | S4-U07 |
| S4-U17 | S4-U08 |
| S4-U18 | S4-U09 |
| S4-U19 | S4-U10 |
| S4-U20 | S4-U11 |
| S4-U21 | S4-U12 |
| S4-U22 | S4-U13 |
| S4-U23 | S4-U14 |
| S4-U24 | S4-U15 |
| S4-U25 | S4-U16 |
| S4-U26 | S4-U17 |
| S4-U27 | S4-U18 |
| S4-U28 | S4-U19 |
| S4-U29 | S4-U20 |
| S4-U30 | S4-U21 |
| S4-U31 | S4-U22 |
| S4-U32 | S4-U23 |
| S4-U33 | S4-U24 |
| S4-U34 | S4-U25 |
| S4-U35 | S4-U26 |
| S4-U36 | S4-U27 |
| S4-U37 | S4-U28 |
| S4-U38 | S4-U29 |
| S4-U39 | S4-U30 |
| S4-U40 | S4-U31 |
| S4-U41 | S4-U32 |
| S4-U42 | S4-U33 |
| S4-U43 | S4-U34 |
| S4-U44 | S4-U35 |
| S4-U45 | S4-U36 |
| S4-U46 | S4-U37 |
| S4-U47 | S4-U38 |
| S4-U49 | S4-U39 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): S4-U01, S4-U02, S4-U07, S4-U08 and S4-U09 (retired in October), S4-U03 to S4-U06 (on-target IMU self-test, removed with `subsystem4_test.c`) and S4-U48 (never run). Removed on 2026-11-20.

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

| Old ID | New ID |
|---|---|
| S4-U40 | S4-U36 |

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): S4-U36, S4-U37, S4-U38, S4-U39.

</details>

## Results

Add a new row for every run. Never edit or delete old rows.

The original template row is retained; it is not a measured result.
Append actual runs below it. A local development commit is not a team GitHub
commit; record the team's actual tested commit for later board tests.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-17 | S4-U01 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U02 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U03 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U04 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U05 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U06 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U07 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U08 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U09 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U10 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U11 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U12 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-11-20 | S4-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U10 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U11 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U12 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U13 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U05 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U14 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U06 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U15 PASS` (data_test) |
| 2026-11-20 | S4-U07 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U16 PASS` (data_test) |
| 2026-11-20 | S4-U08 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U17 PASS` (data_test) |
| 2026-11-20 | S4-U09 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U18 PASS` (data_test) |
| 2026-11-20 | S4-U10 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U19 PASS` (api_test); simulated kernel |
| 2026-11-20 | S4-U11 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U20 PASS` (api_test); simulated kernel |
| 2026-11-20 | S4-U12 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U21 PASS` (api_test); simulated kernel, not real core placement |
| 2026-11-20 | S4-U13 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U22 PASS` (api_test) |
| 2026-11-20 | S4-U14 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U23 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U24 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U25 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U26 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U27 PASS` (api_test) |
| 2026-11-20 | S4-U19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U28 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U29 compact cache:` line ends PASS (data_test); record now 564 bits/71 bytes |
| 2026-11-20 | S4-U21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U30 result codec:` line ends PASS (result_test) |
| 2026-11-20 | S4-U22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U31 result receivers:` line ends PASS; 57 valid + 202 rejected = 259 vectors (test case text says 256) |
| 2026-11-20 | S4-U23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U32 PASS` (odometry_storage_test); odometry 25 bytes, calibration 8 bytes |
| 2026-11-20 | S4-U24 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U33 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U25 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U34 PASS` (data_test) |
| 2026-11-20 | S4-U26 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U35 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U27 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U36 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U28 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U37 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U29 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U38 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U30 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U39 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U31 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U40 PASS` (data_test) |
| 2026-11-20 | S4-U32 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U41 PASS` (recovery_test) |
| 2026-11-20 | S4-U33 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U42 PASS` (recovery_test); software timing only |
| 2026-11-20 | S4-U34 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U43 PASS` (recovery_test) |
| 2026-11-20 | S4-U35 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U44 PASS` (recovery_test); simulated bus |
| 2026-11-25 | S4-U36 | `98c9a60` | Ron Tan | PASS | https://youtu.be/cTbNUiUAZ10 | Dated the day the clip was sent (25 Nov, 09:14); the clip shows no clock. Commit: the image on the robot that morning (checkpoint 8e); S4's tilt code is the same in every build from 8c to 8g. Robot standing still beside the orange hump: Pitch 0° and Roll 0° on the dashboard (0:04-0:06). Set down with one side on the ramp and released (0:08.5): Pitch −3° and Roll −6° from 0:12 to 0:16.4, then Roll −8° (Pitch −3°) to the end at 0:16.7; the robot is off screen from 0:11. Terrain card 5 of 5 supplied, with values updating throughout. The ramp's angle was not measured. |

## Physical checks still to run

USB enumeration; actual identity/configuration; axis response; supply/pull-ups;
bus timeout/recovery under fault; polling/stack headroom; SMP snapshot reads
under the integrated workload. No physical result is recorded by a build.

## Buddy 4 continuation on 27 October 2026

Current local integration starts from published main `0269a02` and reconciles
the supplied integrated handoff. See `verification-20261027.json` for exact
commands, source/tool/artifact identities and all current outcomes.

S4 host groups U10–U28 pass (19 groups), including checked encoder/terrain
input, alignment, faults, repeats, wrap, interrupted humps, maximum retention,
reset, clock failure and smaller/larger completions. GCC `-fanalyzer` passes.
The actual vehicle task is exercised in `INT-S4-11`; B2's eight profiles include
saturated-pool STOP, old-command invalidation and running-motor STOP polling.
The full recovered integration and 302 C/Python aggregate vectors pass. B3's
64,886 host checks, actual kernel headers and ARM objects for SMP=0/1 pass.
The EDR gate passes nine Python methods and 268,307 C checks.

Default integrated and isolated S4Bench ELF/UF2 builds pass using the installed
Pico SDK 2.3.0 and Arm GNU 15.2.1. Neither was flashed. ASan/UBSan could not link
because the installed native compiler lacks libasan/libubsan; see sanitizers.log.
This does not replace physical validation, runtime memory or timing evidence.

The staged physical guide and Buddy 4 ownership recap are in
`docs/BUDDY4_TESTING.md` and `docs/BUDDY4_IMPLEMENTATION.md`.

## CR1 compact storage continuation after checkpoint 373ad6d

The published draft PR #8 preserves the checkpoint. Subsequent local work
replaces persistent and cross-task S4 acquisition records with 500-bit/63-byte
records, narrows task-local terrain fields (165 bits, 21 ARM bytes), and adds
a checked 93-bit/12-byte S4 terrain result consumed by both VC and B1.
See cr1-verification.json for current commands, source hashes and build sizes;
verification-20261027.json remains the earlier checkpoint evidence.

Host groups U10?U31 and GCC static analysis pass, including 259 independent
C/Python result vectors and compact cache raw/time/counter boundaries. The actual
vehicle and telemetry task tests exercise the compact records and rejection
behavior. The unchanged aggregate telemetry remains 50 bytes. Host performance
and before/after memory comparisons are explicitly separate from physical
measurements. ASan/UBSan, physical calibration, load timing and runtime memory
high-water evidence remain outstanding as documented in the current CR1 review.

## Main 58b89c2 reconciliation and completed B4 storage pass

The main-reconciled baseline is commit 5f0b8c7. The current software results,
source/artifact hashes, exact layouts and profile comparisons are in
ready-verification.json and ready-prefixed logs. B4's 23 host groups and GCC
analysis pass, including S4-U32. Integration passes with actual controller and
telemetry consumers; signed shared speed is also tested through the real
50-byte telemetry codec. B3 host/analysis/header/ARM/failure-propagation and the
EDR/B2/S5 gate pass. Default, S4Bench and MQTT ARM builds pass. No firmware was
flashed, no motor command was sent and physical acceptance remains pending.
Linux ASan/UBSan results belong to the exact pushed PR head's CI, separately
from the local Windows checks. The before/after benchmark checksums agree;
host timing is not target latency evidence.
