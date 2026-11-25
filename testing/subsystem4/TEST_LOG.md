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
| S4-U51 | sensor status rejects a NULL output | S4 status (`s4_sensor_get_status()`) | auto (integrated TEST=1); auto (TEST=1 S4_ONLY=1) | `subsystem4_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. Bench: `make -C build_make TEST=1 S4_ONLY=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2`. | `[TEST] S4-U51 PASS  sensor snapshot rejects NULL` |
| S4-U52 | both S4 tasks exist, are not dormant and have base priorities `S4_PRI_SENSOR` and `S4_PRI_MONITOR` | TEAM.md priority table (S4 band 18–19) | auto (integrated TEST=1); auto (TEST=1 S4_ONLY=1) | `subsystem4_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. Bench: `make -C build_make TEST=1 S4_ONLY=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2`. | `[TEST] S4-U52 PASS  both S4 tasks exist and use the assigned priority band` |
| S4-U53 | sensor task heartbeat advances over 500 ms | S4 status (`s4_sensor_get_status()`) | auto (integrated TEST=1); auto (TEST=1 S4_ONLY=1) | `subsystem4_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. Bench: `make -C build_make TEST=1 S4_ONLY=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2`. | `[TEST] S4-U53 PASS  acquisition task keeps running` |
| S4-U54 | IMU configured with `init_result` `LSM303_OK` within 1 s (accelerometer configuration accepted by the S4 task; the magnetometer is checked by S4-U55) | S4 status (`s4_sensor_get_status()`) | auto (integrated TEST=1); auto (TEST=1 S4_ONLY=1) | `subsystem4_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. Bench: `make -C build_make TEST=1 S4_ONLY=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2`. | `[TEST] S4-U54 PASS  IMU accelerometer configuration accepted` |
| S4-U55 | connected IMU delivers fresh accelerometer and magnetometer samples, both usable | S4 status (`s4_sensor_get_status()`) | auto (integrated TEST=1); auto (TEST=1 S4_ONLY=1) | `subsystem4_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. Bench: `make -C build_make TEST=1 S4_ONLY=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_enc_s4_test.uf2`. | `[TEST] S4-U55 PASS  connected IMU delivers fresh, unclipped channel data`; `S4-U55 SKIP` means S4-U54 failed |

*Method:* **host auto** and **auto (host)** = a test runs on a PC; **host auto, simulated kernel** = a PC test with a simulated kernel; **manual (video)** = a person checks and films the result; **auto (integrated TEST=1)** = a `TEST_CHECK` that runs on the robot in the integrated test image; **auto (TEST=1 S4_ONLY=1)** = the same check in the S4 bench test image.

S4-U51 to S4-U55 (added 2026-11-25) are passive on-target checks in `app_program/subsystem4/subsystem4_test.c`, restored after `5f93cb6` deleted the old self-test file. They take new numbers because every lower number has already named another test (see the ID history at the end of this log). They read status only: no motor, servo, sonar or MQTT command. No result is recorded yet; they have not been run on the robot.

Notes on the current test set (added 2026-11-20, checkpoint 8c code):

- S4-U14 to S4-U17, S4-U19 to S4-U24 and S4-U26 to S4-U30 print without the `[TEST]` prefix.
  Search the log for `S4-U`.
- `make -C app_program/subsystem4/tests/host sanitize` runs the same host IDs with ASan/UBSan.
- Manual tests use the normal image (`cd build_make && make`) unless the steps say otherwise.
- S4-U39 replugs the IMU with the power off. The hot-plug times in NFR3 (fault within 1 s, back
  within 2 s) are not tested on the robot; S4-U33 and S4-U34 check the re-init timing in software.
- Next free ID: S4-U56. (S4-U37 to S4-U50 have been used before; see the ID history at the end of this log.)

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
| 2026-10-17 | S4-U01 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U02 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U03 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U04 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U05 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U06 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U07 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U08 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U09 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U10 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U11 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U12 | `9b6de29ae296` (local) [^s4-c1] | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
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

### Restored result rows (audit, 2026-11-25)

These result rows were deleted from the table above in `5f93cb6` (2026-11-25). They are copied back word for word from `d18dd38`, the last commit that had them; the first two columns are the only additions. Their *Test ID* cells use the IDs in force at `d18dd38`, not today's IDs, and some of those numbers now name different tests (second column, and *ID history (git record)* at the end of this file). Nothing was re-run.

| Restored from | Test ID meaning (then → now) | Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|---|
| `d18dd38` (deleted in `5f93cb6`) | S4-U45 at `d18dd38` = level robot: pitch and roll read near 0 after the mount zeros; test removed in `5f93cb6`; no current test uses S4-U45 | 2026-11-19 | S4-U45 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t7_usb_no_imu_broker_restart.txt`, `../system/logs/2026-11-19_t7_mqtt_idle.txt` | Robot disarmed and still on the bench, t=23.5-219.9 s (lines 73-3257). Surface not checked with a level; no 180° turn. Dashboard Pitch and Roll (whole degrees) read 0 in all 439 snapshots of the idle capture. `[S4TILT]` single samples: pitch median +21 cdeg (5-95%: -50 to +112), roll median +84 cdeg (+10 to +136); both within ±100 cdeg in 659 of 936 samples. |
| `d18dd38` (deleted in `5f93cb6`) | S4-U46 at `d18dd38` = six-face orientation: each pose gives about 1 g on the expected …; test removed in `5f93cb6`; no current test uses S4-U46 | 2026-11-19 | S4-U46 | `7a6c373` | Ron Tan | INCOMPLETE | `logs/2026-11-19_t1_usb_hand_tilt.txt` | Normal image, not S4_ONLY; robot disarmed and turned by hand; 2 of 6 poses. Wheels down, t=429.0-483.8 s (lines 69-959): median ax -36, ay -254, az +916 mg; 261 of 262 samples in range (one bump at line 953, az +1479). Upside down about 1.7 s, not held still for 5 s (t=535.50-537.18 s, lines 1991-2019): az -854 to -1049 mg, but ax +224 to +689 and ay +305 to +395 (over 350 mg in 3 and 4 of 9 samples). Nose up/down and side poses not held (ax +909 mg only in passing, line 1984). a_ok 1 throughout. |
| `d18dd38` (deleted in `5f93cb6`) | S4-U47 at `d18dd38` = a firm tap sets the impact flag and count; impacts do not stop the …; test removed in `5f93cb6`; no current test uses S4-U47 | 2026-11-19 | S4-U47 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t1_mqtt_hand_tilt.txt` | Robot disarmed on the bench. Impact True in 2 snapshots (59.3 s, seq 136; 63.3 s, seq 157), each cleared in the next snapshot; state stayed Disarmed, fault 0. Number and timing of taps not logged. This build's `[S4]` line has no `a_impacts` column. Step 4 (armed on a stand) not run. |
| `d18dd38` (deleted in `5f93cb6`) | S4-U49 at `d18dd38` = boot without the IMU, then replug it: fault shown, no reboot, recovery; test removed in `5f93cb6`; no current test uses S4-U49 | 2026-11-19 | S4-U49 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t7_usb_no_imu_broker_restart.txt` | IMU unplugged before boot. t=0-3.57 s (lines 14-70): `[S4]` init 3 (I/O error), a_ok 0, heartbeat 1 to 356; `[S4TILT]` valid 0; `[CSC02]` flags 7 (175 when healthy). Console lines from 3.8 s to 23.5 s missing (lines 72-73), so the 60 s watch is not shown. IMU then plugged in with the power on, not step 5 (time not logged): at t=23.52 s (line 73) a_ok 1, init 0, a_errors 0, pitch 57 / roll 83 cdeg; heartbeat 2350 and uptime continuous to 403.4 s, no reboot. No `inits` column in this build. Steps 4 and 6 not run; dashboard not captured while the IMU was out. |
| `d18dd38` (deleted in `5f93cb6`) | S4-U45 at `d18dd38` = level robot: pitch and roll read near 0 after the mount zeros; test removed in `5f93cb6`; no current test uses S4-U45 | 2026-11-20 | S4-U45 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_flash_8c_boot.txt` | Robot disarmed at rest for 45 s after flashing (t=0.21-45.57 s, lines 28-550). Surface not checked with a level; no 180° turn; dashboard not captured. `[S4TILT]` single samples: pitch median -39 cdeg (5-95%: -139 to +2), roll median +33 cdeg (-58 to +80); both within ±100 cdeg in 189 of 217 samples (largest pitch +698 cdeg, line 451). |

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

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

`5f93cb6` also changed the *Test ID* cells of the twelve 2026-10-17 rows (S4-U10 to S4-U21 became S4-U01 to S4-U12, the same mapping as the table below).

Today's S4-U36 has no earlier ID in git: the 2026-11-25 table lists it as S4-U40, and S4-U40 in the 2026-11-21 numbering does not appear in any committed version before `5f93cb6` (S4-U40 at `d18dd38` is a different, host test; see below).

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| S4-U01 | legacy message buffer retains S4_SUMMARY_MAX | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: sensor identity and initialization faults are checked |
| S4-U02 | retired: monitor used to pause/resume S1 LED | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: sensor readings preserve signs, axes and units |
| S4-U03 | snapshot rejects NULL | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: clipped and invalid sensor samples are rejected |
| S4-U04 | both S4 tasks exist at priorities 18/19 | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: channel failures preserve the healthy sensor |
| S4-U05 | acquisition heartbeat continues | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: driver locks recover and arguments are validated |
| S4-U06 | connected device produces fresh usable channels | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: missing sensors publish no usable data |
| S4-U07 | six-face acceleration signs/scaling | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: cached samples expire without false timestamp updates |
| S4-U08 | magnetic response to rotation | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: sensor channels recover independently after faults |
| S4-U09 | absent-sensor boot does not stop USB/task | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: invalid data and timestamps stay unusable |
| S4-U10 | identity/configuration and initialization faults | S4-U01 | yes | different test: lifecycle failures propagate and clean up |
| S4-U11 | signed decoding, XYZ remapping, units/freshness | S4-U02 | yes | different test: snapshot failures preserve the caller output |
| S4-U12 | clipped/out-of-range data rejected | S4-U03 | yes | different test: tasks use correct scheduling and reject duplicate starts |
| S4-U13 | partial reads and independent channel faults | S4-U04 | yes | different test: wall clock changes leave operating time unchanged |
| S4-U14 | lock recovery and invalid arguments | S4-U05 | yes | different test: encoder records are validated before use |
| S4-U15 | initial/missing sensor state | S4-U06 | yes | different test: encoder freshness and timing gaps are checked |
| S4-U16 | cache age and stale expiry | S4-U07 | yes | different test: encoder counts handle boundaries and discontinuities |
| S4-U17 | per-channel faults, recovery/reinitialization | S4-U08 | yes | different test: hump height handles interruptions and run resets |
| S4-U18 | clipping, future/64-bit time and age saturation | S4-U09 | yes | different test: clock errors preserve published and caller data |
| S4-U19 | init/start order, cleanup and bad arguments | S4-U10 | yes | different test: completed humps only increase the run maximum |
| S4-U20 | snapshot/lock failures and clock wrap | S4-U11 | yes | different test: compact cache preserves raw values and counters |
| S4-U21 | task priority/affinity and duplicate start | S4-U12 | yes | different test: terrain result codec validates records and errors |
| INT-02 | legacy S3 summary lines reach S4 | not an S4 ID: a legacy integration row in the S4 test-case table at `d18dd38`; not in today's S4 table | — | not used |
| S4-U22 | an unset or changed wall clock cannot change the shared operating time | S4-U13 | yes | different test: C and Python result receivers agree |
| S4-U23 | encoder snapshot input: exact length, reserved bits, invalid records, failed API; run maximum kept | S4-U14 | yes | different test: packed odometry preserves motion and calibration values (same test as the October S4-U23 line below this table) |
| S4-U24 | encoder input: duplicates, zero intervals, stale or future input, IMU skew, gaps, reboot history | S4-U15 | yes | different test: clipped acceleration counts as shock without height |
| S4-U25 | encoder counts: unsigned and signed limits, timer wrap, half-range jump, discontinuity | S4-U16 | yes | different test: clipped acceleration is reported but never usable |
| S4-U26 | hump height: completed humps, partial recovery, reversal, shock, pause, calibration, run reset | S4-U17 | yes | different test: plain and deep descents report no height |
| S4-U27 | a clock error cannot publish a fresh sample or overwrite the caller's output | S4-U18 | yes | different test: brief sample faults preserve hump tracking |
| S4-U28 | run maximum only goes up: a smaller, then a larger completed hump | S4-U19 | yes | different test: hump completion latches valid results and counts |
| S4-U29 | compact acquisition record: size, raw and time limits, exact units, counter isolation and wrap | S4-U20 | yes | different test: shock flag clears on the next sample |
| S4-U30 | 12-byte terrain result codec: size, limits, padding bits, fault mapping, field checks; output unchanged on error | S4-U21 | yes | different test: pivots preserve hump tracking until reversal |
| S4-U31 | the C decoder and the Python receiver (`tools/buddy4_result.py`) agree on 256 result vectors; padding bits rejected | S4-U22 | yes | different test: impact and accelerometer faults are counted correctly |
| S4-U32 | packed odometry/calibration, all speeds, signed interval and yaw wraps, invalid input (same test as the October S4-U32 line below this table) | S4-U23 | yes | different test: magnetometer faults leave the accelerometer running |
| S4-U33 | a clipped (±2 g) sample is a shock on every axis, never tilt or height | S4-U24 | yes | different test: sensor reinitialization follows retry and back-off rules |
| S4-U34 | a current clipped accelerometer sample is reported as clipped and is never usable | S4-U25 | yes | different test: unresponsive sensor data triggers reinitialization |
| S4-U35 | a plain descent is DESCENT with no height; a deep far side stays DESCENT | S4-U26 | yes | different test: held data lines recover through clocks and STOP |
| S4-U36 | one shock, clipped, skewed or missing sample keeps a hump; over 60 ms abandons it | S4-U27 | yes | different test: pitch and roll follow the stationary robot tilt |
| S4-U37 | a hump end is latched with a count; an invalid end reports 0 and clears the peak | S4-U28 | yes | not used |
| S4-U38 | the shock flag clears on the next sample, usable or not | S4-U29 | yes | not used |
| S4-U39 | one wheel moving back a count in a pivot keeps the hump; a reversal ends it | S4-U30 | yes | not used |
| S4-U40 | impacts (over 1.8 g or clipped) and accelerometer faults are counted per sample; magnetometer faults are not | S4-U31 | yes | not used |
| S4-U41 | a dead or failing magnetometer never takes the accelerometer down | S4-U32 | yes | not used |
| S4-U42 | re-init on the next cycle after 10 failed polls; 1 s back-off only after a failed attempt; bus cleared first | S4-U33 | yes | not used |
| S4-U43 | a sensor that answers but sends no data is re-initialised | S4-U34 | yes | not used |
| S4-U44 | a held SDA line is clocked free (up to 9 clocks) and ends with a STOP | S4-U35 | yes | not used |
| S4-U45 | level robot: pitch and roll read near 0 after the mount zeros | removed from the table in `5f93cb6` (S4-U36 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S4-U46 | six-face orientation: each pose gives about 1 g on the expected sensor axis | removed from the table in `5f93cb6` (S4-U37 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S4-U47 | a firm tap sets the impact flag and count; impacts do not stop the robot | removed from the table in `5f93cb6` (S4-U38 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S4-U48 | hump height by hand (the driven check at 20% demand is a system test) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S4-U49 | boot without the IMU, then replug it: fault shown, no reboot, recovery | removed from the table in `5f93cb6` (S4-U39 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. A cell that names exactly one existing commit is not marked. Any other cell keeps its original text and carries a footnote; the footnotes are at the end of this file.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened.

Files named in the prose sections of this log that are not in the repository at `0bac48c` (each was deleted in `54a371f`, 2026-10-27; the last versions are in its parent `efedeb1`): `testing/subsystem4/verification-20261027.json`, `testing/subsystem4/cr1-verification.json`, `testing/subsystem4/ready-verification.json`, `testing/subsystem4/sanitizers.log`, the `ready`-prefixed logs, `app_program/subsystem4/README.md`, `docs/BUDDY4_TESTING.md` and `docs/BUDDY4_IMPLEMENTATION.md`.

[^s4-c1]: Hash `9b6de29ae296` not found in repository history (`git cat-file`, all branches, 2026-11-25). The original entry marks it "(local)": a development commit that was never pushed, so the code these rows ran on is not in the repository.
