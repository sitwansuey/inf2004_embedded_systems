# Subsystem 4 — test log

**Subsystem:** Buddy 4, GY-511 / LSM303DLHC acquisition and snapshots
**Student ID:** 2500243 · **Code:** `app_program/subsystem4/`

See `testing/README.md` for evidence/commit conventions and
`app_program/subsystem4/README.md` for the bench/API guide.
Hardware and actual SMP runtime tests remain pending.

## Test cases

Define a test here before running it. Test IDs never change.

Existing IDs are retained. S4-U02 is retired and must not be reused.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | legacy message buffer retains S4_SUMMARY_MAX | IF-03 | on-target auto | S4_ONLY=1 TEST=1 | PASS; no sensor needed |
| S4-U02 | retired: monitor used to pause/resume S1 LED | retired IF-05 | not run | S4 no longer controls S1 | retired; do not reuse ID |
| S4-U03 | snapshot rejects NULL | IF-06 | on-target auto | `subsystem4_test.c` | E_PAR |
| S4-U04 | both S4 tasks exist at priorities 18/19 | lifecycle | on-target auto | `subsystem4_test.c` | PASS |
| S4-U05 | acquisition heartbeat continues | liveness | on-target auto | compare snapshots 500 ms apart | counter increases, including absent sensor |
| S4-U06 | connected device produces fresh usable channels | sensor bring-up | on-target auto, needs hardware | connect verified GY-511; run test image | both sample counters increase; usable flags true; SKIP if not configured |
| S4-U07 | six-face acceleration signs/scaling | calibration preparation | manual | record X+/X-/Y+/Y-/Z+/Z- at rest | gravity vector roughly 1000 mg; signs follow sensor axes |
| S4-U08 | magnetic response to rotation | calibration preparation | manual | rotate slowly away from motors/magnets | XYZ readings respond; no clipped data |
| S4-U09 | absent-sensor boot does not stop USB/task | fault handling | manual | boot without sensor and save 30 s | time/heartbeat progress; unusable values; init retries |
| S4-U10 | identity/configuration and initialization faults | driver | host auto | driver_test.c | PASS |
| S4-U11 | signed decoding, XYZ remapping, units/freshness | driver | host auto | driver_test.c | PASS |
| S4-U12 | clipped/out-of-range data rejected | quality | host auto | driver_test.c | PASS |
| S4-U13 | partial reads and independent channel faults | driver | host auto | driver_test.c | healthy channel preserved |
| S4-U14 | lock recovery and invalid arguments | driver | host auto | driver_test.c | PASS |
| S4-U15 | initial/missing sensor state | snapshot | host auto | data_test.c | no usable data |
| S4-U16 | cache age and stale expiry | snapshot | host auto | data_test.c | NO_DATA does not refresh timestamp |
| S4-U17 | per-channel faults, recovery/reinitialization | snapshot | host auto | data_test.c | invalidation/recovery and counters correct |
| S4-U18 | clipping, future/64-bit time and age saturation | snapshot | host auto | data_test.c | invalid time/data never usable |
| S4-U19 | init/start order, cleanup and bad arguments | lifecycle/API | host auto, simulated kernel | api_test.c | failures propagate; partial init cleaned up |
| S4-U20 | snapshot/lock failures and clock wrap | IF-06 | host auto, simulated kernel | api_test.c | no overwritten output on failure; coherent copy path |
| S4-U21 | task priority/affinity and duplicate start | lifecycle | host auto, simulated kernel | api_test.c | priority 18/core 1, priority 19/any |
| INT-02 | legacy S3 summary lines reach S4 | IF-03 | integrated on-target | existing test; not runnable in S4_ONLY | at least two lines in 3.5 s; SKIP in S4_ONLY |
| S4-U22 | an unset or changed wall clock cannot change the shared operating time | NFR3 (sample age); S4 -> VC/S1 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`api_test.c`) | `[TEST] S4-U22 PASS` |
| S4-U23 | encoder snapshot input: exact length, reserved bits, invalid records, failed API; run maximum kept | S2 -> S4 in VC; FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U23 PASS` line |
| S4-U24 | encoder input: duplicates, zero intervals, stale or future input, IMU skew, gaps, reboot history | S2 -> S4 in VC; NFR3 (encoder data over 60 ms old) | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U24 PASS` line |
| S4-U25 | encoder counts: unsigned and signed limits, timer wrap, half-range jump, discontinuity | S2 -> S4 in VC; FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U25 PASS` line |
| S4-U26 | hump height: completed humps, partial recovery, reversal, shock, pause, calibration, run reset | FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U26 PASS` line |
| S4-U27 | a clock error cannot publish a fresh sample or overwrite the caller's output | NFR3; S4 -> VC/S1 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`api_test.c`) | `[TEST] S4-U27 PASS` |
| S4-U28 | run maximum only goes up: a smaller, then a larger completed hump | FR10 (run maximum) | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U28 PASS` line |
| S4-U29 | compact acquisition record: size, raw and time limits, exact units, counter isolation and wrap | S4 -> VC/S1 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`data_test.c`) | `S4-U29 compact cache:` line ending `PASS` |
| S4-U30 | 12-byte terrain result codec: size, limits, padding bits, fault mapping, field checks; output unchanged on error | S4 -> VC/S1; FR11 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`result_test.c`) | `S4-U30 result codec:` line ending `PASS` |
| S4-U31 | the C decoder and the Python receiver (`tools/buddy4_result.py`) agree on 256 result vectors; padding bits rejected | S4 -> VC/S1; FR13 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`result_vectors.py`) | `S4-U31 result receivers:` line ending `PASS` |
| S4-U32 | packed odometry/calibration, all speeds, signed interval and yaw wraps, invalid input (same test as the October S4-U32 line below this table) | S2 -> S4 in VC; FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`odometry_storage_test.c`) | `S4-U32 PASS` line |
| S4-U33 | a clipped (±2 g) sample is a shock on every axis, never tilt or height | FR3; FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U33 PASS` line |
| S4-U34 | a current clipped accelerometer sample is reported as clipped and is never usable | FR3; NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`data_test.c`) | `[TEST] S4-U34 PASS` |
| S4-U35 | a plain descent is DESCENT with no height; a deep far side stays DESCENT | FR2; FR11 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U35 PASS` line |
| S4-U36 | one shock, clipped, skewed or missing sample keeps a hump; over 60 ms abandons it | FR10; NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U36 PASS` line |
| S4-U37 | a hump end is latched with a count; an invalid end reports 0 and clears the peak | FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U37 PASS` line |
| S4-U38 | the shock flag clears on the next sample, usable or not | FR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U38 PASS` line |
| S4-U39 | one wheel moving back a count in a pivot keeps the hump; a reversal ends it | FR10 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`terrain_input_test.c`) | `S4-U39 PASS` line |
| S4-U40 | impacts (over 1.8 g or clipped) and accelerometer faults are counted per sample; magnetometer faults are not | FR3; NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`data_test.c`) | `[TEST] S4-U40 PASS` |
| S4-U41 | a dead or failing magnetometer never takes the accelerometer down | NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`recovery_test.c`) | `[TEST] S4-U41 PASS` |
| S4-U42 | re-init on the next cycle after 10 failed polls; 1 s back-off only after a failed attempt; bus cleared first | NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`recovery_test.c`) | `[TEST] S4-U42 PASS` |
| S4-U43 | a sensor that answers but sends no data is re-initialised | NFR3 | auto (host) | `make -C app_program/subsystem4/tests/host test` (`recovery_test.c`) | `[TEST] S4-U43 PASS` |
| S4-U44 | a held SDA line is clocked free (up to 9 clocks) and ends with a STOP | NFR3 (I2C reset) | auto (host) | `make -C app_program/subsystem4/tests/host test` (`recovery_test.c`) | `[TEST] S4-U44 PASS` |
| S4-U45 | level robot: pitch and roll read near 0 after the mount zeros | A4 (IMU mount); FR11 | manual | 1. Flash the normal image. Open the dashboard and the USB console. 2. Put the robot, DISARMED, on a surface that a level shows within ±0.2°. 3. Wait 10 s. Read Pitch and Roll on the dashboard (or `pitch_cdeg` and `roll_cdeg` in the `[S4TILT]` lines) for 10 s. 4. Turn the robot 180° on the same spot and repeat step 3. | Pitch and Roll stay within ±1.0° (±100 cdeg) in both directions. |
| S4-U46 | six-face orientation: each pose gives about 1 g on the expected sensor axis | IMU mount (front -Y, left +X, up +Z); FR3 (0.8-1.2 g) | manual | 1. In `build_make`, build `make S4_ONLY=1` and flash it (only S4 runs; the motors are not driven). 2. Open the USB console. 3. Hold the robot still for 5 s in each pose: wheels down, upside down, nose up, nose down, left side up, right side up. 4. In each pose, note `ax_mg`, `ay_mg` and `az_mg` from the `[S4]` lines. | Wheels down: `az` +800 to +1200. Upside down: `az` -800 to -1200. Nose up: `ay` -800 to -1200. Nose down: `ay` +800 to +1200. Left side up: `ax` +800 to +1200. Right side up: `ax` -800 to -1200. The other two axes stay within ±350 mg. `a_ok` is 1 in every pose. |
| S4-U47 | a firm tap sets the impact flag and count; impacts do not stop the robot | FR3; NFR1 | manual | 1. Flash the normal image. Open the dashboard and the USB console. 2. With the robot DISARMED on a desk, give 5 firm taps on the chassis, 5 s apart. 3. After each tap, read Impact on the dashboard and `a_impacts` in the `[S4]` lines. 4. Put the robot on a stand (wheels free). Press `r`, then `g`, and give 3 taps within 2 s. 5. Press `s`. | Each tap: Impact shows Yes within 1 s (held 2 s) and `a_impacts` rises by at least 1. Step 4: the wheels keep turning; no fault 1 (tilt) or STOP follows a tap. |
| S4-U48 | hump height by hand (the driven check at 20% demand is a system test) | FR10; FR13 | manual | 1. Flash the normal image. Open the dashboard. 2. Put the robot, DISARMED, level on the floor. Note Run maximum height (RESET clears it). 3. Make a smooth up-and-down ramp over a block of known height, 20-100 mm (measure it with a ruler). 4. Push the robot slowly over it with the wheels rolling. Stop level on the far side. 5. Read Current hump peak (the dashboard shows cm). 6. Do steps 4-5 five times. | In at least 4 of 5 crossings, Current hump peak is within ±5 mm or ±15% of the block height (whichever is larger). Run maximum height never goes down. |
| S4-U49 | boot without the IMU, then replug it: fault shown, no reboot, recovery | NFR3; NFR2 | manual | 1. Power off. Unplug the IMU Grove cable (never with the power on; hot-plugging is not known to be safe). 2. Power on with the normal image. Open the dashboard and the USB console. 3. Watch for 60 s. 4. Put the robot on a stand. Press `r`, then `g`. 5. Press `s`. Power off, plug the IMU back in and power on. 6. Watch for 10 s. | Step 3: the console and dashboard keep updating with no reboot; `[S4]` shows `a_ok` 0 and `inits` rising by at most 1 per second; IMU fault shows Yes or the accelerometer shows as unavailable. Step 4: fault 1 (tilt) within 1 s and power 0. Step 6: `a_ok` 1 within 2 s of boot, IMU fault No, pitch and roll near 0. |

*Method:* **auto** = `TEST_CHECK` in `app_program/subsystem4/subsystem4_test.c`, printed on the USB
console by the S4_ONLY=1 TEST=1 image (`cd build_make && make S4_ONLY=1 TEST=1`; the image name
ends `_s4_test.uf2`);
**auto (host)** = host test on a PC: `make -C app_program/subsystem4/tests/host test` builds and
runs every host test and prints one PASS line per ID; make stops with an error at the first failure;
**manual** = a person follows the numbered steps and checks the result (USB console, dashboard,
ruler, level). Older rows above say "on-target auto" (= auto) and "host auto" (= auto (host)).

Notes on the current test set (added 2026-11-20, checkpoint 8c code):

- S4-U03 to S4-U06 run only in the S4_ONLY=1 TEST=1 image. The integrated TEST=1 image does not
  call `subsystem4_test()` (`app_program/app_main.c`). S4-U06 prints `[TEST] S4-U06 SKIP` when no
  sensor is configured; record that as SKIP, not PASS.
- S4-U23 to S4-U26, S4-U28 to S4-U33 and S4-U35 to S4-U39 print without the `[TEST]` prefix.
  Search the log for `S4-U`.
- `make -C app_program/subsystem4/tests/host sanitize` runs the same host IDs with ASan/UBSan.
- Manual tests use the normal image (`cd build_make && make`) unless the steps say otherwise.
- S4-U49 replugs the IMU with the power off. The hot-plug times in NFR3 (fault within 1 s, back
  within 2 s) are not tested on the robot; S4-U42 and S4-U43 check the re-init timing in software.
- Retired - do not reuse: S4-U01 (message-buffer check; IF-03 was removed and the check is no
  longer in code), S4-U02 (retired in October), S4-U07 (bare-sensor six-face check; replaced by
  S4-U46 on the mounted robot), S4-U08 (magnetometer rotation check; the magnetometer is not used
  for control and is not in the current plan), S4-U09 (absent-sensor boot on the S4_ONLY bench;
  replaced by S4-U49).
- INT-02 is no longer in `subsystem4_test.c`. Its status is kept in `testing/integration/TEST_LOG.md`.
- Next free ID: S4-U50.

`S4-U10` through `S4-U21` run with
`make -C app_program/subsystem4/tests/host test`.
They test software against mock bus/kernel calls, not real bus timing or
physical cross-core execution.

| S4-U32 | Packed odometry/calibration, all speed values, signed interval/yaw wraps and invalid inputs | CR1 / B2-to-B4 | host auto | `odometry_storage_test.c` | Values preserved; invalid input rejected before narrowing |

## Results

Add a new row for every run. Never edit or delete old rows.

The original template row is retained; it is not a measured result.
Append actual runs below it. A local development commit is not a team GitHub
commit; record the team's actual tested commit for later board tests.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-17 | S4-U10 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U11 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U12 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U13 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U14 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U15 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U16 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U17 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U18 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U19 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U20 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U21 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-11-20 | S4-U10 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U10 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U11 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U11 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U12 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U12 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U13 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U13 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U14 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U14 PASS` (driver_test); mock bus, host only |
| 2026-11-20 | S4-U15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U15 PASS` (data_test) |
| 2026-11-20 | S4-U16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U16 PASS` (data_test) |
| 2026-11-20 | S4-U17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U17 PASS` (data_test) |
| 2026-11-20 | S4-U18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U18 PASS` (data_test) |
| 2026-11-20 | S4-U19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U19 PASS` (api_test); simulated kernel |
| 2026-11-20 | S4-U20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U20 PASS` (api_test); simulated kernel |
| 2026-11-20 | S4-U21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U21 PASS` (api_test); simulated kernel, not real core placement |
| 2026-11-20 | S4-U22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U22 PASS` (api_test) |
| 2026-11-20 | S4-U23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U23 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U24 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U24 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U25 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U25 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U26 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U26 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U27 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U27 PASS` (api_test) |
| 2026-11-20 | S4-U28 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U28 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U29 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U29 compact cache:` line ends PASS (data_test); record now 564 bits/71 bytes |
| 2026-11-20 | S4-U30 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U30 result codec:` line ends PASS (result_test) |
| 2026-11-20 | S4-U31 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U31 result receivers:` line ends PASS; 57 valid + 202 rejected = 259 vectors (test case text says 256) |
| 2026-11-20 | S4-U32 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U32 PASS` (odometry_storage_test); odometry 25 bytes, calibration 8 bytes |
| 2026-11-20 | S4-U33 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U33 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U34 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U34 PASS` (data_test) |
| 2026-11-20 | S4-U35 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U35 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U36 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U36 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U37 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U37 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U38 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U38 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U39 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `S4-U39 PASS` (terrain_input_test) |
| 2026-11-20 | S4-U40 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U40 PASS` (data_test) |
| 2026-11-20 | S4-U41 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U41 PASS` (recovery_test) |
| 2026-11-20 | S4-U42 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U42 PASS` (recovery_test); software timing only |
| 2026-11-20 | S4-U43 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U43 PASS` (recovery_test) |
| 2026-11-20 | S4-U44 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s4.txt` | `[TEST] S4-U44 PASS` (recovery_test); simulated bus |
| 2026-11-19 | S4-U45 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t7_usb_no_imu_broker_restart.txt`, `../system/logs/2026-11-19_t7_mqtt_idle.txt` | Robot disarmed and still on the bench, t=23.5-219.9 s (lines 73-3257). Surface not checked with a level; no 180° turn. Dashboard Pitch and Roll (whole degrees) read 0 in all 439 snapshots of the idle capture. `[S4TILT]` single samples: pitch median +21 cdeg (5-95%: -50 to +112), roll median +84 cdeg (+10 to +136); both within ±100 cdeg in 659 of 936 samples. |
| 2026-11-19 | S4-U46 | `7a6c373` | Ron Tan | INCOMPLETE | `logs/2026-11-19_t1_usb_hand_tilt.txt` | Normal image, not S4_ONLY; robot disarmed and turned by hand; 2 of 6 poses. Wheels down, t=429.0-483.8 s (lines 69-959): median ax -36, ay -254, az +916 mg; 261 of 262 samples in range (one bump at line 953, az +1479). Upside down about 1.7 s, not held still for 5 s (t=535.50-537.18 s, lines 1991-2019): az -854 to -1049 mg, but ax +224 to +689 and ay +305 to +395 (over 350 mg in 3 and 4 of 9 samples). Nose up/down and side poses not held (ax +909 mg only in passing, line 1984). a_ok 1 throughout. |
| 2026-11-19 | S4-U47 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t1_mqtt_hand_tilt.txt` | Robot disarmed on the bench. Impact True in 2 snapshots (59.3 s, seq 136; 63.3 s, seq 157), each cleared in the next snapshot; state stayed Disarmed, fault 0. Number and timing of taps not logged. This build's `[S4]` line has no `a_impacts` column. Step 4 (armed on a stand) not run. |
| 2026-11-19 | S4-U49 | `7a6c373` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-19_t7_usb_no_imu_broker_restart.txt` | IMU unplugged before boot. t=0-3.57 s (lines 14-70): `[S4]` init 3 (I/O error), a_ok 0, heartbeat 1 to 356; `[S4TILT]` valid 0; `[CSC02]` flags 7 (175 when healthy). Console lines from 3.8 s to 23.5 s missing (lines 72-73), so the 60 s watch is not shown. IMU then plugged in with the power on, not step 5 (time not logged): at t=23.52 s (line 73) a_ok 1, init 0, a_errors 0, pitch 57 / roll 83 cdeg; heartbeat 2350 and uptime continuous to 403.4 s, no reboot. No `inits` column in this build. Steps 4 and 6 not run; dashboard not captured while the IMU was out. |
| 2026-11-20 | S4-U45 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_flash_8c_boot.txt` | Robot disarmed at rest for 45 s after flashing (t=0.21-45.57 s, lines 28-550). Surface not checked with a level; no 180° turn; dashboard not captured. `[S4TILT]` single samples: pitch median -39 cdeg (5-95%: -139 to +2), roll median +33 cdeg (-58 to +80); both within ±100 cdeg in 189 of 217 samples (largest pitch +698 cdeg, line 451). |

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
