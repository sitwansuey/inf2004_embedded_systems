# Subsystem 3 — test log

**Subsystem:** Buddy 3 IR line following/barcode decoding  ·  **Student ID:** 2500383  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | IR readings scale between white and black | FR8 | auto (integrated TEST=1); auto (host) | `subsystem3_test.c` via `make TEST=1 NET=lwip MQTT=1 -j8` in `build_make` (requires the `subsystem3_test()` hook in `app_main.c`); flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Host: `test_buddy3.c` via `make -C testing/subsystem3 test`. | `[TEST] S3-U01 PASS  pure reflectance endpoint`; host: `S3-U01 normalization PASS` |
| S3-U02 | calibration validates samples and preserves old endpoints | FR8 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U02 calibration PASS` |
| S3-U03 | line position handles valid and invalid readings | FR8 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U03 line positions PASS` |
| S3-U04 | hysteresis and line recovery handle timer wrap | FR8 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U04 hysteresis/loss/rollover PASS` |
| S3-U05 | invalid configuration values are rejected | FR8, FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U05 configuration bounds PASS` |
| S3-U06 | barcodes decode across scale, direction and speed | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U06 barcode scales/speed/direction PASS` |
| S3-U07 | invalid barcode frames produce no letter | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U07 barcode corruption PASS` |
| S3-U08 | barcode capture decodes once until rearmed | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U08 capture/rearm/duplicates PASS` |
| S3-U09 | failed barcode captures stay within buffer limits | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U09 capture failures/capacity PASS` |
| S3-U10 | navigation handoff respects controller hold and override | FR9, VC <-> S3 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U10 navigation/override PASS` |
| S3-U11 | invalid barcodes and sensor faults prevent steering | FR8, FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U11 unmapped/corrupted/fault PASS` |
| S3-U12 | line recovery rejects invalid clock values | FR8, NFR4 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U12 recovery and time PASS` |
| S3-U13 | task lifecycle failures clean up safely | VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U13 lifecycle/disabled PASS` |
| S3-U14 | controller binding and leases reject stale snapshots | FR1, VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U14 controller leases PASS` |
| S3-U15 | safety overrides prevent stale steering publication | NFR1, VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U15 safety publication race PASS` |
| S3-U16 | event handoff rejects stale records and prioritizes faults | VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U16 event handoff/staleness PASS` |
| S3-U17 | full event buffers hold steering safely | VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U17 event queue full PASS` |
| S3-U18 | IR setup rejects invalid pins and modes | FR8 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U18 sensor configuration PASS` |
| S3-U19 | ADC mode is refused without configured pins | FR8 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` (`BOARD_IR_MODE=0`) | `S3-U19 ADC boundary PASS` |
| S3-U20 | ADC scans preserve channel order and timestamps | FR8, NFR4 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U20 ADC pipeline/timestamp/rollover PASS` |
| S3-U21 | ADC deadlines reject missing or late replies | NFR4 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U21 ADC deadline and late completion PASS` |
| S3-U22 | ADC errors return no valid readings | FR8 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U22 ADC unavailable/read error PASS` |
| S3-U23 | invalid ADC data rejects the whole scan | FR8 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U23 ADC invalid/partial readings PASS` |
| S3-U24 | ADC lifecycle cleanup preserves device ownership |  | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U24 ADC lifecycle/ownership PASS` |
| S3-U25 | invalid ADC requests return explicit errors |  | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U25 ADC transport/request validation PASS` |
| S3-U26 | ADC faults and overrides prevent steering restart | FR8, VC <-> S3 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U26 ADC controller integration PASS` |
| S3-U27 | blocked ADC reads do not block sampling | NFR4 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U27 blocked worker isolation/fault PASS` |
| S3-U28 | acquisition errors publish one fault without raw data | FR8 | auto (host) | `test_adc.c` via `make -C testing/subsystem3 test` | `S3-U28 acquisition errors/raw snapshot PASS` |
| S3-U29 | invalid navigation and control enums are rejected | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3-U29 enum bounds PASS` |
| S3-U30 | invalid controller modes leave state unchanged | VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U30 controller enum bounds PASS` |
| S3-U31 | raw IR mode never enables steering | FR8, FR13 | auto (host) | `test_raw.c` via `make -C testing/subsystem3 test` (`test_raw` motors disabled, `test_raw_unlocked` motors enabled) | `S3-U31 raw bring-up: 1 groups, 135 checks, 0 failures` (`test_raw`); `S3-U31 raw bring-up: 1 groups, 2 checks, 0 failures` (`test_raw_unlocked`) |
| S3-U32 | ADC scan cadence handles kernel ticks and rollover | NFR4 | auto (host) | `test_adc_tick.c` via `make -C testing/subsystem3 test` | `S3-U32 kernel tick cadence/rollover PASS` |
| S3-U33 | late ADC replies are dropped and replaced | NFR4 | auto (host) | `test_adc_tick.c` via `make -C testing/subsystem3 test` | `S3-U33 late reply dropped, resubmitted PASS` |
| S3-U34 | conversion deadlines drop only the current scan | NFR4 | auto (host) | `test_adc_tick.c` via `make -C testing/subsystem3 test` | `S3-U34 conversion deadline drops the scan only PASS` |
| S3-U35 | ADC sampling handles jitter and raw-data authority | FR8, NFR4 | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U35 current scan/jitter/rollover/raw authority PASS` |
| S3-U36 | STOP wins while waiting for ADC replies | NFR1 | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U36 STOP during reply wait PASS` |
| S3-U37 | lease expiry during ADC waits removes steering | VC <-> S3 | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U37 lease expires during reply wait PASS` |
| S3-U38 | ADC waits return clock and transport errors |  | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U38 clock/delivery/transport failure PASS` |
| S3-U39 | missing ADC replies have a bounded wait | NFR4 | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U39 absent reply bounded PASS` |
| S3-U40 | robot IR layout keeps barcode input separate | FR8, FR9 | auto (host) | `test_robot_ir_layout.c` via `make -C testing/subsystem3 test` | `S3-U40 confirmed layout: 15 checks, 0 failures` |
| S3-U41 | measured calibration classifies recorded IR samples | FR8 (A3) | auto (host) | `test_robot_ir_calibration.c` via `make -C testing/subsystem3 test` (`BOARD_IR_MODE` 2, default 3, 0) | `S3-U41 measured calibration: 67 checks, 0 failures` (mode 2); `S3-U41 measured calibration: 9 checks, 0 failures` (default 3); `S3-U41 measured calibration: 2 checks, 0 failures` (mode 0) |
| S3-U42 | paused sample gaps preserve active fault state | NFR4 | auto (host) | `test_paused_time.c` via `make -C testing/subsystem3 test` | `S3-U42 paused timing: 99 checks, 0 failures` |
| S3-U43 | paused calibrated scans handle jitter and rollover | NFR4 | auto (host) | `test_adc_acquire.c` via `make -C testing/subsystem3 test` | `S3-U43 configured paused jitter/rollover PASS` |
| S3-U44 | cyclic sampling avoids bursts and handler ADC calls | NFR4 | auto (host) | `test_sample_schedule.c` via `make -C testing/subsystem3 test` | `S3-U44 cyclic/no burst/no ADC callback PASS` |
| S3-U45 | cyclic handler failures clean up safely |  | auto (host) | `test_sample_schedule.c` via `make -C testing/subsystem3 test` | `S3-U45 cyclic lifecycle cleanup PASS` |
| S3-U46 | publication waits respect freshness and control authority | NFR4, VC <-> S3 | auto (host) | `test_sample_schedule.c` via `make -C testing/subsystem3 test` | `S3-U46 publication wait freshness/STOP/lease/clock PASS` |
| S3-U47 | controller barcode maps validate and apply changes | FR9, FR12 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U47 controller barcode map PASS` |
| S3-U48 | decoded turns survive later latches and publication delays | FR9, VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U48 decoded turn survives a later latch (L1/L6) PASS` |
| S3-U49 | requests use the sampler gap limit | NFR4 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U49 request gap limit aligned (L6) PASS` |
| S3-U50 | controller requests handle bounded lock contention | VC <-> S3 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U50 controller tolerates a busy lock (L7) PASS` |
| S3-U51 | partial and fault-cut barcode reads are distinguished | FR9 | auto (host) | `test_adapter.c` via `make -C testing/subsystem3 test` | `S3-U51 partial and fault-cut barcode reads (L2) PASS` |
| S3-U52 | ADC read overruns stay latched until reboot | NFR4 | auto (host) | `test_adc_tick.c` via `make -C testing/subsystem3 test` | `S3-U52 read overrun still latched PASS` |
| S3-U53 | ADC worker continues after a dropped scan | NFR4 | auto (host) | `test_adc_tick.c` via `make -C testing/subsystem3 test` | `S3-U53 worker loop survives a dropped scan PASS` |
| S3-U54 | four barcode commands produce navigation events | FR9 | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 test` | `S3 four-command navigation handoff PASS` |
| S3-U55 | barcode decoding tolerates valid edge displacement | FR9 | auto (host) | `test_barcode_edge_bias.c` via `make -C testing/subsystem3 test` (`BOARD_IR_MODE=2`) | `S3 barcode edge-bias regressions: 11214 checks, 0 failures` |
| S3-U56 | barcode resynchronisation handles leading noise | FR9 | auto (host) | `test_barcode_resync.c` via `make -C testing/subsystem3 test` | `Barcode resync: 468 A-Z/direction/prefix cases; quiet/timeout/wrap/budget/malformed/held/rearm and measured-F prefix PASS` |
| S3-U57 | barcode completion respects framing and timeout gates | FR9 | auto (host) | `test_barcode_completion.c` via `make -C testing/subsystem3 test` | `Completion: 210 A-Z/direction/scale/wrap and recorded-A cases;380ms vs760ms; strict/invalid/partial/resync/abort/timeout gates PASS` |
| S3-U58 | digital barcode capture waits for end quiet | FR9 | auto (host) | `test_barcode_end_quiet.c` via `make -C testing/subsystem3 test` | `S3 GP28 end-quiet barcode: 169 checks, 0 failures` |
| S3-U59 | barcode interruptions produce the correct read status | FR9 | auto (host) | `test_barcode_fault_fail.c` via `make -C testing/subsystem3 test` | `S3 L2 fault-cut and partial barcode reads: 44 checks, 0 failures` |
| S3-U60 | reference captures preserve navigation and fault handling | FR9 | auto (host) | `test_barcode_import.c` via `make -C testing/subsystem3 test` | `PASS: reference capture (both directions/noise prefix/junction) preserves main A-D navigation; unmapped E stays unmapped; retry and sensor-fault behavior verified.` |
| S3-U61 | ADC line sampling works with digital barcodes | FR8, FR9 | auto (host) | `test_mixed_barcode.c` via `make -C testing/subsystem3 test` (`BOARD_IR_MODE=3`) | `ADC line + digital GP28 barcode PASS`; `exit code 0` |
| S3-B01 | subsystem sources parse with real kernel headers |  | auto (host) | `subsystem3.c`, `subsystem3_test.c` via `make -C testing/subsystem3 check-headers` | `Checking real kernel headers, SMP=0`; `Checking real kernel headers, SMP=1`; `exit code 0` |
| S3-B02 | firmware builds with both console configurations |  | auto (host) | `subsystem3.c` via `make -C build_make -j8`, then `make -C build_make CONSOLE=uart WIFI=none -j8` | Both commands: `exit code 0`, `.uf2` written. |
| S3-B03 | subsystem sources compile for Cortex-M0+ in both SMP modes |  | auto (host) | `subsystem3.c`, `subsystem3_test.c` via `make -C testing/subsystem3 arm-check` | `PASS: Buddy 3 ARM object compilation SMP=0`; `PASS: Buddy 3 ARM object compilation SMP=1` |
| S3-B04 | host tests pass with static analysis enabled |  | auto (host) | `test_buddy3.c` and host suite via `make -C testing/subsystem3 analyze` | `exit code 0`; no analyzer warnings, 0 failures. |
| S3-CI01 | a failed assertion makes the test target fail |  | auto (host) | `test_buddy3.c` via `make -C testing/subsystem3 verify-failure` | `PASS: test target propagated a real failed assertion` |

The rows were added on 2026-11-20 for the checkpoint 8c code (15aebfc). The older definition tables further down ("Buddy 3 implementation test definitions", "Phase 2 definitions and final verification") are kept unchanged and use the old IDs.

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

*Method:* **auto (integrated TEST=1)** = a test runs in the integrated test image; **auto (host)** = a test runs on a PC without the robot.

**auto (host)** = a host program run by `make -C testing/subsystem3 test` from the repo root (gcc).
On Windows, run it from PowerShell with `C:\msys64\ucrt64\bin;C:\msys64\usr\bin` at the front of PATH.
Each program prints PASS lines or "0 failures". Any failure makes `make` stop with an error.
Also check that the output has no FAIL line. *N* is the printed check count; it changes as tests are added.
On-target **auto** rows name their build profile. S3 has only the integrated `make TEST=1 NET=lwip MQTT=1` profile.

<details>
<summary>ID changes on 2026-11-21 (old ID → new ID)</summary>

On 2026-11-21 the IDs were renumbered so they run in order with no gaps. The test cases, the result rows' *Test ID* column and the test programs use the new IDs. The saved logs in `logs/`, output quoted in the *Notes* column of older rows and the October records further down show the old IDs.

| Old ID | New ID |
|---|---|
| S3-U02 | S3-U01 |
| S3-U04 | S3-U02 |
| S3-U05 | S3-U03 |
| S3-U06 | S3-U04 |
| S3-U07 | S3-U05 |
| S3-U08 | S3-U06 |
| S3-U09 | S3-U07 |
| S3-U10 | S3-U08 |
| S3-U11 | S3-U09 |
| S3-U12 | S3-U10 |
| S3-U13 | S3-U11 |
| S3-U14 | S3-U12 |
| S3-U15 | S3-U13 |
| S3-U16 | S3-U14 |
| S3-U17 | S3-U15 |
| S3-U18 | S3-U16 |
| S3-U19 | S3-U17 |
| S3-U20 | S3-U18 |
| S3-U21 | S3-U19 |
| S3-U22 | S3-U20 |
| S3-U23 | S3-U21 |
| S3-U24 | S3-U22 |
| S3-U25 | S3-U23 |
| S3-U26 | S3-U24 |
| S3-U27 | S3-U25 |
| S3-U28 | S3-U26 |
| S3-U29 | S3-U27 |
| S3-U30 | S3-U28 |
| S3-U31 | S3-U29 |
| S3-U32 | S3-U30 |
| S3-U33 | S3-U31 |
| S3-U34 | S3-U32 |
| S3-U35 | S3-U33 |
| S3-U36 | S3-U34 |
| S3-U37 | S3-U35 |
| S3-U38 | S3-U36 |
| S3-U39 | S3-U37 |
| S3-U40 | S3-U38 |
| S3-U41 | S3-U39 |
| S3-U42 | S3-U40 |
| S3-U43 | S3-U41 |
| S3-U44 | S3-U42 |
| S3-U45 | S3-U43 |
| S3-U48 | S3-U44 |
| S3-U49 | S3-U45 |
| S3-U50 | S3-U46 |
| S3-U51 | S3-U47 |
| S3-U52 | S3-U48 |
| S3-U53 | S3-U49 |
| S3-U54 | S3-U50 |
| S3-U55 | S3-U51 |
| S3-U56 | S3-U52 |
| S3-U57 | S3-U53 |
| S3-U58 | S3-U54 |
| S3-U59 | S3-U55 |
| S3-U60 | S3-U56 |
| S3-U61 | S3-U57 |
| S3-U62 | S3-U58 |
| S3-U63 | S3-U59 |
| S3-U64 | S3-U60 |
| S3-U65 | S3-U61 |
| S3-B01 | S3-B01 (unchanged) |
| S3-B02 | S3-B02 (unchanged) |
| S3-B03 | S3-B03 (unchanged) |
| S3-B04 | S3-B04 (unchanged) |
| S3-CI01 | S3-CI01 (unchanged) |
| S3-M12 | S3-M01 |
| S3-M13 | S3-M02 |
| S3-M14 | S3-M03 |
| S3-M15 | S3-M04 |

Removed before the renumbering (old IDs; a new ID with the same number is a different test): S3-U01 and S3-M01 to S3-M11 (never run) and S3-U03 (default configuration check, removed from `subsystem3_test.c`), all removed on 2026-11-20. Retired earlier: S3-U46 and S3-U47 (barcode capture checks, `test_barcode_capture.c` removed in 9c6b8bc), S3-I01 (S3 ran the S2 host tests) and S3-I02 (October host probe for missing shared headers).

</details>

<details>
<summary>ID changes on 2026-11-25 (old ID → new ID)</summary>

On 2026-11-25 every test whose latest result was INCOMPLETE was removed together with its result rows (its saved logs stay in `logs/`). Tests filmed for the video set were narrowed to what their video shows. The remaining IDs were renumbered so they run in order with no gaps; Notes cells and saved logs keep the IDs they had when written.

Removed on 2026-11-25 (latest result INCOMPLETE; a new ID with the same number is a different test): S3-M01, S3-M02, S3-M03, S3-M04.

</details>

## Buddy 3 implementation test definitions

S3-U01 above belongs to the retired sample statistics consumer. Its template
row is retained and is not execution evidence. New identifiers are not reused.
These definitions precede the final committed verification run.

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S3-U02 | Normalization, clamping, inverse polarity and division/range guards | Native; on-target smoke also provided | Correct darkness or explicit failure |
| S3-U03 | Default config has no navigation mapping | On-target smoke; native configuration/mapping groups also cover behavior | No fabricated A/Z command |
| S3-U04 | Labelled calibration, contrast, variance, sample capacity | Native | Stable valid endpoints only; failed finish unchanged |
| S3-U05 | Left/centre/right error, all-black/split junction, invalid mask/raw | Native | Correct position or invalid/ambiguous state |
| S3-U06 | Hysteresis, continuous loss debounce, recovery deadline and timer wrap | Native | No chatter; precise bounded transitions |
| S3-U07 | Geometry overlap, bad thresholds, enum/range and interval guards | Native | Invalid configurations rejected |
| S3-U08 | A/Z PDF fixtures, scales, direction and bounded speed change | Native | Known symbols decoded at all valid scales |
| S3-U09 | Partial, unknown, corrupt framing/gaps, zero/large widths | Native | No valid symbol on bad input |
| S3-U10 | Capture at two scales/speeds, rollover, duplicate latch/rearm | Native | One decode per physically cleared marker |
| S3-U11 | Noise, timeout, partial data, 29-run capacity and interrupted capture | Native | Explicit bounded error; no overrun |
| S3-U12 | Navigation once, steering handoff, no competing recovery, override/resume | Native | One command, hold until explicit resume |
| S3-U13 | Unmapped symbol, corrupt barcode and invalid sensor | Native | No turn; fault suppresses steering |
| S3-U14 | Core recovery/stop events and duplicate/backward/stale clocks | Native | One bounded recovery/stop; explicit error |
| S3-U15 | Kernel resource failures, task start and default-disabled acquisition | Native actual adapter with boundary fakes | Cleanup/retry; zero hardware access |
| S3-U16 | Controller binding, lease renew/expiry and stale snapshots | Native boundary | Only current controller can authorize tracking |
| S3-U17 | Safety directive during publication and mutex contention | Native boundary | Override wins; no stale steering authority |
| S3-U18 | Copied event handoff, generation/age rejection and fault preemption | Native boundary | No duplicate or obsolete turn |
| S3-U19 | Actual four-record event buffer full | Native boundary | Transport fault; steering held |
| S3-U20 | Reserved/duplicate pins and GPIO error | Native boundary | Configuration/read errors surfaced |
| S3-U21 | Unsafe kernel ADC adapter contract | Native boundary | E_NOSPT and no pad access, never a fake read |
| S3-CI01 | Failing assertion propagation | Same Make test target with --force-failure | Inner test returns nonzero, outer verification succeeds |
| S3-B01 | Real kernel header compatibility in SMP=0 and SMP=1 | Host syntax check | S3 files parse; not an Arm build |
| S3-B02 | Default and UART/no-radio firmware builds | Actual project Makefile | Real build result recorded, never substituted |

Manual tests S3-M01 through S3-M11 are defined in HARDWARE_VALIDATION.md. All are
NOT RUN. On-target S3-U02/U03 smoke checks also remain NOT RUN on hardware.
The native CI target runs its tests unconditionally and contains no skip path.

## Results

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S3-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-11-20 | S3-U01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U02 normalization PASS`; same in the `analyze` rerun. Host part only; the on-target TEST=1 part was not run |
| 2026-11-20 | S3-U02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U04 calibration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U05 line positions PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U06 hysteresis/loss/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U05 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U07 configuration bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U06 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U08 barcode scales/speed/direction PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U07 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U09 barcode corruption PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U08 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U10 capture/rearm/duplicates PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U09 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U11 capture failures/capacity PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U10 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U12 navigation/override PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U11 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U13 unmapped/corrupted/fault PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U12 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U14 recovery and time PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U13 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U15 lifecycle/disabled PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U14 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U16 controller leases PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U17 safety publication race PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U18 event handoff/staleness PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U19 event queue full PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U20 sensor configuration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U21 ADC boundary PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U22 ADC pipeline/timestamp/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U23 ADC deadline and late completion PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U24 ADC unavailable/read error PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U25 ADC invalid/partial readings PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U24 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U26 ADC lifecycle/ownership PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U25 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U27 ADC transport/request validation PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U26 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U28 ADC controller integration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U27 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U29 blocked worker isolation/fault PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U28 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U30 acquisition errors/raw snapshot PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U29 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U31 enum bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U30 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U32 controller enum bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U31 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, both builds: `test_raw` (motors disabled) 1 group, 135 checks, 0 failures; `test_raw_unlocked` (motors enabled) 1 group, 2 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-U32 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U34 kernel tick cadence/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U33 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U35 late reply dropped, resubmitted PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U34 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U36 conversion deadline drops the scan only PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U35 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U37 current scan/jitter/rollover/raw authority PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U36 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U38 STOP during reply wait PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U37 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U39 lease expires during reply wait PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U38 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U40 clock/delivery/transport failure PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U39 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U41 absent reply bounded PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U40 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_robot_ir_layout`: `S3-U42 confirmed layout: 15 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U41 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, all three builds: `BOARD_IR_MODE=2` 67 checks, default 9 checks, `BOARD_IR_MODE=0` 2 checks, 0 failures each; same in the `analyze` rerun |
| 2026-11-20 | S3-U42 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_paused_time`: `S3-U44 paused timing: 99 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U43 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U45 configured paused jitter/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U44 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U48 cyclic/no burst/no ADC callback PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U45 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U49 cyclic lifecycle cleanup PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U46 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U50 publication wait freshness/STOP/lease/clock PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U47 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U51 controller barcode map PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U48 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U52 decoded turn survives a later latch (L1/L6) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U49 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U53 request gap limit aligned (L6) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U50 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U54 controller tolerates a busy lock (L7) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U51 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U55 partial and fault-cut barcode reads (L2) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U52 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U56 read overrun still latched PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U53 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U57 worker loop survives a dropped scan PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U54 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3 four-command navigation handoff PASS`; same in the `analyze` rerun (group prints no ID) |
| 2026-11-20 | S3-U55 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_edge_bias` built with `BOARD_IR_MODE=2`: 11214 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-U56 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_resync`: 468 A-Z/direction/prefix cases plus the quiet/timeout/wrap/budget/malformed/held/rearm and measured-F prefix cases, PASS; same in the `analyze` rerun |
| 2026-11-20 | S3-U57 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_completion`: 210 A-Z/direction/scale/wrap and recorded-A cases plus the strict/invalid/partial/resync/abort/timeout gates, PASS; same in the `analyze` rerun |
| 2026-11-20 | S3-U58 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_end_quiet`: `S3 GP28 end-quiet barcode: 169 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U59 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_fault_fail`: `S3 L2 fault-cut and partial barcode reads: 44 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U60 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_import`: `PASS: reference capture (both directions/noise prefix/junction) ...`, unmapped E stays unmapped, retry and sensor fault verified; same in the `analyze` rerun |
| 2026-11-20 | S3-U61 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_mixed_barcode` built with `BOARD_IR_MODE=3`: `ADC line + digital GP28 barcode PASS`, 106 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-B01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make check-headers`: `Checking real kernel headers, SMP=0` and `SMP=1`, no error, exit 0 |
| 2026-11-20 | S3-B03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make arm-check` with arm-none-eabi-gcc 14.2.1 (Arm GNU Toolchain 14.2.Rel1): `PASS: Buddy 3 ARM object compilation SMP=0` and `SMP=1`, exit 0; objects only, no link |
| 2026-11-20 | S3-B04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make analyze`: every host program rebuilt with `-fanalyzer -O1` and `-Werror`, no analyzer warning; the same PASS lines and check counts as `test`, 0 failures, exit 0 |
| 2026-11-20 | S3-CI01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make verify-failure`: the forced assertion (`test_buddy3.c:601`, `2 + 2 == 5`) gave 1 failure and `make[1]` Error 1, then `PASS: test target propagated a real failed assertion`, exit 0. The one failed check is the intended forced one |
| 2026-11-25 | S3-B02 | `ac5485d` | Ron Tan | PASS | `logs/2026-11-25_firmware_default.txt`, `logs/2026-11-25_firmware_uart.txt` | Clean export of checkpoint 8g with the local Wi-Fi secrets file. `make -C build_make -j8`: exit 0, 0 errors, 0 warnings, wrote `mtk3pico_smp1_usb_cdc_wifi_enc.uf2`. `make -C build_make CONSOLE=uart WIFI=none -j8`: exit 0, 0 errors, 0 warnings, wrote `mtk3pico_smp1_uart_enc.uf2`. |

## Verified implementation results

Compiler: GCC 15.2.0 (MSYS2 UCRT64), GNU Make 4.4.1. Final source/test commit:
93f71ae0effb76f3a535e948e8c1e098934c7ee8. Results below refer to that exact commit;
this later records-only commit does not change code or the workflow.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | S3-U02, S3-U04 through S3-U21 | 93f71ae | Codex/native GCC | PASS | [native](logs/2026-10-15_native.txt) | 19 groups: 529 algorithm checks plus 15,273 adapter/boundary assertions, 0 failures, 0 compiler warnings; no skipped native tests |
| 2026-10-15 | S3-U02, S3-U04 through S3-U21 with GCC analyzer | 93f71ae | Codex/native GCC | PASS | [static analysis](logs/2026-10-15_static_analysis.txt) | -fanalyzer -Wshadow -Wconversion; 0 diagnostics; boundary assertion count includes repeated fake RTOS invariants, not 15,273 independent test cases |
| 2026-10-15 | S3-CI01 | 93f71ae | Codex/native GCC | PASS | [failure propagation](logs/2026-10-15_failure_propagation.txt), [direct failing target](logs/2026-10-15_negative_probe.txt) | Real false assertion: binary exits 1, Make test exits 2. The deliberate failure is expected and verified; normal test run above executes both binaries |
| 2026-10-15 | S3-B01 | 93f71ae | Codex/host syntax | PASS | [real headers](logs/2026-10-15_headers.txt) | SMP=0/1, TEST=1; suppresses host/target pointer-size casts only; no Arm ABI/link verification |
| 2026-10-15 | S3-B02 default firmware | 93f71ae | Codex/project Makefile | BLOCKED | [default build](logs/2026-10-15_firmware_default.txt) | Real exit 2 at pico_rp2040.mk:130: arm-none-eabi-gcc absent |
| 2026-10-15 | S3-B02 UART/no-radio firmware | 93f71ae | Codex/project Makefile | BLOCKED | [UART build](logs/2026-10-15_firmware_uart.txt) | Real exit 2, same missing compiler. SDK not installed; independent source blockers documented in AUDIT.md |
| 2026-10-15 | Whitespace | 93f71ae | Codex/Git | PASS | [diff check](logs/2026-10-15_whitespace.txt) | git diff --check clean |
| 2026-10-15 | GitHub Actions / PR | 93f71ae | GitHub connector | NOT RUN / BLOCKED | [publication](logs/2026-10-15_publication.txt) | Connector cannot create remote branch (403); no Actions run or PR link exists |
| 2026-10-15 | On-target smoke S3-U02/U03 and S3-M01 through S3-M11 | 93f71ae | No hardware available | NOT RUN | HARDWARE_VALIDATION.md | No flashing, ADC read, waveform capture or physical robot verification |

Exact root-relative commands (local PATH was prefixed with MSYS2 ucrt64/bin and
usr/bin; no provisioning or fake compiler wrapper was used):

    make -B -C testing/subsystem3 test
    make -C testing/subsystem3 verify-failure
    make -C testing/subsystem3 test TEST_ARGS=--force-failure  # expected exit 2
    make -C testing/subsystem3 check-headers
    make -B -C testing/subsystem3 test CFLAGS="-std=c11 -Wall -Wextra -Werror -pedantic -Wshadow -Wconversion -fanalyzer -O1"
    make -C build_make -j2
    make -C build_make CONSOLE=uart WIFI=none -j2
    git diff --check

The -B option forces a complete native rebuild for recorded compiler evidence.
Without -B, the documented/CI test command still executes both test binaries.

Saved console logs normalize CRLF to LF and remove trailing whitespace only.


## Phase 2 definitions and final verification

Code commit: e0a44481e0d9034adcd53233cbfda0be215f7cae (following f5461c1).
The subsequent evidence commit changes documentation/logs and removes a trailing
blank line from the host fixture only. All original 19 native groups remain.
S3-U21 still checks that the actual unconfigured ADC pad defaults fail safely;
its old E_NOSPT gate is now supplemented by the pad-enabled worker tests below.

| ID | What is tested | Method | Expected result |
|---|---|---|---|
| S3-U22 | ADC three-channel permutation, pipelining, oldest timestamp and wrap | Actual client/worker with native device fakes | Complete correctly ordered scan; one request outstanding |
| S3-U23 | Deadline edge, no response, late 1000 ms read and rollover | Native fake time | Invalid/held at deadline; no retry or stale success |
| S3-U24 | Unavailable exclusive device and returned read error | Native boundary | Original error; zero valid readings |
| S3-U25 | Out-of-range, short/oversized read, invalid mask/time/sequence | Native boundary | Entire scan rejected; terminal hold |
| S3-U26 | MBF/task allocation/start failures and sole-owner access | Native boundary | Cleanup/retry before activation; no unauthorized ADC calls |
| S3-U27 | Bad channel/request, malformed/full MBFs and clock error | Native boundary | Explicit errors without blocking or unsafe reads |
| S3-U28 | ADC tracking/status/raw data, override and failed-ADC resume | Actual subsystem service with native boundaries | Fresh tracking only; override/terminal fault cannot revive steering |
| S3-U29 | Sampler runs while a fake legacy read remains blocked | Deterministic interleaving | Deadline fault event delivered before worker returns; no client device call |
| S3-U30 | Unavailable/corrupt/short acquisition faults and raw snapshot | Actual subsystem service | Correct acquisition ER, one fault, zero invalid raw values |
| S3-B03 | Real Cortex-M0+ compilation of all Buddy 3 sources, SMP=0/1 | ARM compiler object check | Real ARM objects; separately labelled from full firmware link |
| S3-I01 | Existing Buddy 2 host tests, all six modes | Unchanged S2 source/fixtures | Existing tests pass without S2 edits |
| S3-I02 | Existing shared-source dependency failures | Host syntax-only diagnostic probe | Expose missing board pins and S2/S4 stale declarations; never called an ARM build |

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | All 28 native groups | e0a4448 | Codex/GCC 15.2.0 | PASS | [native](logs/2026-10-15_phase2_native.txt) | 529 algorithm + 15,261 adapter + 1,012 ADC assertions = 16,802; zero failures/warnings; all three binaries execute |
| 2026-10-15 | All 28 groups, SMP=0 boundary build | e0a4448 | Codex/native GCC | PASS | [SMP=0](logs/2026-10-15_phase2_native_smp0.txt) | 529 + 15,253 + 973 assertions; no errors/warnings; affinity-specific fake invariants omitted only because this is the real single-core preprocessor profile |
| 2026-10-15 | GCC static analysis and full native suite | e0a4448 | Codex/-fanalyzer | PASS | [analysis](logs/2026-10-15_phase2_static_analysis.txt) | Strict -Werror/-Wshadow/-Wconversion; zero diagnostics; all 28 groups execute |
| 2026-10-15 | S3-B01 | e0a4448 | Codex/real kernel header parsing | PASS | [headers](logs/2026-10-15_phase2_headers.txt) | SMP=0/1 and TEST=1, including the new ADC worker; host parsing only |
| 2026-10-15 | S3-CI01 | e0a4448 | Codex/actual test target | PASS | [verification](logs/2026-10-15_phase2_failure_propagation.txt), [direct negative](logs/2026-10-15_phase2_negative_probe.txt) | False assertion makes binary exit 1 and Make exit 2; verification target correctly recognizes failure |
| 2026-10-15 | S3-I01 | e0a4448 | Codex/existing S2 tests | PASS | [six modes](logs/2026-10-15_phase2_buddy2_regression.txt) | run.ps1 was execution-policy blocked; compiled/ran its unchanged fixtures and six exact flag sets directly with GCC; all six pass, zero compiler diagnostics |
| 2026-10-15 | S3-I02 | e0a4448 | Codex/host dependency probe | BLOCKED (expected diagnostic) | [probe](logs/2026-10-15_phase2_integration_probe.txt) | Missing production board_pins.h; removed SAMPLE_MSG/sem_slots/mpf_samples/S2_PERIOD_MS; not an ARM firmware result |
| 2026-10-15 | Toolchain provisioning | e0a4448 | Codex/environment | BLOCKED | [provisioning](logs/2026-10-15_phase2_provisioning.txt) | ARM GCC/binutils/SDK absent; official download networking blocked; WSL enumeration denied; no installer completed |
| 2026-10-15 | S3-B03 | e0a4448 | Codex/actual arm-check command | BLOCKED | [ARM objects](logs/2026-10-15_phase2_arm_objects.txt) | Compiler executable absent; Make exit 2, no ARM object produced |
| 2026-10-15 | S3-B02 default firmware | e0a4448 | Codex/project Makefile | BLOCKED | [default](logs/2026-10-15_phase2_firmware_default.txt) | Actual exit 2 at missing Arm GCC guard; no successful compile/link/UF2 |
| 2026-10-15 | S3-B02 UART/no-radio firmware | e0a4448 | Codex/project Makefile | BLOCKED | [UART](logs/2026-10-15_phase2_firmware_uart.txt) | Actual exit 2 at same prerequisite; not a substituted SDK/stub build |
| 2026-10-15 | Ownership and whitespace | Phase 2 tree | Codex/Git | PASS | [scope](logs/2026-10-15_phase2_ownership.txt) | Other buddies, kernel/device/config/lib and shared app code unchanged; only empty ADC ownership row claimed in TEAM.md |
| 2026-10-15 | GitHub upload, PR, CI, merge | Phase 2 local commits | GitHub API | BLOCKED / NOT RUN | [publication](logs/2026-10-15_phase2_publication.txt) | Git tree and PR creation HTTP 403 despite account Write/Admin; no successful refresh action exposed; remote branch still e292334, no PR exists |
| 2026-10-15 | S3-M01..M11 and on-target smoke | Phase 2 | No hardware | NOT RUN | [bench protocol](HARDWARE_VALIDATION.md) | No physical measurements, flashing or vehicle movement |

Exact root-relative commands:

~~~sh
make -B -C testing/subsystem3 test CPPFLAGS="-I../../app_program/subsystem3 -DTK_SUPPORT_SMP=0"
make -C testing/subsystem3 analyze
make -B -C testing/subsystem3 test
make -C testing/subsystem3 check-headers
make -C testing/subsystem3 verify-failure
make -C testing/subsystem3 test TEST_ARGS=--force-failure  # expected Make exit 2
make -C testing/subsystem3 arm-check
make -C build_make -j2
make -C build_make CONSOLE=uart WIFI=none -j2
git diff --exit-code e292334 -- app_program/subsystem1 app_program/subsystem2 app_program/subsystem4 app_program/subsystem5 app_program/app.h app_program/app_main.c device kernel config lib
git diff --check e292334
~~~

The S2 log contains exact GCC argument arrays for its six existing modes. No
host board fixture was copied into production app_program/board_pins.h. Native
assertion totals include repeated fake-kernel invariants, not independent cases.
The CI firmware job installs real Arm GCC/newlib and SDK dependencies, compiles
Buddy 3 ARM objects and attempts the complete standard Make build; it does not
hide missing dependencies or ignore failures. It has not executed remotely yet.
Main remains unchanged. Firmware success, required CI, controller/board integration
and team approvals remain merge gates. There is no merge commit or main CI result.

## 2026-10-18: ARM controller enum portability correction

Baseline: f1fbb024de5d6b62a0f6d3b444e04255e09b860f; PR #4 on
feat/buddy3-line-barcode-ci. Working tree was clean, fetch found no remote
changes, and origin/main (774a24c3c31fbc65a2bd3e7bac784dd379f13bb7) is an
ancestor of this branch. No other buddy's source or shared configuration changed.

Root cause: ARM GCC selects an unsigned representation for s3_mode_t, so
s3_controller_control's mode < S3_MODE_PAUSED (zero) is always false and fails
-Werror=type-limits. Reproduced locally with the real Cortex-M0+ target before
repair. Use the same unsigned upper-bound comparison already present in
buddy3_core.c and drv_ir.c; preserve all lease, marker-clear and caller checks.
Audited subsystem3 sources, tests and buddy3-ci.yml for related enum/signedness
issues; no other confirmed portability correction or workflow change was needed.
No warnings were suppressed and -Werror remains enabled.

New S3-U31 checks reject -1, the first value above the maximum, and 99 in both
navigation mappings and core control; invalid control leaves the core unchanged.
S3-U32 checks the same controller mode boundaries with zero/nonzero leases,
preserves published status/control state on rejection, checks validation before
clock access, and retains successful PAUSED/TRACK/OVERRIDE transitions.
Existing S3-U20 retains negative and oversized IR-mode regression coverage.

Executed through Windows PowerShell using the already installed Ubuntu WSL2
GCC 15.2.0 and arm-none-eabi-gcc 14.2.1. No tools were installed.

| Command (repository root) | Actual result |
|---|---|
| make -B -C testing/subsystem3 test | PASS: 30 groups, 16,887 checks, zero failures (551 algorithm + 15,324 adapter + 1,012 ADC) |
| make -B -C testing/subsystem3 analyze | PASS: same tests under GCC -fanalyzer, zero diagnostics |
| make -B -C testing/subsystem3 check-headers | PASS: actual kernel headers, SMP=0 and SMP=1 |
| make -B -C testing/subsystem3 verify-failure | PASS: deliberate false assertion fails the nested test target and is correctly detected |
| make -B -C testing/subsystem3 arm-check | PASS: all Buddy 3 ARM objects, SMP=0 and SMP=1; this is not a complete firmware link |
| make -k -C build_make -j2 | BLOCKED, exit 2: pico_rp2040.mk:175, PICO_SDK_PATH is not set |
| make -k -C build_make CONSOLE=uart WIFI=none -j2 | BLOCKED, exit 2: subsystem4/sdk.mk:5 also requires Pico SDK 2.2.0 |
| git diff --check | PASS |

CI after publication must independently establish the full firmware result
using its provisioned SDK. Existing ADC hardware, board-pin, Vehicle Controller
and affected-owner review gates remain subject to verification; no sensor/robot
hardware test or successful firmware link is claimed by these local results.