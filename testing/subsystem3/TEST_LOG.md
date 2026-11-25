# Subsystem 3 — test log

**Subsystem:** Buddy 3 IR line following/barcode decoding  ·  **Student ID:** 2500383  ·  **Code:** `app_program/subsystem3/`

## Test cases

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
| S3-U66 | IR sampler is configured and keeps publishing fresh scans with acquisition `E_OK` | FR8; IF-08 (`s3_status_get()`) | auto (integrated TEST=1) | `subsystem3_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S3-U66 PASS  IR sampler configured and publishing fresh scans` |
| S3-U67 | latest IR scan has a non-zero valid mask and every raw channel within 0..4095 | FR8; IF-08 (`s3_status_get()`) | auto (integrated TEST=1) | `subsystem3_test.c` via `make -C build_make TEST=1 NET=lwip MQTT=1 -j8`; flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`. Power up DISARMED, at rest, with every sensor connected; save the USB console from `[TEST] === test run start ===` to `[TEST] === test run end ===`. | `[TEST] S3-U67 PASS  IR raw readings within the ADC range` |

The rows were added on 2026-11-20 for the checkpoint 8c code (15aebfc). The older definition tables further down ("Buddy 3 implementation test definitions", "Phase 2 definitions and final verification") are kept unchanged and use the old IDs.

The IDs were renumbered on 2026-11-21 and again on 2026-11-25 so they run in order with no gaps. The old IDs are listed under *ID changes* below.

*Method:* **auto (integrated TEST=1)** = a test runs in the integrated test image; **auto (host)** = a test runs on a PC without the robot.

**auto (host)** = a host program run by `make -C testing/subsystem3 test` from the repo root (gcc).
On Windows, run it from PowerShell with `C:\msys64\ucrt64\bin;C:\msys64\usr\bin` at the front of PATH.
Each program prints PASS lines or "0 failures". Any failure makes `make` stop with an error.
Also check that the output has no FAIL line. *N* is the printed check count; it changes as tests are added.
On-target **auto** rows name their build profile. S3 has only the integrated `make TEST=1 NET=lwip MQTT=1` profile.

S3-U66 and S3-U67 (added 2026-11-25) run after S3-U01 in the same `subsystem3_test()`. They read S3's published status only. They take new numbers because S3-U02 to S3-U65 have all been used. The project member reported these checks complete on 2026-11-25. [Latest completion status](../README.md#completion-of-pico-w-and-sanitizer-checks).

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

The template previously used S3-U01 for the retired sample statistics
consumer. The current S3-U01 above covers IR normalization.

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

The October record lists S3-M01 through S3-M11 and the former S3-U02/U03
on-target smoke checks as NOT RUN for that revision.
The native CI target runs its tests unconditionally and contains no skip path.

## Results

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
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

### Restored result rows (audit, 2026-11-25)

These result rows were deleted from the table above in `5f93cb6` (2026-11-25). They are copied back word for word from `d18dd38`, the last commit that had them; the first two columns are the only additions. Their *Test ID* cells use the IDs in force at `d18dd38`, not today's IDs, and some of those numbers now name different tests (second column, and *ID history (git record)* at the end of this file). Nothing was re-run.

| Restored from | Test ID meaning (then → now) | Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|---|---|
| `d18dd38` (deleted in `5f93cb6`) | S3-M12 at `d18dd38` = IR raw levels on white and black, and the room light; test removed in `5f93cb6`; no current test uses S3-M12 | 2026-11-20 | S3-M12 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run 1, not the defined steps: raw values from the 5 Hz telemetry while line tracking at 20% (415 samples, 119.7-571.1 s; 18 with both line IRs ≥ 3700 left out), not 5 s holds on white paper and on the line under each sensor. Floor: GP27 min 186, median 250; GP26 min 182, median 248 (95% of floor samples ≤ 710 and ≤ 764). On the line with the other sensor on the floor: GP27 up to 3437 (median 3122, 57 samples), GP26 up to 3504 (median 2867, 19 samples). GP28 read 1 in 47 of 178 samples with both line IRs below 1000 (this run crossed barcode cards). Ramp and lux not measured; the floor surface is not logged. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M13 at `d18dd38` = barcode read rate for *A*, *F* and *Z* at 30% demand; test removed in `5f93cb6`; no current test uses S3-M13 | 2026-11-20 | S3-M13 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | 3 card passes, not 20 per letter; demand 20% (START commands c10314…), not 30%; A, F and Z mapped to action 3 (Straight) at 75.4-82.8 s (lines 10108-11666), not action 0. Pass 1: attempt 1 failed, 17/29 bars and spaces (123.47 s, line 17364); fault 7 at 125.21 s while reversing for attempt 2. Pass 2: attempt 1 failed, 13/29 (406.82 s); attempt 2 (reverse) failed, 18/29 (411.22 s); fault 1 Tilt at 413.5 s before attempt 3. Pass 3: decoded A on attempt 1, 29/29 (568.89 s, line 76420), then "A -> Straight". Which card was under the robot is not logged. The telemetry also shows 29 other "Reading..." stretches with no attempt event. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M14 at `d18dd38` = line following on a 1 m straight at 30% demand: touch length and dead …; test removed in `5f93cb6`; no current test uses S3-M14 | 2026-11-20 | S3-M14 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Line tracking on the track at 20% demand, not a 1 m straight at 30%; no video; S, W and (S - W)/2 not measured. 12 tracking stretches, longest 37.7 s (183.3-221.0 s). No fault 2 in the log (fault codes 0, 1, 7 and 8 only). Stretches ended by obstacle acquisition (5), fault 7 (125.2, 263.1 s), fault 8 (374.1, 393.8 s), fault 1 Tilt (413.5, 491.5 s) and STOP (571.1 s). The motor direction shows 34 pivots, the longest in 2 consecutive snapshots (seq 89-90 at 190.6 s; seq 38-39 at 486.0 s). The 300 ms limit needs the video: the telemetry is 5 Hz and loses snapshots. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M15 at `d18dd38` = lift detection inputs from S3: both line IRs near full scale and GP28 …; test removed in `5f93cb6`; no current test uses S3-M15 | 2026-11-20 | S3-M15 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | No disarmed level lift of 50-100 mm held 5 s; lift height not logged. On the line while tracking, with the other sensor on the floor: GP27 up to 3437 (119.73 s), GP26 up to 3504 (123.62 s), below 3700. Both line IRs dark at once (GP27/GP26): 2938/3041 (172.6 s); 3594/3681 with GP28 = 1 at 411.30 s (pitch −1°, reversing for a barcode re-read), 19 below the limit; what was under the robot is not logged. All 18 tracking samples with both line IRs ≥ 3700 had GP28 = 1 (3832-4095). Fault 8 at 393.81 s (line 49555) after both read 3955-4095 with GP28 = 1 from 391.05 s (pitch down to −58°); the other fault 8 (374.09 s) followed a 10 s telemetry gap. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M12 at `d18dd38` = IR raw levels on white and black, and the room light; test removed in `5f93cb6`; no current test uses S3-M12 | 2026-11-20 | S3-M12 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run 3, not the defined steps: raw values from the 5 Hz telemetry while line tracking at 20% (174 samples, 42.5-2749.7 s; 13 with both line IRs ≥ 3700 left out). Floor: GP27 min 183, median 233; GP26 min 187, median 221 (95% of floor samples ≤ 281 and ≤ 333). On the line with the other sensor on the floor: GP27 up to 3583 (median 2975, 9 samples), GP26 up to 3523 (median 3258, 14 samples). GP28 read 0 in 112 of 116 samples with both line IRs below 1000. Ramp and lux not measured; the floor surface is not logged. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M14 at `d18dd38` = line following on a 1 m straight at 30% demand: touch length and dead …; test removed in `5f93cb6`; no current test uses S3-M14 | 2026-11-20 | S3-M14 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Line tracking on the track at 20% demand, not a 1 m straight at 30%; no video; S and W not measured. 11 tracking stretches, longest 17.8 s (2587.7-2605.4 s). No fault 2 in the log (fault codes 0, 1 and 7 only). Stretches ended by obstacle acquisition (2), fault 7 (77.5 s), fault 1 Tilt (1994.8, 2039.0, 2392.0, 2489.2 s) and STOP (4). 21 pivots seen; one PIVOT_LEFT was in 5 consecutive snapshots (seq 214-218, 800 ms from first to last, 2595.7-2598.4 s), 0.7 s after a "Hump cleared" event; whether that part of the track is straight is not logged. |
| `d18dd38` (deleted in `5f93cb6`) | S3-M15 at `d18dd38` = lift detection inputs from S3: both line IRs near full scale and GP28 …; test removed in `5f93cb6`; no current test uses S3-M15 | 2026-11-20 | S3-M15 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | No disarmed level lift of 50-100 mm held 5 s; lift height not logged. On the line while tracking, with the other sensor on the floor: GP27 up to 3583 (2595.00 s, line 108697), GP26 up to 3523 (2593.15 s), below 3700. Both line IRs dark at once (GP27/GP26): 3406/3366 with GP28 = 0 (47.66 s). All 13 tracking samples with both line IRs ≥ 3700 had GP28 = 1 (3737-4095); these stretches ended with STOP (94.0 s) or fault 1 Tilt (1994.8, 2039.0, 2392.0 s); no fault 8 in the log. |

## Verified implementation results

Compiler: GCC 15.2.0 (MSYS2 UCRT64), GNU Make 4.4.1. Final source/test commit:
93f71ae0effb76f3a535e948e8c1e098934c7ee8. Results below refer to that exact commit;
this later records-only commit does not change code or the workflow.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| 2026-10-15 | S3-U02, S3-U04 through S3-U21 | 93f71ae | AI-assisted/native GCC | PASS | [native](logs/2026-10-15_native.txt) | 19 groups: 529 algorithm checks plus 15,273 adapter/boundary assertions, 0 failures, 0 compiler warnings; no skipped native tests |
| 2026-10-15 | S3-U02, S3-U04 through S3-U21 with GCC analyzer | 93f71ae | AI-assisted/native GCC | PASS | [static analysis](logs/2026-10-15_static_analysis.txt) | -fanalyzer -Wshadow -Wconversion; 0 diagnostics; boundary assertion count includes repeated fake RTOS invariants, not 15,273 independent test cases |
| 2026-10-15 | S3-CI01 | 93f71ae | AI-assisted/native GCC | PASS | [failure propagation](logs/2026-10-15_failure_propagation.txt), [direct failing target](logs/2026-10-15_negative_probe.txt) | Real false assertion: binary exits 1, Make test exits 2. The deliberate failure is expected and verified; normal test run above executes both binaries |
| 2026-10-15 | S3-B01 | 93f71ae | AI-assisted/host syntax | PASS | [real headers](logs/2026-10-15_headers.txt) | SMP=0/1, TEST=1; suppresses host/target pointer-size casts only; no Arm ABI/link verification |
| 2026-10-15 | S3-B02 default firmware | 93f71ae | AI-assisted/project Makefile | BLOCKED | [default build](logs/2026-10-15_firmware_default.txt) | Real exit 2 at pico_rp2040.mk:130: arm-none-eabi-gcc absent |
| 2026-10-15 | S3-B02 UART/no-radio firmware | 93f71ae | AI-assisted/project Makefile | BLOCKED | [UART build](logs/2026-10-15_firmware_uart.txt) | Real exit 2, same missing compiler. SDK not installed; independent source blockers documented in AUDIT.md |
| 2026-10-15 | Whitespace | 93f71ae | AI-assisted/Git | PASS | [diff check](logs/2026-10-15_whitespace.txt) | git diff --check clean |
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
| 2026-10-15 | All 28 native groups | e0a4448 | AI-assisted/GCC 15.2.0 | PASS | [native](logs/2026-10-15_phase2_native.txt) | 529 algorithm + 15,261 adapter + 1,012 ADC assertions = 16,802; zero failures/warnings; all three binaries execute |
| 2026-10-15 | All 28 groups, SMP=0 boundary build | e0a4448 | AI-assisted/native GCC | PASS | [SMP=0](logs/2026-10-15_phase2_native_smp0.txt) | 529 + 15,253 + 973 assertions; no errors/warnings; affinity-specific fake invariants omitted only because this is the real single-core preprocessor profile |
| 2026-10-15 | GCC static analysis and full native suite | e0a4448 | AI-assisted/-fanalyzer | PASS | [analysis](logs/2026-10-15_phase2_static_analysis.txt) | Strict -Werror/-Wshadow/-Wconversion; zero diagnostics; all 28 groups execute |
| 2026-10-15 | S3-B01 | e0a4448 | AI-assisted/real kernel header parsing | PASS | [headers](logs/2026-10-15_phase2_headers.txt) | SMP=0/1 and TEST=1, including the new ADC worker; host parsing only |
| 2026-10-15 | S3-CI01 | e0a4448 | AI-assisted/actual test target | PASS | [verification](logs/2026-10-15_phase2_failure_propagation.txt), [direct negative](logs/2026-10-15_phase2_negative_probe.txt) | False assertion makes binary exit 1 and Make exit 2; verification target correctly recognizes failure |
| 2026-10-15 | S3-I01 | e0a4448 | AI-assisted/existing S2 tests | PASS | [six modes](logs/2026-10-15_phase2_buddy2_regression.txt) | run.ps1 was execution-policy blocked; compiled/ran its unchanged fixtures and six exact flag sets directly with GCC; all six pass, zero compiler diagnostics |
| 2026-10-15 | S3-I02 | e0a4448 | AI-assisted/host dependency probe | BLOCKED (expected diagnostic) | [probe](logs/2026-10-15_phase2_integration_probe.txt) | Missing production board_pins.h; removed SAMPLE_MSG/sem_slots/mpf_samples/S2_PERIOD_MS; not an ARM firmware result |
| 2026-10-15 | Toolchain provisioning | e0a4448 | AI-assisted/environment | BLOCKED | [provisioning](logs/2026-10-15_phase2_provisioning.txt) | ARM GCC/binutils/SDK absent; official download networking blocked; WSL enumeration denied; no installer completed |
| 2026-10-15 | S3-B03 | e0a4448 | AI-assisted/actual arm-check command | BLOCKED | [ARM objects](logs/2026-10-15_phase2_arm_objects.txt) | Compiler executable absent; Make exit 2, no ARM object produced |
| 2026-10-15 | S3-B02 default firmware | e0a4448 | AI-assisted/project Makefile | BLOCKED | [default](logs/2026-10-15_phase2_firmware_default.txt) | Actual exit 2 at missing Arm GCC guard; no successful compile/link/UF2 |
| 2026-10-15 | S3-B02 UART/no-radio firmware | e0a4448 | AI-assisted/project Makefile | BLOCKED | [UART](logs/2026-10-15_phase2_firmware_uart.txt) | Actual exit 2 at same prerequisite; not a substituted SDK/stub build |
| 2026-10-15 | Ownership and whitespace | Phase 2 tree | AI-assisted/Git | PASS | [scope](logs/2026-10-15_phase2_ownership.txt) | Other buddies, kernel/device/config/lib and shared app code unchanged; only empty ADC ownership row claimed in TEAM.md |
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

## Audit notes (2026-11-25): record history

Both renumberings described under *ID changes* above (dated 2026-11-21 and 2026-11-25 there) reached git in one commit, `5f93cb6` (2026-11-25). The last commit with the earlier IDs is `d18dd38`. `testing/README.md` says test IDs never change and are never reused; the last column of the table below shows which numbers now name a different test. Nothing in this section changes a test or a result; it records what the git history shows.

### ID history (git record)

IDs as they stood at `d18dd38` and what they became in `5f93cb6`. "Test case text changed" compares the whole test-case row (what, requirement, method, steps, expected result) at `d18dd38` with the row that has the new ID today. "This number today" says whether today's table uses the same number for a different test.

The October tables further up (*Buddy 3 implementation test definitions*, *Phase 2 definitions and final verification* and their result rows) use a third, earlier numbering that was not renumbered and is not listed here; for example, S3-U02 in the October table is "Normalization, clamping, inverse polarity and division/range guards".

| ID at `d18dd38` | What was tested at `d18dd38` (verbatim) | ID after `5f93cb6` | Test case text changed | This number today |
|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: IR readings scale between white and black |
| S3-U02 | IR reading scaled to 0-1000 between white and black: clamping, inverted polarity, zero-span and range guards | S3-U01 | yes | different test: calibration validates samples and preserves old endpoints |
| S3-U03 | default S3 configuration is valid and maps no letter to a turn (A and Z = none) | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: line position handles valid and invalid readings |
| S3-U04 | calibration from labelled white/black samples: contrast, noise and sample count checks; a failed finish keeps the old endpoints | S3-U02 | yes | different test: hysteresis and line recovery handle timer wrap |
| S3-U05 | line position: left/centre/right error, all-black junction, invalid mask or raw value | S3-U03 | yes | different test: invalid configuration values are rejected |
| S3-U06 | black/white hysteresis (no chatter), line-loss debounce, recovery deadline, timer wrap | S3-U04 | yes | different test: barcodes decode across scale, direction and speed |
| S3-U07 | configuration bounds: overlapping geometry, bad thresholds, enum and interval ranges are rejected | S3-U05 | yes | different test: invalid barcode frames produce no letter |
| S3-U08 | Code 39 `*A*` and `*Z*` decode at every scale, in both directions and with a speed change | S3-U06 | yes | different test: barcode capture decodes once until rearmed |
| S3-U09 | partial, unknown or corrupt frames, bad gaps, zero or huge widths give no letter | S3-U07 | yes | different test: failed barcode captures stay within buffer limits |
| S3-U10 | capture at two scales and speeds with timer rollover; one decode per marker until re-armed | S3-U08 | yes | different test: navigation handoff respects controller hold and override |
| S3-U11 | noise, timeout, partial data, 29-run capacity and an interrupted capture give a bounded error, no overrun | S3-U09 | yes | different test: invalid barcodes and sensor faults prevent steering |
| S3-U12 | one navigation event per marker; steering held until the controller resumes; override wins | S3-U10 | yes | different test: line recovery rejects invalid clock values |
| S3-U13 | unmapped letter or corrupt barcode gives no turn; invalid sensor data removes steering | S3-U11 | yes | different test: task lifecycle failures clean up safely |
| S3-U14 | line-loss recovery and stop events; duplicate, backward or stale clock values are rejected | S3-U12 | yes | different test: controller binding and leases reject stale snapshots |
| S3-U15 | S3 task lifecycle: kernel object failures clean up; acquisition stays off until configured, with no hardware access | S3-U13 | yes | different test: safety overrides prevent stale steering publication |
| S3-U16 | one controller binding; lease renew and expiry; stale snapshots refused | S3-U14 | yes | different test: event handoff rejects stale records and prioritizes faults |
| S3-U17 | a safety override during status publication or lock contention wins; no stale steering | S3-U15 | yes | different test: full event buffers hold steering safely |
| S3-U18 | copied event handoff; old-generation or over-age events rejected; a fault pre-empts | S3-U16 | yes | different test: IR setup rejects invalid pins and modes |
| S3-U19 | four-record event buffer full: transport fault, steering held | S3-U17 | yes | different test: ADC mode is refused without configured pins |
| S3-U20 | IR pin set-up: bad mode, reserved pins (GP19, GP23, GP29) and duplicate pins rejected; GPIO errors returned | S3-U18 | yes | different test: ADC scans preserve channel order and timestamps |
| S3-U21 | board without IR ADC pins: ADC mode refused (E_NOSPT), no pad access, no fake reading | S3-U19 | yes | different test: ADC deadlines reject missing or late replies |
| S3-U22 | ADC worker: three-channel order, pipelining, oldest timestamp, wrap | S3-U20 | yes | different test: ADC errors return no valid readings |
| S3-U23 | ADC deadline: no reply, a late 1000 ms read and rollover give invalid data, never a stale success | S3-U21 | yes | different test: invalid ADC data rejects the whole scan |
| S3-U24 | ADC device unavailable or read error: the error is returned, no valid reading | S3-U22 | yes | different test: ADC lifecycle cleanup preserves device ownership |
| S3-U25 | out-of-range, short or oversized reads and a bad mask, time or sequence reject the whole scan | S3-U23 | yes | different test: invalid ADC requests return explicit errors |
| S3-U26 | ADC worker lifecycle: buffer and task create failures clean up; only the owner calls the ADC | S3-U24 | yes | different test: ADC faults and overrides prevent steering restart |
| S3-U27 | ADC requests: bad channel or request, malformed or full buffers and a clock error give explicit errors | S3-U25 | yes | different test: blocked ADC reads do not block sampling |
| S3-U28 | S3 service with the ADC: tracking, status and raw data; override or a failed ADC cannot restart steering | S3-U26 | yes | different test: acquisition errors publish one fault without raw data |
| S3-U29 | the sampler keeps running while an ADC read is blocked; the deadline fault arrives first | S3-U27 | yes | different test: invalid navigation and control enums are rejected |
| S3-U30 | acquisition errors (unavailable, corrupt, short) give one fault and no raw values | S3-U28 | yes | different test: invalid controller modes leave state unchanged |
| S3-U31 | enum bounds: -1, max+1 and 99 rejected in the letter map and in core control | S3-U29 | yes | different test: raw IR mode never enables steering |
| S3-U32 | controller mode bounds with zero and non-zero lease; a rejected call changes nothing | S3-U30 | yes | different test: ADC scan cadence handles kernel ticks and rollover |
| S3-U33 | raw bring-up mode: raw IR levels can be read but never steer; refused while the motors are enabled | S3-U31 | yes | different test: late ADC replies are dropped and replaced |
| S3-U34 | ADC scan cadence with the real kernel tick (a 10 ms delay runs at +20 ms), with rollover | S3-U32 | yes | different test: conversion deadlines drop only the current scan |
| S3-U35 | a late ADC reply is dropped and a new scan is sent | S3-U33 | yes | different test: ADC sampling handles jitter and raw-data authority |
| S3-U36 | a conversion past its deadline drops only that scan | S3-U34 | yes | different test: STOP wins while waiting for ADC replies |
| S3-U37 | sampler and ADC worker together: current scan with jitter and rollover; raw values only with authority | S3-U35 | yes | different test: lease expiry during ADC waits removes steering |
| S3-U38 | STOP during the ADC reply wait wins | S3-U36 | yes | different test: ADC waits return clock and transport errors |
| S3-U39 | lease expiry during the reply wait removes steering | S3-U37 | yes | different test: missing ADC replies have a bounded wait |
| S3-U40 | clock, delivery or transport failure during the wait returns an error | S3-U38 | yes | different test: robot IR layout keeps barcode input separate |
| S3-U41 | a missing ADC reply is waited for a bounded time only | S3-U39 | yes | different test: measured calibration classifies recorded IR samples |
| S3-U42 | robot IR layout: GP26 right, GP27 left, GP28 barcode only; codebook A U-turn, F left, Z right, B-D no turn; the barcode sensor cannot steer or make a junction | S3-U40 | yes | different test: paused sample gaps preserve active fault state |
| S3-U43 | measured endpoints in `robot_ir_calibration.h` classify the recorded white and black samples; a paused core never steers | S3-U41 | yes | different test: paused calibrated scans handle jitter and rollover |
| S3-U44 | paused timing: sample gaps while paused do not latch a fault; an active fault stays latched | S3-U42 | yes | different test: cyclic sampling avoids bursts and handler ADC calls |
| S3-U45 | calibrated scan while paused, with jitter and rollover | S3-U43 | yes | different test: cyclic handler failures clean up safely |
| S3-U46 | retired - do not reuse. Was: barcode capture copy/overflow/no authority (`test_barcode_capture.c`, removed in 9c6b8bc). | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: publication waits respect freshness and control authority |
| S3-U47 | retired - do not reuse. Was: barcode capture invalid/init cleanup (same removed file). | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | different test: controller barcode maps validate and apply changes |
| S3-U48 | cyclic sampler release: no burst after a delay, no ADC call from the handler | S3-U44 | yes | different test: decoded turns survive later latches and publication delays |
| S3-U49 | cyclic handler create or start failure cleans up | S3-U45 | yes | different test: requests use the sampler gap limit |
| S3-U50 | status publish wait: freshness, STOP, lease and clock failure | S3-U46 | yes | different test: controller requests handle bounded lock contention |
| S3-U51 | letter map set by the controller (MAP command): bad letter, action, caller or a busy lock refused; applied at the next marker; letter 0 action 0 restores the codebook | S3-U47 | yes | different test: partial and fault-cut barcode reads are distinguished |
| S3-U52 | a decoded turn still reaches the controller after a later silent latch or a late publish | S3-U48 | yes | different test: ADC read overruns stay latched until reboot |
| S3-U53 | other requests use the sampler's own gap limit; a 30 ms old status is still current | S3-U49 | yes | different test: ADC worker continues after a dropped scan |
| S3-U54 | mode and lease requests wait briefly for a busy lock; past the bound, E_TMOUT changes nothing | S3-U50 | yes | different test: four barcode commands produce navigation events |
| S3-U55 | a capture that ends by itself with 3-8 elements counts as partial; one cut by a sampler hiccup counts as failed | S3-U51 | yes | different test: barcode decoding tolerates valid edge displacement |
| S3-U56 | a driver read that overran its deadline stays latched for the boot | S3-U52 | yes | different test: barcode resynchronisation handles leading noise |
| S3-U57 | the ADC worker loop keeps running after a dropped scan | S3-U53 | yes | different test: barcode completion respects framing and timeout gates |
| S3-U58 | `*A*` to `*D*` each give one navigation event: left, right, straight, U-turn | S3-U54 | yes | different test: digital barcode capture waits for end quiet |
| S3-U59 | barcode edge bias: all 26 letters and framing, scale and direction, edge displacement, invalid shapes, capture budget | S3-U55 | yes | different test: barcode interruptions produce the correct read status |
| S3-U60 | barcode resync: A-Z in both directions after leading noise; quiet, timeout, wrap, malformed and re-arm cases; a measured `*F*` with a prefix | S3-U56 | yes | different test: reference captures preserve navigation and fault handling |
| S3-U61 | barcode completion: A-Z, both directions, four scales, timer wrap, a recorded `*A*`; strict, invalid, partial, abort and timeout gates | S3-U57 | yes | different test: ADC line sampling works with digital barcodes |
| S3-U62 | GP28 digital capture ends only after the quiet time with no edge; bars and result reported in both directions; noise, a trailing mark or overflow do not end it early | S3-U58 | yes | not used |
| S3-U63 | fault-cut and partial reads: a hiccup mid-marker is a failed read (re-read), 3-8 elements is partial, 2 is a crossing line; a mode change is not a failure | S3-U59 | yes | not used |
| S3-U64 | reference capture through the main navigation path: A-D in both directions with a noise prefix and a junction; unmapped `*E*` gives no turn; retry and sensor fault | S3-U60 | yes | not used |
| S3-U65 | mission IR mode 3 through the real adapter: ADC line on GP26/GP27 plus digital GP28 barcode; GP28 is never read as ADC | S3-U61 | yes | not used |
| S3-B01 | S3 sources parse with the real kernel headers, SMP=0 and SMP=1, TEST=1 | S3-B01 | yes | same test |
| S3-B02 | firmware builds with the S3 sources: default, and UART with no Wi-Fi | S3-B02 | yes | same test |
| S3-B03 | S3 sources compile for Cortex-M0+ with `arm-none-eabi-gcc`, SMP=0 and SMP=1 (objects only, no link) | S3-B03 | yes | same test |
| S3-B04 | every host test again under `gcc -fanalyzer -O1`, warnings as errors | S3-B04 | yes | same test |
| S3-CI01 | a failing check makes the test target fail | S3-CI01 | yes | same test |
| S3-I01 | retired - do not reuse. Was: S3 ran the S2 host tests as a regression; S2 now keeps its own tests. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-I02 | retired - do not reuse. Was: October host probe for missing shared headers; the probe no longer exists. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M01 | retired - do not reuse. Was: electrical identification (October bench plan `HARDWARE_VALIDATION.md`, removed in 54a371f). | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M02 | retired - do not reuse. Was: pin and peripheral ownership. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M03 | retired - do not reuse. Was: sensor geometry. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M04 | retired - do not reuse. Was: calibration and lighting (now covered by S3-M12). | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M05 | retired - do not reuse. Was: analogue worker and deadline. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M06 | retired - do not reuse. Was: captured A/Z waveforms. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M07 | retired - do not reuse. Was: sample budget and speed. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M08 | retired - do not reuse. Was: controller and motor handoff. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M09 | retired - do not reuse. Was: line loss and bounded recovery. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M10 | retired - do not reuse. Was: override and marker clearance. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M11 | retired - do not reuse. Was: marker spacing and turn timing. | removed from the table in `5f93cb6` (not listed in its 2026-11-21 table) | — | not used |
| S3-M12 | IR raw levels on white and black, and the room light | removed from the table in `5f93cb6` (S3-M01 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S3-M13 | barcode read rate for `*A*`, `*F*` and `*Z*` at 30% demand | removed from the table in `5f93cb6` (S3-M02 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S3-M14 | line following on a 1 m straight at 30% demand: touch length and dead band | removed from the table in `5f93cb6` (S3-M03 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |
| S3-M15 | lift detection inputs from S3: both line IRs near full scale and GP28 black when lifted; the black line stays below the limit | removed from the table in `5f93cb6` (S3-M04 in its 2026-11-21 numbering; its 2026-11-25 list removes it as INCOMPLETE) | — | not used |

### Commit cells checked

Every hash in a *Commit* cell of this file, including the restored rows, was looked up with `git cat-file -t` in this repository (all branches) on 2026-11-25. A cell that names exactly one existing commit needs no note. The recorded commit values are retained. Entries needing clarification are listed by date and test ID under *Notes on unclear commit and evidence cells* at the end of this file.

### Evidence paths checked

Every `logs/` path named in a *Log / evidence* cell of this file, including the restored rows, exists at `0bac48c`. YouTube links were not opened. Exception: two October rows name `HARDWARE_VALIDATION.md`, which is no longer in the repository (see the notes at the end of this file); the sentence "Manual tests S3-M01 through S3-M11 are defined in HARDWARE_VALIDATION.md" refers to the same deleted file.

### Notes on unclear commit and evidence cells

- **2026-10-15 Ownership and whitespace; 2026-10-15 GitHub upload, PR, CI, merge; 2026-10-15 S3-M01..M11 and on-target smoke:** No commit hash recorded in the original entry (it says only "Phase 2 ..."). The Phase 2 rows above it name `e0a4448` (2026-10-15), but which code this row describes is not recorded.
- **2026-10-15 On-target smoke S3-U02/U03 and S3-M01 through S3-M11; 2026-10-15 S3-M01..M11 and on-target smoke:** `testing/subsystem3/HARDWARE_VALIDATION.md` is not in the repository at `0bac48c`: it was deleted in `54a371f` (2026-10-27). Its last version is in `efedeb1`, the parent of `54a371f` (`git show efedeb1:testing/subsystem3/HARDWARE_VALIDATION.md`).
