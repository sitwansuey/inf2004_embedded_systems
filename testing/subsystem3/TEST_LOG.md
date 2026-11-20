# Subsystem 3 — test log

**Subsystem:** Buddy 3 IR line following/barcode decoding  ·  **Student ID:** 2500383  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |
| S3-U02 | IR reading scaled to 0-1000 between white and black: clamping, inverted polarity, zero-span and range guards | FR8 | auto (integrated TEST=1); auto (host) | 1. On target: in `build_make` run `make TEST=1 NET=lwip MQTT=1 -j8`, flash `mtk3pico_smp1_usb_cdc_wifi_lwip_mqtt_enc_test.uf2`, save the USB console to `logs/`. Needs a build where the integrated `APP_TEST` block in `app_main.c` calls `subsystem3_test()` (15aebfc does not). 2. Host: `make -C testing/subsystem3 test`, program `test_buddy3.c`. | `[TEST] S3-U02 PASS  pure reflectance endpoint`; host: `S3-U02 normalization PASS` |
| S3-U03 | default S3 configuration is valid and maps no letter to a turn (A and Z = none) | FR9 | auto (integrated TEST=1) | Step 1 of S3-U02. | `[TEST] S3-U03 PASS  no unverified navigation mapping` |
| S3-U04 | calibration from labelled white/black samples: contrast, noise and sample count checks; a failed finish keeps the old endpoints | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U04 calibration PASS` |
| S3-U05 | line position: left/centre/right error, all-black junction, invalid mask or raw value | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U05 line positions PASS` |
| S3-U06 | black/white hysteresis (no chatter), line-loss debounce, recovery deadline, timer wrap | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U06 hysteresis/loss/rollover PASS` |
| S3-U07 | configuration bounds: overlapping geometry, bad thresholds, enum and interval ranges are rejected | FR8, FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U07 configuration bounds PASS` |
| S3-U08 | Code 39 `*A*` and `*Z*` decode at every scale, in both directions and with a speed change | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U08 barcode scales/speed/direction PASS` |
| S3-U09 | partial, unknown or corrupt frames, bad gaps, zero or huge widths give no letter | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U09 barcode corruption PASS` |
| S3-U10 | capture at two scales and speeds with timer rollover; one decode per marker until re-armed | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U10 capture/rearm/duplicates PASS` |
| S3-U11 | noise, timeout, partial data, 29-run capacity and an interrupted capture give a bounded error, no overrun | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U11 capture failures/capacity PASS` |
| S3-U12 | one navigation event per marker; steering held until the controller resumes; override wins | FR9, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U12 navigation/override PASS` |
| S3-U13 | unmapped letter or corrupt barcode gives no turn; invalid sensor data removes steering | FR8, FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U13 unmapped/corrupted/fault PASS` |
| S3-U14 | line-loss recovery and stop events; duplicate, backward or stale clock values are rejected | FR8, NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U14 recovery and time PASS` |
| S3-U15 | S3 task lifecycle: kernel object failures clean up; acquisition stays off until configured, with no hardware access | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U15 lifecycle/disabled PASS` |
| S3-U16 | one controller binding; lease renew and expiry; stale snapshots refused | FR1, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U16 controller leases PASS` |
| S3-U17 | a safety override during status publication or lock contention wins; no stale steering | NFR1, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U17 safety publication race PASS` |
| S3-U18 | copied event handoff; old-generation or over-age events rejected; a fault pre-empts | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U18 event handoff/staleness PASS` |
| S3-U19 | four-record event buffer full: transport fault, steering held | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U19 event queue full PASS` |
| S3-U20 | IR pin set-up: bad mode, reserved pins (GP19, GP23, GP29) and duplicate pins rejected; GPIO errors returned | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U20 sensor configuration PASS` |
| S3-U21 | board without IR ADC pins: ADC mode refused (E_NOSPT), no pad access, no fake reading | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c`, built with `BOARD_IR_MODE=0` | `S3-U21 ADC boundary PASS` |
| S3-U22 | ADC worker: three-channel order, pipelining, oldest timestamp, wrap | FR8, NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U22 ADC pipeline/timestamp/rollover PASS` |
| S3-U23 | ADC deadline: no reply, a late 1000 ms read and rollover give invalid data, never a stale success | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U23 ADC deadline and late completion PASS` |
| S3-U24 | ADC device unavailable or read error: the error is returned, no valid reading | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U24 ADC unavailable/read error PASS` |
| S3-U25 | out-of-range, short or oversized reads and a bad mask, time or sequence reject the whole scan | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U25 ADC invalid/partial readings PASS` |
| S3-U26 | ADC worker lifecycle: buffer and task create failures clean up; only the owner calls the ADC | | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U26 ADC lifecycle/ownership PASS` |
| S3-U27 | ADC requests: bad channel or request, malformed or full buffers and a clock error give explicit errors | | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U27 ADC transport/request validation PASS` |
| S3-U28 | S3 service with the ADC: tracking, status and raw data; override or a failed ADC cannot restart steering | FR8, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U28 ADC controller integration PASS` |
| S3-U29 | the sampler keeps running while an ADC read is blocked; the deadline fault arrives first | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U29 blocked worker isolation/fault PASS` |
| S3-U30 | acquisition errors (unavailable, corrupt, short) give one fault and no raw values | FR8 | auto (host) | `make -C testing/subsystem3 test`: `test_adc.c` | `S3-U30 acquisition errors/raw snapshot PASS` |
| S3-U31 | enum bounds: -1, max+1 and 99 rejected in the letter map and in core control | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c` | `S3-U31 enum bounds PASS` |
| S3-U32 | controller mode bounds with zero and non-zero lease; a rejected call changes nothing | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U32 controller enum bounds PASS` |
| S3-U33 | raw bring-up mode: raw IR levels can be read but never steer; refused while the motors are enabled | FR8, FR13 | auto (host) | `make -C testing/subsystem3 test`: `test_raw.c`, built twice (`test_raw` motors disabled, `test_raw_unlocked` motors enabled) | `S3-U33 raw bring-up: 1 groups, N checks, 0 failures`, once per build |
| S3-U34 | ADC scan cadence with the real kernel tick (a 10 ms delay runs at +20 ms), with rollover | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_tick.c` | `S3-U34 kernel tick cadence/rollover PASS` |
| S3-U35 | a late ADC reply is dropped and a new scan is sent | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_tick.c` | `S3-U35 late reply dropped, resubmitted PASS` |
| S3-U36 | a conversion past its deadline drops only that scan | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_tick.c` | `S3-U36 conversion deadline drops the scan only PASS` |
| S3-U37 | sampler and ADC worker together: current scan with jitter and rollover; raw values only with authority | FR8, NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U37 current scan/jitter/rollover/raw authority PASS` |
| S3-U38 | STOP during the ADC reply wait wins | NFR1 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U38 STOP during reply wait PASS` |
| S3-U39 | lease expiry during the reply wait removes steering | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U39 lease expires during reply wait PASS` |
| S3-U40 | clock, delivery or transport failure during the wait returns an error | | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U40 clock/delivery/transport failure PASS` |
| S3-U41 | a missing ADC reply is waited for a bounded time only | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U41 absent reply bounded PASS` |
| S3-U42 | robot IR layout: GP26 right, GP27 left, GP28 barcode only; codebook A U-turn, F left, Z right, B-D no turn; the barcode sensor cannot steer or make a junction | FR8, FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_robot_ir_layout.c` | `S3-U42 confirmed layout: N checks, 0 failures` |
| S3-U43 | measured endpoints in `robot_ir_calibration.h` classify the recorded white and black samples; a paused core never steers | FR8 (A3) | auto (host) | `make -C testing/subsystem3 test`: `test_robot_ir_calibration.c`, built three times (`BOARD_IR_MODE` 2, default 3, 0) | `S3-U43 measured calibration: N checks, 0 failures`, once per build |
| S3-U44 | paused timing: sample gaps while paused do not latch a fault; an active fault stays latched | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_paused_time.c` | `S3-U44 paused timing: N checks, 0 failures` |
| S3-U45 | calibrated scan while paused, with jitter and rollover | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_acquire.c` | `S3-U45 configured paused jitter/rollover PASS` |
| S3-U46 | retired - do not reuse. Was: barcode capture copy/overflow/no authority (`test_barcode_capture.c`, removed in 9c6b8bc). | | | | |
| S3-U47 | retired - do not reuse. Was: barcode capture invalid/init cleanup (same removed file). | | | | |
| S3-U48 | cyclic sampler release: no burst after a delay, no ADC call from the handler | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_sample_schedule.c` | `S3-U48 cyclic/no burst/no ADC callback PASS` |
| S3-U49 | cyclic handler create or start failure cleans up | | auto (host) | `make -C testing/subsystem3 test`: `test_sample_schedule.c` | `S3-U49 cyclic lifecycle cleanup PASS` |
| S3-U50 | status publish wait: freshness, STOP, lease and clock failure | NFR4, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_sample_schedule.c` | `S3-U50 publication wait freshness/STOP/lease/clock PASS` |
| S3-U51 | letter map set by the controller (MAP command): bad letter, action, caller or a busy lock refused; applied at the next marker; letter 0 action 0 restores the codebook | FR9, FR12 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U51 controller barcode map PASS` |
| S3-U52 | a decoded turn still reaches the controller after a later silent latch or a late publish | FR9, VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U52 decoded turn survives a later latch (L1/L6) PASS` |
| S3-U53 | other requests use the sampler's own gap limit; a 30 ms old status is still current | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U53 request gap limit aligned (L6) PASS` |
| S3-U54 | mode and lease requests wait briefly for a busy lock; past the bound, E_TMOUT changes nothing | VC <-> S3 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U54 controller tolerates a busy lock (L7) PASS` |
| S3-U55 | a capture that ends by itself with 3-8 elements counts as partial; one cut by a sampler hiccup counts as failed | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_adapter.c` | `S3-U55 partial and fault-cut barcode reads (L2) PASS` |
| S3-U56 | a driver read that overran its deadline stays latched for the boot | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_tick.c` | `S3-U56 read overrun still latched PASS` |
| S3-U57 | the ADC worker loop keeps running after a dropped scan | NFR4 | auto (host) | `make -C testing/subsystem3 test`: `test_adc_tick.c` | `S3-U57 worker loop survives a dropped scan PASS` |
| S3-U58 | `*A*` to `*D*` each give one navigation event: left, right, straight, U-turn | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_buddy3.c`, group "S3 four-command navigation handoff" (prints no ID) | `S3 four-command navigation handoff PASS` |
| S3-U59 | barcode edge bias: all 26 letters and framing, scale and direction, edge displacement, invalid shapes, capture budget | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_edge_bias.c`, built with `BOARD_IR_MODE=2` (prints no ID) | `S3 barcode edge-bias regressions: N checks, 0 failures` |
| S3-U60 | barcode resync: A-Z in both directions after leading noise; quiet, timeout, wrap, malformed and re-arm cases; a measured `*F*` with a prefix | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_resync.c` (prints no ID) | `Barcode resync: N A-Z/direction/prefix cases; ... PASS` |
| S3-U61 | barcode completion: A-Z, both directions, four scales, timer wrap, a recorded `*A*`; strict, invalid, partial, abort and timeout gates | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_completion.c` (prints no ID) | `Completion: N A-Z/direction/scale/wrap and recorded-A cases; ... PASS` |
| S3-U62 | GP28 digital capture ends only after the quiet time with no edge; bars and result reported in both directions; noise, a trailing mark or overflow do not end it early | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_end_quiet.c` (prints no ID) | `S3 GP28 end-quiet barcode: N checks, 0 failures` |
| S3-U63 | fault-cut and partial reads: a hiccup mid-marker is a failed read (re-read), 3-8 elements is partial, 2 is a crossing line; a mode change is not a failure | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_fault_fail.c` (prints no ID) | `S3 L2 fault-cut and partial barcode reads: N checks, 0 failures` |
| S3-U64 | reference capture through the main navigation path: A-D in both directions with a noise prefix and a junction; unmapped `*E*` gives no turn; retry and sensor fault | FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_barcode_import.c` (prints no ID) | `PASS: reference capture (both directions/noise prefix/junction) ...` |
| S3-U65 | mission IR mode 3 through the real adapter: ADC line on GP26/GP27 plus digital GP28 barcode; GP28 is never read as ADC | FR8, FR9 | auto (host) | `make -C testing/subsystem3 test`: `test_mixed_barcode.c`, built with `BOARD_IR_MODE=3` (prints no ID) | `ADC line + digital GP28 barcode PASS`, then `N checks, 0 failures` |
| S3-B01 | S3 sources parse with the real kernel headers, SMP=0 and SMP=1, TEST=1 | | auto (host) | `make -C testing/subsystem3 check-headers` | `Checking real kernel headers, SMP=0` and `SMP=1`, no error, exit 0 |
| S3-B02 | firmware builds with the S3 sources: default, and UART with no Wi-Fi | | auto (host) | `make -C build_make -j8`, then `make -C build_make CONSOLE=uart WIFI=none -j8` | both exit 0 and write a `.uf2` |
| S3-B03 | S3 sources compile for Cortex-M0+ with `arm-none-eabi-gcc`, SMP=0 and SMP=1 (objects only, no link) | | auto (host) | `make -C testing/subsystem3 arm-check` | `PASS: Buddy 3 ARM object compilation SMP=0` and `SMP=1` |
| S3-B04 | every host test again under `gcc -fanalyzer -O1`, warnings as errors | | auto (host) | `make -C testing/subsystem3 analyze` | no analyzer warning; the same PASS lines as `test`, 0 failures |
| S3-CI01 | a failing check makes the test target fail | | auto (host) | `make -C testing/subsystem3 verify-failure` (runs `test` with `TEST_ARGS=--force-failure`) | `PASS: test target propagated a real failed assertion` |
| S3-I01 | retired - do not reuse. Was: S3 ran the S2 host tests as a regression; S2 now keeps its own tests. | | | | |
| S3-I02 | retired - do not reuse. Was: October host probe for missing shared headers; the probe no longer exists. | | | | |
| S3-M01 | retired - do not reuse. Was: electrical identification (October bench plan `HARDWARE_VALIDATION.md`, removed in 54a371f). | | | | |
| S3-M02 | retired - do not reuse. Was: pin and peripheral ownership. | | | | |
| S3-M03 | retired - do not reuse. Was: sensor geometry. | | | | |
| S3-M04 | retired - do not reuse. Was: calibration and lighting (now covered by S3-M12). | | | | |
| S3-M05 | retired - do not reuse. Was: analogue worker and deadline. | | | | |
| S3-M06 | retired - do not reuse. Was: captured A/Z waveforms. | | | | |
| S3-M07 | retired - do not reuse. Was: sample budget and speed. | | | | |
| S3-M08 | retired - do not reuse. Was: controller and motor handoff. | | | | |
| S3-M09 | retired - do not reuse. Was: line loss and bounded recovery. | | | | |
| S3-M10 | retired - do not reuse. Was: override and marker clearance. | | | | |
| S3-M11 | retired - do not reuse. Was: marker spacing and turn timing. | | | | |
| S3-M12 | IR raw levels on white and black, and the room light | FR8; assumption A3 | manual | 1. Flash the normal image. Start the dashboard. Robot disarmed, sensors at normal height. 2. Stand the robot on white paper. Watch Right IR (GP26), Left IR (GP27) and Barcode IR (GP28) for 5 s. Write down the lowest and highest value. 3. Move the robot so the black line is under GP26, then GP27, then GP28. Write down the same. 4. Repeat step 2 on the ramp surface. 5. Write down the light level in lux (meter or phone app). 6. If a limit is missed, re-measure the endpoints in `app_program/robot_ir_calibration.h`. | White about 190, black about 3200 (A3). GP26: white ≤ 1850, black ≥ 2460. GP27: white ≤ 1830, black ≥ 2430 (550 and 750 per mille of the spans 187-3214 and 189-3177). Ramp below the black limit. GP28: 0 on white, 1 on black. Lux written down. |
| S3-M13 | barcode read rate for `*A*`, `*F*` and `*Z*` at 30% demand | FR9 | manual | 1. Print `*A*`, `*F*` and `*Z*` at course size. Tape one to the right of a straight line (GP28 side), at least 300 mm from the start. 2. Flash the normal image. Start the dashboard and open its Events tab. With MAP, set the letter to action 0 (read, no turn). 3. Put the robot on the line at the start. Run `python tools/robot_console.py --port COMx --seconds 20 --arm --log testing/subsystem3/logs/<date>_fr9_<letter>_<pass>.txt` (30% demand). Press Ctrl-C after the barcode event; the tool then sends STOP. 4. Do 20 passes per letter. 5. For each pass, note the attempt (1, 2 or 3) that decoded the letter, or "failed". Copy the event log into `logs/`. | For each letter: ≥ 16 of 20 decoded on attempt 1 and ≥ 19 of 20 within 3 attempts. No wrong letter. Each attempt shows one barcode event with the bars and spaces read. |
| S3-M14 | line following on a 1 m straight at 30% demand: touch length and dead band | FR8 | manual | 1. Measure the sensor spacing S (GP26 to GP27, centre to centre) and the tape width W with a ruler. Write down (S - W)/2. 2. Lay a 1 m straight line with a 300 mm lead-in. Flash the normal image. Start the dashboard. 3. Film from above at 60 fps. 4. Put the robot centred on the line. Run `python tools/robot_console.py --port COMx --seconds 15 --arm --log testing/subsystem3/logs/<date>_fr8_run<n>.txt`. Press Ctrl-C after the 1 m. 5. Do 5 runs. 6. In the video, count the frames of each correction pivot. | All 5 runs cover the 1 m with no fault and without losing the line. Every pivot lasts ≤ 300 ms (≤ 18 frames at 60 fps). S, W and (S - W)/2 written down. |
| S3-M15 | lift detection inputs from S3: both line IRs near full scale and GP28 black when lifted; the black line stays below the limit | FR1 (fault 8), VC <-> S3 | manual | 1. Flash the normal image. Start the dashboard. Robot disarmed. 2. On the track, put GP26 on the black line, then GP27. Write down each raw value. 3. Lift the robot level, 50-100 mm above the floor. Hold 5 s. Write down Right IR, Left IR and Barcode IR. 4. Do step 3 three times. The VC stop itself (fault 8 after 2 s while armed) is not part of this test. | Lifted, every time: Right IR and Left IR ≥ 3700 (`BOARD_LIFT_RAW`) and Barcode IR = 1. On the black line: GP26 and GP27 stay below 3700. |

Rows from S3-U02 down were added on 2026-11-20 for the checkpoint 8c code (15aebfc). The older definition tables further down ("Buddy 3 implementation test definitions", "Phase 2 definitions and final verification") are kept unchanged and the IDs keep their meaning. S3-U01 (first row) is retired - do not reuse.

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

**auto (host)** = a host program run by `make -C testing/subsystem3 test` from the repo root (gcc).
On Windows, run it from PowerShell with `C:\msys64\ucrt64\bin;C:\msys64\usr\bin` at the front of PATH.
Each program prints PASS lines or "0 failures". Any failure makes `make` stop with an error.
Also check that the output has no FAIL line. *N* is the printed check count; it changes as tests are added.
On-target **auto** rows name their build profile. S3 has only the integrated `make TEST=1 NET=lwip MQTT=1` profile.


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
| 2026-11-20 | S3-U02 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U02 normalization PASS`; same in the `analyze` rerun. Host part only; the on-target TEST=1 part was not run |
| 2026-11-20 | S3-U04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U04 calibration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U05 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U05 line positions PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U06 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U06 hysteresis/loss/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U07 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U07 configuration bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U08 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U08 barcode scales/speed/direction PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U09 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U09 barcode corruption PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U10 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U10 capture/rearm/duplicates PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U11 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U11 capture failures/capacity PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U12 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U12 navigation/override PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U13 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U13 unmapped/corrupted/fault PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U14 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U14 recovery and time PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U15 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U15 lifecycle/disabled PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U16 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U16 controller leases PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U17 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U17 safety publication race PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U18 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U18 event handoff/staleness PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U19 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U19 event queue full PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U20 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U20 sensor configuration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U21 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U21 ADC boundary PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U22 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U22 ADC pipeline/timestamp/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U23 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U23 ADC deadline and late completion PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U24 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U24 ADC unavailable/read error PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U25 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U25 ADC invalid/partial readings PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U26 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U26 ADC lifecycle/ownership PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U27 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U27 ADC transport/request validation PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U28 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U28 ADC controller integration PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U29 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U29 blocked worker isolation/fault PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U30 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc` (9 groups, 1171 checks, 0 failures): `S3-U30 acquisition errors/raw snapshot PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U31 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3-U31 enum bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U32 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U32 controller enum bounds PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U33 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, both builds: `test_raw` (motors disabled) 1 group, 135 checks, 0 failures; `test_raw_unlocked` (motors enabled) 1 group, 2 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-U34 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U34 kernel tick cadence/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U35 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U35 late reply dropped, resubmitted PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U36 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U36 conversion deadline drops the scan only PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U37 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U37 current scan/jitter/rollover/raw authority PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U38 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U38 STOP during reply wait PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U39 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U39 lease expires during reply wait PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U40 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U40 clock/delivery/transport failure PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U41 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U41 absent reply bounded PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U42 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_robot_ir_layout`: `S3-U42 confirmed layout: 15 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U43 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, all three builds: `BOARD_IR_MODE=2` 67 checks, default 9 checks, `BOARD_IR_MODE=0` 2 checks, 0 failures each; same in the `analyze` rerun |
| 2026-11-20 | S3-U44 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_paused_time`: `S3-U44 paused timing: 99 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U45 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_acquire` (6 groups, 2144 checks, 0 failures): `S3-U45 configured paused jitter/rollover PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U48 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U48 cyclic/no burst/no ADC callback PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U49 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U49 cyclic lifecycle cleanup PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U50 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_sample_schedule` (3 groups, 449 checks, 0 failures): `S3-U50 publication wait freshness/STOP/lease/clock PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U51 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U51 controller barcode map PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U52 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U52 decoded turn survives a later latch (L1/L6) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U53 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U53 request gap limit aligned (L6) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U54 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U54 controller tolerates a busy lock (L7) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U55 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adapter` built with `BOARD_IR_MODE=0` (13 groups, 52212 checks, 0 failures): `S3-U55 partial and fault-cut barcode reads (L2) PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U56 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U56 read overrun still latched PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U57 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_adc_tick` (5 groups, 465 checks, 0 failures): `S3-U57 worker loop survives a dropped scan PASS`; same in the `analyze` rerun |
| 2026-11-20 | S3-U58 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_buddy3` (14 groups, 579 checks, 0 failures): `S3 four-command navigation handoff PASS`; same in the `analyze` rerun (group prints no ID) |
| 2026-11-20 | S3-U59 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_edge_bias` built with `BOARD_IR_MODE=2`: 11214 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-U60 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_resync`: 468 A-Z/direction/prefix cases plus the quiet/timeout/wrap/budget/malformed/held/rearm and measured-F prefix cases, PASS; same in the `analyze` rerun |
| 2026-11-20 | S3-U61 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_completion`: 210 A-Z/direction/scale/wrap and recorded-A cases plus the strict/invalid/partial/resync/abort/timeout gates, PASS; same in the `analyze` rerun |
| 2026-11-20 | S3-U62 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_end_quiet`: `S3 GP28 end-quiet barcode: 169 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U63 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_fault_fail`: `S3 L2 fault-cut and partial barcode reads: 44 checks, 0 failures`; same in the `analyze` rerun |
| 2026-11-20 | S3-U64 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_barcode_import`: `PASS: reference capture (both directions/noise prefix/junction) ...`, unmapped E stays unmapped, retry and sensor fault verified; same in the `analyze` rerun |
| 2026-11-20 | S3-U65 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make test`, `test_mixed_barcode` built with `BOARD_IR_MODE=3`: `ADC line + digital GP28 barcode PASS`, 106 checks, 0 failures; same in the `analyze` rerun |
| 2026-11-20 | S3-B01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make check-headers`: `Checking real kernel headers, SMP=0` and `SMP=1`, no error, exit 0 |
| 2026-11-20 | S3-B03 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make arm-check` with arm-none-eabi-gcc 14.2.1 (Arm GNU Toolchain 14.2.Rel1): `PASS: Buddy 3 ARM object compilation SMP=0` and `SMP=1`, exit 0; objects only, no link |
| 2026-11-20 | S3-B04 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make analyze`: every host program rebuilt with `-fanalyzer -O1` and `-Werror`, no analyzer warning; the same PASS lines and check counts as `test`, 0 failures, exit 0 |
| 2026-11-20 | S3-CI01 | `ed197b3` | Ron Tan | PASS | `logs/2026-11-20_host_s3.txt` | `make verify-failure`: the forced assertion (`test_buddy3.c:601`, `2 + 2 == 5`) gave 1 failure and `make[1]` Error 1, then `PASS: test target propagated a real failed assertion`, exit 0. The one failed check is the intended forced one |
| 2026-11-20 | S3-M12 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Track run 1, not the defined steps: raw values from the 5 Hz telemetry while line tracking at 20% (415 samples, 119.7-571.1 s; 18 with both line IRs ≥ 3700 left out), not 5 s holds on white paper and on the line under each sensor. Floor: GP27 min 186, median 250; GP26 min 182, median 248 (95% of floor samples ≤ 710 and ≤ 764). On the line with the other sensor on the floor: GP27 up to 3437 (median 3122, 57 samples), GP26 up to 3504 (median 2867, 19 samples). GP28 read 1 in 47 of 178 samples with both line IRs below 1000 (this run crossed barcode cards). Ramp and lux not measured; the floor surface is not logged. |
| 2026-11-20 | S3-M13 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | 3 card passes, not 20 per letter; demand 20% (START commands c10314…), not 30%; A, F and Z mapped to action 3 (Straight) at 75.4-82.8 s (lines 10108-11666), not action 0. Pass 1: attempt 1 failed, 17/29 bars and spaces (123.47 s, line 17364); fault 7 at 125.21 s while reversing for attempt 2. Pass 2: attempt 1 failed, 13/29 (406.82 s); attempt 2 (reverse) failed, 18/29 (411.22 s); fault 1 Tilt at 413.5 s before attempt 3. Pass 3: decoded A on attempt 1, 29/29 (568.89 s, line 76420), then "A -> Straight". Which card was under the robot is not logged. The telemetry also shows 29 other "Reading..." stretches with no attempt event. |
| 2026-11-20 | S3-M14 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | Line tracking on the track at 20% demand, not a 1 m straight at 30%; no video; S, W and (S - W)/2 not measured. 12 tracking stretches, longest 37.7 s (183.3-221.0 s). No fault 2 in the log (fault codes 0, 1, 7 and 8 only). Stretches ended by obstacle acquisition (5), fault 7 (125.2, 263.1 s), fault 8 (374.1, 393.8 s), fault 1 Tilt (413.5, 491.5 s) and STOP (571.1 s). The motor direction shows 34 pivots, the longest in 2 consecutive snapshots (seq 89-90 at 190.6 s; seq 38-39 at 486.0 s). The 300 ms limit needs the video: the telemetry is 5 Hz and loses snapshots. |
| 2026-11-20 | S3-M15 | `89f1bdc` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run1_mqtt.txt` | No disarmed level lift of 50-100 mm held 5 s; lift height not logged. On the line while tracking, with the other sensor on the floor: GP27 up to 3437 (119.73 s), GP26 up to 3504 (123.62 s), below 3700. Both line IRs dark at once (GP27/GP26): 2938/3041 (172.6 s); 3594/3681 with GP28 = 1 at 411.30 s (pitch −1°, reversing for a barcode re-read), 19 below the limit; what was under the robot is not logged. All 18 tracking samples with both line IRs ≥ 3700 had GP28 = 1 (3832-4095). Fault 8 at 393.81 s (line 49555) after both read 3955-4095 with GP28 = 1 from 391.05 s (pitch down to −58°); the other fault 8 (374.09 s) followed a 10 s telemetry gap. |
| 2026-11-20 | S3-M12 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Track run 3, not the defined steps: raw values from the 5 Hz telemetry while line tracking at 20% (174 samples, 42.5-2749.7 s; 13 with both line IRs ≥ 3700 left out). Floor: GP27 min 183, median 233; GP26 min 187, median 221 (95% of floor samples ≤ 281 and ≤ 333). On the line with the other sensor on the floor: GP27 up to 3583 (median 2975, 9 samples), GP26 up to 3523 (median 3258, 14 samples). GP28 read 0 in 112 of 116 samples with both line IRs below 1000. Ramp and lux not measured; the floor surface is not logged. |
| 2026-11-20 | S3-M14 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | Line tracking on the track at 20% demand, not a 1 m straight at 30%; no video; S and W not measured. 11 tracking stretches, longest 17.8 s (2587.7-2605.4 s). No fault 2 in the log (fault codes 0, 1 and 7 only). Stretches ended by obstacle acquisition (2), fault 7 (77.5 s), fault 1 Tilt (1994.8, 2039.0, 2392.0, 2489.2 s) and STOP (4). 21 pivots seen; one PIVOT_LEFT was in 5 consecutive snapshots (seq 214-218, 800 ms from first to last, 2595.7-2598.4 s), 0.7 s after a "Hump cleared" event; whether that part of the track is straight is not logged. |
| 2026-11-20 | S3-M15 | `662c1ef` | Ron Tan | INCOMPLETE | `../system/logs/2026-11-20_track_run3_mqtt.txt` | No disarmed level lift of 50-100 mm held 5 s; lift height not logged. On the line while tracking, with the other sensor on the floor: GP27 up to 3583 (2595.00 s, line 108697), GP26 up to 3523 (2593.15 s), below 3700. Both line IRs dark at once (GP27/GP26): 3406/3366 with GP28 = 0 (47.66 s). All 13 tracking samples with both line IRs ≥ 3700 had GP28 = 1 (3737-4095); these stretches ended with STOP (94.0 s) or fault 1 Tilt (1994.8, 2039.0, 2392.0 s); no fault 8 in the log. |

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