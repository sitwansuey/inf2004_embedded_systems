# Team and subsystems (CSC02)

This is the single place that says **who owns what** (pins, peripherals,
priorities, kernel objects and the interfaces between subsystems), so that five
people can work in one firmware image without clashing. Keep it up to date.

A **subsystem** is one student's part of the robot, with its own folder. It may
contain several tasks and own several peripherals (section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Telemetry: binary EDR16 snapshots and robot events to MQTT, MQTT/USB commands, the dashboard | 2500238 | ronbotan (Ron Tan) | `app_program/subsystem1/`, `app_program/external_systems/Dashboard/` | `testing/subsystem1/`, `app_program/external_systems/Dashboard/test_*.py` |
| 2 | Motion control: motor PWM, PID wheel-speed control, wheel encoders and odometry input | 2500613 | jordecoder (Jordan Goh) | `app_program/subsystem2/` | `testing/subsystem2/`, `app_program/subsystem2/tests/host/` |
| 3 | IR line following (two straddle sensors) and Code 39 barcode reading | 2500383 | youzerwan (Jun Kang) | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | GY-511 IMU: tilt, impacts and terrain (humps and their height) | 2500243 | vojm (Joshua Tan) | `app_program/subsystem4/` | `testing/subsystem4/`, `app_program/subsystem4/tests/host/` |
| 5 | Ultrasonic ranger on a servo: obstacle detection, edge scan and width | 2502324 | ncht88 (Terrence Ng) | `app_program/subsystem5/` | `testing/subsystem5/` |

**Shared integration code** belongs to the whole team:

- the Vehicle Controller (VC): `app_program/vehicle.c`, `vehicle_policy.c`, the
  `vehicle_*.h` helpers, `ir_line_trace.c/.h`, `app_state.c/.h`, `app_logging.c/.h`;
- the watchdog supervisor: `app_watchdog.c/.h`, `shared/watchdog_policy.c`,
  `include/tm/robot_health.h`, `lib/libtm/sysdepend/pico_rp2040/robot_health.c`;
- the robot profile: `robot_config.h`, `robot_ir_config.h`,
  `robot_ir_calibration.h`, `board_pins.h`, `robot_config.local.example.h`;
- `app_program/shared/`, `app_main.c`, `app.h`, `test_report.h` and this file;
- every change to the template's kernel, library, driver, configuration and
  build files (section 5, *Changes to shared template files*).

**Who owns the VC, and why it is not in a subsystem folder.** The VC is the
one task that commands motion in the mission build. It uses the interfaces of
all five subsystems (section 5), so it is no single student's part of the
robot. It has no individual owner: the five members own it together, like the
template's `app_main.c` and `app.h`, and it sits beside them at the top of
`app_program/` rather than in a `subsystemN/` folder, where the section 1 rule
would make one student the only person allowed to change it. Any member may
propose a change to shared code, by a pull request; `.github/CODEOWNERS`
requests a review from all five members for every file outside the subsystem
and dashboard folders. The data formats between the VC and each subsystem
are agreed with that subsystem's owner (section 5, *Agreed by*).

Shared code is changed only by agreement of the team; claiming an empty row in
this file (a pin, a peripheral, an interface ID) is the exception. It is tested
in `testing/integration/` and `testing/system/`.

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it. Two
subsystems on one pin build without any error but fail on the bench, and two
outputs driving one pin can damage it. Only the *reserved* rows are fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio (CYW43439: power, PIO-SPI data, chip select, clock) |
| GP0 | S5 | Ultrasonic ECHO input. With `CRASH_REPORT=1` or `CONSOLE=uart` it is UART0 TX instead and S5 is not built. |
| GP1 | S5 | Ultrasonic TRIG output (10 µs pulse). With `CRASH_REPORT=1` or `CONSOLE=uart` it is UART0 RX instead. |
| GP2 | S4 | I2C1 SDA to the GY-511 (100 kHz, internal pull-up) |
| GP3 | S4 | I2C1 SCL to the GY-511 |
| GP4 | S2 | Right-wheel (motor 2) encoder channel A, input, both-edge interrupt |
| GP5 | S2 | Right-wheel (motor 2) encoder channel B, input, both-edge interrupt |
| GP6 | | free (PWM slice 3 A: shares slice 3 with the GP22 diagnostic buzzer) |
| GP7 | | free (PWM slice 3 B, same note) |
| GP8 | S2 | Left motor (motor 1) driver input A, PWM slice 4 A (20 kHz) |
| GP9 | S2 | Left motor driver input B, PWM slice 4 B |
| GP10 | S2 | Right motor (motor 2) driver input A, PWM slice 5 A (20 kHz) |
| GP11 | S2 | Right motor driver input B, PWM slice 5 B |
| GP12 | S5 | Ranger servo, PWM slice 6 A (50 Hz, 750–2250 µs pulse) |
| GP13 | | free, but never as PWM: it is slice 6 B, the servo's slice |
| GP14 | | free (PWM slice 7 A) |
| GP15 | | free (PWM slice 7 B) |
| GP16 | S2 | Left-wheel (motor 1) encoder channel A, input, both-edge interrupt |
| GP17 | S2 | Left-wheel (motor 1) encoder channel B, input, both-edge interrupt |
| GP18 | | free (PWM slice 1 A) |
| GP19 | S1 | Heartbeat LED (LED + 330 Ω to GND), toggled every 250 ms |
| GP20 | | free (PWM slice 2 A) |
| GP21 | | free (PWM slice 2 B) |
| GP22 | S5 | Diagnostic buzzer (PWM slice 3 A, 2 kHz), only in the S5 acquisition diagnostics (`TEST=1` with `APP_ACQUISITION_TEST`, `APP_ACQUISITION_LEFT_TEST` or `APP_S5_MASTER_TEST`); free in the mission build |
| GP26 | S3 | Right straddle line sensor, analog input (ADC channel 0) |
| GP27 | S3 | Left straddle line sensor, analog input (ADC channel 1) |
| GP28 | S3 | Barcode sensor module DO pin, digital input (high = black) |

## 3. Peripherals

**Each peripheral is driven by exactly one task, its *owning task*.** Every
other task asks the owning task through a kernel object or the owner's API
(section 5) and never touches the peripheral itself: this port runs tasks on
both cores at once, and the drivers have no locking of their own. A driver is
code inside the subsystem that needs the hardware (`drv_<name>.c`), not a
subsystem of its own.

| Peripheral | Owner subsystem | Owning task | Pins |
|---|---|---|---|
| UART0 | *kernel* (crash report / UART console) | none in the mission build; `tm_printf` writes it only when built with `CRASH_REPORT=1` or `CONSOLE=uart` | GP0, GP1 (S5 otherwise) |
| UART1 | | free | |
| I2C0 | | free | |
| I2C1 | S4 | `s4_sensor_task` (18, core 1) | GP2, GP3 |
| SPI0 | | free | |
| SPI1 | | free | |
| ADC | S3 | `s3_adc_worker` (17, core 0), through the kernel ADC driver `adca` | GP26 (ch 0), GP27 (ch 1) |
| PWM slices 4, 5 | S2 | `motor` (14, core 0) | GP8–GP11 |
| PWM slice 6 | S5 | `obstacle scan` (20, core 1) | GP12 (GP13 must not be PWM) |
| PWM slice 3 | S5, acquisition diagnostic builds only | S5 diagnostic task (20, core 1) | GP22 |
| PWM slices 1, 2, 7 | | free (slice 1 only on GP18; slice 2 on GP20/GP21; slice 7 on GP14/GP15) | |
| PWM slice 0 | | no free pin (GP0/GP1 and GP16/GP17 are taken); usable only as an internal counter | |
| GPIO interrupt (IO_IRQ_BANK0) | S2 | set up by `motor` (14, core 0); the encoder interrupt runs on core 0 | GP4, GP5, GP16, GP17 |
| Watchdog block | shared (watchdog supervisor) | cyclic handler `cyc_watchdog` (100 ms, core 1) feeds it and writes SCRATCH0–3 only (SCRATCH4–7 belong to the bootrom). The kernel sets its 1 MHz tick at boot; S4 (`drv_pico_i2c.c`) and S5 (`s5_hardware.c`) write the same tick value again at start-up. | — |
| TIMER (1 MHz free-running) | *kernel* (started at boot) | shared, read only: VC and S2 (timing statistics), S4 (I2C timeouts), S5 (echo timing), the network task. S4 and S5 also release its reset again at start-up. | — |
| USB controller | *kernel* | `usb_service_task` (3, core 0) | USB |
| WiFi radio, TCP/IP, MQTT (with PIO and DMA) | *kernel* | `cyw43_utk_task` (4, core 0) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's WiFi service task,
`cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). No subsystem task
calls `cyw43_...`, lwIP or MQTT functions. Any task may read the connection
state with `cyw43_utk_get_status()`.

S1 sends and receives over MQTT by code that **runs inside the service task**
(`lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`), which only polls S1's kernel
objects and the shared `app_state` mutex with `TMO_POL` and never blocks
(interfaces IF-15, IF-16, IF-17 and IF-19; IF-18 is the same kind of hook
inside the USB service task). This changes the shared network library; the
team's agreement is in the *Agreed by* column of those rows. It is built with
`NET=lwip MQTT=1`.

## 4. Task priorities

1 is the highest; 1–4 are used by the kernel. Each subsystem keeps its own
band; change it only by team agreement. Core: `TP_PRC1` = core 0, `TP_PRC2` =
core 1, or "any".

| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| kernel | 1–4 | initial task – 1 – any; `usb_service_task` – 3 – core 0; `cyw43_utk_task` – 4 – core 0 |
| shared (VC) | 5 | `vehicle` – 5 – core 0 (in the mission build the only task that commands motion, so it runs above every subsystem task) |
| 1 | 12–13 | `led_task` – 12 – any; `telemetry` – 13 – core 1 |
| 2 | 14–15 | `motor` – 14 – core 0 |
| 3 | 16–17 | `consumer_task` (line and barcode sampler) – 16 – core 0; `s3_adc_worker` – 17 – core 0 |
| 4 | 18–19 | `s4_sensor_task` – 18 – core 1; `s4_monitor_task` – 19 – any |
| 5 | 20–21 | `obstacle scan` – 20 – core 1 (the S5 diagnostic task in the S5 test builds) |

Cyclic handlers: S1 LED (250 ms, any core), S3 sampler (10 ms, core 0), S4
sensor (10 ms, core 1), watchdog supervisor (100 ms, core 1).

## 5. Interfaces between subsystems

Every arrow between two subsystems (and the shared VC) is listed here. The
**provider** creates the kernel object or owns the API and declares it in a
header in its own folder (`subsystemN.h`; S1 also uses `control.h` and
`network_bridge.h`, S2 `s2_data.h`; `s1_event_post()` is declared in
`shared/events.h`). The **user** only sends to it, waits on it or calls it.
Every interface except IF-05 and IF-22 has at least one integration test
(`testing/integration/TEST_LOG.md`, which lists the same IF IDs); for IF-18 the
tests cover the command frames but not the USB hook itself. *Agreed by* names the owners on both sides who agreed the data format; *team (VC)* is the shared VC side, agreed by all members.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by | Tested by |
|---|---|---|---|---|---|---|
| IF-01 | VC → S2 | mailbox + fixed memory pool (owned by S2) | `h_movement_mailbox`, `h_movement_pool` (8 blocks); `s2_controller_bind()`, `s2_movement_send()`, `s2_follow_send()`, `s2_reverse_follow_send()` | 16-byte `s2_movement_msg_t`: command 1–6 and power 1–100 % (a STOP or zero power goes through IF-02 and is never queued), steering −400..400 (follow sends only), issue time, STOP epoch. Only the bound VC task may send. Renewed every VC tick; after 1000 ms without one, S2 switches PWM off and publishes FAULT, which the VC turns into fault 7. | Jordan Goh (S2), team (VC) | INT-01, INT-13, INT-14, INT-15, INT-17, INT-18, INT-32 |
| IF-02 | VC → S2 | shared 32-bit epoch, atomic (owned by S2) | `stop_epoch`; `s2_emergency_stop()` | Urgent STOP, never queued. The bound VC bumps the epoch; the motor task switches PWM off on its next poll (about 20 ms at most) or before its next PWM write, and drops movements queued under an older epoch. | Jordan Goh (S2), team (VC) | INT-01, INT-11, INT-30 (VC side; the S2 side is in the S2 host tests) |
| IF-03 | S2 → VC, S1 | shared status under a semaphore (owned by S2) | `h_status_lock`; `s2_motion_get_status()` | `s2_motion_status_t`: commanded motion, power, state (STOPPED/MOVING/FAULT), last error and command time. | Jordan Goh (S2), Ron Tan (S1), team (VC) | INT-11, INT-32, INT-10 |
| IF-04 | S2 → VC (→ S4 terrain) | function API, lock-free sequence counter (owned by S2) | `s2_encoder_snapshot()` | Exact 13-byte encoder snapshot: left and right forward counts, sample time, flags; the VC checks age, wrap and IMU skew. | Jordan Goh (S2), Joshua Tan (S4), team (VC) | INT-02, INT-11 |
| IF-05 | S2 → S3 | function API, lock-free sequence counter (owned by S2) | `s2_encoder_get_status()` | Wheel travel for measuring barcode bars (half-counts, about 0.17 mm each). | Jordan Goh (S2), Jun Kang (S3) | none yet (the S3 unit tests feed positions directly) |
| IF-06 | S2 → S5 | function API, lock-free sequence counter (owned by S2) | `s2_encoder_get_status()` | Encoder counts before and after each echo (motion during a scan sample). | Jordan Goh (S2), Terrence Ng (S5) | INT-08, INT-26 |
| IF-07 | VC → S3 | function API with a priority-inheritance mutex (owned by S3) | `mtx_stats`; `s3_controller_bind()`, `s3_controller_control()`, `s3_controller_renew()`, `s3_controller_set_map()`, `s3_controller_barcode_end()` | Mode PAUSED / TRACK / OVERRIDE; TRACK needs a lease (the VC uses 100 ms and renews it every tick); marker clear after a decoded turn; MAP letter (1–26) → action (0–4) for future decodes; barcode end once the VC has measured 30 mm without a GP28 edge. | Jun Kang (S3), team (VC) | INT-22, INT-23, INT-29, INT-31 |
| IF-08 | S3 → VC, S1 | shared status under a priority-inheritance mutex (owned by S3) | `mtx_stats`; `s3_status_get()` | `s3_status_t`: line result, raw IR values, barcode state and elements, decoded letter; a sample older than 30 ms is not used. | Jun Kang (S3), Ron Tan (S1), team (VC) | INT-15, INT-16, INT-17, INT-18, INT-20, INT-23, INT-24, INT-29, INT-34 |
| IF-09 | S3 → VC | message buffer (owned by S3) | `h_events`; `s3_event_receive()` | 14-byte events (navigation, stop request, fault, recovery request): kind, turn command, letter, sequence, time; at most 4 queued; delivered only from the current generation and when younger than 100 ms. The VC acknowledges each decoded marker (IF-07 marker clear). | Jun Kang (S3), team (VC) | INT-06, INT-22, INT-23, INT-29 |
| IF-10 | S4 → VC, S1 | shared record under a priority-inheritance mutex (owned by S4) | `status_lock`; `s4_sensor_get_record()` | 71-byte acquisition record: accelerometer and magnetometer, usable flags, impact and fault counters. The VC runs S4's terrain algorithm on it (hump, crest, height). | Joshua Tan (S4), Ron Tan (S1), team (VC) | INT-03, INT-11, INT-33, INT-10, INT-34 |
| IF-11 | VC → S5 | function API with a priority-inheritance mutex (owned by S5) | `status_mutex`; `s5_controller_bind()`, `s5_navigation_set()` | Scan id, navigation mode (MONITOR, APPROACH, OVERCOME, REORG) and look angle −90..90°; an old reading is invalid until the servo settles. | Terrence Ng (S5), team (VC) | INT-09, INT-30 |
| IF-12 | S5 → VC, S1 | shared packed record under a priority-inheritance mutex (owned by S5) | `status_mutex`; `s5_get_status()`; `s5_width_reported()` | 92-byte status: scan state, echo, actual and requested angles, scan id, distance, edge bounds, width and bypass plan; `s5_width_reported()` gives the beam-corrected width shown to the user. | Terrence Ng (S5), Ron Tan (S1), team (VC) | INT-04, INT-08, INT-09, INT-10, INT-12, INT-15, INT-16, INT-19, INT-26, INT-27, INT-28, INT-30 |
| IF-13 | VC → S1 | shared snapshot under a priority-inheritance mutex (shared code) | `app_state_publish()`, `app_get_state()` | `app_state_t`: VC phase, fault and cause, motion, terrain result, line and barcode fields for telemetry. | Ron Tan (S1), team (VC) | INT-05, INT-10, INT-21, INT-32, INT-34 |
| IF-14 | VC → S1 | message buffer, never blocks (owned by S1) | `event_buffer` (160 bytes); `s1_event_post()` | 2-byte robot events (barcode attempt, instruction, hump, obstacle); a full queue drops and counts. | Ron Tan (S1), team (VC) | INT-06, INT-07, INT-33 |
| IF-15 | S1 → dashboard (through the MQTT broker) | message buffer, polled by the network task (owned by S1) | `telemetry_buffer` (2 snapshots); `s1_telemetry_take()` | 55-byte snapshot, published as 33 two-byte EDR16 records (`docs/DATA_REPRESENTATION.md`): every value the dashboard shows from S1-S5 and the VC (IF-03, IF-08, IF-10, IF-12, IF-13, IF-21). | Ron Tan (S1), team | INT-05, INT-06, INT-07, INT-10, INT-21 |
| IF-16 | S1 → dashboard (through the MQTT broker) | message buffer, polled by the network task (owned by S1) | `event_buffer`; `s1_event_take()` | Robot events from IF-14, one per poll, published on the events topic after any pending command reply. Link events are not in this buffer: the MQTT port makes them on (re)connect, holds the latest one itself and sends it before the queued robot events. | Ron Tan (S1), team | INT-06, INT-07 |
| IF-17 | dashboard (through the MQTT broker) → VC | message buffer + STOP word (owned by S1) | `command_buffer` (4 deep); `s1_command_offer()`, `s1_command_take()` | 6-byte command frames: STOP, ARM (power, lease), RESET, START, MAP (letter → action). STOP bypasses the queue; a command 100 ms old or more is refused. | Ron Tan (S1), team | INT-01, INT-06, INT-07, INT-22, INT-31 |
| IF-18 | USB console → VC | hook inside the USB service task (owned by S1) | `app_usb_rx_byte()` → `s1_command_offer()` | Keys `s` (STOP), `a` (ARM 30 %, 500 ms lease), `r` (RESET), `g` (START 30 %) or raw 6-byte frames. | Ron Tan (S1), team (VC) | none for the hook itself (INT-31 replays the same frames into the VC; INT-06 tests `s1_command_offer()`) |
| IF-19 | VC, network task, S1 → dashboard (through the MQTT broker) | message buffer (owned by S1) | `ack_buffer`; `s1_command_ack()` (VC), `s1_command_reply()` (network task and S1, with phase and fault from `app_get_state()`), `s1_command_ack_take()` | 6-byte command replies: sequence, opcode, result (accepted, refused, RESET needed, stale), VC phase and fault. "RESET needed" comes only from the network task: after an MQTT (re)connect or an MQTT STOP, ARM and START are refused until a RESET has been passed to the VC's command queue. A full buffer drops the oldest reply and counts it. | Ron Tan (S1), team | INT-06, INT-07, INT-31 |
| IF-20 | VC, S2, S4, S1, network → watchdog | one heartbeat counter per loop, one writer each (shared code) | `health_heartbeat[]` (`include/tm/robot_health.h`); `cyc_watchdog` | Each monitored loop adds one per pass; a loop stale for 1000 ms writes a crash record to watchdog SCRATCH0–3 and the board reboots. The network task also flags its first join (`health_net_ready`). | all five members (team) | INT-25 |
| IF-21 | S2 → S1 | function API, lock-free sequence counter (owned by S2) | `s2_encoder_get_status()` | Left and right forward encoder counts for the telemetry snapshot (EDR16 IDs 1 and 2) and the encoder-available flag, shown on the dashboard. | Jordan Goh (S2), Ron Tan (S1) | INT-10 (S1 side, with S2 stubbed) |
| IF-22 | S2, network → VC | shared counters, one writer per record, no lock (shared code) | `app_task_timing[]` (`shared/task_timing.h`) | Loop timing (runs, late releases, longest run and gap) for the motor and network loops; the VC prints it as the USB `[TIMING]` line every 10 s. Only with `BOARD_TIMING_STATS 1`. | Jordan Goh (S2), team (VC) | none (USB diagnostic only) |

### Data to the dashboard (through S1)

Every value on the dashboard reaches it through S1 (Buddy 1): each owner's data
goes to S1 over the interface named here, and S1 publishes it over MQTT (IF-15,
IF-16).

| From | Data shown on the dashboard | Interface | Agreed by |
|---|---|---|---|
| S2 (Jordan Goh) | left and right wheel encoder counts; motor state, command and power | IF-21, IF-03 | Jordan Goh (S2), Ron Tan (S1) |
| S3 (Jun Kang) | raw line and barcode IR values; barcode bars read, result and decoded letter | IF-08 | Jun Kang (S3), Ron Tan (S1) |
| S4 (Joshua Tan) | acceleration and magnetic field, impact and IMU fault; pitch, roll, terrain state and hump height (worked out by the VC with S4's terrain code) | IF-10, IF-13 | Joshua Tan (S4), Ron Tan (S1) |
| S5 (Terrence Ng) | obstacle distance, servo angle, scan state, obstacle width and bypass side | IF-12 | Terrence Ng (S5), Ron Tan (S1) |
| VC (team) | mission phase, fault and cause; robot events (barcode, hump, obstacle) | IF-13, IF-14 | team (VC), Ron Tan (S1) |
| S1 (Ron Tan) | all of the above as EDR16 records on MQTT; command replies | IF-15, IF-16, IF-19 | Ron Tan (S1), team |

Interface numbers from before this table (the template's IF-03 S3 → S4 summary
and IF-05 LED task control) are retired. The comments in `subsystem1.h` and
`subsystem4/sensor_data.h` now say so. Still to update by their owners: the
comment on `tid_led` in `subsystem1/subsystem1.c` (still "IF-05") and S4-U11
in `testing/subsystem4/TEST_LOG.md` (still "IF-06"; the interface is IF-10).

### Changes to shared template files

The team imported the course template in commit `359b763` (2026-10-10). Every
file under `kernel/`, `lib/`, `include/`, `device/`, `config/` and
`build_make/` that differs from that import is listed here (compared with
`git diff 359b763`, ignoring line endings; `config/config.h` and the other
kernel configuration files are unchanged). These files are shared by the whole
team, and the template asks for MQTT changes to the shared network library to
be agreed "with the team and the instructor first". *Agreement* says what this
file records; "not recorded" means no agreement is written down here, not that
there was none. *Commits* are those that changed the file after the import
(`git log 359b763..HEAD -- <file>`).

| File(s) | What changed | Why | Needed by | Commits | Agreement |
|---|---|---|---|---|---|
| `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`, `mqtt_utk.h` | The template's heartbeat publish is replaced by S1's MQTT link: EDR16 telemetry, robot events and command replies are published, `csc02/command` is subscribed, the RESET gate and optional broker login are added, and `mqtt_utk_poll_telemetry()` and `mqtt_utk_link_lost()` are new. Includes `app_program/subsystem1/network_bridge.h`. | lwIP and MQTT may only be called from the network task (section 3), so S1's code that sends and receives runs inside it and polls S1's kernel objects. | S1 (and the dashboard) | `7c3d323`, `176587a`, `373ad6d`, `56d0d37`, `6764fd7`, `dd3e555`, `85cc435`, `921f76d`, `3ca4ecb`, `7a6c373`, `14b6c4f` | team: IF-15 to IF-17 and IF-19 rows; instructor: not recorded |
| `lib/liblwip/sysdepend/pico_rp2040/lwipopts.h` | `MQTT_OUTPUT_RINGBUF_SIZE` 2048 and `MQTT_REQ_MAX_IN_FLIGHT` 64 (lwIP defaults 256 and 4). | The defaults held only about 8 EDR16 publishes and capped telemetry near 30 messages a second. | S1 | `921f76d` | not recorded |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.c`, `cyw43_utk.h` | Calls `mqtt_utk_poll_telemetry()` every poll; turns Wi-Fi power save off after bring-up and every join; re-joins after an AP loss or a rejected join; adds the network watchdog heartbeat and `health_net_ready`; holds at boot after repeated crashes; adds loop timing. Includes `app_program/shared/task_timing.h` (and through it `robot_config.h`). | Power save added hundreds of ms to command replies; a lost association left the robot off the network until reboot; the watchdog must see the network loop. | S1 (MQTT), shared watchdog, S2 timing report | `373ad6d`, `7a6c373`, `14b6c4f`, `89f1bdc` | heartbeat: IF-20 row; timing: IF-22 row; power save and re-join: not recorded |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_compat.c` | Watchdog heartbeat inside the driver's delay hooks; the weak `panic()` and `hard_assertion_failure()` write the crash record and reboot instead of halting. | A slow but bounded driver call must not look like a frozen task; a halted core gave no message. | shared watchdog | `89f1bdc` | IF-20 row (heartbeat); kernel/library edit: not recorded |
| `lib/libwifi/sysdepend/pico_rp2040/cyw43_utk_kernel.c` | Comment only (real poll period). | — | — | `7a6c373` | — |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_console.c`, `usb_tinyusb_glue.c` | USB input is passed byte by byte (up to 32 per pass) to a weak `app_usb_rx_byte()` instead of being discarded; new `tm_tusb_read_char()`. | S1's USB command keys (IF-18). | S1 | `373ad6d` | team: IF-18 row; kernel/library edit: not recorded |
| `lib/libtm/sysdepend/pico_rp2040/usb/usb_sdk_compat.c`, `lib/liblwip/sysdepend/pico_rp2040/sys_arch.c` | `panic()`, `hard_assertion_failure()` and `lwip_utk_assert()` write the crash record and reboot through the watchdog instead of spinning. | A spin froze every lower-priority task on that core with no message. | shared watchdog | `89f1bdc` | not recorded |
| `include/tm/robot_health.h`, `lib/libtm/sysdepend/pico_rp2040/robot_health.c` (new) | Heartbeat counters, hardware watchdog and crash-record access. | It lives under `include/` and `lib/` so that the kernel's fault handler and the SDK/CYW43/lwIP ports can use it without including `app_program/` headers. | shared watchdog (IF-20) | `89f1bdc` | IF-20 row (all five members) |
| `kernel/sysdepend/cpu/core/armv6m/exc_hdr.c` | HardFault writes the crash record, starts a 500 ms watchdog and reboots (Pico only). | A halted core left the other core running with this core's tasks dead. | shared watchdog | `89f1bdc` | not recorded |
| `kernel/sysdepend/pico_rp2040/hw_setting.c` | Stops the watchdog at boot, starts its 1 MHz tick and releases the TIMER from reset. | Crash-record uptime and the fault handler's watchdog are valid from boot, not only after S4 or S5 start. | shared watchdog | `89f1bdc` | not recorded |
| `include/sys/inittask.h` | Initial task stack 4 KB in `TEST=1` images (1 KB otherwise). | The on-target self-test runs in the initial task. | `TEST=1` builds (on-target checks of S1 to S5) | `ed197b3`, `5f93cb6` | not recorded |
| `device/adc/sysdepend/rp2040/adc_cnf_sysdep.h` | Includes `app_program/robot_config.h` and enables the GP26–GP28 analogue pads from `BOARD_IR_MODE` (GP26/GP27 in modes 2 and 3, GP28 only in mode 2); conversion timeout 1000 → 5 ms. | The pads must match S3's wiring: in mode 3 GP28 is a digital input and must keep its digital function. The short timeout bounds each conversion wait well inside S3's 20 ms conversion deadline (`subsystem3/drv_ir_adc.h`). | S3 | `373ad6d`, `921f76d` | not recorded |
| `device/adc/sysdepend/rp2040/adc_rp2040.c` | The ADC ready waits give up after 100000 polls (`E_TMOUT`) instead of spinning for ever. | A stuck ADC must not hang S3's worker or the start-up. | S3 | `373ad6d` | not recorded |
| `config/wifi_secrets.example.h` | Adds `MQTT_USERNAME` and `MQTT_PASSWORD` (both `NULL`). | Optional broker login (S1). | S1 | `7c3d323`, `14b6c4f` | not recorded |
| `build_make/pico_rp2040.mk` | New options `S2_ONLY`, `S2_ENCODER_ENABLED`, `S2_COMMAND_TIMEOUT_MS`, `S4_ONLY` (checks, image-name suffixes, `-D` macros, profile stamp); `LNKFILE` moved to `include/etc/` (below). | S2 and S4 bench builds and S2's encoder and timeout settings. | S2, S4 | `7da5095`, `edee296`, `25589b3`, `373ad6d`, `58b89c2`, `5f0b8c7`, `ac7d949` | not recorded |
| `build_make/mtkernel_3/app_program/subdir.mk` | Source lists for `S2_ONLY=1` and `S4_ONLY=1`; includes `app_program/subsystem4/sdk.mk` in every other build; `-Wall -Wextra -Werror` for all `app_program` objects. | Bench builds; S4 needs SDK I2C code. | S2, S4, team | `7da5095`, `edee296`, `25589b3`, `e59a4fe`, `373ad6d`, `ac7d949`, `5f93cb6` | not recorded |
| `build_make/mtkernel_3/lib/libtm/sysdepend/pico_rp2040/subdir.mk` | Builds `robot_health.o`. | Watchdog support code. | shared watchdog | `89f1bdc` | not recorded |
| `build_make/makefile` | Links with `-lm`. | The VC and S4 terrain code use `<math.h>`. | VC, S4 | `373ad6d` | not recorded |
| `etc/linker/` → `include/etc/linker/` (all ten linker scripts, unchanged) | Moved. No commit message explains it: it first appears in the merge `211c6c8` ("Latest push", 2026-10-27), whose parents both still have `etc/`. `pico_rp2040.mk` was pointed at the new place in `ac7d949`; the five non-Pico board makefiles (`iote_*.mk`, `nucleo_h723.mk`) kept `../etc/` and could not find their scripts until their paths were updated on 2026-11-25. The Pico build does not use those five makefiles. No non-Pico board is built or tested in this repository (no CI job or test record). | — | none (Pico build unchanged) | `211c6c8`, `ac7d949` | not recorded |
| `app_program/subsystem4/sdk.mk` (included by the shared `subdir.mk`) | Adds `-DPICO_NO_HARDWARE=0 -DNDEBUG -ffunction-sections -fdata-sections`, the S4 SDK include paths and `-I` for the USB port to the global `CFLAGS`, and `--gc-sections` to `LFLAGS`, so they apply to the kernel and every subsystem, not only S4. Compiles SDK `i2c.c`, `timeout_helper.c` and `ticks.c` (plus `gpio.c` and `timer.c` when `WIFI=none`). Stops the build when `PICO_SDK_PATH` is missing, including `CONSOLE=uart WIFI=none`, which the template could build without the SDK. | S4 uses the SDK's I2C driver. | S4 (all builds) | `edee296`, `373ad6d`, `ac7d949` | not recorded |

`sdk.mk` is left as it is: limiting its flags to S4's objects, or building
without the SDK again, would change the kernel's and the other subsystems'
compiled code in every robot image. That is a firmware change the team would
have to agree and re-test on the robot. Every build of this robot, including
`CONSOLE=uart WIFI=none`, therefore needs `PICO_SDK_PATH`.

**Why `device/` and `lib/` include `app_program/` headers.** Three shared
files reach into the application: `adc_cnf_sysdep.h` includes
`robot_config.h` (the ADC pads follow `BOARD_IR_MODE`), `cyw43_utk.c` includes
`shared/task_timing.h` (loop timing, and `BOARD_WIFI_POWER_SAVE` from
`robot_config.h`), and `mqtt_utk.c` includes `subsystem1/network_bridge.h`
(S1's command frame and queue API). They use the robot's settings or S1's
queues from code that must run in the kernel's driver or network task. The
cost is that these kernel and library files now depend on application
headers: a change to `robot_config.h`, `task_timing.h` or `network_bridge.h`
also rebuilds and can change them, so such a change is reviewed as a
shared-file change.

## 6. Kernel object budget

Limits are set in `config/config.h`. Counts are for the mission build
(`NET=lwip MQTT=1`, IR mode 3). The kernel itself also uses 2 semaphores and 4
event flags: the ADC driver `adca` (1 semaphore, 1 event flag), the UART0
serial driver `sera` (1 semaphore, 2 event flags; registered in every build,
unused in the mission build) and the device manager's lock (1 event flag).
S3's ADC worker opens `adca` at run time, which adds 1 semaphore.

| Object | Limit | S1 | S2 | S3 | S4 | S5 | Shared (VC) | Kernel |
|---|---|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 2 | 2 | 1 | 1 | 3 |
| Semaphore | 16 | | 1 | | | | | 3 |
| Event flag | 16 | | | | | | | 4 |
| Mailbox | 12 | | 1 | | | | | |
| Mutex | 12 | | | 1 | 1 | 1 | 1 | |
| Message buffer | 12 | 4 | | 3 | | | | |
| Fixed memory pool | 12 | | 1 | | | | | |
| Variable memory pool | 12 | | | | | | | |
| Cyclic handler | 12 | 1 | | 1 | 1 | | 1 | |
| Alarm handler | 12 | | | | | | | |

S1's message buffers are `telemetry_buffer`, `command_buffer`, `ack_buffer` and
`event_buffer`; S3's are `h_events` and the ADC worker's request and reply
buffers. The tightest limit is message buffers, 7 of 12.

## 7. Integration notes

**Start-up.** Every interface object (S1's four message buffers and LED
cyclic, S2's mailbox, pool and semaphore, S3's mutex and event buffer, and the
S4, S5 and `app_state` mutexes) is created in the `subsystemN_init()` calls
before any application task starts. Created later: the S3 and S4 sampler
cyclic handlers (in their `_start`), the S3 ADC worker task and its two
message buffers (by the S3 task on its first pass), and the watchdog cyclic
handler (after the VC starts). A failed init or start call keeps the VC from
starting, and the watchdog is then not started either. The robot boots
DISARMED; `app_program/robot_config.h` holds the measured profile (IR mode 3,
odometry and terrain calibration, PID, obstacle bypass).

**Stopping.** The motor task alone handles the urgent STOP epoch, discards
stale queued movement, polls about every 20 ms and stops on its own 1000 ms
renewal watchdog. It never returns. A status copy that misses the lock is
retried on the next poll. A motor or encoder set-up failure keeps PWM off and
publishes FAULT (VC fault 7 once armed); the task keeps its heartbeat and
drops commands, so the watchdog does not reboot-loop. Zero PWM is not
guaranteed to brake at once; the physical stopping time still has to be
measured.

**Timing.** The kernel tick is 10 ms and every relative wait gets one extra
tick, so the VC, motor, telemetry and network loops (which ask for 10 ms, or
2 ms for the network poll) really run about every 20 ms; values counted in VC
ticks assume 20 ms. The S3 and S4 cyclic samplers run at exactly 10 ms. With
`BOARD_TIMING_STATS 1`, a USB `[TIMING]` line every 10 s gives the longest
run, the longest gap and the late count for the VC, motor and network loops (a
gap over 25 ms counts as late; the motor figure excludes direction dead time).

**Watchdog.** The VC, motor, network, telemetry and S4 sensor loops each add
one to their own heartbeat counter per pass (`include/tm/robot_health.h`).
`app_program/app_watchdog.c` checks them every 100 ms from a core-1 cyclic
handler and feeds the RP2040 watchdog (1500 ms) only while all of them moved
within 1000 ms. It arms 20 s after boot at the earliest, once every loop has
run and the network task has finished its first join. A stale loop is written
to watchdog SCRATCH0–3 and the board reboots at once (the 1500 ms timeout is
the backup). The record names one loop: on each core the highest-priority
stale loop, and of those the one stale longest; it also lists every loop
silent for over 500 ms. The priority and core table is in
`app_program/shared/watchdog_policy.c`; keep it in step if a task's priority
or core changes. The handler is released by the core-0 kernel tick, so a
core-0 interrupt-level hang or a stopped core 1 stops the supervisor itself;
the timeout then reboots the board and the next boot reports that the
supervisor stopped feeding. `panic()`, `hard_assertion_failure()`,
`lwip_utk_assert()` and the HardFault handler write the same record and reboot
at once. The next boot prints `[RESET] ...` on USB every 10 s for the first
60 s. After 3 crashes in a row, each within 20 s of boot, the board holds at
boot instead of starting (no tasks, no WiFi, motors off) and repeats the line
every 2 s; power-cycle to try again. A new monitored loop must beat at least
every 1000 ms, also inside long waits. Not monitored: S5, the USB task, the S3
sampler and ADC worker, the S4 monitor and the LED.

**Stacks.** The VC task has 4096 bytes and telemetry 3072; the other
application tasks have 2048 (`TASK_STACK_SIZE`). The initial task (`usermain`
and the `[RESET]`/`[WDT]` report loop) has 1 KB, or 4 KB in `TEST=1` images.
The kernel adds 128 bytes to each, and interrupts run on the interrupted
task's stack.

**Build profiles.** `make` with no options builds `NET=none MQTT=0` (no
dashboard link); the mission build is `NET=lwip MQTT=1`. The UART and
crash-report builds (`CONSOLE=uart`, `CRASH_REPORT=1`) need GP0/GP1 for UART0,
so S5 is not built and the integrated robot cannot drive in them: an ARM or START stops with fault 1 (sonar) within 1 s.
`S2_ONLY=1` and `S4_ONLY=1` are bench builds that compile and start only that
subsystem (no VC, S1, S3, S5 or watchdog). `TEST=1` adds the passive on-target checks (`subsystem1_test()` to
`subsystem5_test()` in the integrated image, `subsystem2_test()` in `S2_ONLY=1`,
`subsystem4_test()` in `S4_ONLY=1`), run by the initial task while the robot runs. The S5 test builds (`TEST=1`
with `APP_S5_MASTER_TEST`, `APP_ACQUISITION_TEST`, `APP_ACQUISITION_LEFT_TEST`
or `APP_S5_SERVO_TEST`) replace the S5 scan task with a diagnostic task (20,
core 1) and do not start the VC; that task binds to S2 and drives the motors
itself (the acquisition tests drive forward at 30 % to 15 cm), and all but the
servo test use GP22. A git-ignored `app_program/robot_config.local.h`
overrides any `robot_config.h` value, for example `BOARD_MOTORS_DISABLED 1`
for bench work.

**Robustness.** The VC tolerates three missed motor-status copies in a row; a
published FAULT still stops at once. An odometry gap holds the robot stopped
for at most 200 ms. S3 lock timeouts are retried within the 100 ms lease, and
a decoded marker held with no pending turn is re-armed after 100 ms. A decoded
turn is still delivered after a later sampler latch; other S3 events older
than 100 ms are dropped. A barcode capture of 3 or more elements cut short by
an S3 sampler fault counts as a failed read and starts the re-read
(`BOARD_BARCODE_FAULT_FAILS 1`); shorter partial captures are dropped. S4
bridges up to 60 ms of unusable terrain input and keeps a completed hump
height separately; its counters feed the VC's shock and IMU-fault counts. A
lift never waives the armed tilt check: both line sensors at 3700 or more with
GP28 black for 2 s stops the run with fault 8 (lifted). S5 keeps its plan after
READY, allows one free restart after its initial braking and checks the side
corridor before READY. The VC ray-checks a shallower heading after reversing
and waits for a forward reading after every pan. The MQTT wire format
(two-byte EDR16 records, status ID 0) is in `docs/DATA_REPRESENTATION.md`.
