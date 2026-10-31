# CSC02 subsystem ownership and integrated interfaces

The CR1 integration retains micro T-Kernel 3. Subsystem ownership and checked
interfaces below describe the integrated candidate. Physical validation and
whole-team CR1 acceptance remain separate.

<<<<<<< HEAD
Fill this in at the start of the project and keep it up to date. It is the
single place that says **who owns what**, so that five people can work in
one firmware image without clashing over pins, priorities, or kernel objects.

A **subsystem** is one student's part of the product, with its own folder.
It may contain several tasks and own several peripherals (see section 3).

Rule: **only the owner edits a subsystem's folders.** To change another
subsystem, ask its owner or open a pull request for them to review.

## 1. Members and subsystems

| # | Subsystem (what it does) | Student ID | GitHub username | Code | Tests |
|---|---|---|---|---|---|
| 1 | Buddy 1: shared EDR16 telemetry / MQTT / dashboard | 2500238 | | `app_program/subsystem1/` | `testing/data_representation/` |
| 2 | PWM motion control and dual-wheel encoder diagnostics | 2500613 | | `app_program/subsystem2/` | `app_program/subsystem2/tests/host/` |
| 3 | IR line following, barcode decoding, navigation events | 2500383 | | `app_program/subsystem3/` | `testing/subsystem3/` |
| 4 | Buddy 4: GY-511 / LSM303DLHC acquisition and sensor snapshots | | | `app_program/subsystem4/` | `testing/subsystem4/` |
| 5 | | | | `app_program/subsystem5/` | `testing/subsystem5/` |

Shared files (`app_program/app_main.c`, `app_program/app.h`, this file)
are changed only by agreement of the team. The exception is claiming an empty
row in this file (a pin, a peripheral, an interface ID), which anyone may do
(section 2).

## 2. Pins

One owner per pin. Claim a pin here **before** you wire or code it.

**Why:** two subsystems using the same pin is one of the most common and
hardest-to-find mistakes in a team project. The code builds without any
error, but the hardware misbehaves. Each subsystem can pass its own tests
and then fail once they are combined, and two outputs driving the same pin
against each other can damage it. Writing the claim down first means a clash
is spotted when someone edits this table, not hours later on the bench.

**No instructor check needed.** The team keeps this table itself. You do not
need the instructor, or the rest of the team, to check or approve a claim:
if the row is empty, fill it in and commit. If two people claim the same
pin, they agree between themselves who moves. Only the *reserved* rows are
fixed.

| Pin | Owner | Used for |
|---|---|---|
| GP23, GP24, GP25, GP29 | *reserved* | Pico W radio |
| GP0 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP1 | | (taken whenever the team builds with `CRASH_REPORT=1`) |
| GP2 | S4 | GY-511 I2C1 SDA, 3.3 V logic |
| GP3 | S4 | GY-511 I2C1 SCL, 3.3 V logic |
| GP4 | S2 | right wheel (motor 2) encoder A |
| GP5 | S2 | right wheel (motor 2) encoder B |
| GP6 | | |
| GP7 | | |
| GP8 | S2 | left wheel motor PWM A |
| GP9 | S2 | left wheel motor PWM B |
| GP10 | S2 | right wheel motor PWM A |
| GP11 | S2 | right wheel motor PWM B |
| GP12 | | |
| GP13 | | |
| GP14 | | |
| GP15 | | |
| GP16 | S2 | left wheel (motor 1) encoder A |
| GP17 | S2 | left wheel (motor 1) encoder B |
| GP18 | | |
| GP19 | *example:* S1 | external LED (LED + 330 Ω to GND) |
| GP20 | | |
| GP21 | | |
| GP22 | | |
| GP26 | | (ADC0 capable) |
| GP27 | | (ADC1 capable) |
| GP28 | | (ADC2 capable) |

## 3. Peripherals

**Subsystems, tasks and peripherals are three different things:**

| Term | What it is | Example |
|---|---|---|
| **Subsystem** | one student's part of the product, one folder | "environment sensing", "motor control", "user interface" |
| **Task** | one thread of execution. A subsystem may have **as many tasks as it needs** | a sampling task and a filtering task, both in "environment sensing" |
| **Peripheral** | one hardware block on the RP2040 | I2C0, UART1, the ADC |

A driver is **not** a subsystem. It is code *inside* the subsystem that
needs that hardware: the "environment sensing" subsystem contains the I2C
sensor code, and owns I2C0. Write it as `drv_<name>.c` / `drv_<name>.h` in
that subsystem's folder, never under `device/`, and call it only from the
peripheral's owning task. The kernel's own drivers in `device/` (serial,
I2C, ADC) may be used instead; switching one on in `config/config_device.h`
changes shared configuration, so record it in the table below first.

**The rule: each peripheral is driven by exactly one task, its *owning
task*.** That task can be in any subsystem, and one subsystem may own
several peripherals (one owning task each, or one task owning all of them).
Every other task, in the same subsystem or another one, asks the owning task
through a kernel object (section 5) and never touches that peripheral itself.

*Why:* this port runs tasks on both cores at the same time. If two tasks
drive the same I2C or UART block at once, their transfers get mixed up, and
the drivers have no locking to stop it.

| Peripheral | Owner subsystem | Owning task | Pins |
=======
| Owner | Responsibility | Task priority and core | Hardware |
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
|---|---|---|---|
| Vehicle Controller | sole motion authority; safety, leases, line/terrain/scan handoffs | 5, core 0 | no direct motor/peripheral writes |
| Buddy 1 (2500238) | binary telemetry, copied command/telemetry buffers, dashboard | telemetry 13/core 1; LED 12/any | LED GP19 |
| Buddy 2 (2500613) | PWM output owner, watchdog, encoder IRQ and status | motor 14/core 0; GPIO IRQ 13/core 0 | motor GP8-11 (PWM 4/5); encoder GP16/17 and GP4/5 |
| Buddy 3 (2500383) | line/barcode sampler and bounded navigation events | sampler 16/core 0; optional ADC worker 17/core 0 | user confirms GP27 left line, GP26 right line, GP28 barcode; raw bench works, calibrated control pending |
| Buddy 4 (2500243) | GY-511 acquisition, protected snapshots, terrain algorithm | sensor 18/core 1; monitor 19/any; pure terrain processing called by controller | I2C1 GP2/3 |
| Buddy 5 (2502324) | ultrasonic/servo owner, stationary edge acquisition, geometry | scan 20/core 1 | trigger GP1, echo GP0 (existing Buddy 5 wiring), servo GP12/PWM slice 6 |
| Kernel services | USB and CYW43/lwIP/MQTT ownership | USB 3/core 0; network 4/core 0 | radio GP23/24/25/29; USB controller |

GP13 cannot share PWM slice 6 with another owner. GP0/1 are ultrasonic pins
only on USB builds with CRASH_REPORT=0. UART/crash builds disable Buddy 5 and
therefore cannot arm the integrated robot. GP23/24/25/29 remain radio reserved.
No other task calls the SDK I2C driver, ultrasonic/servo driver, PWM output or
MQTT API. USB input is delivered by a bounded hook inside the existing USB owner.

All objects are initialized before tasks start. Any startup failure prevents
the controller from starting. Default configuration is DISARMED, with IR,
odometry/terrain calibration and autonomous bypass disabled. The motor task
alone handles the urgent STOP epoch, discards stale queued movement, polls at
10 ms and checks its independent 1000 ms renewal watchdog. Physical stopping
time still requires measurement; zero PWM is not guaranteed immediate braking.

| Interface | Contract |
|---|---|
| VC -> S2 | controller-bound mailbox, validated 3-bit command/7-bit power, issued time and STOP epoch; movement renewed every controller iteration |
| S2 -> VC/S1 | bounded protected commanded-status and consistent encoder snapshots; operating-clock timestamps; measured x1 scale |
| S2 -> S4 in VC | exact 13-byte EDR encoder payload; checked validity, age, IMU skew, count/time wrap and interval continuity; no native struct fusion |
| VC <-> S3 | single controller binding, modes/100 ms lease, fresh snapshot (<30 ms), bounded 13-byte events (<100 ms), explicit marker acknowledgement |
| S4 -> VC/S1 | 63-byte protected acquisition record; per-channel usability/age decoded locally; 12-byte checked terrain result in shared app_state; acquisition remains sole I2C owner |
| VC -> S5 | controller-bound navigation mode/direction, scan ID, old reading invalidated until servo settles |
| S5 -> VC/S1 | copied status, matching actual/requested sample angle, freshness, static scan geometry and bounds |
| VC -> S1 | protected compact controller/terrain snapshot; no network calls |
| S1 -> network | two-entry copied binary inbox; nonblocking bounded polling on the existing network service |
| network/USB -> VC | copied commands, urgent STOP outside queue, 100 ms queue age and expiring ARM lease |

The obsolete sample producer was removed in published main PR #6 and remains
absent here. Retired sample interfaces are not used as sensor telemetry.
The old `app_vehicle_request` accepts STOP only; it cannot bypass controller
policy with positive movement. Controller snapshots are published with a
nonblocking priority-inheritance mutex. Peripheral ownership is unchanged.

Kernel limits remain those in config/config.h. Application usage is 8 tasks
(9 with the ADC worker), four application mutexes, one semaphore, two mailboxes,
one fixed pool, four message buffers (six with ADC), one event flag and one
cyclic handler, plus the kernel USB/network/device service objects. No new
scheduler, MPU, resource-sharing redesign or CR2/CR3 feature was introduced.

27 October Buddy 4 continuation: see `docs/BUDDY4_IMPLEMENTATION.md` for current
integration provenance, owner boundaries, verification and limitations. The
EDR envelope and the aggregate 50-byte dashboard schema remain separate.
Buddy 1 interface review remains retrospective.
See docs/MAIN_RECONCILIATION_20261027.md for adopted S5 storage and retained safety
contracts, and docs/BUDDY1_RETROSPECTIVE_REVIEW.md for interface review.

<<<<<<< HEAD
| Subsystem | Priority band | Tasks (name – priority – core) |
|---|---|---|
| 1 | 12–13 | led_task – 12 – any; s1_sensor_telemetry_task – 13 – any (network builds) |
| 2 | 14–15 | motor_task – 14 – core 0 |
| 3 | 16–17 | `consumer_task` (S3 sampler) – 16 – core 0; `s3_adc_worker` – 17 – core 0 (optional) |
| 4 | 18–19 | `s4_sensor_task` – 18 – core 1; `monitor_task` – 19 – any |
| 5 | 20–21 | |

Core: `TP_PRC1` = core 0, `TP_PRC2` = core 1, or "any".

## 5. Interfaces between subsystems

Every arrow between two subsystems is listed here. The **provider** creates
the kernel object and publishes its ID in its own `subsystemN.h`; the **user** only
sends to it or waits on it. Agree on the data format before coding.

| ID | From → To | Mechanism | Object ID name | Data / meaning | Agreed by |
|---|---|---|---|---|---|
| IF-01 | S2 → S3 | mailbox (owned by S3) | `mbx_samples` | `SAMPLE_MSG` in `subsystem2.h` | *example* |
| IF-02 | S3 → S2 | semaphore + fixed memory pool (owned by S2) | `sem_slots`, `mpf_samples` | receiver returns the block, then 1 slot | *example* |
| IF-03 | S3 → S4 | message buffer + event flag (owned by S4) | `mbf_summary`, `flg_events` | text line ≤ `S4_SUMMARY_MAX` bytes, then `FLG_BATCH` | *example* |
| IF-04 | S4 → S3 | retired example | `app_stats`, `mtx_stats` | S4 no longer reads S3 demo statistics | retired in S4 adaptation |
| IF-05 | S4 → S1 | retired example | `tid_led` | S4 no longer suspends/resumes S1 | retired in S4 adaptation |
| IF-06 | B2–B5 → B1 → network service | nonblocking copied message buffer | `s1_telemetry_record`, `s1_telemetry_u12` | one EDR16 record: data[15:4], ID[3:0]; `shared/edr.h` | repository standard requested by project owner |
| IF-07 | B2 → B4/controller | local full-width snapshot getter | `s2_encoder_get_status()` | both 32-bit counts, timestamp and validity; use for local fusion | implementation present |
| IF-08 | All buddies → B1 | shared EDR16 codec | `edr_encode`, `edr_decode` | exactly two bytes per record; B2 adapters return two records each | see `docs/DATA_REPRESENTATION.md` |
| IF-09 | B4 → B1/controller | local mutex-protected snapshot | `s4_sensor_get_status()` | full raw/scaled samples, per-channel usability and ages | implementation present |

Each interface is tested by at least one integration test
(`testing/integration/TEST_LOG.md`).

## 6. Kernel object budget

Limits are set in `config/config.h`. If the team needs more, raise the
limit there. The numbers below include the network-enabled B1 collector.

| Object | Limit | S1 | S2 | S3 | S4 | S5 |
|---|---|---|---|---|---|---|
| Task | 32 | 2 | 1 | 1 | 2 | |
| Semaphore | 16 | | 1 | | | |
| Event flag | 16 | | | | 1 | |
| Mailbox | 12 | | | 1 | | |
| Mutex | 12 | | | 1 | 1 | |
| Message buffer | 12 | 1 | | | 1 | |
| Fixed memory pool | 12 | | 1 | | | |
| Variable memory pool | 12 | | | | | |
| Cyclic handler | 12 | 1 | | | | |
| Alarm handler | 12 | | | | | |

(The kernel's USB and WiFi service tasks also use a few of these.)
=======
GP22 is retained for the explicitly selected S5 acquisition diagnostic buzzer;
it is not used by the default or passive-test profiles.
>>>>>>> 54a371fcc23b8e6c657b60791f4818656bc0053e
