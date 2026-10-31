# Team and subsystems

The tables are pre-filled with the **example** in `app_program/` (marked
*example*). Replace those rows with your own as you replace the example code.

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
|---|---|---|---|
| UART1 | | | |
| I2C0 | | | |
| I2C1 | S4 | `s4_sensor_task` | GP2 SDA, GP3 SCL |
| SPI0 | | | |
| SPI1 | | | |
| ADC | S3 (optional; disabled pending verified analogue wiring) | `s3_adc_worker` (priority 17, core 0) | GP26-GP28 capability only; no pin claim; GP29 reserved for radio |
| PWM slices 4/5 | S2 | motor task | GP8-GP11 |
| IO_BANK0 IRQ 13, core 0 | S2 (when encoders enabled) | encoder interrupt handler | GP4/5, GP16/17 |
| WiFi radio, TCP/IP, MQTT | *kernel* | `cyw43_utk_task` (built in) | GP23–25, GP29 |

### WiFi and MQTT are special

The radio and the network stack belong to the kernel's built-in WiFi service
task, `cyw43_utk_task` (`lib/libwifi/sysdepend/pico_rp2040/`). **No student
task may call `cyw43_...`, lwIP or MQTT functions, not even from a
"WiFi subsystem".** The calls are not safe from any other task, on either core.

What students *can* do:

- **Read the connection state** from any task, which is always safe:
  ```c
  #include "cyw43_utk.h"
  T_CYW43_UTK_STATUS st;
  cyw43_utk_get_status(&st);   /* st.ready, st.link_status, st.ip_addr, ... */
  ```
- **Send or receive data over the network** by adding code that *runs
  inside the service task*. The MQTT client is in
  `lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c`. It publishes a heartbeat
  today. A "cloud / reporting" subsystem would add a kernel object
  (e.g. a message buffer) that application tasks fill. The MQTT code then
  reads it with `TMO_POL` (never blocking) and publishes it.
  Because this changes shared library code, **agree it with the team and
  the instructor first**, and record it as an interface in section 5.
  It needs the `NET=lwip MQTT=1` build (README section 3).

## 4. Task priorities

1 is the highest. 1–4 are used by the kernel (USB console 3, WiFi 4). Each subsystem gets its own band by default;
change it only by team agreement. Write down every task you create.

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
