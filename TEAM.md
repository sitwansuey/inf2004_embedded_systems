# CSC02 subsystem ownership and integrated interfaces

The CR1 integration retains micro T-Kernel 3. Subsystem ownership and checked
interfaces below describe the integrated candidate. Physical validation and
whole-team CR1 acceptance remain separate.

| Owner | Responsibility | Task priority and core | Hardware |
|---|---|---|---|
| Vehicle Controller | sole motion authority; safety, leases, line/terrain/scan handoffs | 5, core 0 | no direct motor/peripheral writes |
| Watchdog supervisor | heartbeat check, RP2040 watchdog feed, crash record | 100 ms cyclic handler/core 1 | watchdog block (SCRATCH0..3 only) |
| Buddy 1 (2500238) | binary telemetry, copied command/telemetry buffers, dashboard | telemetry 13/core 1; LED 12/any | LED GP19 |
| Buddy 2 (2500613) | PWM/PID output owner, watchdog, encoder IRQ and status | motor 14/core 0; GPIO IRQ 13/core 0 | motor GP8-11 (PWM 4/5); encoder GP16/17 and GP4/5 |
| Buddy 3 (2500383) | line/barcode sampler and bounded navigation events | sampler 16/core 0; optional ADC worker 17/core 0 | user confirms GP27 left line, GP26 right line, GP28 barcode; calibrated ADC line (GP26/27) and digital GP28 barcode in use |
| Buddy 4 (2500243) | GY-511 acquisition, protected snapshots, terrain algorithm | sensor 18/core 1; monitor 19/any; pure terrain processing called by controller | I2C1 GP2/3 |
| Buddy 5 (2502324) | ultrasonic/servo owner, stationary edge acquisition, geometry | scan 20/core 1 | trigger GP1, echo GP0 (existing Buddy 5 wiring), servo GP12/PWM slice 6 |
| Kernel services | USB and CYW43/lwIP/MQTT ownership | USB 3/core 0; network 4/core 0 | radio GP23/24/25/29; USB controller |

GP13 cannot share PWM slice 6 with another owner. GP0/1 are ultrasonic pins
only on USB builds with CRASH_REPORT=0. UART/crash builds disable Buddy 5 and
therefore cannot arm the integrated robot. GP23/24/25/29 remain radio reserved.
No other task calls the SDK I2C driver, ultrasonic/servo driver, PWM output or
MQTT API. USB input is delivered by a bounded hook inside the existing USB owner.

All objects are initialized before tasks start. Any startup failure prevents
the controller from starting. The firmware boots DISARMED;
`app_program/robot_config.h` holds the measured profile (IR mode 3,
odometry/terrain calibration, PID and obstacle bypass). The motor task alone
handles the urgent STOP epoch, discards stale queued movement, polls every
~20 ms and checks its independent 1000 ms renewal watchdog. It never returns:
a status copy that misses the shared lock is retried on the next poll, and a
motor/encoder setup failure keeps PWM off with FAULT published (VC fault 7
once armed). Physical stopping
time still requires measurement; zero PWM is not guaranteed immediate braking.

Timing: the kernel tick is 10 ms and every relative wait gets one extra tick,
so the VC, motor, telemetry and network loops (which ask for 10 ms, or 2 ms
for the network poll) really run about every 20 ms; tunables counted in VC
ticks assume 20 ms. The S3 and S4 cyclic samplers run at exactly 10 ms.

Watchdog (checkpoint 8c): the VC, motor, network, telemetry and S4 IMU loops
each add one to their own heartbeat counter per loop
(`include/tm/robot_health.h`; one writer each). `app_program/app_watchdog.c`
checks them every 100 ms from a core-1 cyclic handler and feeds the RP2040
watchdog (1500 ms) only while all advanced within 1000 ms. It arms 20 s after
boot at the earliest, once every loop has run and the network task has
finished its first join. A stale loop is written to watchdog SCRATCH0..3 and
the board reboots. panic(), hard_assertion_failure(), lwip_utk_assert() and
the HardFault handler write the same record and reboot at once. The next
boot prints `[RESET] ...` on USB. A new monitored loop must beat at least
every 1000 ms, including inside long waits. S5 and USB are not monitored.
The record names the highest-priority stale loop on its core, using the
priority table in `app_program/shared/watchdog_policy.c`; keep it in step
if a task priority changes.

Stacks: the VC task has 4096 bytes and telemetry 3072 (`app_start_task_stack`);
other application tasks keep `TASK_STACK_SIZE` (2048). The kernel adds 128
bytes to each, and interrupts run on the interrupted task's stack.

| Interface | Contract |
|---|---|
| VC -> S2 | controller-bound mailbox, validated 3-bit command/7-bit power, issued time and STOP epoch; movement renewed every controller iteration |
| S2 -> VC/S1 | bounded protected commanded-status and consistent encoder snapshots; operating-clock timestamps; measured x1 scale |
| S2 -> S4 in VC | exact 13-byte internal encoder snapshot; checked validity, age, IMU skew, count/time wrap and interval continuity; no native struct fusion |
| VC <-> S3 | single controller binding, modes/100 ms lease, fresh snapshot (<30 ms), bounded 14-byte events (<100 ms), explicit marker acknowledgement |
| S4 -> VC/S1 | 71-byte protected acquisition record with monotonic impact/fault counters; per-channel usability/age decoded locally; 12-byte checked terrain result in shared app_state; acquisition remains sole I2C owner |
| VC -> S5 | controller-bound navigation mode/direction, scan ID, old reading invalidated until servo settles |
| S5 -> VC/S1 | copied status, matching actual/requested sample angle, freshness, static scan geometry and bounds |
| VC -> S1 | protected compact controller/terrain snapshot; bounded robot-event records (s1_event_post, 160-byte message buffer (20 two-byte records including kernel overhead), never blocking; a full queue drops and counts); no network calls |
| S1 -> network | two-entry copied binary inbox, newest first with old drops counted and flags preserved; robot events taken one per poll (s1_event_take) on Robopico/events/edr16/v1; link events held separately and sent first; nonblocking bounded polling on the existing network service |
| network/USB -> VC | copied commands, urgent STOP outside queue, 100 ms queue age; timed ARM for benches or one-shot autonomous START |

The obsolete sample producer was removed in published main PR #6 and remains
absent here. Retired sample interfaces are not used as sensor telemetry.
Controller snapshots are published with a nonblocking priority-inheritance
mutex. Peripheral ownership is unchanged.

Kernel limits remain those in config/config.h. Application usage is 8 tasks
(9 with the ADC worker), four application mutexes, one semaphore, one mailbox,
one fixed pool, five message buffers (seven with ADC; CNF_MAX_MBFID is 12) and three cyclic handlers,
plus the kernel USB/network/device service objects. No new scheduler, MPU,
resource-sharing redesign or CR2/CR3 feature was introduced.

The 55-byte mailbox snapshot is internal only. MQTT uses two-byte EDR16
records, including ID 0 mission status; the retired envelope/aggregate is not
accepted by the dashboard. See docs/DATA_REPRESENTATION.md and
docs/MISSION_SETUP.md.

GP22 is retained for the explicitly selected S5 acquisition diagnostic buzzer;
it is not used by the default or passive-test profiles.

The VC tolerates three consecutive motor-status copy misses; a published
FAULT still stops at once. An odometry gap holds stopped for at most 200 ms.
S3 lock timeouts retry within the existing 100 ms lease. Navigation survives
a later sampler latch. A marker held with no pending turn is re-armed after
100 ms. Fault-cut reads of at least three elements count as failed attempts
(`BOARD_BARCODE_FAULT_FAILS 1`); short partials get USB logs only.

S4 bridges up to 60 ms of unusable terrain input and keeps completed hump
height separately. Its counters feed VC shock and IMU-fault counts. The
armed lift detector never waives tilt. S5 retains its plan after READY, allows
one free initial braking restart, and checks the side corridor before READY.
The VC also ray-checks a shallower reverse replan and waits for forward data
after pans. Peripheral ownership and all STOP/lease checks stay in force.

With `BOARD_TIMING_STATS 1`, a USB `[TIMING]` line every 10 s gives maximum
run time, release gap and late/run counts for VC, motor and network. A gap
over 25 ms counts as late. The motor measurement excludes direction dead time.
