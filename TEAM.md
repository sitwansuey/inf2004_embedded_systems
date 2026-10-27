# CSC02 subsystem ownership and integrated interfaces

The local CR1 integration retains micro T-Kernel 3. The user authorized changes
across the five buddies' implementations and explicitly authorized publishing
and merging PR #8. Physical validation remains separate.

| Owner | Responsibility | Task priority and core | Hardware |
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
Publication/merge was authorized by the user; Buddy 1 review is retrospective.
See docs/MAIN_RECONCILIATION_20261027.md for adopted S5 storage and retained safety
contracts, and docs/BUDDY1_RETROSPECTIVE_REVIEW.md for interface review.

GP22 is retained for the explicitly selected S5 acquisition diagnostic buzzer;
it is not used by the default or passive-test profiles.
