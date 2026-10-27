# Buddy 4 implementation and ownership

## Current software baseline

Buddy 4 is integrated on `buddy4/cr1-integration` with main `58b89c2`.
Integration PR: https://github.com/sitwansuey/inf2004_embedded_systems/pull/9.
See MAIN_RECONCILIATION_20261027.md for the startup/controller conflict decisions.
This branch is a software baseline for testing, not final hardware acceptance.
It boots disarmed and motor-locked. Calibration and autonomous bypass remain
unset by default. No board was flashed or driven during this implementation.

## Sensor-to-controller path

1. The sensor task alone owns I2C1 on GP2 SDA / GP3 SCL. The GY-511 contains
   an LSM303DLHC accelerometer and magnetometer, not a gyroscope. Channel quality,
   clipping, overrun, freshness, absent devices and reinitialization are handled
   by the existing driver/acquisition path.
2. Authoritative acquisition storage and protected cross-task copies use a
   500-bit / 63-byte record. `s4_sensor_get_record()` copies under the bounded
   priority-inheritance mutex; `s4_data_snapshot()` creates a temporary decoded
   view after the consumer obtains current operating time. Wide diagnostic
   views are not the persistent cache or the binary transmission format.
3. The controller obtains Buddy 2's exact 13-byte encoder payload through
   `s2_encoder_encode()`. `s4_encoder_terrain_step()` checks API result, format,
   validity, age, IMU alignment, duplicates, count/time wrap and continuity.
   Both sources use `tk_get_otm()`. IMU microsecond-shaped timestamps are
   millisecond operating time multiplied by 1000, not microsecond precision.
4. Checked odometry converts counts using measured calibration. Its owner-local
   state uses 181 bits / 23 ARM bytes; persistent calibration uses 38 bits /
   5 ARM bytes. Full-range counts/timestamps and cumulative distance retain
   32 bits; speed, interval distance and yaw use their proven smaller domains.
   Public configuration views and checked arithmetic remain wide until validated.
5. Terrain processing uses gravity-derived pitch and measured encoder travel:
   incremental rise is travel times sin(pitch). Mounting calibration, usable
   aligned data, flat-baseline confirmation and continuity gate the estimate.
   The task-local terrain state uses 165 bits / 21 ARM bytes.
6. Only an ordered, valid ascent/crest/descent and return to baseline completes
   a hump. Reversal, shock, sensor loss or a broken interval abandons the partial
   traversal. A completed hump can increase the retained run maximum; pause and
   recoverable faults retain it. Stopped/disarmed RESET or reboot starts a new run.
7. The controller publishes a checked 93-bit / 12-byte terrain result under the
   existing app-state mutex. Controller and Buddy 1 use `s4_result_decode()`.
   B1 adapts the result to the existing 394-bit / 50-byte aggregate telemetry.
   Optional standalone EDR framing is source 4 / kind 2, total 14 bytes; it is
   not silently substituted for the aggregate MQTT schema.

## Ownership and interfaces

| Area | Buddy 4 responsibility | Other owner boundary |
|---|---|---|
| Acquisition | IMU driver/data, quality, timestamps and protected copies | No other task accesses I2C |
| Encoder consumption | Checked B2 decoding, age/skew and terrain association | B2 owns GPIO IRQ, counts, PWM and motor watchdog |
| Terrain | Mounting/zero calibration, pitch/roll, state, height and maximum | Controller chooses speed/STOP; B2 actuates |
| Representation | Compact B4 cache/history/result and odometry dependency | B1 owns publication/dashboard contracts; controller owns arbitration |
| Validation | B4 software regressions, physical IMU and ramp evidence | Whole-vehicle timing/network/drive tests involve the affected owners |

The interfaces needed by the current B4 path are implemented. Buddy 1's
retrospective review should confirm schema selection, units, validity/history,
source/type routing and fault aggregation using BUDDY1_RETROSPECTIVE_REVIEW.md.
No universal payload or new per-buddy decoder is necessary if the existing
checked contracts are retained. A changed external schema needs a coordinated
version/receiver change, not a raw struct copy.

## Scope and remaining physical work

The professor's revised final demonstration uses approximately the smaller
practice ramp's height; the higher ramp can be disregarded. The numeric height,
slope and acceptance tolerance remain to be measured/agreed. No expected answer
is pre-entered into the estimator and no new narrow height bound is inferred.
Follow BUDDY4_TESTING.md for sensor-only, mounting, encoder scale, complete ramp,
maximum/reset, invalid-data, controller and dashboard checks.

Initial configurable assumptions remain encoder age 60 ms, IMU/encoder skew
30 ms and interval gap 60 ms; flat band 3 degrees, slope entry 5 degrees,
maximum accepted roll 15 degrees, return residual 20 mm and minimum hump 5 mm.
Review these against smaller-ramp geometry and measured noise. They are not
accuracy or timing guarantees. Gravity-based tilt is unreliable under sustained
dynamic acceleration; slip, sensor mounting and wheelbase geometry affect height.
Absolute compass heading, PID speed regulation and audio playback are not
implemented by this B4 work.

B4's stored field review, tables and software regressions are implemented,
including the previously outstanding odometry storage. This does not certify
whole-team CR1 completion: B1 commands, B3 persistent/ADC records, B5 ranges and
shared controller state retain findings in the dated CR1 team review.
Target accuracy, latency/jitter, fault-to-PWM response and runtime stack/heap
high-water evidence remain required before Week 13 acceptance.

## Verification records

`testing/subsystem4/ready-verification.json` identifies the current sources,
artifacts and checks. `ready-baseline.json` is the same-profile main-reconciled
baseline before odometry packing. Host boundary tests and a before/after
production-code benchmark supplement the driver/terrain/codec and actual
controller/telemetry tests. ARM layouts and full-image static RAM are measured
separately from native layouts. See BUDDY4_CR1_STORAGE.md for the precise domains,
compiler-layout limits and performance tradeoffs. Older verification/merge/cr1
records remain dated engineering evidence, not instructions for another device.
