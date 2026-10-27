# Buddy 4 integration and ownership recap

Prepared 27 October 2026, Singapore. This records local software work; physical
validation is the next stage. No firmware was flashed and no motors were started.

## Current reconciled software baseline

Checkpoint 373ad6d and CR1 continuation 8f14916 are reconciled with main f8107be
(Buddy 5 PR #7) in PR #8. The integrated controller, STOP protections and common
operating clock are retained; Buddy 5's checked packed status is adopted. See
MAIN_RECONCILIATION_20261027.md for conflict decisions and current verification,
and BUDDY1_RETROSPECTIVE_REVIEW.md for the requested follow-up review.

Buddy 4 uses a 63-byte acquisition record, a 21-byte ARM terrain working state
and a 12-byte checked result for controller/B1. B1 retains the 50-byte aggregate
schema. Physical calibration, accuracy, load timing and runtime memory evidence
remain open. Nothing was flashed or started during software reconciliation.

The following sections describe the original checkpoint and its provenance.

## Starting sources

The original clean checkout was main `056a92823bb7bbd87b17f0bb6eb6ed7bd7308e63`.
GitHub was refreshed and the new local branch
`buddy4/integration-cr1-20261027` starts at main
`0269a02da74cdf7cd9ebd0a6bb46597dcfbc3677`, including Buddy 2
`fc1dd8dfac1d30a06369f8ed08f29a7d718125fd` and published Buddy 3
`16f319c5169967ff6a916e8ce22cfed0c84f6acd`. Main already removed the obsolete
producer that caused the old build failure.

The handoff ZIP is a broader integrated snapshot, based on local `b02e335` plus
uncommitted work, rather than only Buddy 3. All 1,250 manifested files matched
SHA256 and length. Its 1,164 repository files were compared with main using
`056a928` as the reference. Nonoverlapping work was recovered; overlapping
Buddy 2 source was reconciled explicitly. The original checkout branch and ZIP
remain intact. The supplied restoration script copied the snapshot but then
failed on an empty deleted-files entry; no deleted source was required.

Restored files include the prior controller, STOP epochs, operating-clock fixes,
IR integration, sonar integration, binary telemetry, dashboard, tests, tools and
historical evidence. Those are inherited work, not newly authored Buddy 4 work.
See `BUDDY4_CHANGE_PROVENANCE.json` for recovered paths and conflict decisions.

## Final data path

1. Buddy 2 remains the sole encoder/PWM owner. Its sequence-checked getter now
   timestamps with `tk_get_otm()`, including the motor watchdog and command lease.
   This preserves the local integration's monotonic operating-clock correction.
2. The controller calls `s2_encoder_encode()` into a fresh caller-owned buffer.
   Success supplies exactly the existing 13-byte EDR v1 encoder payload:
   left count u32 LE, right count u32 LE, operating milliseconds u32 LE, flags u8.
   Counts are signed cumulative positions represented modulo 2^32. Flags are
   VALID, left A/B and right A/B in bits 0..4; bits 5..7 must be zero.
3. Buddy 4 `encoder_input.c` uses the existing checked decoder. It rejects API
   errors, wrong lengths, noncanonical invalid records, reserved bits,
   stale/future samples, inconsistent repeats, ambiguous count changes and gaps.
   Odometry uses measured x1 count scale and wheel dimensions, never PWM.
4. Source age is checked after the potentially waiting snapshot calls.
   IMU read timestamps are explicitly divided by 1000, then compared in the same
   operating-millisecond domain. That scaling adds no timestamp resolution.
5. Existing gravity-derived pitch/roll and geometric height integration are
   retained. Height requires calibration, valid aligned inputs, a confirmed flat
   baseline and an ordered ascent/crest/descent/flat traversal. Shocks, overrun,
   loss, reversal or discontinuity cancel completion. Either wheel reversing
   cancels height even between repeated IMU samples; usable tilt and navigation
   odometry can remain available during an intentional turn.
6. The controller receives bounded state through the existing application
   snapshot. MQTT is outside the control decision. Existing STOP precedence,
   queue invalidation and peripheral ownership remain. A full movement pool
   cannot prevent the controller's STOP epoch from being offered.
7. Buddy 1 keeps the existing aggregate 394-bit/50-byte telemetry and Python
   dashboard contract. The 15-byte EDR framed encoder packet is a separate
   format; no global envelope migration was made. `CR1_PROTOCOL.md` defines the
   aggregate bit offsets, units and availability flags. Terrain availability
   now also requires a recent application snapshot and usable acceleration.

The maximum updates only on a valid completed hump. Faults, pause and ordinary
loss preserve it, while invalidating current height. The stopped/disarmed RESET
command clears terrain and encoder history; reboot also starts a new in-memory
run. ARM/renewal alone does not clear the maximum. No nonvolatile history exists.

## Configurable assumptions and limitations

Initial encoder maximum age is 60 ms, IMU/encoder skew 30 ms and encoder/terrain
gap 60 ms. These accommodate a nominal 20 ms IMU period plus the 10 ms controller
cycle; they are configurable assumptions requiring load measurements. They are
not established Tribunal timing guarantees. Flat confirmation is five samples,
flat band 3 degrees, slope entry 5 degrees, accepted roll at most 15 degrees,
base return tolerance 20 mm and minimum completed hump 5 mm. Named macros in
`terrain.h` and `encoder_input.h` expose these provisional settings.

LSM303DLHC has an accelerometer and magnetometer, no gyro. Tilt assumes gravity
dominates the measured acceleration. Height is an estimate, not validated
accuracy. Wheel slip, mounting flex, vibration and sample association contribute
error. A reset with identical plausible count/time values is indistinguishable
from continuous motion without a producer boot/session identifier; local state
must be reset whenever a producer session changes. Neither physical timing nor
runtime stack/heap high-water usage has been measured in this session.

## Buddy 4 ownership

| Area | Buddy 4 responsibility | Other owner or shared boundary |
|---|---|---|
| GY-511 acquisition | Driver validation, I2C1 task, per-channel quality, freshness, protected snapshots | GP2/GP3 and I2C1 have one S4 owner |
| Incoming encoder use | Decode and validate the agreed bytes; check time alignment and reject bad fusion input | B2 owns IRQ counts, wheel signs and measured encoder calibration |
| Terrain | Mounting/zero calibration, pitch/roll, hump state, height validity, maximum/reset rules | Controller decides speed/STOP; B2 writes PWM |
| Compact information | S4 state/flags, field ranges, units, validity and documented output meaning | B1 owns MQTT queue/transport and dashboard; shared aggregate schema |
| Evidence | S4 software regressions and sensor/terrain physical evidence | Cross-subsystem tests involve B1/B2/controller owners |

New or revised Buddy 4 files: `encoder_input.c/.h`, `terrain.c/.h`,
`drv_lsm303dlhc.h`, `sensor_data.c/.h`, `subsystem4.c`, `task_sensor.c`,
`task_monitor.c`, host tests and size probe. The driver implementation itself is
reused; flags/enums in its cached types were narrowed.

Necessary shared/B2 edits: controller encoder adapter call, checked modular
odometry deltas, EDR-compatible packed motor state plus preserved STOP epochs,
operating-clock timestamps, bounded motor-status waits, polling STOP at most
every configured 10 ms wait, host fixtures and S4-only link dependencies.
The portable B2 runner and codecs are kept from latest main. B1 gets only
terrain freshness gating and passive terrain diagnostics beyond restored work.
Windows build helpers now have a native MSYS2 backend using the installed
extension toolchain, while retaining WSL support.

This is local integration for owner review. It does not claim team approval to
publish another buddy's subsystem. Only CR1 is addressed; CR2/CR3 are excluded.

## Verification

The exact commands, tool versions, source hashes, ELF/UF2 hashes and sizes are
recorded in `../testing/subsystem4/verification-20261027.json` and its logs.
Checks cover S4 acquisition, data/API behavior, production encoder/terrain
adapter, actual vehicle task, B2 EDR/watchdog profiles, recovered integration,
C/Python vectors, B3 regressions and Cortex-M0+ object compilation. Native
GCC static analysis was run on the S4 suite. Default integrated and isolated
S4Bench firmware were built with Pico SDK 2.3.0 and Arm GNU 15.2.

ASan/UBSan were attempted but could not link because the installed native GCC
lacks their runtime libraries. This is a toolchain limitation, not a sanitizer
pass. Historical sanitizer logs in the handoff remain historical evidence.

Continue with `BUDDY4_TESTING.md`; the first physical step is acquisition-only
S4Bench with motor power disconnected. Calibration defaults remain unset.
