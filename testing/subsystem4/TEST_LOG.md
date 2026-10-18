# Subsystem 4 — test log

**Subsystem:** Buddy 4, GY-511 / LSM303DLHC acquisition and snapshots
**Student ID:** <student ID> · **Code:** `app_program/subsystem4/`

See `testing/README.md` for evidence/commit conventions and
`app_program/subsystem4/README.md` for the bench/API guide.
Hardware and actual SMP runtime tests remain pending.

## Test cases

Existing IDs are retained. S4-U02 is retired and must not be reused.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S4-U01 | legacy message buffer retains S4_SUMMARY_MAX | IF-03 | on-target auto | S4_ONLY=1 TEST=1 | PASS; no sensor needed |
| S4-U02 | retired: monitor used to pause/resume S1 LED | retired IF-05 | not run | S4 no longer controls S1 | retired; do not reuse ID |
| S4-U03 | snapshot rejects NULL | IF-06 | on-target auto | `subsystem4_test.c` | E_PAR |
| S4-U04 | both S4 tasks exist at priorities 18/19 | lifecycle | on-target auto | `subsystem4_test.c` | PASS |
| S4-U05 | acquisition heartbeat continues | liveness | on-target auto | compare snapshots 500 ms apart | counter increases, including absent sensor |
| S4-U06 | connected device produces fresh usable channels | sensor bring-up | on-target auto, needs hardware | connect verified GY-511; run test image | both sample counters increase; usable flags true; SKIP if not configured |
| S4-U07 | six-face acceleration signs/scaling | calibration preparation | manual | record X+/X-/Y+/Y-/Z+/Z- at rest | gravity vector roughly 1000 mg; signs follow sensor axes |
| S4-U08 | magnetic response to rotation | calibration preparation | manual | rotate slowly away from motors/magnets | XYZ readings respond; no clipped data |
| S4-U09 | absent-sensor boot does not stop USB/task | fault handling | manual | boot without sensor and save 30 s | time/heartbeat progress; unusable values; init retries |
| S4-U10 | identity/configuration and initialization faults | driver | host auto | driver_test.c | PASS |
| S4-U11 | signed decoding, XYZ remapping, units/freshness | driver | host auto | driver_test.c | PASS |
| S4-U12 | clipped/out-of-range data rejected | quality | host auto | driver_test.c | PASS |
| S4-U13 | partial reads and independent channel faults | driver | host auto | driver_test.c | healthy channel preserved |
| S4-U14 | lock recovery and invalid arguments | driver | host auto | driver_test.c | PASS |
| S4-U15 | initial/missing sensor state | snapshot | host auto | data_test.c | no usable data |
| S4-U16 | cache age and stale expiry | snapshot | host auto | data_test.c | NO_DATA does not refresh timestamp |
| S4-U17 | per-channel faults, recovery/reinitialization | snapshot | host auto | data_test.c | invalidation/recovery and counters correct |
| S4-U18 | clipping, future/64-bit time and age saturation | snapshot | host auto | data_test.c | invalid time/data never usable |
| S4-U19 | init/start order, cleanup and bad arguments | lifecycle/API | host auto, simulated kernel | api_test.c | failures propagate; partial init cleaned up |
| S4-U20 | snapshot/lock failures and clock wrap | IF-06 | host auto, simulated kernel | api_test.c | no overwritten output on failure; coherent copy path |
| S4-U21 | task priority/affinity and duplicate start | lifecycle | host auto, simulated kernel | api_test.c | priority 18/core 1, priority 19/any |
| INT-02 | legacy S3 summary lines reach S4 | IF-03 | integrated on-target | existing test; not runnable in S4_ONLY | at least two lines in 3.5 s; SKIP in S4_ONLY |

`S4-U10` through `S4-U21` run with
`make -C app_program/subsystem4/tests/host test`.
They test software against mock bus/kernel calls, not real bus timing or
physical cross-core execution.

## Results

The original template row is retained; it is not a measured result.
Append actual runs below it. A local development commit is not a team GitHub
commit; record the team's actual tested commit for later board tests.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S4-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
| 2026-10-17 | S4-U10 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U11 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U12 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U13 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U14 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U15 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U16 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U17 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U18 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U19 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U20 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |
| 2026-10-17 | S4-U21 | `9b6de29ae296` (local) | Codex / host | PASS | `logs/2026-10-17_host.txt`, `logs/2026-10-17_sanitizers.txt` | software / simulated kernel and bus only |

## Physical checks still to run

USB enumeration; actual identity/configuration; axis response; supply/pull-ups;
bus timeout/recovery under fault; polling/stack headroom; SMP snapshot reads
under the integrated workload. No physical result is recorded by a build.
