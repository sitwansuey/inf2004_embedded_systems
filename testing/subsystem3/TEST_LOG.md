# Subsystem 3 — test log

**Subsystem:** Consumer (example)  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem3/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S3-U01 | statistics self-consistent (min ≤ max, 0..100) | | auto | `subsystem3_test.c` | `[TEST] S3-U01 PASS` |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).


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
