# Latest isolated Buddy 3 validation — 25 October 2026

Native GCC analyzer/regressions, expected-failure propagation, real kernel
headers and real ARM objects passed. See PUBLISH_VALIDATION.md for commands,
counts, scope and the unchanged Buddy 2 full-firmware build blocker.
Historical test definitions/results follow.

# Subsystem 3 — test log

**Subsystem:** Buddy 3 IR line following/barcode decoding  ·  **Student ID:** 2500383  ·  **Code:** `app_program/subsystem3/`

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