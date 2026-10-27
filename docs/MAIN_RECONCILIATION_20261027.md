# Main reconciliation on 27 October 2026

## Second reconciliation: main 58b89c2

Integration record: PR #9, prepared on buddy4/cr1-integration.

Main added 9c3bd3d and 58b89c2 after the first reconciliation below. The
Buddy 4 branch now incorporates those commits. Their changes add IR line and
barcode processing, wheel duty trims (97/98 percent), movement renewal and
global ADC pad enablement around the older timed controller.

The wheel trims and updated motor assertions are retained. Startup retains
the existing single explicit-ARM controller with live S3/S4 acquisition,
encoder-based navigation, STOP epochs and monotonic operating time. The new
main controller would replace S4 with a placeholder, automatically start
motion after a delay, permit a sensor-free indefinite straight mode and treat
NO_FALL as forward clearance. Those policies are not used by this integration.
No-echo remains unknown space, with normal sensor-fault handling.

Main's straddling-sensor steering map is extracted into stateless
vehicle_line_command() and used by the existing explicitly supervised
continuous-line test. Freshness, calibration and movement authorization stay
with their existing owners. A second barcode decoder/logging pipeline is not
started; Buddy 3 owns acquisition, capture and bounded events. Both-white
forward behavior cannot establish line presence and is not enabled as the
normal autonomous tracking policy. The existing dedicated test profiles remain
available; full course/physical barcode acceptance remains open.

ADC pad setup follows BOARD_IR_MODE==2 in the kernel configuration, preserving
GP26/27/28 for configured analogue IR but leaving digital/disabled and isolated
IMU profiles alone. Main's unconditional command-line enables are removed.
The 5 ms ADC timeout and bounded READY implementation are preserved.

The reconciled pre-packing Default ARM image and integration suite pass;
ready-baseline.json and ready-baseline-build.log under testing/subsystem4
record the baseline for the subsequent B4 odometry storage comparison. These
are software checks; no flashing or hardware action was performed.

## First reconciliation: main f8107be

This is an integrated software baseline for review. Physical calibration,
accuracy, motor-response timing and runtime memory evidence remain separate
work. PR #8 remains unmerged; Buddy 1 review is retrospective. No board was
flashed or started.

## Sources and decisions

Buddy 4 checkpoint 373ad6d and compact-storage continuation 8f14916 were
reconciled with main f8107be (Buddy 5 PR #7). Both histories are retained.

| Area | Resolved implementation and reason |
|---|---|
| Controller/startup | Retain the explicit-ARM encoder-based controller in vehicle.c and startup in app_main.c. Main's older timed S5 controller treated S3/S4 as placeholders and started cruise after a delay; it cannot replace working terrain/line integration or STOP epochs. No second controller task is started. |
| Shared application state | Retain protected integrated app_state with the 12-byte checked B4 result. Do not restore S3/S4 placeholder records or publish terrain history as a native struct. |
| S5 storage/API | Adopt main's checked 84-byte s5_data codec, status adapter and packed persistent status. Keep its full-range decoded API so values cannot truncate during codec decode. |
| S5 synchronization | Preserve controller binding, finite inheritance waits, initialization checks and lock/unlock error propagation. Getter output remains unchanged on failure. Encode/decode and packed read-modify-write occur inside the same mutex. |
| S5 scan behavior | Preserve encoder movement rejection during stationary acquisition, sample-angle association, lost-bound rejection, selected-heading verification and treatment of echo timeout as unknown space. Retain the reviewed clamped servo pulse range. |
| S5 clocks | Use operating time tk_get_otm consistently with S2/S4/controller rather than comparing against adjustable system time. Imported diagnostic code uses the same timebase. |
| Wide S5 consumer fields | Validate range and plan domains before narrowing into controller inputs. Oversized ranges do not authorize movement; oversized plans do not wrap into small plausible plans. B1 masks unavailable distance/width instead of losing an entire aggregate packet. |
| S5 diagnostics | Preserve upstream acquisition/master/sweep sources as explicit APP_TEST plus S5 diagnostic opt-ins. Passive TEST=1 does not automatically run motor/servo test sequences. The diagnostic hardware function checks its owning task; integrated controller startup is suppressed for explicit diagnostic builds. Motor lock remains the default. |
| Logging and pins | Adapt upstream logging to real integrated controller state in the telemetry task. Retain conditional board_pins definitions and GP22 for the explicit diagnostic buzzer, avoiding duplicate unconditional motor/pin assignments. |
| S3 | Add an explicit NULL guard before raw-config validation/dereference. Existing NULL tests pass and GCC analysis no longer depends on inferring another compilation unit's NULL behavior. |

## Verification

merge-verification.json and merge-prefixed logs under testing/subsystem4 record
exact source/artifact identities and local results. Checks cover B4 host/analysis,
B3 host/analysis/kernel/ARM checks, B2 watchdog profiles, the new S5 codec/API
profiles, actual controller/telemetry consumers, binary queues/MQTT substitutes,
and C/Python protocol vectors. Default, S4Bench, passive-test and MQTT builds
are recorded separately where run. MQTT compilation uses temporary nondeployable
example credentials, removed afterwards; no real credentials are committed.

A dedicated GitHub workflow runs B4/integration tests with ASan/UBSan on Linux,
where those runtimes are available. Local Windows sanitizer unavailability is
not represented as a pass. Check PR #8 for final remote check outcomes.

The merge does not claim whole-team CR1 minimality, calibrated hump accuracy,
physical stopping performance or final hardware acceptance. Motor lock and
uncalibrated defaults remain. See BUDDY4_TESTING.md for staged validation.

## Retrospective review

Buddy 1 should review BUDDY1_RETROSPECTIVE_REVIEW.md against the new main before
adopting any separate MQTT branch.
