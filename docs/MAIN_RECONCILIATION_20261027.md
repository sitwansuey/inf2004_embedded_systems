# Main reconciliation on 27 October 2026

The user authorized reconciling current Buddy 4 work with latest main, pushing
and merging PR #8, with Buddy 1 reviewing retrospectively. This is an integrated
software baseline. Physical calibration, accuracy, motor-response timing and
runtime memory evidence remain separate work. No board was flashed or started.

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
adopting any separate MQTT branch. No message was sent to teammates by the agent.
