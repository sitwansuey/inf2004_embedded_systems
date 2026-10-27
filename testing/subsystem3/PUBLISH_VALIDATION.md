# Buddy 3-only publication validation — 25 October 2026

Target: feat/buddy3-line-barcode-ci, based on bac31d8f775066c2a1e111e4c05773a58d3500ab.
Changed paths are restricted to app_program/subsystem3 and testing/subsystem3.
No Vehicle Controller, other subsystem, kernel/device, global build profile,
robot launcher, compiled firmware, credentials or hardware auto-start is included.

## Checks on the isolated tree

- make -C testing/subsystem3 analyze: PASS, 64,886 assertions across
  15 result summaries, zero failures under GCC -fanalyzer.
- verify-failure: PASS. Its deliberate 2+2==5 assertion fails and propagates
  a nonzero result; this expected negative check is not a regression failure.
- check-headers: PASS with actual kernel headers, SMP=0 and SMP=1.
- arm-check: PASS for actual Cortex-M0+ objects, SMP=0 and SMP=1.
- Additional real ARM objects: raw, calibrated, passive capture and sampler
  decoder profiles PASS in both core configurations (8 combinations).
- git diff --check: PASS before commit.

The analyzer found an API validation boundary that relied on a validator in
another translation unit; s3_configure_raw now explicitly checks NULL before
dereferencing. Its existing invalid-input tests cover this path. Standalone
raw bring-up requires a coordinator-declared motor lock; native positive
fixtures declare a test-only lock, and unlocked refusal remains tested.

## Whole-firmware assessment

The complete branch build remains blocked in unchanged Buddy 2 code:
app_program/subsystem2/task_producer.c refers to removed SAMPLE_MSG,
sem_slots, mpf_samples and S2_PERIOD_MS. Buddy 3 actual ARM compilation passes.
Those unrelated files were left unchanged as explicitly requested. This
Buddy 3-only commit does not claim a complete integrated firmware build.

## Hardware scope

See HARDWARE_VALIDATION.md. The separate local integrated v6 test completed
30% PWM / 20 seconds and the user confirms turns and straight crossing.
Its controller/motor/USB changes are excluded from this branch. Full-course
autonomy and complete physical barcode decoding remain unvalidated.

## Successful native summaries

- Buddy 3 algorithms: 13 groups, 551 checks, 0 failures
- Buddy 3 adapter: 8 groups, 20114 checks, 0 failures
- Buddy 3 ADC: 9 groups, 1095 checks, 0 failures
- Buddy 3 ADC tick: 3 groups, 328 checks, 0 failures
- Buddy 3 ADC acquisition: 6 groups, 2088 checks, 0 failures
- S3-U33 raw bring-up: 1 groups, 135 checks, 0 failures
- S3-U33 raw bring-up: 1 groups, 2 checks, 0 failures
- S3-U42 confirmed layout: 14 checks, 0 failures
- S3-U43 measured calibration: 68 checks, 0 failures
- S3-U43 measured calibration: 2 checks, 0 failures
- S3-U44 paused timing: 99 checks, 0 failures
- Buddy 3 barcode capture: 2 groups, 388 checks, 0 failures
- Buddy3 sample schedule: 3 groups, 449 checks, 0 failures
- S3 motor scan sampler: 4 groups, 34500 checks, 0 failures
- S3 barcode edge-bias regressions: 5053 checks, 0 failures
