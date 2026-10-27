# Final local checkout validation

Historical checkout validation; source identity is recorded below.

Branch: local/csc02-cr1-integration-20261021. Source integration commit:
a3dbb7b6116ba2ab6f3e2f06fce2cb5735f809af. A subsequent local commit fixes the
Windows helper and records this validation; use `git log -1` for its exact ID.

The direct checkout checks passed: Buddy 2's eight native configurations;
Buddy 3 native/header/ARM SMP 0/1 checks; Buddy 4 host tests; integration tests
including the full scan sequence, C/Python codec, copied queues and actual MQTT
callbacks with a fake transport. The Windows build helper initially failed to
pass drive paths through wslpath; direct mounted-drive conversion and --exec
fixed the problem. The corrected helper compiled this checkout with a SDK
path containing spaces. No firmware was flashed and no serial port was opened.

The final default ELF is text/data/bss 326748/456/25552 bytes, static RAM 26008
bytes. The comparable build-repaired baseline with encoders is 25172 bytes:
integration increases static RAM by 836 bytes despite smaller messages.
MQTT's lwIP allocation is separately recorded in MEASUREMENTS.json. Individual
compiler stack estimates are telemetry 864 bytes, vehicle worker 608 bytes,
motor 120 bytes and sensor acquisition 272 bytes. These exclude callees,
interrupt nesting and RTOS frames; they do not measure runtime stack peaks.

Default UF2: 654848 bytes, SHA-256
38be0e24a4796e5bfc3ac90a52ef293395f199af4914cec0123dcdd32c93fe56. Its RP2040 family, block order/count and flash address bounds
were inspected. It is built with IR/odometry/terrain calibration disabled and
boots DISARMED. The network-enabled build used a compile-only dummy config in
the isolated snapshot; it is not the distributed firmware.

All 92 installed source/evidence file hashes matched the reviewed manifest
before adding these final notes and measurement data. The complete final
manifest and raw logs are in the task's outputs folder. Repository log copies
trim trailing console whitespace; raw output logs remain unchanged.

Hardware testing was deferred at the user's request. PID speed regulation,
absolute compass heading and audio remain unimplemented. Refer to
HARDWARE_ACCEPTANCE.md before configuring or operating the robot. The project
must not be described as physically complete or accuracy-validated.
