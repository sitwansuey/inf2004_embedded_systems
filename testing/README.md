# Software testing

The repository tracks executable tests, shims, schemas, golden inputs and CI
workflows. Reports, historical logs, physical captures and test diaries remain
local and are ignored. Source tests are not evidence of hardware accuracy.

Run the commands in the root README. Relevant entry points:

- `data_representation/run.py`: EDR C/Python codecs and B2/S5 profiles.
- `subsystem3/Makefile`: IR algorithms, adapter, ADC and ARM/header checks.
- `../app_program/subsystem4/tests/host/Makefile`: B4 driver, compact storage,
  checked encoder/terrain/result path, GCC analysis and sanitizer targets.
- `integration/Makefile`: controller, queue and actual telemetry/MQTT boundaries.
- `../tools/robot_bench/TEST_HELPERS.ps1`: host-only capture/replay helper checks.

`fixtures/robot_bench/` contains immutable regression inputs consumed by the
helper test, not new capture outputs. Preserve failure outcomes; a PC replay
is not an onboard decoding pass. New build/serial results should be saved
locally until selected for submission. No test command here flashes or starts
a robot. `verify-failure` intentionally generates one failed assertion to check
that failures propagate; its successful wrapper is not a regression failure.

Final Week 13 submission still requires representation tables, measured RAM
and message comparisons, and correctness/performance evidence in GitHub. Those
records are intentionally local for now and must be selected and tracked before
submission. Runtime memory, latency, electrical STOP and calibration need real
hardware measurements on the final firmware.
