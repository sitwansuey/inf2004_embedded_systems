# Software testing

The repository tracks executable tests, shims, schemas, golden inputs and CI
workflows. Reports, historical logs, physical captures and test diaries remain
local and are ignored. Source tests are not evidence of hardware accuracy.

Run the commands in the root README. Relevant entry points:

- `data_representation/run.py` (or `check_data.cmd` / `check_data.sh` from the
  root): EDR C/Python codecs, the S5 codec, every Dashboard `test_*.py`, the
  audio tool and the B2 host profiles.
- `subsystem3/Makefile`: IR algorithms, adapter, ADC and ARM/header checks.
- `../app_program/subsystem2/tests/host/run.py`: B2 adapters and watchdog in
  nine host profiles (also stage 3 of the EDR gate).
- `../app_program/subsystem4/tests/host/Makefile`: B4 driver, compact storage,
  checked encoder/terrain/result path, GCC analysis and sanitizer targets.
- `integration/Makefile`: controller, avoidance, queue and actual
  telemetry/MQTT boundaries; `integration/run.py` runs the same recipes with
  any native compiler (Windows included).

`app_program/robot_config.h` defaults are the robot's mission profile. Host
builds define `ROBOT_CONFIG_NO_LOCAL` so a git-ignored `robot_config.local.h`
never leaks into a test; a test that needs another scenario passes explicit
`-D` flags (for example `-DBOARD_IR_MODE=2` or `-DBOARD_MOTORS_DISABLED=1`).

New build/serial results should be saved locally until selected for
submission. No test command here flashes or starts a robot. `verify-failure`
intentionally generates one failed assertion to check that failures propagate;
its successful wrapper is not a regression failure.

Final Week 13 submission still requires representation tables, measured RAM
and message comparisons, and correctness/performance evidence in GitHub. Those
records are intentionally local for now and must be selected and tracked before
submission. Runtime memory, latency, electrical STOP and calibration need real
hardware measurements on the final firmware.
