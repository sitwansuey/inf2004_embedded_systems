# EDR16 verification

From the repository root, run:

```sh
python -B testing/data_representation/run.py
```

Use Python 3 (`python3` where `python` is not found) and native GCC or Clang.
`--compiler PATH` selects the compiler;
`--report FILE` writes results, tool versions and input hashes. An Arm cross
compiler cannot execute these host tests. `.\check_data.cmd --no-pause`
(Windows) and `sh check_data.sh` (Git Bash, MSYS2, macOS or Linux) run the
same suite and write `edr-check-report.json` (git-ignored) in the repository
root. The shell launcher forwards runner arguments; the Windows launcher
accepts only `--no-pause`.

The three stages are:

1. **Python:** shared EDR16 codec, exact bytes, lengths, units, ranges and
   invalid values, including mission faults 0 to 8 (8 = lifted; 9 and above
   rejected); S5 packed-status checks; all Dashboard `test_*.py` files,
   including commands and robot events.
2. **C:** shared codec with exhaustive 16-bit words, fixed vectors, invalid
   buffers, unchanged outputs on failure and local arithmetic; S1 copied
   telemetry/command/event queue checks.
3. **S2:** nine simulated motor profiles covering commands, encoders, expiry,
   urgent STOP, errors, PID, kicks and stalls, plus the exact bytes of S2's
   EDR16 motion and encoder adapters.

The runner sets `CC` for the S5 Python check. The dashboard event-packing
check separately needs `gcc` on `PATH`. Browser routing needs Node.js on
`PATH`; without it, that part is omitted. The dashboard's C width-producer
check is skipped unless `testing/integration/.build/width_avoidance` (`.exe`
on Windows) exists, built by `python -B testing/integration/run.py` or
`make -C testing/integration test`. Review skips and omission messages
before calling the coverage complete.

The [wire contract](../../docs/DATA_REPRESENTATION.md) defines one two-byte
record per MQTT payload. S2's four-byte adapter outputs contain two records
and must be published separately. The private 55-byte snapshot is checked
by the integration protocol and stream tests:

```sh
python -B testing/integration/run.py
```

Record the tested commit, command, result and saved output using
[testing/README.md](../README.md). The [TEST_LOG.md](TEST_LOG.md) here contains
historical formats as well as EDR16 records; read the migration notes before
reusing an old result. Current subsystem and integration results are also
in their own `TEST_LOG.md` files and tracked `logs/` folders. Host checks do
not establish physical sensor accuracy or live MQTT performance.
