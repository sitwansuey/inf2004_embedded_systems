# EDR16 verification

From the repository root, run `python -B testing/data_representation/run.py`.
Or use `check_data.cmd` / `check_data.sh`. Supply `--compiler PATH` for a native
GCC/Clang compiler not on PATH. `--report FILE` writes results and source hashes.
An ARM cross compiler can compile tests but cannot execute the host harnesses.

The three stages are:

1. Python shared codec and dashboard receiver: exhaustive words, ranges,
   exact lengths, explicit byte vectors, units, malformed/old packet rejection.
2. C shared codec: exhaustive words, fixed bytes, unchanged outputs on failure,
   null/short buffers and local arithmetic; B1 copied queue and length handling.
3. Buddy 2's eight existing host profiles: encoder wiring, commands, watchdog,
   rollover, error propagation, plus the updated EDR16 adapter bytes.

Current specification: [DATA_REPRESENTATION.md](../../docs/DATA_REPRESENTATION.md).
Each record is two bytes. B2 adapters return two records in a four-byte array;
publish them separately. Old source/kind envelope fixtures are no longer valid.
Tests never authorize motor actuation or claim physical sensor validation.
