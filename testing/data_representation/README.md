# External data representation checks

## Simple launchers

**Windows:** double-click `check_data.cmd` in the repository root. It runs all
three stages, saves `edr-check-report.json` beside the script, and keeps the
window open so you can read the result. From a terminal or automation, use
`.\check_data.cmd --no-pause` to return immediately after the tests.

**Linux/WSL/macOS:** run `sh check_data.sh` from the repository root. Options
such as `sh check_data.sh --compiler clang` are passed to the Python runner.
Both launchers find Python 3 and work when invoked by their full path from a
different working directory. Python 3 and a native GCC/Clang are still required.
They do not install software or flash the robot.

## Direct command and prerequisites

Every buddy should run this before starting integration, after pulling changes
to the shared representation, and again before opening a pull request. From
the repository root, with Python 3 and a **native** GCC or Clang installed:

```sh
python -B testing/data_representation/run.py --report edr-check-report.json
```

On Linux/WSL/macOS, use `python3` if that is the installed command. The same
runner works on Windows. The PowerShell equivalent is:

```powershell
powershell -ExecutionPolicy Bypass -File testing/data_representation/run.ps1 -Report edr-check-report.json
```

Expect the final line to start with **`EDR CHECK PASS`** and exit status zero.
A failed assertion, build error or missing compiler returns nonzero; the gate
stops on the first failed stage. Keep the detailed failure output when asking
the field owner to investigate. Do not work around a failure by changing the
golden expected bytes just to match a changed implementation.

The gate runs three stages:

1. Python decoder and independent known-byte contract tests.
2. Native C99 codec tests, including every buddy/source, record kind and length.
3. Actual Buddy 2 adapters and watchdog in eight simulated RTOS/hardware profiles.

No Pico, firmware SDK, MQTT broker, Wi-Fi credentials, PowerShell on Unix or
third-party Python packages are required. The compiler must produce programs
for your computer: `arm-none-eabi-gcc` cannot run these host tests. Start by
checking `python --version` and `gcc --version` if setup fails.

Use `--compiler clang` or `--compiler /path/to/gcc` to select a compatible
compiler (`-Compiler clang` in PowerShell). The runner creates its generated C
fixture header and executable in an OS temporary directory, then removes them.

## Sharing results

`--report` is optional. Its JSON report records success/failure for each stage,
tool versions, Git revision/dirty status, and SHA-256 hashes of the covered
source, fixtures and test inputs. A failed run also writes a failed report.
Compare the `inputs.sha256` value with a buddy's report to check that you tested
the same covered inputs. Checkout CRLF/LF differences are normalized. Different
hashes can reflect harmless source/test edits; this fingerprint is not the
protocol version or proof of wire incompatibility. The report does not cover
unrelated source files or prove on-device timing.

The example report filename is ignored by Git. A report with
`working_tree_modified: true` describes local changes in addition to the listed
commit. Keep that distinction when sharing test evidence.

## Automatic pull-request check

`.github/workflows/data-representation.yml` runs the same gate on pushes, pull
requests and manual dispatch, using native GCC and Clang. It prints the JSON
evidence report in each job log, including on failure. No cloud run is claimed
until this workflow is committed/pushed and the run finishes.

To prevent merges after a regression, a repository administrator can make
`EDR contract (gcc)` and `EDR contract (clang)` required status checks in the
branch rules. The workflow file alone does not enable that branch setting.

`vectors.json` is the common, machine-readable source for expected bytes and
field values. Python reads it directly. The runner translates its literal data
to a temporary C header without calling either codec. Both implementations
encode to the known packet bytes and decode to the known fields. Tests do not
rely only on encode/decode round trips.

Coverage includes:

- All 65,536 motor words, including all 2,121 valid combinations and unchanged
  outputs on failed unpacking; every byte-valued pack input is range checked.
- All header bytes, all envelope lengths 0–255, short/trailing packets, all
  known-schema payload lengths, and generic opaque payloads for buddies 1–5.
- All 5 sources x 8 record kinds x 256 lengths: 10,240 combinations in each
  language, checking exact headers, routing, zero bytes and payload contents.
- Every fault byte and encoder flag byte, reserved bits, canonical unavailable
  encoder records, null C arguments, short buffers and failure output handling.
- Fixed byte-order vectors, zero and `UINT32_MAX`, modulo negative tick deltas,
  rollover, ambiguous half-range rejection and freshness limits.
- Dashboard CLI JSON output, binary file input, importable decode API and
  malformed input exit status.

The C build treats conversion, signedness, shadowing, prototypes and pedantic
warnings as errors under C99. Its output prints payload/packet sizes, host ABI
DTO sizes and an encoder encode/decode microbenchmark. DTO sizes are transient
host values, not wire layouts. Timing excludes RTOS locks, hardware and MQTT;
it cannot establish RP2040 timing or target RAM savings. Target measurements
require a firmware build and on-device execution.

The dashboard entry point is `tools/edr_decode.py`. `decode_packet(bytes)`
returns a JSON-compatible dictionary. Known Buddy 2 motion and encoder records
are validated and decoded; other valid source/kind combinations return
`supported: false` with the unmodified payload in hexadecimal. Each MQTT
message must contain exactly one binary packet.

## When a buddy adds their part

A PASS currently establishes the common framing and Buddy 2 field/adapter
contract. Buddy 3–5 payloads are opaque until their owners define schemas, so
it cannot establish their sensor interpretation or application behavior yet.

For each new payload, the owner should:

1. Document source/kind, exact byte length, ranges, units, invalid values and
   timestamp/rollover rules in `docs/DATA_REPRESENTATION.md`.
2. Add checked C encoding/decoding and matching Python decoding. Keep the shared
   envelope stable and preserve existing known vectors.
3. Add independent expected byte vectors for zero, minimum, maximum, signed
   values and rollover, plus malformed/truncated/reserved-code tests. Add the
   C tests and any new C source to the gate's build list. Python `test_*.py`
   files in this directory are discovered automatically. Register new source
   dependencies in `run.py`'s `INPUTS` list so the report fingerprints them too.
4. Test the real producer snapshot/adapter and the consuming buddy's decoding,
   including stale data and failed API calls; register their host test command
   as another gate stage when ready.
5. Rerun this gate and their subsystem tests. Measure hardware timing and actual
   MQTT copying/delivery separately when integrating the robot.

If a wire change is intentional, review the specification/version and expected
vectors together with the affected buddies. A green test with silently changed
expectations cannot establish compatibility with older consumers.
