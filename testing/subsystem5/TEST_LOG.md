# Subsystem 5 — test log

**Subsystem:** <name>  ·  **Student ID:** <student ID>  ·  **Code:** `app_program/subsystem5/`

See `testing/README.md` for how to run tests and record results.

## Test cases

Define a test here before running it. Test IDs never change.

| ID | What is tested | Requirement / interface | Method | Steps / how | Expected result |
|---|---|---|---|---|---|
| S5-U01 | example: runs in a task context | | auto | `subsystem5_test.c` | `[TEST] S5-U01 PASS` |
| S5-U02 | | | manual | 1. … 2. … | |

*Method:* **auto** = `TEST_CHECK` in code, result printed on the console;
**manual** = a person observes the result (LED, scope, meter, phone app…).

## Results

### 2026-10-27: packed application status (host)

Command: `python -B testing/data_representation/run.py --report edr-check-report.json`

Result: **EDR CHECK PASS**, all three gate stages. S5 tests compile the actual
codec, shared-status API, scan, width and policy in normal and `APP_TEST=1`
configurations using native GCC with warnings as errors. Both profiles pass.
Observed native status size: 136 bytes; authoritative packed status: 84 bytes;
EDR frame: 86 bytes. Coverage includes reserved bits, angles, all 32-bit field
positions, signed extrema, C/Python agreement, mutex failure, navigation
invalidation, fault fallback and rollover freshness. The existing eight
Buddy 2 host profiles also pass. See `docs/SUBSYSTEM5_DATA.md` for the contract.

No board was flashed. Firmware timing, stack high-water marks and physical
sensor/navigation behavior were not measured by these host tests.

### 2026-10-27: pre-push verification and full ARM build

Repeated the complete EDR gate: PASS (10 Python test methods, 268,307 C
checks with zero failures, both S5 configurations and all eight S2 profiles).
The full ARM build initially exposed a missing `app_program/board_pins.h`:
the S5 branch removed it while the merged motor module still includes it.
Restored the guarded pin definitions from `origin/main`, preserving the
existing pin values.

Built the complete default firmware using WSL Ubuntu, ARM GCC and the local
Pico SDK: `PICO_SDK_PATH=/home/ncht/pico/pico-sdk make -C build_make -j2`.
The initial run used `-B` to force recompilation; after restoring the header,
the resumed build passed and generated both ELF and UF2. Image:
`mtk3pico_smp1_usb_cdc_wifi`; text=290420, data=172, bss=24900 bytes.
No board was flashed and no hardware behavior is claimed.

Add a new row for every run. Never edit or delete old rows.

| Date | Test ID | Commit | Tester | Result | Log / evidence | Notes |
|---|---|---|---|---|---|---|
| YYYY-MM-DD | S5-U01 | `abc1234` | | PASS / FAIL | `logs/YYYY-MM-DD_….txt` | |
