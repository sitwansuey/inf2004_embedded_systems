# GP28 decoder v5: saved A capture passes on the PC

The original 24 October 21:55 manual recording now decodes A, forward, at t=3358980 ms
using production C on Linux and Windows. All 2510 original ADC samples and timestamps
are preserved. One A match, zero rejects, zero wrong matches, zero discontinuities.

**V5 is installed. Normal USB v5 READY and STOP were verified.**
The earlier21:37 commanded-motor recording involved manual dragging; it was not an
isolated motor-only test. Later manual captures had motor power disconnected and zero
serial commands. No motor commands were sent during this software work.

## Change and boundaries

- `buddy3_barcode.c` requires three clearly separated wide intervals: smallest wide
  at least1.5 times largest narrow. A bounded BLACK/WHITE edge offset per character
  must fit every corrected interval into the existing narrow0.75..1.25 and wide
  2.25..3.75 bands. Bias cannot exceed half a narrow unit. Both WHITE separators,
  exact29-run count, framing, known A/Z, forward/reverse and adjacent scale bounds
  remain checked. Ranking alone cannot accept an inconsistent width/outlier.
- `robot_ir_config.h` selects a20ms minimum only for calibrated ADC profiles at the
  existing nominal10ms acquisition rate. Generic default40ms, hysteresis400/600,
  ADC endpoints,30ms continuity/freshness and10s timeout remain unchanged.
  Isolated10ms pulses remain rejected at the nominal cadence. No faster acquisition
  or reliable recognition of10ms optical bars is claimed.
- New regression test and sampler fixture validate short biased frames, A/Z both ways,
  multiple scales and corruption. App/host markers identify v5. Motor cap remains20%,
  maximum6000ms, host lease250ms, independent motor watchdog150ms, boot STOPPED.
- No new tasks/buffers/persistent decoder fields, sensor owners, line mappings, navigation
  commands or wire formats. CR1 only; no Requirements2/3 or remote changes.

The bounded offset is an engineering model for observed pulse imbalance, not proof of
whether optics, threshold crossing, mounting or hand movement caused it. This capture was
used during diagnosis, so replay success is a regression result, not an independent
physical acceptance test. A checksum was not specified or invented; a corruption that
forms another valid supported symbol can remain indistinguishable.

## Checks actually performed

- Integrated Buddy3 suite: 64886 assertions, zero failures.
- Dedicated edge-bias tests:5053 assertions, zero failures; also run with AddressSanitizer
  and UndefinedBehaviorSanitizer, with no reported errors. All512 middle bit patterns
  accept only A/Z with valid framing; all512 alternatives for each frame are checked.
  Tests include excessive bias, zero/extreme/ambiguous/outlier widths, bad separators,
  truncated frames, both directions and the20ms calibrated budget.
- Sampler integration includes a short biased frame, ownership, STOP, freshness/lease,
  gaps and clock failures, without line/event authority. GCC static analysis passes.
- All seven saved complete manual CSV recordings were replayed: final A passes; six
  earlier incomplete/malformed captures remain rejected. No field accuracy rate is claimed.
- Controller/protocol/MQTT suite passes:339 motor policy checks, USB/urgent STOP,
  302 codec vectors,74 bound rejections,50-byte telemetry and13-byte events.
- Real-header ARM object compilation: SMP0 and SMP1 pass. Forced full ARM link passes;
  RP2040 UF2 blocks/profile markers/payload verified against ARM ELF. Build-only checks preceded the separate installation below.
- Source hashes, unchanged cached remote refs and git whitespace checks pass.
  Prior user edits preserved on `local/csc02-ir-motor-scan-20261024`. No push, fetch, PR or remote merge.

## Before and after sizes (bytes)

| Measure | Installed v3 | Built v5 | Difference |
|---|---:|---:|---:|
| Text | 313516 | 313900 | +384 |
| Data | 160 | 160 | +0 |
| BSS | 25436 | 25436 | +0 |
| Static RAM | 25596 | 25596 | +0 |
| UF2 file | 627712 | 628224 | +512 |
| Telemetry payload | 50 | 50 | +0 |

Static link/representation sizes exclude runtime peak RAM and network framing. Task
stack allocation remains2048bytes. Separate Cortex-M0+ O2 stack reports are retained:
`barcode_stack_before.su` / `barcode_stack.su`. Frames exclude callers/callees/library use;
on-device stack high-water remains outstanding. No static RAM increase is claimed as
proof that peak stack use is unchanged.

## Artifact and hardware next step

Reviewed artifact: `tools/robot_bench/artifacts/ir_decoder_v5/CSC02_IR_MOTOR_SCAN_USB.uf2`.
v5 SHA256: `34d36e25406708316a6c4a1a2798108792486087cc52972ff6e09992b54d9ab3`.
Pre-upgrade v3 SHA256: `f6b8cdc71ed544503f20e4810e63b1f61ad9d9ef75d12dc9a9cf68742f289076`.

Default portable PC replay and its source manifest are updated to v5; prior artifacts
are backed up under workspace BEFORE. Original v3 capture/replay results remain
unchanged. v5 uses a separate identical recording copy in HOST_V5.

Keep motor power disconnected. After the operator identifies the robot BOOTSEL drive,
copy only this verified artifact, then verify the v5 normal-USB STOP handshake.
Next physical trial: unpowered manual A, then Z. Do not infer onboard decoding from
PC replay. Raw USB capture still evaluates data on the PC; an explicitly prepared
onboard observer is required for onboard validation. No movement is queued.

Evidence: `evidence/2026-10-24/ir_decoder_v5/`. Older reports are historical.

## Installation and first independent live recording

User identified robot RPI-RP2 on D: with motor power disconnected. Source/UF2/ELF
and packaged artifact were checked before copy. Normal USB v5 READY and final STOP
were acknowledged. This verifies copy plus runtime identity, not physical flash readback.

BARCODE_V5_MANUAL_A_20261024_222313_079 contains2502 valid10ms samples, zero gaps/drops,
25 complete STOP/fault-zero controller reports and zero capture commands. Independent
PC replay did NOT decode A: one40ms BLACK interval, then INCOMPLETE after200ms WHITE.
That WHITE interval eventually lasted250ms and further bars followed. Full record44edges,
ending WHITE. This is distinct from the corrected BLACK/WHITE width-bias issue.
Partial-frame quiet detection needs review before further repeats. Motor limits unchanged.
Both saved-recording success and independent live failure are retained.
