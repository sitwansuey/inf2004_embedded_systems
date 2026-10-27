# Buddy 3 physical recording, 07:22/07:23 +08:00, 22 October 2026

The sampling correction is ARM built, flashed by the user, and verified in
real recordings. Firmware SHA256:
346dc09e75f348f36719e24ff6f88015ae4901b50e14397bea07f483bba26d2e.
All 84 tracked source/test/document hashes matched before this turn's
documentation-only updates. No firmware source was edited this turn.

| Physical recording | A | Z |
|---|---:|---:|
| Complete distinct valid ADC scans | 2512 | 2509 |
| Consecutive intervals | all 10ms | all 10ms |
| Gap/drop/error records | 0 | 0 |
| GP28 raw range | 118..3925 | 186..3929 |
| Threshold edges | 54 | 30 |
| First complete sweep | 5450ms | 4260ms |
| Native production replay | BAD_PATTERN | BAD_PATTERN |
| Decoded expected/wrong symbols | 0 / 0 | 0 / 0 |

All 5021 samples are retained, with actual timestamps and no interpolation,
rescaling or partial-marker stitching. Each recording is one continuous
decoder segment. Fifty controller status records remain STOP, DISARMED,
power 0, fault 0; no robot command was sent. Both line sensors remain dark,
but the host GP28 optics test does not use them to gate decoding. Onboard
controller/line gates are unchanged and decoding remains PAUSED on the board.

The cyclic/mutex correction fixed the previously observed sampling gaps in
these two recordings. This is an observed result under this workload, not a
hard real-time proof for every workload. Physical symbol decoding remains
NOT PASSED. Both cards now fit the retained 10-second timeout and reach
pattern validation. The strict decoder rejects their measured width ratios.

A's first black edge is 1150ms after its first recorded sample; the first
30-edge sweep finishes at 6600ms. A second partial sweep appears at
16360..20040ms. The host decoder holds after the first rejection, as designed;
it does not silently rearm for a later partial sweep. Z has one 30-edge sweep
at 5890..10150ms. These offsets use firmware sample time, not host wall time.

For Z the first nine widths are 50,160,110,70,210,110,500,100,100ms. Their
estimated narrow unit is 100ms: 50ms and 500ms fall outside the unchanged
75..125% narrow / 225..375% wide limits. Ranking the three largest widths
gives the expected frame/body/frame arrangement for Z, but ranking alone is
not a valid decoding pass. The A middle group likewise resembles A by rank
while failing ratio checks. Tolerances and timing limits were not relaxed.

The user reports sliding the card on a flat surface. Variation in hand speed,
optical transition position, scan path or sensor/card geometry remains possible;
the logs cannot establish which is responsible. Neither a broken sensor nor
an incorrect card is established. Supplied Barcode Sample (1).pdf pages 1/3
were visually checked; the printed letter interrupts the bars at the middle
of the card, so the next sweep should cross uninterrupted bars away from it.
Barcode & Line Specification (1).pdf specifies 18mm line width, 20mm gap and
approximately 141mm barcode length; actual installed sensor spacing is not
measured. Moving line control remains a separate outstanding validation.

| Image | text | data | bss | static RAM | UF2 | telemetry |
|---|---:|---:|---:|---:|---:|---:|
| Previous capture 78ED21 | 331052 | 456 | 25584 | 26040 | 663040 | 50 |
| Installed cyclic capture 346DC0 | 331884 | 456 | 25584 | 26040 | 665088 | 50 |

The correction adds 832 text bytes and zero linked static RAM bytes versus
the preceding capture image. The optional kernel capture buffer increases
128 to 160 bytes; kernel object overhead and full runtime heap occupancy are
unmeasured. ELF PT_LOAD comparison matched 332340 firmware bytes in the UF2.
Source-level cyclic ID growth is not a measured increase in linked BSS.

## Next test with the current firmware

Use task-output TEST_BARCODE_RETRY.ps1 -Symbol Z in normal PowerShell.
No BOOTSEL or flash is needed. Keep motor power off and software lock on.
Keep the chassis still, existing calibration/height/wires unchanged. On the
same flat surface use a straight guide, passing the complete barcode once
across GP28 at steady pace in about 4-6 seconds. Start on WHITE, wait for
SCAN NOW/start beep, and finish on WHITE without returning the card across
the sensor. Cross uninterrupted bars away from the printed letter. Hold
WHITE until recording ends. The helper permits at most three prompted
attempts and stops early on a host replay pass or on a capture/controller
fault. Each attempt preserves uniquely named raw/warmup/CSV/result logs.

REPLAY_BARCODE_LOG.ps1 runs a PC executable compiled directly from unchanged
production barcode/normalization code. Executable and ten source-input hashes
are checked first. It preserves every row, and reports a pass only for the
expected symbol with no wrong symbol or continuity break. It sends no serial
command and makes no motion/onboard-decoding claim. Native limits remain
30ms max gap, 40ms minimum run and 10000ms timeout. Synthetic A/Z forward and
reverse pass; wrong symbol, partial markers across gap/invalid/drop, slow
timeout, controller fault and serial damage reject. Mock retry tests confirm
early success, at most three failed attempts, and immediate fault exit.
Both actual recordings still reject with the packaged host executable.

## Evidence and outstanding work

Task outputs BARCODE_CARDS_0723/ retain the original logs, summary records,
UF2, ELF/map and manifests. BARCODE_CARDS_0723_ANALYSIS.json contains raw
edges/width audit. IR_0723_FIRMWARE_VERIFIED.json preserves the 84 matched
pre-documentation hashes. IR_BARCODE_FEEDBACK_TESTS.log distinguishes
synthetic/mock checks from physical recordings. BARCODE_REPLAY_BUILD.json
records the host build command and source/executable hashes.

Prior stationary calibrated IR classification remains 161/161 on 3E84E2.
Physical A/Z decoding, onboard decoding, moving line following, electrical
motor shutdown/braking, encoder geometry and remaining integrated motion,
terrain/obstacle accuracy and live MQTT validations remain outstanding.
Motor lock remains 1; no CR2/CR3 implementation, push, PR or remote change.
