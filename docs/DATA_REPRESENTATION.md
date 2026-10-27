# Shared external data representation (EDR v1)

Buddy 2 implements the motor and encoder records below. All five buddies can
use the envelope and pure C codec in `app_program/shared/edr.h`. This is the
implementation contract for this change; adoption by the other branches and
physical integration still need verification.

## Requirements and scope

Source material: `Week8_Tribunal1_Template (2).docx` and
`Change Requirement 1.docx`, supplied for this task. The relevant tribunal
decisions are the 3/7/2-bit motor fields, a common binary contract, encoder
decoding for Buddy 4, the existing kernel clock, and separate controller and
telemetry paths. CR1 also asks for range, RAM, message-size and timing evidence.
The source Word files are unchanged; meeting declarations and planned tests are
not treated as completed engineering evidence.

Implemented here:

- Explicit masks/shifts for authoritative Buddy 2 motor storage and messages.
- One portable byte format, checked encoders/decoders and snapshot adapters.
- A common five-buddy envelope and shared fault-bit assignments.
- A Python receiver and C/Python test vectors.

This does not implement another buddy's algorithms, a Vehicle Controller,
MQTT transport, PID, distance calibration, stall detection, or terrain fusion.
Those features are absent or incomplete in this checkout. In particular, the
full application still refers to retired `SAMPLE_MSG` demo interfaces.

## Envelope: one record per MQTT payload

All offsets are zero-based. Multibyte integers are **little-endian**. Bit zero
is the least significant bit. Use byte APIs and explicit lengths, including
embedded zero bytes; never `strlen`, `%s`, raw struct copies, C bit-field
layout, `#pragma pack`, or enum memory layout as the transport format.

| Byte | Bits | Meaning | Valid range |
|---|---|---|---|
| 0 | 0..2 | Source buddy | 1..5; 0, 6, 7 rejected |
| 0 | 3..5 | Source-local record kind | 0..7 |
| 0 | 6..7 | Protocol version | 1; all other versions rejected |
| 1 | 0..7 | Payload length in bytes | 0..255 |
| 2 onward | All | Owner-defined payload | Exactly the advertised length |

The source needs three bits for five buddies. Kind is an eight-slot protocol
namespace per owner; version is a four-slot protocol namespace. These are
framing metadata rather than widened sensor values. The maximum payload is a
protocol bound, not a requirement to allocate 255 bytes for every record.
Use a 9-byte packet for motor status and a 15-byte packet for encoders.
MQTT supplies the record boundary; the explicit length also detects a wrong
record size or an accidental NUL-terminated transport. It is not a checksum.

`edr_frame_decode()` checks only framing and returns a borrowed payload view.
Its buffer must remain alive while the view is used. Route by **both** source
and kind, then invoke the owner's checked decoder. Unknown record kinds must
not be interpreted as motor commands or sensor values. No decoder actuates
hardware or authorizes movement.

| Buddy | Source | Implemented records | Integration responsibility |
|---|---|---|---|
| 1, networking | 1 | Payload definitions pending | Copy bounded byte records; publish actual length inside the network-owning task |
| 2, motors | 2 | Kind 0 motion; kind 1 encoders | Produce records using `s2_data.h` |
| 3, line/barcode | 3 | Payload definitions pending | Define actual sensor/state bounds and retain local freshness checks |
| 4, IMU/terrain | 4 | Payload definitions pending | Consume kind 1 encoder samples; define sensor scale, validity and event formats |
| 5, obstacles/controller | 5 | Kind 0: 84-byte obstacle snapshot | [Bit layout and checked codec](SUBSYSTEM5_DATA.md); retain command arbitration |

The codec accepts other sources without imposing guessed sensor ranges. Each
owner must add a field table, checked payload codec and shared vectors before
claiming that their application data satisfies CR1.

## Buddy 2 field representation table

| Application field | Valid values / units | Required bits | Stored / transmitted representation |
|---|---|---:|---|
| Movement command | Seven codes, 0..6 | 3 | Motor word bits 0..2; 7 rejected |
| PWM power | Integer 0..100 percent | 7 | Motor word bits 3..9; 101..127 rejected |
| Motion state | Three codes, 0..2 | 2 | Motor word bits 10..11; 3 rejected |
| Motor-word padding | No application value | 0 | Container bits 12..15 must be zero |
| Shared faults | Seven independent conditions | 7 | One byte, bits 0..6; bit 7 rejected |
| Command timestamp | Boot milliseconds modulo 2^32 | 32 | `uint32_t`, four little-endian bytes |
| Left/right position | Cumulative signed x1 count modulo 2^32 | 32 each | Two unsigned 32-bit containers; no narrowing or saturation |
| Encoder timestamp | Boot milliseconds modulo 2^32 | 32 | `uint32_t`, four little-endian bytes |
| Encoder usable flag | False/true | 1 | Flags bit 0 |
| Four encoder GPIO levels | Low/high for left A/B and right A/B | 1 each | Flags bits 1..4 |
| Encoder flags padding | No application value | 0 | Bits 5..7 must be zero |
| Exact kernel error | Signed `ER` domain, not a bounded application state | 32 retained | Internal diagnostic; external status maps to fault bits |
| Diagnostic positive/negative pulse and edge counters | 0..2^32-1, modulo cumulative counters | 32 each | Existing ISR-owned atomic counters; omitted from routine telemetry |
| Encoder readiness / sequence | Cross-core atomic protocol metadata | 32 each | Existing aligned atomic words, not packed read-modify-write flags |

Native `s2_motion_status_t` and `s2_encoder_status_t` remain compatibility views
at the RTOS API boundary. Their `sizeof` is not the wire size. Kernel mailbox
headers, pointers, object IDs, locks, allocator alignment, arithmetic
intermediates and temporary decoded values are not serialized application
fields. Public callers must rebuild because the internal movement-message
layout and encoder snapshot declaration changed.

Command codes: STOP=0, FORWARD=1, REVERSE=2, LEFT=3, RIGHT=4,
PIVOT_RIGHT=5, PIVOT_LEFT=6. LEFT rotates counterclockwise; RIGHT rotates
clockwise. Power is commanded PWM percentage, **not measured wheel speed**.
Motion states: STOPPED=0, MOVING=1, FAULT=2. Codecs validate field ranges;
the motor owner retains normalization and safety rules such as STOP/zero
power, command renewal and expiry. A syntactically valid word alone is never
movement authorization.

## Motion record (source 2, kind 0): 7-byte payload

| Payload offset | Size | Field |
|---|---:|---|
| 0 | 2 bytes | 12-bit motor word, little-endian |
| 2 | 1 byte | Shared fault mask |
| 3 | 4 bytes | Time the last command was issued, low kernel milliseconds |

Fault allocation: bit 0 motor stall, 1 encoder fault, 2 IMU fault, 3 no echo,
4 Wi-Fi loss, 5 motor command timeout, 6 other motor/driver error. Bit 7 is
reserved and rejected. Buddy 2 currently emits only timeout or other motor
error from its existing error status. A zero stall bit is **not** proof that
stall detection exists. Other owners must explicitly report their own faults;
this adapter does not manufacture a combined whole-vehicle health report.
The exact `last_error` remains available through the native status getter.

`command_ms` is not a periodic sample timestamp or a renewed authorization
lease for the receiver. A stopped vehicle can legitimately have an old
command time. Use packet arrival/connection health for dashboard liveness;
do not equate receipt of telemetry with continued movement permission.

## Encoder record (source 2, kind 1): 13-byte payload

| Payload offset | Size | Field |
|---|---:|---|
| 0 | 4 bytes | Left wheel forward count, modulo 2^32 |
| 4 | 4 bytes | Right wheel forward count, modulo 2^32 |
| 8 | 4 bytes | Capture time, low kernel milliseconds |
| 12 | 1 byte | valid, left A, left B, right A, right B in bits 0..4 |

Left is motor 1: negative pulses minus positive pulses. Right is motor 2:
positive pulses minus negative pulses, matching the current wiring signs.
These are measured **x1 rising-A counts**, not four-times quadrature counts.
The GPIO bits describe sampled levels; pulse counters remain authoritative.

An invalid/startup record is exactly thirteen zero bytes. Invalid records
with nonzero counts, time or GPIO flags are rejected. Timestamp zero with
valid=1 is legitimate at boot or rollover. Disabled encoders return `E_NOSPT`
from the adapter and leave the output unchanged; do not publish that old
buffer as a new valid sample. Enabled-but-not-started acquisition returns
`E_OK` with the all-zero invalid record.

The getter samples `tk_get_tim().lo` inside the successful sequence-checked
count/GPIO copy. Any intervening encoder IRQ causes a retry. A clock failure
returns the kernel error and clears the native output; the byte adapter
leaves its caller's output unchanged. This is a count-snapshot time, not an
individual edge timestamp or a guaranteed periodic acquisition interval.

## Time, signed counts and Buddy 4 consumption

All local times use the existing kernel boot clock; there is no extra clock
task. EDR stores its low 32 bits in milliseconds, wrapping every 49.71 days.
Buddy 4 currently exposes kernel milliseconds scaled to microseconds in a
64-bit value: convert units explicitly before comparing with an EDR sample.

Compute age as unsigned `(now_ms - sampled_ms)`. Use
`edr_age_is_fresh(now_ms, sampled_ms, maximum_ms)` only with an agreed maximum
below 2^31 ms (24.85 days), and discard history on reset/reboot/reconnect.
It accepts ages up to and including the configured limit. No timestamp-only
scheme can identify a reboot or a sample retained for a whole clock wrap.
Do not compare a PC wall clock or MQTT arrival time to the kernel boot clock.
Consumers still need source validity, an agreed age limit, and alignment with
their own sensor samples. The representation does not establish that limit.

For interval position use `edr_count_delta(current, previous, &delta)`.
It subtracts unsigned modulo-32 counts, then converts only a proven signed
range. Exactly 2^31 ticks is ambiguous and rejected. Consumers must sample
often enough that actual motion is less than 2^31 ticks per interval; a
larger movement can alias to the opposite sign. Reject invalid/old samples,
zero elapsed intervals and ambiguous deltas. Reinitialize after reboot or
loss of history; do not compute an interval from an invalid startup sample.

Only after measuring ticks per wheel revolution and wheel circumference can
Buddy 4 calculate distance or speed. Combine **measured** wheel intervals
with sensor intervals; never infer distance from PWM percentage. No such
calibration or hump-height accuracy is claimed by these tests.

## Integration and concurrency

Include `../shared/edr.h` from any subsystem. Buddy 2 producers additionally
include `../subsystem2/s2_data.h`. A telemetry producer calls the corresponding
snapshot adapter, checks `E_OK`, wraps the exact payload with
`edr_frame_encode()`, and hands a **copy plus actual byte length** to Buddy 1.
Use source 2 and kind 0/1. The codec takes caller-owned storage, has no mutable
globals, performs no allocation, and is reentrant for distinct buffers.
Input/output storage must not overlap. A false/error result leaves outputs
untouched, so callers must check it rather than transmit a previous packet.

For example, the local construction of a motor telemetry packet is:

```c
uint8_t payload[EDR_MOTION_SIZE];
uint8_t packet[EDR_HEADER_SIZE + EDR_MOTION_SIZE];
size_t packet_size = 0U;
ER result = s2_motion_encode(payload, sizeof(payload));

if (E_OK == result)
{
    if (edr_frame_encode(EDR_SOURCE_MOTOR, EDR_KIND_MOTION,
                         payload, sizeof(payload), packet, sizeof(packet),
                         &packet_size))
    {
        /* Copy packet_size bytes into Buddy 1's agreed telemetry handoff. */
    }
}
```

Buddy 4 can obtain and decode `s2_encoder_encode()` locally without MQTT.
First check the API result, then `edr_encoder_decode()`, then valid, age and
alignment. Buddy 1's dashboard uses `tools/edr_decode.py` with the same field
map. New owners can add source-local record kinds without touching Buddy 2.

The motor task remains the sole PWM writer; packing adds no actuator owner.
The motor command pool contains copied packed words and original issue times.
The shared status semaphore protects the **entire** packed snapshot on both
cores. Encoder counters retain single-ISR ownership and sequence-checked
atomic loads. Never update one packed field through an unprotected
read-modify-write, share a scratch packet buffer, or enqueue a pointer to a
stack buffer without copying it. The pure codec does not synchronize a
caller's shared buffer for them.

The existing controller bind/access check and command watchdog remain in
force. Remote movement command ingestion is not implemented. A network
receiver must validate a separately agreed command contract and hand it to
the Vehicle Controller, which arbitrates before calling Buddy 2. Telemetry
and MQTT reconnects must not renew a movement command.

Buddy 1 still needs a bounded copied telemetry queue, nonblocking consumption
inside the network-owning task, explicit full-buffer handling, publication
rate and disconnect policy. Application tasks must not call lwIP/MQTT/CYW43
directly. No new shared RTOS object or network API is silently introduced by
this change. A full movement pool can still reject STOP; retain result
checking and validate the existing watchdog under concurrent load.

## BARR-C and verification

New codec and adapter code follows the relevant rules from
[BARR-C:2018](https://barrgroup.com/sites/default/files/barr_c_coding_standard_2018.pdf):
fixed-width wire integers, unsigned bit operations, documented conversions,
module-prefixed lowercase names, const input buffers, include guards,
private static helpers, explicit braces and four-space indentation.
Serialization uses byte operations rather than compiler-specific packing.
The shared codec supports C99; the firmware retains its existing GNU C11
configuration. Existing RTOS/vendor code and unrelated legacy module style
have not been certified or comprehensively reformatted.

Before and after integrating any buddy's part, run the common hardware-free
gate from the repository root. It includes all-buddy framing, C/Python contract
checks and the eight Buddy 2 adapter/watchdog profiles:

```sh
python -B testing/data_representation/run.py --report edr-check-report.json
```

Expect `EDR CHECK PASS`. See the [gate guide](../testing/data_representation/README.md)
for prerequisites, reporting, CI and extending coverage to new payload schemas.
Individual entry points remain available:

```powershell
powershell -ExecutionPolicy Bypass -File testing/data_representation/run.ps1
powershell -ExecutionPolicy Bypass -File app_program/subsystem2/tests/host/run.ps1
python tools/edr_decode.py "42 07 f1 04 00 78 56 34 12"
```

See `testing/data_representation/TEST_LOG.md` for observed software results,
RAM/message-size evidence and the remaining hardware acceptance work. Host
test speed does not establish motor deadlines, MQTT latency, calibration,
cross-core contention or physical stopping distance.

