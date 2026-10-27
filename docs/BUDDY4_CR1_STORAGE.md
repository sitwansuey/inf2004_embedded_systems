# Buddy 4 CR1 storage and field review

The same source can have different host and ARM layouts. Internal bitfields
are never copied onto the wire; explicit EDR and telemetry codecs own byte
order, padding, range checks and exact lengths. Only the sensor task mutates
its working frame. The published S4 copy uses its existing priority-inheritance
mutex. Controller terrain/input state is task-local; `app_state` publication
uses the existing mutex. Neither volatile nor local IRQ masking substitutes
for cross-core protection.

| Stored information | Range and resolution | Information bits | Actual representation and invalid handling |
|---|---|---:|---|
| Per-channel fresh/valid/overrun | Boolean | 1 each | Three internal u8 bitfields; unusable readings retain numbers with validity cleared |
| Per-channel acquisition status | Codes 0..4 | 3 | Internal 3-bit field; no-data, OK, clipped, I/O error, lock-drained |
| Init/poll result | Codes 0..5 | 3 each | Internal 3-bit fields; configured false on unsuccessful init |
| Configured/accel usable/mag usable | Boolean | 1 each | Internal single-bit fields, coherent published snapshot |
| Raw acceleration XYZ | -2048..2047, 1 LSB = 1 mg in configured mode | 12 each | int16 array, smallest standard integer container; no truncation. Aggregate wire uses 12-bit mg fields |
| Raw magnetic XYZ including corrupt/overflow diagnostics | Full signed 16-bit read | 16 each | int16; valid configured sensor range is narrower. Aggregate wire uses checked 12-bit valid raw values |
| Scaled XYZ | Acceleration mg; magnetic mG | 12 valid accel; up to 17 magnetic diagnostic bits | int32 arithmetic/compatibility arrays; magnetic full raw diagnostics divided by configured gain can exceed int16. Packing 12-bit raw accel internally would not reduce the measured aligned ARM sample size without a separate storage codec |
| Channel read time / observed time | 64-bit operating ms scaled by 1000 | 64 retained | uint64; actual resolution 1 ms. UINT64_MAX is clock error; no-data has fresh=false |
| Sample ages | 0..UINT32_MAX-1 ms | 32 | uint32; UINT32_MAX means unavailable; large ages saturate below sentinel |
| Heartbeat, samples, retry, error, overrun, publish and legacy counters | Modulo 2^32 diagnostics | 32 each | uint32; narrowing would shorten diagnostic wrap with no agreed bound |
| Terrain phase | Six states | 3 | Internal 3-bit state; INVALID/SHOCK explicitly distinguished |
| Tilt/height/shock/initialized flags | Boolean | 1 each | Internal single-bit fields |
| Flat confirmation count | 0..configured threshold, default 5 | 3 at default | uint8 supports configurable 1..255 threshold; never shared or serialized directly |
| Pitch | -9000..9000 centidegrees | 15 | int16, wire 15; tilt_valid gates meaning |
| Roll | -18000..18000 centidegrees | 16 | int16, wire 16; mounting/quality/roll threshold gates control use |
| Height and run maximum | 0..1000 mm, 1 mm display resolution | 10 each | uint16 native containers, 10-bit aggregate wire; height_valid gates last completion. Maximum>0 is retained completed-hump history |
| Height/peak/distance history | Signed micrometres | 32 retained | int32 cumulative state; int64 and double safe calculation intermediates. Checked range, no modulo geometric distance |
| Terrain sample time | Modulo 2^32 ms | 32 | uint32; repeated time does not integrate again; gaps invalidate traversal |
| Incoming encoder counts/time | Modulo 2^32 | 32 each | Exact 13-byte EDR payload; decode to transient fields; checked modular deltas reject half-range |
| Encoder valid/GPIOs | Five Boolean bits | 5 | EDR byte bits 0..4; bits 5..7 reserved and rejected |
| Encoder adapter diagnostic | Six named results | 3 | Task-local 3-bit field; no raw enum copied across transport |
| Mutex IDs and atomic sequence/STOP words | Kernel/atomic domain | 32 retained | Naturally aligned RTOS/atomic storage; never packed across cores |

Minimum information bits, standard C container width, alignment and final
wire width are deliberately distinguished. This is a scoped CR1 review with
explicit retained-width justifications, not proof that every byte of the team
firmware is globally minimal. Shared team schema changes require owner review.

## Measured sizes

`size_probe.c` reproduces the supplied old type declarations and compares them
with current types. Native GCC 15.2 x86_64: sample **40 -> 32 bytes**, cached
status **144 -> 120**, terrain **28**, encoder-input state **36**. Arm GNU 15.2
Cortex-M0+: sample **32 -> 32**, cached status **128 -> 120**, terrain **28**,
encoder-input state **36**. ARM values come from object-symbol lengths, not
host execution or assumptions about enum size.

The persistent `published_status` therefore saves 8 ARM bytes against that
type baseline. Stack copies of this type are also smaller, but actual task
stack high-water savings have not been measured. The encoder adapter occupies
36 bytes including the previous 32-byte odometry state, a 4-byte type-size
increase in that controller-local record; this is not a net whole-firmware RAM-saving
claim. Whole-image static data/BSS values and hashes are recorded per build in
`../testing/subsystem4/verification-20261027.json`. No runtime heap/stack
measurement is claimed.

## Compact outputs and fault meanings

The aggregate schema remains exactly 50 bytes: acceleration 12 bits/axis in mg,
magnetic raw 12 bits/axis, pitch 15, roll 16, terrain 3, last height and maximum
10 each. `available` bits 3/7 mean usable acceleration/magnetometer; bits 5/6
mean current usable tilt/completed-height status. Stale controller or sensor
data clears terrain availability. Retained maximum survives unavailable current
height and is displayed separately. See `CR1_PROTOCOL.md` for all offsets.

The aggregate controller reason is an enum, not the EDR fault mask. It must
never be cast to a mask. Meanings are:

| Controller reason | Relationship to EDR diagnostics |
|---|---|
| VC_OK | No controller fault; not proof all sensor fields are valid |
| VC_SENSOR | Could concern IMU, odometry or sonar; cannot uniquely select EDR_FAULT_IMU, EDR_FAULT_ENCODER or EDR_FAULT_NO_ECHO |
| VC_LINE | No dedicated EDR line bit; retain controller reason |
| VC_SHOCK | IMU-related condition, but not a motor error |
| VC_TRANSPORT | Local command/event transport; does not necessarily mean Wi-Fi |
| VC_TIMEOUT | Lease expiry corresponds conceptually to TIMEOUT |
| VC_UNCALIBRATED | No dedicated EDR calibration bit |
| VC_MOTOR | Motor-related condition corresponds conceptually to MOTOR |

`s2_motion_encode()` independently maps its own exact motor error to TIMEOUT or
MOTOR. This integration does not manufacture a combined mask from ambiguous
controller reasons or change other buddies' packet schemas.
