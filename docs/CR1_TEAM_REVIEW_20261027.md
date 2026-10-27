# Buddy implementation and CR1 review — 27 October 2026

Reviewed candidate: `29fb124168e4b12df4cf6af53bc8ac02631b91bc`, branch
`buddy4/cr1-integration` (renamed from `buddy4/integration-cr1-20261027`). It
incorporates main `f8107be` through the reviewed reconciliation. PR #8 is open
and unmerged. Integration/merge is on hold at the user's request.

Desktop handover: the latest fetched main is now `58b89c2`, including
`9c3bd3d` (line tracing, barcode and ultrasonic integration) and `58b89c2`
(check fixes). Those updates have not been integrated or assessed by this
review. Continue from the remote `buddy4/cr1-integration` branch, preserve its
Buddy 4 work, and compare the new main before resolving overlapping changes.
This review describes candidate `29fb124`; its findings need rechecking after
that reconciliation. Calibration, physical testing and evidence collection
remain separate work. This handover changes documentation only.

This is a source/evidence review, not a new hardware acceptance run or a
certificate that every application field has been exhaustively inventoried.
The supplied post-Tribunal document supplies decisions and historical status;
its old "not implemented" statements are checked against current code. The
handover's embedded assignment is context, not a new instruction to publish,
merge, flash or change other owners' implementations.

## Overall assessment

All five subsystem foundations are represented in this candidate. Full original
functionality, whole-firmware minimum-bit compliance and Week 13 evidence are
not complete. Compact telemetry alone does not establish CR1 compliance.
An ordinary C byte/enum/word can still exceed a bounded field's required bits;
compiler allocation/padding can also mean a correctly sized bit-field saves no
whole bytes. Wide checked arithmetic, RTOS metadata and necessary aligned
atomic storage have different purposes and should not be narrowed blindly.

| Owner | Base implementation found | CR1 position | Remaining work |
|---|---|---|---|
| Buddy 1 | Copied binary telemetry/command queues, one network owner, aggregate dashboard decoder, reconnect handling and urgent STOP | Binary telemetry implemented in this candidate; command and persistent-state review incomplete | Pack bounded command fields; review network flags/counters; reconcile separate MQTT branch; live broker/dashboard/load evidence; confirmed-action/audio path if retained in final scope |
| Buddy 2 | Sole PWM writer, expiring commands, independent watchdog, urgent STOP, dual encoder acquisition and checked encoder codec | Core motor storage/messages and encoder contract are well advanced; subsystem-wide sign-off still pending | Inventory remaining stored flags/odometry fields and justify full ranges; physical encoder, watchdog and shutdown tests; PID remains absent, consistent with the agreed initial open-loop baseline |
| Buddy 3 | IR acquisition, calibration/line algorithms, bounded A/Z barcode decoder, leases/freshness and controller events | Partially implemented: several output fields and 13-byte events packed; persistent core/capture/ADC records remain wider | Pack bounded capture/core states, flags, count and ADC records; complete representation table/comparisons; complete physical barcode and course acceptance |
| Buddy 4 | GY-511 acquisition, checked B2 decoding/time alignment, ordered terrain estimation, retained completed-hump maximum, controller/B1 result path | Main B4 acquisition/terrain/result representations implemented with detailed evidence; inherited odometry state still needs review | Close shared odometry/storage inventory, then calibration, ramp/vehicle validation, target timing and runtime memory evidence |
| Buddy 5 | Ultrasonic/servo owner, stationary bound scans, width geometry and avoidance-plan/controller handoff | Checked 84-byte shared codec implemented; persistent control/context and several numeric domains still need work | Pack stored navigation flags/context; establish bounded range/duration/geometry contracts and narrow codec fields; physical scan/width/bypass validation |
| Shared controller | Explicit ARM/lease, priority arbitration, line/terrain/scan integration and encoder-based relative turns | Partial: many flags/state fields packed, others exceed current validated domains | Owner-reviewed field-width pass, concurrent-fault representation and real end-to-end acceptance |

## Concrete CR1 findings

These are examples establishing unfinished work, not an exhaustive replacement
for each owner's field table. Bit counts below include zero where stored.

| Area / source | Current representation | Observed domain / required width | Review action |
|---|---|---|---|
| B1 `subsystem1/telemetry.c`, `network_bridge.h` | Six-byte binary command; opcode byte, power byte, lease two bytes | Three opcodes need 2 bits; ARM power 1..50 (plus zero for other commands) needs 6; lease 0..1000 needs 10 | Binary is implemented, but these application fields are not minimum-width. Define a versioned compact command and update every receiver/tool; preserve queue-independent STOP. Protocol framing/sequence have separate justifications. |
| B3 `buddy3_barcode.h` | `s3_barcode_t.count` is uint32; capture state is a native enum; flags are separate bools | Stored count 0..29 needs 5 bits; four capture states need 2; each flag 1 | Pack authoritative capture state. The `s3_barcode_decode` function's wide count argument must still validate malformed input before narrowing. |
| B3 `buddy3_core.h`, `buddy3_line.h` | Persistent core mode/error enums, separate Boolean members, byte black masks | Three modes 2 bits; seven errors 3; three-sensor mask 3; each Boolean 1 | Pack stored core/line state while preserving locks, API validation and temporal behavior. |
| B3 `drv_ir_adc.h` | Copied ADC request has three byte-sized channels; reply embeds native `s3_sample_t` | ADC channels 0..2 need 2 bits each; ADC readings 0..4095 need 12 each; valid mask 0..7 needs 3 | Add a checked compact request/reply contract or justify an applicable exception. Generic algorithm views may support a wider range; the fixed ADC queue does not need to inherit it. |
| B5 `subsystem5.c`, `s5_scan.h` | Shared recenter/turn/navigation flags are separate BOOLs; scan context contains wide samples/counters/angles | Flags 1 bit each; bound averaging count 0..3 needs 2; acquisition/retry counters need widths derived from their thresholds | Pack persistent fields; shared whole-container updates must stay inside the existing mutex. |
| B5 `s5_data.c`, `s5_hardware.c` | Distance and echo duration occupy 32 bits each in the 84-byte payload | Actual hardware sample stores distance 0..5000 (13 bits), echo duration 0..25000 (15 bits) | Existing codec preserves the broad API domain. Agree whether that is truly required diagnostic information; then narrow/check actual bounded production fields. Do not truncate arbitrary existing API inputs. |
| B5 `s5_width.c`, `SUBSYSTEM5_DATA.md` | Width/plan values retain 32-bit external domains | Current default geometry has a much smaller calculated bound; aggregate telemetry already uses width 0..500 (9 bits) | Derive bounds for supported configurations, invalid values and diagnostics, then change the contract and tests. The aggregate's bound alone is not proof that every S5 diagnostic has that domain. |
| Shared `vehicle_policy.h/.c` | Persistent lease uint32; maximum power 7 bits; fault reason 4 bits | ARM lease 0..1000 needs 10; maximum power 0..50 needs 6; eight implemented reason codes need 3 | Formalize accepted domains and narrow safely. `vc_stop` currently masks four bits, so update validation/mask and callers together. A reason code is not a simultaneous fault mask. |
| Shared B2/B4 `odometry.h/.c`, `app_state.h` | Persistent speed is int32 and duplicated into app state | Accepted odometry intervals enforce speed -2000..2000 mm/s, requiring 12 offset bits | Pack persistent speed with a checked accessor; retain wide distance/time/count calculations. Review delta/yaw/configuration domains separately. B4's encoder-input object embeds this odometry state. |

Full 32-bit modulo wheel counts/timestamps are not automatically excessive.
Their rollover behavior and diagnostic lifetime must be preserved unless a
smaller domain is explicitly agreed. Likewise, the presence of decoded local
snapshots or arithmetic intermediates does not by itself establish a CR1 gap.
The persistent and copied records above require the closest attention.

The shared EDR fault allocation exists. B4 maps encoder/IMU bits explicitly.
The team still needs an inventory showing how independent simultaneous faults
are stored/reported. The controller's exclusive reason code cannot stand in
for every independent fault condition. Absent detection (for example motor
stall) must not be presented as a successfully monitored fault just because
its bit is defined.

## Branch differences and base-function limitations

- The fetched `origin/buddy1/MQTT` reference is `211c6c8`. Its `telemetry.c`
  still constructs decimal/Boolean/quoted text and uses separate topic IDs.
  It does not satisfy the binary-payload part of CR1. This candidate's binary
  integration must be retained/reconciled when Buddy 1's changes are adopted.
- B2's base agreed for initial integration is open-loop PWM plus encoder
  feedback/status. PID speed regulation is not implemented. Encoder-based
  distance/relative-turn control in the shared controller does not add PID or
  establish physical accuracy.
- B3 has actual stationary calibration and limited motion observations in the
  historical evidence. Current docs still record unresolved thin-bar sensing,
  incomplete onboard barcode acceptance and no complete-course acceptance.
  Do not turn a PC replay or a synthetic waveform test into an onboard PASS.
- B4's software path now implements what the older Tribunal status called
  pending: B2 decode, terrain estimation and compact result delivery. It has
  no gyroscope; absolute compass accuracy and calibrated height accuracy are
  not established. Physical acquisition observations from older images are
  distinct from acceptance of the current result/terrain implementation.
- B5's historical evidence establishes some basic echoes/status copies, not
  calibrated width or successful autonomous bypass. The current integrated
  controller uses scan plans with odometry; its complete vehicle behavior
  still needs testing. Default bypass/calibration gates remain unset.
- The external audio/confirmed-action subscriber path is not implemented in
  this candidate. If that remains a submission requirement, it is software
  work in addition to testing, owned by the team/B1 rather than B4's IMU path.

## Evidence checked and Week 13 status

This review checked all 112 application-source hashes in
`testing/subsystem4/merge-verification.json`: all matched the working source.
No firmware code was changed and no tests/hardware runs were repeated for
this documentation review. Previously recorded local evidence covers B4's
22 host groups, B3's 64,886 checks, the EDR/S2/S5 gate, actual integration
consumers and ARM firmware builds. The exact candidate's B4 Linux CI also
passed host/integration and address/undefined-behavior sanitizer steps:
[B4 integration run](https://github.com/sitwansuey/inf2004_embedded_systems/actions/runs/40487837478).
That corrects the report's then-pending remote sanitizer status; native Windows
sanitizers remain unavailable. Passing tests do not prove minimum field widths.

| Week 13 item | Evidence available | Still needed |
|---|---|---|
| Field representation table | B2 EDR table, B4 detailed tables, B5 codec table, aggregate schema | Complete actual persistent/shared/queue fields, including B3, commands and controller; remove stale claims that describe older integration layouts |
| RAM comparison | Historical ARM layouts/full-image data+BSS, B4 120->63 byte cache and 28->12 byte shared result, B5 136->84 byte host shared-status comparison | Final same-profile baseline/current comparison with exact source/compiler IDs; distinguish host/ARM, static RAM, runtime stack/heap and added integration features |
| Message comparison | B3 event 16->13 bytes; aggregate 50 bytes; B2 13-byte encoder payload; B4 12-byte result; B5 84-byte payload | Inventory all relevant queue/wire records and accurate before sizes; record command/ADC changes when implemented. A new format has no observed old packet by default. |
| Correctness/performance | Host regressions, analysis, codec vectors, firmware builds and Linux sanitizers | Target latency/jitter/load, runtime memory, network behavior and physical accuracy/safety evidence on the final integrated firmware |

The report does not support signing off any Buddy as fully Week-13-complete.
B4's principal subsystem-specific software is substantially implemented, but
the shared odometry and team interfaces need the remaining CR1 pass. Await the
next main, reconcile the candidate, close the software gaps by owner, then
collect final-profile hardware evidence and merge when authorized.
