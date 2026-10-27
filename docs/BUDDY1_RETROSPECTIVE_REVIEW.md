# Buddy 1 retrospective integration review

The user authorized merging the integrated software baseline before this review.
This checklist identifies the exact interfaces to inspect; it does not assert
that Buddy 1 has already approved them. The separate buddy1/MQTT branch was not
merged wholesale; reconcile it against this main before later adoption.

## Formats are distinct

| Record | Size | Meaning and implementation |
|---|---:|---|
| Aggregate dashboard telemetry | 50 bytes / 394 bits | Existing csc02/telemetry schema; subsystem1/telemetry_codec.c, tools/telemetry_schema.json and Python dashboard decoder. |
| B2 encoder | 13-byte payload / 15-byte EDR packet | EDR v1 source 2/kind 1; subsystem2/s2_data.c and shared/edr.c. |
| B4 acquisition | 63 bytes / 500 bits | Protected internal record for controller/B1; s4_sensor_get_record plus s4_data_snapshot. Not a replacement MQTT dashboard payload. |
| B4 terrain result | 12 bytes / 93 bits | app_state copied result, checked by subsystem4/result_data.c in both controller and B1. Optional external EDR source 4/kind 2 adds two bytes; tools/buddy4_result.py decodes it. |
| B5 obstacle status | 84-byte payload / 86-byte EDR packet | Published source 5/kind 0, subsystem5/s5_data.c; full diagnostic ranges retained. |

Do not route one format into another decoder by matching a subset of fields.
Record versions, units, byte/bit ordering, lengths and invalid flags are
specified in CR1_PROTOCOL.md, BUDDY4_CR1_STORAGE.md and SUBSYSTEM5_DATA.md.

## Code and behavior to review

- subsystem1/task_telemetry.c obtains compact B4 records, decodes after source
  copies against operating time, and packs the agreed aggregate. Current tilt
  and height availability clear on stale/invalid data; maximum remains retained
  completed-hump history. Invalid S5 ranges/widths do not appear as valid zeroes.
- subsystem1/telemetry.c and network_bridge.h use copied bounded records and
  explicit byte lengths. Embedded NUL bytes must survive; do not restore strlen
  or printable-text validation on binary payloads.
- lib/liblwip/sysdepend/pico_rp2040/mqtt_utk.c publishes only from the existing
  network-owning task. Reconnection, bounded drains, backpressure and stale
  frames are covered by host substitutes; actual network timing remains open.
- USB/MQTT commands share the checked six-byte command contract. STOP bypasses
  the normal command queue, invalidates old motion and remains independent of
  telemetry delivery. Keep the independent B2 watchdog and expiring ARM lease.
- Aggregate controller fault is a reason code, not the shared EDR fault bitmask.
  B4 explicitly maps encoder/IMU bits; do not cast a generic sensor reason into
  an asserted specific sensor fault. Whole-team fault aggregation needs review.
- The logger runs in the telemetry task on state changes, not inside the
  priority-5 controller. It performs no network or peripheral calls.
- Default build has no MQTT enabled and remains motor-locked. Mqtt profile
  requires private ignored wifi_secrets.h; test builds used mock values only.

## Evidence and remaining checks

See testing/subsystem4/merge-verification.json and PR #8's CI results. Native
checks exercise C/Python codecs, embedded zeros, queue-full cases, fragmented
commands, reconnect handling and actual telemetry consumers. Separately verify
real broker delivery, dashboard units/availability/history, disconnect behavior,
latency under concurrent load and any separate B1 payload changes. Preserve the
motor lock for data-only checks. Physical evidence remains to be collected.
