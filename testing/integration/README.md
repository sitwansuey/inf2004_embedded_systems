# CSC02 integration verification

Run `make -C testing/integration test` on a native Linux/WSL compiler. Run
`make -C testing/integration sanitize` for AddressSanitizer and UBSan.

The tests execute the actual controller policy, odometry, terrain processor,
obstacle scan/geometry, event codec, binary codec, command/telemetry handoff and
MQTT port. The RTOS, GPIO and transport boundaries use deterministic substitutes.
These tests establish software behavior, not physical sensor or timing accuracy.

| ID | Checks |
|---|---|
| INT-CR1-01 | Safety arbitration, command leases, wrap, turn completion, stopped phase handoffs, bounded bypass and reacquisition |
| INT-CR1-02 | Encoder wrap, scale, speed, relative yaw, stale gaps and discontinuities |
| INT-CR1-03 | Mounting gate, tilt, ordered ascent/crest/descent, completed hump maximum, shock and invalid data |
| INT-CR1-04 | Angle association, 200 mm stop threshold, echo failure, motion during scanning, width bounds |
| INT-CR1-05 | Independent C/Python wire agreement, 302 vectors, 74 out-of-range inputs, length/version/padding rejection |
| INT-CR1-06 | Copied queues, full-buffer behavior, urgent STOP, invalidated old ARM requests, stale telemetry and 13-byte events |
| INT-CR1-07 | Actual MQTT callbacks with fake transport: fragments, binary NULs, backpressure, stale pending frame, reconnect STOP and retry guard |
| INT-CR1-08 | Production sonar driver with GP1 Trigger/GP0 Echo, pulse timing, echo faults, timer wrap/start failure and narrow status publication |
| INT-CR1-09 | Preempted S5 publisher handoff, bounded copy/control waits, controller ownership, timeout/unlock errors and lifecycle |
| INT-CR1-10 | Production telemetry task and real codec: post-copy age reference, new/stale snapshots, failed copies/clock and line age |

`protocol_golden.json` records deterministic reference packets. The schema is
`tools/telemetry_schema.json`. No live MQTT connection is made by these tests.

The existing Buddy 2 native suite also checks a full movement pool followed by
urgent STOP, no energization from pre-STOP messages, and controller ownership.

Physical acceptance remains separate: refer to `docs/HARDWARE_ACCEPTANCE.md`.
