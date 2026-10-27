# Bounded S5 status copies and post-copy freshness

The 22 October 2026 04:33 pin-fix firmware restored ultrasonic acquisition:
12 fresh valid ranges, two no-fall timeouts and zero no-rise results were
recorded. However, 16 of 30 status copies were unavailable. The capture did
not include copy error codes, so not every failed copy can be attributed to
a specific kernel error.

The code has a reproducible contention failure. S5 publishes at priority 20
on core 1, while telemetry runs at priority 13 on that core. Telemetry can
preempt S5 between locking and unlocking its status mutex. TMO_POL then gives
up immediately; it does not enqueue a priority-inheriting waiter to let the
publisher finish. The core 0 controller also uses the same snapshot API.

The public S5 APIs now request a finite 5 ms mutex wait with the existing
TA_INHERIT mutex, matching the existing S4 approach. The kernel tick is 10 ms:
this is a requested timeout, not a claim of a hard 5 ms wall-clock bound.
No GPIO, sensor timing, peripheral owner, kernel source or MQTT layout changes.
Lock/unlock errors propagate; failed reads do not fall back to cached data.
Startup rejects an unsuccessful S5 controller bind, and lifecycle checks reject
starting before initialization or allocating a duplicate status mutex.

Controller and telemetry freshness clocks are obtained after copying the
snapshots. Otherwise a new sample published during a wait could appear newer
than the reader's old clock and unsigned age subtraction could mark it stale.
Telemetry also reevaluates line freshness after the possible wait. If its age
clock fails, no packet is published for that iteration. The controller retains
its STOP behavior on clock failure and unavailable/stale required sensors.

Motor-locked S5RAW output now appends error=<kernel copy return code>; Echo
result remains a separate field. The recorder accepts both old and new formats
and counts unavailable copies and error codes. The binary packet stays 50 bytes
and s5_status_t has no new fields. The new startup marker lets the build/flash
helper reject the previous image. Nothing was pushed or remotely modified.

Native checks pass: actual API with a preempted-publisher fixture, timeout and
unlock errors, controller ownership and lifecycle; a negative control using
the old polling implementation fails as expected. The production telemetry
task with the real codec passes new/stale snapshot, failed-copy/clock and line
age cases. Real kernel headers compile for SMP 0/1 and lock 0/1; lock=0 was
only a syntax check. See the integration test log for commands/results.

The corrected status-copy source still requires an ARM build and physical
capture. The on-board pin-fix build remains locked and provides basic ranging;
its static RAM was 26008 bytes (data 456 + bss 25552). Do not claim the copy
loss rate, calibrated distance accuracy, electrical motor shutdown or stopping
distance has been validated by the native fixtures.
