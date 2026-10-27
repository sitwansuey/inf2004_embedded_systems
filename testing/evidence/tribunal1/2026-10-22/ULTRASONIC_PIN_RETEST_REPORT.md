# Ultrasonic pin correction: physical retest

22 October 2026, COM5. Corrected locked firmware was built and copied at
04:33:31 +08:00; the read-only 30-second capture started at 04:33:36 +08:00.
SHA256: `15644919870cd0cc8b775f6e24aa6c4a74d371e7fef8a6deae318806401fd00b`.

- 30 complete controller records: all stopped, disarmed, fault-free. No commands sent.
- 143 IMU records: accelerometer and magnetometer usable, zero reported I/O errors.
- 30 sonar records: 14 successful copies, including 12 fresh valid ranges
  (138..856 mm) and two no-fall timeouts. No successful copy reports no-rise.
- 16 status copies were unavailable. Zero-filled unavailable records are not ranges.

Restoring TRIG=GP1 / ECHO=GP0 restored echo acquisition with the existing wiring.
The earlier supply diagnosis was premature. This establishes basic acquisition,
not calibrated distance accuracy: the log includes an early 138 mm observation
which triggered scanning, and the 40/60/40 cm prompt sequence was not independently
measured. Servo orientation, target size/position and range accuracy remain open.
Both encoder counters also changed early; physical wheel mapping is still open.

ARM text/data/bss: 326140/456/25552 bytes. Static RAM remains 26008 bytes;
the 50-byte application telemetry packet is unchanged. Compared with the previous
reversed-pin diagnostic image, code grew 48 bytes and UF2 size stayed 653312 bytes.

The build helper verified locked source hashes and the compiled lock marker.
The live capture missed the startup banner after its warm-up drain, so it must
not be described as a live observation of that banner. Electrical motor shutdown,
stopping distance, full obstacle avoidance and sensor voltage levels were not measured.

The next software investigation concerns immediate-poll status mutex reads.
Motor lock remains on. Keep wiring and supply unchanged for the comparison.
