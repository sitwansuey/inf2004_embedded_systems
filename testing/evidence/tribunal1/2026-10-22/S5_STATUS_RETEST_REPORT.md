# Status-copy correction: physical result

22 October 2026. Firmware copied at 04:46:13 +08:00; COM5 captured 30 seconds
from 04:46:19. All 30 status copies succeeded with error=0, all were fresh,
and all returned Echo OK. This improves the previous 14/30 successful copies.
There were 26 valid forward ranges (372..841 mm); four samples were correctly
invalidated during servo movement. Distance variability remains, so calibrated
accuracy is not established. All 30 controller records were stopped/disarmed
and fault-free. All 143 IMU records had usable accelerometer/magnetometer data
and zero reported I/O errors. No commands were sent.

SHA256 c0a4a67a31cf1227d7a35a3cc68fa9450f74f03748d55f2f82a1762f863de49d.
ARM text/data/bss: 326316/456/25552; static RAM 26008 bytes, unchanged.
UF2 653824 bytes (+512), text +176 versus the pin-fix image; telemetry 50 bytes.
This is a short capture, not a long stress test or electrical stop measurement.
Startup banners were drained before capture; the helper checked their compiled
markers. Existing wiring and 3V3 supply were retained. Motor lock stays on.

The user has prioritized Buddy 3 IR bring-up next; encoder mapping is deferred.
