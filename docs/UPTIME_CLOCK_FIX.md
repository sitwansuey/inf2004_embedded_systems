# Shared operating-time clock after USB bring-up

The 22 October 2026 COM5 recording confirmed working USB/controller reports
and usable LSM303 accelerometer/magnetometer samples, but exposed two issues.
One incomplete console record was incorrectly classified by the host recorder
as unexpected motor state. Old buffered console data also produced gaps in the
capture. The recorder now discards a short opening backlog and treats malformed
records separately from complete moving/fault records.

All complete controller records in the first capture were disarmed, command
STOP, zero power and fault zero. IMU initialization succeeded, reported I/O
errors remained zero and axis values varied. No fresh valid ultrasonic range
was indicated; IR was deliberately disabled. This does not measure motor pin
levels, sensor accuracy or stopping distance.

Application deadlines, leases, sample timestamps and telemetry now consistently
use `tk_get_otm`, the kernel operating-time API. `tk_get_tim` is the wall-clock
API; before it is set, TRON time can be negative. The previous unsigned 64-bit
conversion and multiplication in Buddy 4 wrapped, so its printed milliseconds
did not match controller milliseconds. No kernel or peripheral ownership was
changed, and all counter/message widths and the drive lock remain unchanged.

Verification: all ten Buddy 2 native configurations passed, Buddy 3 algorithm,
adapter and ADC suites passed, real kernel-header checks passed for SMP 0/1,
Buddy 4 driver/data/API tests passed and the actual binary command queue checks
passed. New regression S4-U22 supplies an unset/changed wall clock alongside
operating time and proves the public IMU clock never reads wall time. Existing
32-bit rollover, 64-bit IMU clock, clock errors and command expiry checks pass.

The updated firmware still requires an ARM build and physical reflash through
the reviewed helper from the user's normal PowerShell session. A new banner
identifies the uptime build. The helper rejects the earlier locked image if
that banner is absent. Sensor calibration and motion validation remain pending.
