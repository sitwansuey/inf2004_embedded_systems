# USB bench motor lock, 22 October 2026

Requested drive motor disable is the default compiled configuration:
`BOARD_MOTORS_DISABLED=1`. Buddy 2 holds its four drive inputs GPIO-low and
does not select or enable their PWM. Both the send API and lower motor-apply
function reject nonzero movement. The vehicle controller rejects ARM. STOP and
encoder diagnostics remain usable. Servo operation is separate from drive.
USB/MQTT cannot clear the lock; it requires deliberate configuration and rebuild.

Verified using actual Buddy 2 sources with native GCC and simulated RTOS/GPIO:
`app_program/subsystem2/tests/host/run.ps1 -Compiler C:/msys64/ucrt64/bin/gcc.exe`.
All eight existing configurations passed with drive explicitly enabled only
in simulation. Two new configurations use the real default lock (integrated
with encoders and standalone without encoders): all movement commands rejected,
zero nonzero-duty writes, no motor PWM mux/enabling, STOP works, encoders work.

ARM rebuild attempted through `tools/build_windows.ps1 -Profile Default` and
the existing Pico SDK. This session returned `Wsl/E_ACCESS_DENIED`; no new
locked UF2 was produced or flashed. Previous default DISARMED UF2 artifacts
predate this hard lock. Native PASS is not an ARM build or hardware test.

Keep motor power off until a new build is identified and flashed to the robot's
Pico W. Holding BOOTSEL while connecting USB prevents the previously installed
application from starting during that connection. Never select the separate
LoRaWAN board. Verify the new boot banner contains `MOTOR LOCK ON` before
relying on this firmware configuration, then measure the actual drive pins.
