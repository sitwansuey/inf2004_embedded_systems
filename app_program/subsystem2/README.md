# Buddy 2 Motion Control

Student ID: 2500613

This module currently provides open-loop PWM motor commands. Encoder mounting
and wiring are unavailable, so measured speed/distance, PID tuning, straight-line
correction and motion accuracy evaluation remain pending.

## Integration

The team project must supply `../app.h`, `../board_pins.h`, T-Kernel and the board
drivers. Call `subsystem2_init()` and `subsystem2_start()`, checking their return
values. In the vehicle-controller task, call `s2_controller_bind()` before sending
commands. `app_vehicle_request()` remains an external vehicle-controller API;
Buddy 2 does not implement it.

Moving commands now expire **1000 ms after they are sent**. The vehicle controller
must renew `s2_movement_send(command, power_percent)` periodically, for example
every 250 ms, and check each return value. Sending an identical command renews
its lease without interrupting PWM. STOP or zero power switches PWM off; invalid
commands or power above 100 return `E_PAR`.

If renewals stop, the motor task switches all four PWM outputs off and reports
`S2_MOTION_FAULT` with `last_error == E_TMOUT`. Expired queued commands cannot
restart motion. A fresh valid command clears the fault. Configure the timeout
with `S2_COMMAND_TIMEOUT_MS` consistently across the project.

`s2_motion_get_status()` gives a semaphore-protected snapshot of commanded output:
`STOPPED`, `MOVING` or `FAULT`, effective command/power, last error and the last
accepted command timestamp. This is not confirmation that the wheels are moving
or stationary. PWM off does not guarantee braking; that depends on the driver.
The watchdog depends on the motor task continuing to run, and is not a hardware
watchdog. A full command pool can reject even STOP; check the send result.

## Encoders And Hardware Tests

`S2_ENCODER_ENABLED` defaults to `0`. No encoder GPIOs or interrupts are accessed
in this mode; `s2_encoder_get_status()` zeroes its output and returns `E_NOSPT`.
When the hardware is ready, enable diagnostics with `APP_S2_ONLY=1` and
`S2_ENCODER_ENABLED=1`. These diagnostics count pulses only; they are not PID
feedback. They are intentionally not enabled in the integrated build.

Confirm motor 1 is the left wheel and motor 2 the right wheel. Check the forward
signs in `subsystem2.c` and all pins in the team's `board_pins.h`. The report assigns
motor 1 encoder GP4/GP5 and motor 2 encoder GP16/GP17; this code uses the
`BOARD_MOTOR2_ENCODER_A_GPIO` and `BOARD_MOTOR2_ENCODER_B_GPIO` definitions and logs
those actual pin numbers. Reconcile these definitions before wiring an encoder.

With the wheels raised, build with `APP_TEST=1` and `APP_S2_ONLY=1`, then invoke
`subsystem2_test()` after initialization/start. It tests directions for two seconds
with command renewals, explicit STOPs, and one deliberate watchdog timeout. With
encoders disabled it skips encoder readings and missing-pulse warnings.

`task_producer.c` is retained because the team uses that task for the vehicle
controller. Its code and callers have not been changed by these motor updates.
The checked-in version on this branch still contains the sample-generator
template. Controller integration must be checked against the team's controller
implementation, including command binding and periodic movement renewal.

## Host Verification

Run `powershell -ExecutionPolicy Bypass -File .\tests\host\run.ps1` with GCC on
PATH. The harness uses simulated RTOS calls, time and GPIO/PWM registers. It checks
command validation, renewal, expiry, stale messages, stop/fault handling and
encoder build modes. A full board build and physical motor testing still require
the team project and hardware.

The optional encoder register layout follows the
[Raspberry Pi SDK definitions](https://github.com/raspberrypi/pico-sdk/blob/master/src/rp2040/hardware_structs/include/hardware/structs/io_bank0.h).
The status semaphore uses the
[SMP T-Kernel semaphore API](https://www.tron.org/wp-content/themes/dp-magjam/pdf/specifications/en_US/TEF021-S002-01.00.00_en.pdf).
