# Robot hardware acceptance

No robot was connected, flashed, driven or calibrated during the code work on
21 October 2026. The user asked to work on code first. The robot Pico is separate
from the LoRaWAN Pico; do not select or reflash the latter.

1. Connect the robot Pico W by USB with motor power off. Confirm the board and
   port identity. Photograph/check the motor driver, encoder wiring, sensor
   labels and whether the three IR outputs are AO or DO. Confirm the ultrasonic
   echo interface is safe for the Pico's 3.3 V input.
2. With the robot supported and wheels clear, manually select BOOTSEL on this
   board only. Verify `INFO_UF2.TXT` identifies an RP2040. Review the firmware hash
   and copy only the selected passive/default UF2. No flashing helper runs
   automatically. Initial firmware boots DISARMED and has IR/motion calibration
   disabled.
3. Record the USB output before enabling motion. Check the task startup banner,
   faults, IMU response on each axis, fresh/stale flags, servo direction and
   ultrasonic distance against measured targets. Missing echoes must keep the
   vehicle stopped. Check straight-ahead readings correspond to angle zero.
4. Measure digital polarity or black/white ADC endpoints under the actual
   lighting. Configure `robot_config.local.h`. Three line-facing sensors cannot
   simultaneously serve an independent barcode path. Reserve a physically
   separate barcode sensor and remove it from `BOARD_IR_LINE_MASK`, or keep
   barcode decoding disabled. Set A/Z navigation mappings only from the actual
   course codebook.
5. Use the explicitly selected S2 bench test only with wheels raised. Check all
   seven motor commands, zero PWM on STOP, both encoder directions, one complete
   wheel revolution in x1 counts, and command-timeout shutdown. Check the motor
   and encoder sign conventions before recording circumference and track width.
   Correct those constants in Buddy 2 if the physical wiring differs.
6. Establish rigid IMU mounting with X forward, Y left and Z up. Record flat
   pitch/roll offsets and known tilt responses. Set the terrain calibration flag
   only after validation. Height is a tilt/encoder geometric estimate; compare
   against measured humps at several speeds. Reject slip, shocks, reversal,
   incomplete traversal and dynamic acceleration. No gyroscope or accuracy
   guarantee is provided. Absolute compass heading is not implemented.
7. Start at low power on a controlled line course, with a nearby operator and
   manual motor power cutoff. USB `a` renews a 500 ms tracking lease; `s` stops;
   `r` resets a latched fault/new-run maximum while stopped. Stop renewal, unplug
   USB, disconnect a sensor, lose the line and saturate communications. Record
   electrical PWM shutdown and physical stopping distance separately. The
   software polls at 10 ms; a measured worst-case response is still required.
8. Validate scan width against known objects and backgrounds. All edge/clearance
   decisions require real echo returns. No echo is unknown space. The geometric
   interval models angle quantization, not beam width, material reflection or
   measured accuracy. Confirm chassis clearance, servo settling and encoder
   travel before enabling `BOARD_BYPASS_VALIDATED`. Test all avoidance stages,
   new obstacles during turns, aborts and stopped line reacquisition.
9. With real local Wi-Fi/broker configuration, build `NET=lwip MQTT=1`, run the
   dashboard, and compare binary values with serial/measured references. Test
   broker loss, reconnect, stale packets, malformed/fragmented commands and
   lease expiry while motors operate. MQTT requires RESET then ARM after each
   connection; never retain command messages. Audio playback is not implemented.

Record logs/video, actual commit, supply/wiring, calibration, speed, reference
measurements and failures in `testing/system/TEST_LOG.md`. Host PASS and firmware
link PASS do not substitute for these results.
