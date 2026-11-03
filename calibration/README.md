# RoboPico calibration checkpoint (2026-11-03)

To reproduce the USB wheel or line test on this robot, copy the profile from the repository root before building:

    Copy-Item calibration/robot_config.local.h app_program/robot_config.local.h

Keep any existing local overrides before replacing that file. Build the IRMotorScan or IRLineTrace profile. The settings apply only to those two bench profiles. The reviewed copy under calibration belongs on Git; the active per-checkout copy and config/wifi_secrets.h are ignored. Put subsequent measured changes in the reviewed copy and sync it to the active copy.

- Wheel geometry: 1,889 left and 1,894 right counts over three revolutions. Use 631 counts/rev, 220 mm rollout and 113 mm tyre-centre spacing.
- Straight-drive baseline: left/right trim 73/98, PID Kp/Ki/Kd 30/10/0 (milli units), 20% output cap. Target was 375 counts/s; the Ki-10 floor run recorded 265 left and 274 right in the final second. Speed calibration is unfinished.
- Line following: latest firmware applies a further 990/1000 to the right PWM after PID rounding. This is a trial value; short passes did not establish an improvement over 995/1000. The motor and line tests use a 200 mm forward clearance threshold. Intermittent echoes can stop a run.
- IR endpoints (right, left, barcode) remain in app_program/robot_ir_calibration.h: white 187/189/197, black 3214/3177/2472. Barcode capture and replay are next: GP28 saw contrast but the A sweeps were incomplete. Check one narrow bar and its neighbouring white gap first.
- Magnetometer reported valid XYZ samples. Original vs returned orientation differed, so no compass-heading offsets have been accepted. Terrain mounting, autonomous bypass and PID calibration are not signed off.
