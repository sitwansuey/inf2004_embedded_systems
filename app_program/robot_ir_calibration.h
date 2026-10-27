/* Order: GP26 right line, GP27 left line, GP28 dedicated barcode.
 * GP26/27 retain 22 Oct 2026 floor endpoints (61 white / 60 black samples).
 * GP28 replacement: 24 Oct 2026, actual printed sheet at mounted height.
 * Integer means from 806 WHITE / 810 BLACK samples; independent 810/810
 * repeat samples validate classification without refitting these endpoints.
 * Black strength varied by placement (2449..2493 then 3046..3090); both
 * remain separated from white (177..229). Exact optical gap is unmeasured.
 * See docs/IR_GP28_REPLACEMENT_CALIBRATION.md for logs and limitations.
 * GP26/27 have NOT been revalidated after borrowing/replacing a line module.
 * Selected by IRCalibrated / IRBarcodeCapture (motor locked), and the
 * explicit IRMotorScan USB diagnostic profile (enabled motors, boot STOPPED).
 * Repeat calibration after changing module, mounting, lighting or print.
 */
#ifndef ROBOT_IR_CALIBRATION_H
#define ROBOT_IR_CALIBRATION_H
#define BOARD_IR_ADC_CALIBRATED 1
#define BOARD_IR_ADC_WHITE {187U,189U,197U}
#define BOARD_IR_ADC_BLACK {3214U,3177U,2472U}
#endif
