/* Robot-specific measured endpoints, GP26 RIGHT / GP27 LEFT / GP28 barcode.
 * GP26/27: 22 Oct endpoints; 24 Oct independent RIGHT/LEFT/WHITE validation
 * passed 2,418 samples without refitting. GP28 replacement: 24 Oct first
 * WHITE/BLACK means from 806/810 samples; repeat 810/810 wide-bar samples
 * separate WHITE and BLACK. Optical gap unmeasured; thin bars and complete
 * physical barcode A/Z reliability remain unvalidated. Recalibrate after
 * changing mounting, module, lighting or print. Only opt-in calibrated
 * profiles select these values; defaults do not enable IR or motion. */
#ifndef BUDDY3_CALIBRATION_H
#define BUDDY3_CALIBRATION_H
#define BOARD_IR_ADC_CALIBRATED 1
#define BOARD_IR_ADC_WHITE {187U,189U,197U}
#define BOARD_IR_ADC_BLACK {3214U,3177U,2472U}
#endif
