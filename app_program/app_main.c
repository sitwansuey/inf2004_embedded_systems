#include "app.h"
#include "subsystem2/subsystem2.h"
#include "barcode_line_follow.h"

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = TASK_STACK_SIZE;	/* kernel allocates the stack */
	ctsk.tskatr  = TA_HLNG		/* written in a high-level language (C) */
		     | TA_RNG3;		/* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0) {
		ctsk.tskatr |= TA_ASSPRC;	/* honour the affinity below */
		ctsk.assprc  = prc;		/* which core(s) it may run on */
	}
#else
	(void)prc;			/* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK) {
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	if (er < E_OK) {
		CHECK(tk_del_tsk(tid), "task start cleanup");
		return er;
	}
	return tid;
}


#include "app_state.h"
#include "robot_ir_config.h"
#include "subsystem3/subsystem3.h"
#include "subsystem3/drv_ir_adc.h"
#if TM_MQTT
#include "../lib/libwifi/sysdepend/pico_rp2040/cyw43_utk.h"
#endif
#include "subsystem5/subsystem5.h"
#include "subsystem1/telemetry_codec.h"
#if !APP_S2_ONLY && !APP_S4_ONLY && !BOARD_WHEEL_CAL_BENCH
static ER configure_line(void) {
    if(BOARD_IR_MODE==S3_IR_DISABLED)return E_OK;
#if BOARD_IR_RAW_BENCH
    const s3_ir_config_t raw = {BOARD_IR_MODE,{26,27,28}};
    return s3_configure_raw(&raw);
#else
    s3_adapter_config_t c={0};s3_config_defaults(&c.algorithms);
    c.ir.mode=BOARD_IR_MODE;c.ir.pin[0]=26;c.ir.pin[1]=27;c.ir.pin[2]=28;
    robot_ir_apply_layout(&c.algorithms);
    if(BOARD_IR_MODE==S3_IR_DIGITAL) {
        c.algorithms.line.raw_max=1;
        for(unsigned i=0;i<3;++i) {c.algorithms.line.black[i]=BOARD_IR_BLACK_LEVEL;c.algorithms.line.white[i]=1-BOARD_IR_BLACK_LEVEL;}
    } else {
        if(!robot_ir_apply_adc_calibration(&c.algorithms)) return E_NOSPT;
    }
    return s3_configure(&c);
#endif
}
#endif
EXPORT INT usermain(void) {
    T_RVER ver;tk_ref_ver(&ver);
    tm_printf((UB *)"\nCSC02 CR1 / micro T-Kernel 3, SMP=%u, encoders=%u\n",TK_SUPPORT_SMP,S2_ENCODER_ENABLED);
#if BOARD_WHEEL_CAL_BENCH
    tm_printf((UB *)"WHEEL CAL: bounded forward ARM only; clear floor path or wheels raised; cap30; PI=%u\n",BOARD_PID_ENABLED);
    tm_printf((UB *)"WHEEL CAL: boot stopped, no auto/START; encoder and motor only; ADC/line/sonar inactive\n");
    tm_printf((UB *)"WHEEL CAL: forward lease maximum3000ms; onboard expiry; no renewal\n");
#if BOARD_PID_ENABLED
    tm_printf((UB *)"WHEEL CAL: fractional PWM feedback; forward target500/%u at command30\n",
        (30U*BOARD_PID_MAX_CPS/100U)*BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE/1000U);
#endif
#elif BOARD_BARCODE_DEMO
#if BOARD_PID_ENABLED
    tm_printf((UB *)"BARCODE DEMO30: motors ENABLED; measured wheel PI; cap30; ADC GP26/27; DIGITAL GP28\n");
    tm_printf((UB *)"BARCODE WHEEL PI: fractional PWM; forward balance=%u/%u permille\n",
        BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE,BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE);
#else
    tm_printf((UB *)"BARCODE DEMO30: motors ENABLED; equal PWM; PID off; ADC GP26/27; DIGITAL GP28\n");
#endif
    tm_printf((UB *)"BARCODE DIGITAL: GP28 DO black_level=%u min_run=%ums sample=10ms\n",
        BOARD_BARCODE_DIGITAL_BLACK_LEVEL,BOARD_BARCODE_MIN_RUN_MS);
    tm_printf((UB *)"BARCODE DEMO30: invalid capture=discard/retry after WHITE; retry=%u\n",BOARD_BARCODE_RETRY);
    tm_printf((UB *)"BARCODE DEMO30: no decoded/mapped instruction=FORWARD; default=%u\n",BOARD_BARCODE_DEFAULT_FORWARD);
    tm_printf((UB *)"BARCODE DEMO30: white-centre; sonar/terrain bypass DISABLED; encoder turns\n");
    tm_printf((UB *)"BARCODE DEMO30: wait MQTT map + WHITE/fresh sensors 3000ms; auto once per boot=%u\n",BOARD_BARCODE_AUTO_START);
    tm_printf((UB *)"BARCODE MQTT DIAG: v1 widths29 QoS1 heartbeat1000ms; EDR cursor fair\n");
    tm_printf((UB *)"BARCODE CAPTURE: active scan survives both-black; junction needs clearance then new detection\n");
    tm_printf((UB *)"BARCODE RESYNC: retry capture slides up to8 leading pulse pairs; full frame and quiet required\n");
    tm_printf((UB *)"BARCODE WIDTH FIT: compensated separation; joint bounded edge/gap fit\n");
#if BOARD_BARCODE_SCAN_STEERING
    tm_printf((UB *)"BARCODE LINE CORRECTION: forward-only analog P; cap=%upermille\n",BARCODE_SCAN_MAX_STEERING);
#endif
#elif BOARD_IR_CONTINUOUS_BENCH
    tm_printf((UB *)"IR CONTINUOUS BENCH v6: boot STOPPED; supervised USB start only\n");
    tm_printf((UB *)"IRLINE continuous: 30pct 20000ms pattern-stop=0 encoder-stop=0\n");
    tm_printf((UB *)"IRLINE emergency STOP/USB lease/ADC/motor faults retained; ultrasonic DISABLED\n");
#elif BOARD_IR_LINE_ONLY_BENCH
    tm_printf((UB *)"IR LINE ONLY BENCH v5: boot STOPPED; ultrasonic DISABLED; supervised USB start only\n");
    tm_printf((UB *)"IRLINE limits: PWM<=50%% run<=20000ms heartbeat<250ms motor-watchdog<=150ms progress<=750ms\n");
    tm_printf((UB *)"IRLINE no obstacle protection; white cannot prove line presence; no navigation\n");
    tm_printf((UB *)"IRLINE course: cross=straight recent-side=hold bounded=500/800ms\n");
#elif BOARD_IR_LINE_TRACE_BENCH
    tm_printf((UB *)"IR LINE TRACE BENCH v2: boot STOPPED; both white=assumed centre; supervised USB start only\n");
    tm_printf((UB *)"IRLINE limits: PWM<=20%% run<=6000ms heartbeat<250ms motor-watchdog<=150ms\n");
    tm_printf((UB *)"IRLINE GP27=left GP26=right; both black=STOP; white cannot prove line presence\n");
#elif BOARD_IR_MOTOR_SCAN_BENCH
    tm_printf((UB *)"IR MOTOR SCAN BENCH v5: boot STOPPED; USB framed start only; no navigation\n");
    tm_printf((UB *)"IRSCAN barcode: min_run=20ms sample=10ms bias<=half_narrow\n");
    tm_printf((UB *)"IRSCAN limits: PWM<=20%% run<=6000ms heartbeat<250ms motor-watchdog<=150ms\n");
#else
    tm_printf((UB *)"Boot DISARMED; USB s=STOP, r=reset fault, a=500 ms track lease, g=autonomous mission START. MQTT EDR=%u bytes.\n",EDR_RECORD_SIZE);
#endif
    if(BOARD_MOTORS_DISABLED)tm_printf((UB *)"MOTOR LOCK ON: drive GPIOs held low; all movement/ARM commands rejected.\n");
    tm_printf((UB *)"Clock: kernel operating time (monotonic uptime)\n");
#if BOARD_IR_RAW_BENCH
    tm_printf((UB *)"IR RAW BENCH: input-only; no calibrated line or barcode authority\n");
    tm_printf((UB *)"[S3] raw input mode=%u pins=GP26,GP27,GP28\n",BOARD_IR_MODE);
#if BOARD_IR_MODE == 2
    tm_printf((UB *)"[S3] ADC timing: conversion<%ums reply<%ums\n",
              S3_ADC_DEADLINE_MS,S3_ADC_REPLY_DEADLINE_MS);
    tm_printf((UB *)"[S3] ADC acquisition: bounded reply wait; no prefetched next scan\n");
#endif
#endif
#if BOARD_IR_CALIBRATED_BENCH
    tm_printf((UB *)"IR CALIBRATED BENCH: measured endpoints; motor lock required; no motion test\n");
    tm_printf((UB *)"[S3] roles: GP26=right GP27=left GP28=barcode line-mask=3\n");
    {
        const uint16_t white[S3_SENSOR_COUNT]=BOARD_IR_ADC_WHITE;
        const uint16_t black[S3_SENSOR_COUNT]=BOARD_IR_ADC_BLACK;
        tm_printf((UB *)"[S3] ADC endpoints white=%u,%u,%u black=%u,%u,%u\n",
                  (unsigned)white[0],(unsigned)white[1],(unsigned)white[2],
                  (unsigned)black[0],(unsigned)black[1],(unsigned)black[2]);
    }
    tm_printf((UB *)"[S3] paused scan gaps reset observations; TRACK gaps still fault\n");
#endif
#if BOARD_IR_BARCODE_CAPTURE
#if BOARD_IR_LINE_TRACE_BENCH
    tm_printf((UB *)"IRLINE capture: all ADC scans; steering uses only GP26/27\n");
#elif BOARD_IR_MOTOR_SCAN_BENCH
    tm_printf((UB *)"IRSCAN capture: GP28 ADC; unchanged A/Z decoder; timed forward diagnostic only\n");
#else
    tm_printf((UB *)"IR BARCODE CAPTURE: copied scans; motor locked; host decoder replay\n");
#endif
    tm_printf((UB *)"[S3] scan schedule: cyclic10; capture receive without status mutex\n");
#endif
    ER er=E_OK;
#if BOARD_WHEEL_CAL_BENCH
    IMPORT ER wheel_calibration_start(void);
    er=subsystem1_init();
    if(er>=E_OK)er=subsystem2_init();
    if(er>=E_OK)er=app_state_init();
    if(er>=E_OK)er=subsystem2_start();
    if(er>=E_OK)er=subsystem1_start();
    if(er>=E_OK)er=wheel_calibration_start();
#elif APP_S2_ONLY
    er=subsystem2_init();
    if(er>=0)er=subsystem2_start();
#if APP_TEST
    if(er>=0)subsystem2_test(); /* explicitly selected raised-wheel bench profile */
#endif
#elif APP_S4_ONLY
    er=subsystem4_init();
    if(er>=0)er=subsystem4_start();
#if APP_TEST
    if(er>=0)subsystem4_test();
#endif
#else
    er=subsystem1_init();
    if(er>=0)er=subsystem2_init();
    if(er>=0)er=subsystem3_init();
    if(er>=0)er=configure_line();
    if(er>=0)er=subsystem4_init();
    if(er>=0)er=subsystem5_init();
    if(er>=0)er=app_state_init();
    /* All shared objects exist before any application task starts. */
    if(er>=0)er=subsystem2_start();
#if TM_MQTT
    /* Radio firmware upload can preempt ADC work for >30 ms at boot.
     * Wait while stopped, before opening the one-shot ADC transport. Never
     * relax acquisition deadlines or recover an unsafe conversion in place. */
    if(er>=0) {
        SYSTIM began,current;
        er=tk_get_otm(&began);
        while(er>=0) {
            T_CYW43_UTK_STATUS radio={0};cyw43_utk_get_status(&radio);
            if(radio.init_complete)break;
            s2_emergency_stop();
            er=tk_get_otm(&current);
            if(er<0)break;
            if(current.lo-began.lo>=10000U){er=E_TMOUT;break;}
            er=tk_dly_tsk(10);
        }
    }
#endif
    if(er>=0)er=subsystem3_start();
    if(er>=0)er=subsystem4_start();
    if(er>=0)er=subsystem5_start();
    if(er>=0)er=subsystem1_start();
#if !APP_S5_DIAGNOSTICS
    if(er>=0)er=vehicle_start();
#else
    if(er>=0)tm_printf((UB *)"Explicit S5 diagnostic owns the hardware; integrated controller not started.\n");
#endif
#if APP_TEST
    if(er>=0) {subsystem5_test();tm_printf((UB *)"[TEST] passive integrated checks completed; no motor test requested\n");}
#endif
#endif
    if(er<0)tm_printf((UB *)"[STARTUP FAIL] %d; controller not armed\n",er);
    else tm_printf((UB *)"Selected subsystems started. Calibration and physical validation required before motion.\n");
    for(;;)tk_slp_tsk(TMO_FEVR);
}
