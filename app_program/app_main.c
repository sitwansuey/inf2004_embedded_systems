#include "app.h"
#include "app_state.h"
#include "robot_ir_config.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"
#include "shared/task_timing.h"
#if !APP_S2_ONLY && !APP_S4_ONLY
#include "app_watchdog.h"
#endif

#if !APP_S2_ONLY && !APP_S4_ONLY && !APP_S5_DIAGNOSTICS && BOARD_IR_MODE != 2 && BOARD_IR_MODE != 3
/* The vehicle controller follows the line with the two calibrated ADC
 * straddle sensors (GP26/GP27, robot_ir_calibration.h). With mode 1 (three
 * digital sensors) or 0 (no IR) it has no calibration and stops at boot. */
#error The integrated robot build needs BOARD_IR_MODE 3 (ADC line sensors, digital GP28 barcode) or 2 (all ADC); modes 0 and 1 cannot drive
#endif

#if BOARD_TIMING_STATS
/* Loop timing records (shared/task_timing.h): here because every firmware
 * build links this file. Each loop writes only its own entry. */
task_timing_t app_task_timing[TASK_TIMING_COUNT];
#endif

/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	return app_start_task_stack(entry, pri, prc, name, TASK_STACK_SIZE);
}

/* The same with a stack size other than TASK_STACK_SIZE (app.h). */
EXPORT ID app_start_task_stack(FP entry, PRI pri, UW prc, const char *name, INT stack_bytes)
{
	T_CTSK	ctsk = {0};
	ID	tid;
	ER	er;

	ctsk.task    = entry;		/* function the task runs */
	ctsk.itskpri = pri;		/* initial priority (1 = highest) */
	ctsk.stksz   = stack_bytes;	/* kernel allocates the stack */
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

#if !APP_S2_ONLY && !APP_S4_ONLY
static ER configure_line(void) {
    if(BOARD_IR_MODE==S3_IR_DISABLED)return E_OK;
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
}
#endif
EXPORT INT usermain(void) {
    tm_printf((UB *)"\nCSC02 CR1 / micro T-Kernel 3, SMP=%u, encoders=%u\n",TK_SUPPORT_SMP,S2_ENCODER_ENABLED);
#if !APP_S2_ONLY && !APP_S4_ONLY
    /* Why the previous run ended; before any task runs. After repeated
     * early crashes: hold here, nothing started, the line repeated. */
    if(app_reset_capture())app_boot_hold_loop();
#endif
    tm_printf((UB *)"Boot DISARMED; USB s=STOP, r=reset fault, a=500 ms track lease, g=autonomous mission START. MQTT EDR=%u bytes.\n",EDR_RECORD_SIZE);
    if(BOARD_MOTORS_DISABLED)tm_printf((UB *)"MOTOR LOCK ON: drive GPIOs held low; all movement/ARM commands rejected.\n");
    tm_printf((UB *)"Clock: kernel operating time (monotonic uptime)\n");
    ER er=E_OK;
#if APP_S2_ONLY
    er=subsystem2_init();
    if(er>=0)er=subsystem2_start();
#if APP_TEST
    /* Test image (make TEST=1 S2_ONLY=1): S2's passive checks on their own.
     * They read the motion status and encoders; no motor is driven. */
    if(er>=0) {
        tm_printf((UB *)"[TEST] === test run start ===\n");
        subsystem2_test();
        tm_printf((UB *)"[TEST] === test run end ===\n");
    }
#endif
#elif APP_S4_ONLY
    er=subsystem4_init();
    if(er>=0)er=subsystem4_start();
#if APP_TEST
    /* Test image (make TEST=1 S4_ONLY=1): the IMU checks on their own. */
    if(er>=0) {
        tm_printf((UB *)"[TEST] === test run start ===\n");
        subsystem4_test();
        tm_printf((UB *)"[TEST] === test run end ===\n");
    }
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
    /* Test image (make TEST=1 NET=lwip MQTT=1): run every subsystem's
     * passive on-target checks in turn, here in the initial task, while the
     * system runs (a few seconds: each test waits, bounded, with
     * tk_dly_tsk() for its tasks' first samples). They only read state;
     * none sends a motion command, moves the servo or changes the
     * controller (S2-U33 checks that a STOP from this task is refused).
     * The initial task has a 4 KB stack in test images (inittask.h). */
    if(er>=0) {
        tm_printf((UB *)"[TEST] === test run start ===\n");
        subsystem1_test();
        subsystem2_test();
        subsystem3_test();
        subsystem4_test();
        subsystem5_test();
        tm_printf((UB *)"[TEST] passive integrated checks completed; no motor test requested\n");
        tm_printf((UB *)"[TEST] === test run end ===\n");
    }
#endif
#endif
    if(er<0)tm_printf((UB *)"[STARTUP FAIL] %d; controller not armed\n",er);
    else tm_printf((UB *)"Selected subsystems started. Calibration and physical validation required before motion.\n");
#if !APP_S2_ONLY && !APP_S4_ONLY
    /* Watchdog supervisor (it arms itself once boot has finished), then the
     * repeated "[RESET]" line and the arming report. */
    if(er>=0)CHECK(app_watchdog_start(),"watchdog supervisor");
    app_watchdog_report_loop();
#endif
    for(;;)tk_slp_tsk(TMO_FEVR);
}
