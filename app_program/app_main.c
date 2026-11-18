#include "app.h"
#include "app_state.h"
#include "robot_ir_config.h"
#include "subsystem2/subsystem2.h"
#include "subsystem3/subsystem3.h"

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
    tm_printf((UB *)"Boot DISARMED; USB s=STOP, r=reset fault, a=500 ms track lease, g=autonomous mission START. MQTT EDR=%u bytes.\n",EDR_RECORD_SIZE);
    if(BOARD_MOTORS_DISABLED)tm_printf((UB *)"MOTOR LOCK ON: drive GPIOs held low; all movement/ARM commands rejected.\n");
    tm_printf((UB *)"Clock: kernel operating time (monotonic uptime)\n");
    ER er=E_OK;
#if APP_S2_ONLY
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
