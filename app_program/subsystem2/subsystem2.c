/*
 *	subsystem2.c
 *	Subsystem 2 -- Consumer: applies movement commands from app_main.
 *
 *	APIs shown: tk_cre_mbx, tk_cre_mpf, tk_rcv_mbx
 */

#include "subsystem2.h"
#include "../board_pins.h"
#include <tk/syslib.h>

static ID controller_tid;
EXPORT void s2_controller_bind(void) { controller_tid = tk_get_tid(); }

static ID h_movement_mailbox;
static ID h_movement_pool;

/* PWM duty = compare / (wrap + 1). */
#define MOTOR_PWM_WRAP 6249U

/* Set either to -1 if that wheel rotates backwards during FORWARD.
 * Motor 1 is assumed to be the left wheel, motor 2 the right wheel.
 */
#define MOTOR1_FORWARD_SIGN 1
#define MOTOR2_FORWARD_SIGN (-1) /* motor 2 wiring is reversed */

#if APP_S2_ONLY
/* IO_BANK0 IRQ 13 on core 0. Both channels report edges for diagnostics.
 * B high defines positive counts; swap the wires/sign interpretation if needed.
 * No printing or kernel calls in the interrupt handler. */
#define S2_ENCODER_IRQ 13U
#define S2_ENCODER_INTR (IO_BANK0_BASE + 0xf0U)
#define S2_ENCODER_INTE (IO_BANK0_BASE + 0x100U)
#define S2_ENCODER_EDGE (8U << (BOARD_MOTOR2_ENCODER_A_GPIO * 4U))
#define S2_ENCODER_A_EDGES (12U << (BOARD_MOTOR2_ENCODER_A_GPIO * 4U))
#define S2_ENCODER_B_EDGES (12U << (BOARD_MOTOR2_ENCODER_B_GPIO * 4U))
#define S2_ENCODER_ALL_EDGES (S2_ENCODER_A_EDGES | S2_ENCODER_B_EDGES)
static volatile UW encoder_positive, encoder_negative;
static volatile UW encoder_a_edges, encoder_b_edges;

static void encoder_interrupt(UINT intno)
{
    (void)intno;
    UW pending = in_w(S2_ENCODER_INTR) & S2_ENCODER_ALL_EDGES;
    if (pending) {
        out_w(S2_ENCODER_INTR, pending);
        if (pending & S2_ENCODER_A_EDGES) ++encoder_a_edges;
        if (pending & S2_ENCODER_B_EDGES) ++encoder_b_edges;
        if (pending & S2_ENCODER_EDGE) {
            if (gpio_get_val(BOARD_MOTOR2_ENCODER_B_GPIO)) ++encoder_positive;
            else ++encoder_negative;
        }
    }
}

static ER encoder_init(void)
{
    ER er = gpio_set_pin(BOARD_MOTOR2_ENCODER_A_GPIO, GPIO_MODE_IN);
    if (er < E_OK) return er;
    er = gpio_set_pin(BOARD_MOTOR2_ENCODER_B_GPIO, GPIO_MODE_IN);
    if (er < E_OK) return er;
    /* Pull up for open-collector outputs; input voltage must be 3.3 V. */
    set_w(GPIO(BOARD_MOTOR2_ENCODER_A_GPIO), GPIO_PUE | GPIO_SHEMITT);
    clr_w(GPIO(BOARD_MOTOR2_ENCODER_A_GPIO), GPIO_PDE);
    set_w(GPIO(BOARD_MOTOR2_ENCODER_B_GPIO), GPIO_PUE | GPIO_SHEMITT);
    clr_w(GPIO(BOARD_MOTOR2_ENCODER_B_GPIO), GPIO_PDE);
    T_DINT cfg = {0};
    cfg.intatr = TA_HLNG;
    cfg.inthdr = (FP)encoder_interrupt;
    er = tk_def_int(S2_ENCODER_IRQ, &cfg);
    if (er < E_OK) return er;
    out_w(S2_ENCODER_INTR, S2_ENCODER_ALL_EDGES);
    set_w(S2_ENCODER_INTE, S2_ENCODER_ALL_EDGES);
    ClearInt(S2_ENCODER_IRQ);
    EnableInt(S2_ENCODER_IRQ, 3);
    tm_printf((UB *)"[S2 ENCODER INIT] core=%d enable=0x%08x expected=0x%08x "
              "GP4_ctrl=0x%08x GP5_ctrl=0x%08x\n", tk_get_prc() - 1,
              in_w(S2_ENCODER_INTE), S2_ENCODER_ALL_EDGES,
              in_w(GPIO_CTRL(BOARD_MOTOR2_ENCODER_A_GPIO)),
              in_w(GPIO_CTRL(BOARD_MOTOR2_ENCODER_B_GPIO)));
    return E_OK;
}

#endif

EXPORT ER s2_encoder_get_status(s2_encoder_status_t *copy)
{
    if (!copy) return E_PAR;
#if APP_S2_ONLY
    /* Each aligned 32-bit counter read is atomic; counters never reset. */
    copy->positive_pulses = encoder_positive;
    copy->negative_pulses = encoder_negative;
    copy->a_edges = encoder_a_edges;
    copy->b_edges = encoder_b_edges;
    UW pins = in_w(GPIO_IN);
    copy->a_level = (pins >> BOARD_MOTOR2_ENCODER_A_GPIO) & 1U;
    copy->b_level = (pins >> BOARD_MOTOR2_ENCODER_B_GPIO) & 1U;
#else
    *copy = (s2_encoder_status_t){0};
#endif
    return E_OK;
}

static void motor_stop(void)
{
	pwm_set_cc(BOARD_MOTOR1_A_GPIO, 0);
	pwm_set_cc(BOARD_MOTOR1_B_GPIO, 0);
	pwm_set_cc(BOARD_MOTOR2_A_GPIO, 0);
	pwm_set_cc(BOARD_MOTOR2_B_GPIO, 0);
}

static void motor_init(void)
{
	const UINT pins[] = {
		BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
		BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO
	};

	/* Hold all driver inputs low before connecting them to PWM. */
	for (UINT i = 0; i < 4; ++i) {
		gpio_set_val(pins[i], 0);
		gpio_set_pin(pins[i], GPIO_MODE_OUT);
	}

	/* GP8/9 share slice 4; GP10/11 share slice 5. */
	pwm_set_enabled(BOARD_MOTOR1_A_GPIO, FALSE);
	pwm_set_enabled(BOARD_MOTOR2_A_GPIO, FALSE);
	pwm_set_wrap(BOARD_MOTOR1_A_GPIO, MOTOR_PWM_WRAP);
	pwm_set_wrap(BOARD_MOTOR2_A_GPIO, MOTOR_PWM_WRAP);
	motor_stop();

	for (UINT i = 0; i < 4; ++i) {
		pwm_set_pin(pins[i]);
	}
	pwm_set_enabled(BOARD_MOTOR1_A_GPIO, TRUE);
	pwm_set_enabled(BOARD_MOTOR2_A_GPIO, TRUE);
}

static void motor_set_pair(UINT pin_a, UINT pin_b, INT direction, UW duty)
{
	/* The inactive direction input stays low. */
	if (direction > 0) {
		pwm_set_cc(pin_a, duty);
	} else if (direction < 0) {
		pwm_set_cc(pin_b, duty);
	}
}

static void motor_apply(s2_move_command_t command, UINT power_percent)
{
	INT left = 0;
	INT right = 0;

	/* Clear both directions before applying a new command. */
	motor_stop();
	if (power_percent == 0 || power_percent > 100U) {
		return;
	}

	switch (command) {
	case S2_MOVE_FORWARD: left = 1; right = 1; break;
	case S2_MOVE_REVERSE: left = -1; right = -1; break;
	case S2_MOVE_LEFT: left = -1; right = 1; break;
	case S2_MOVE_RIGHT: left = 1; right = -1; break;
	case S2_MOVE_PIVOT_RIGHT: left = 1; right = 0; break;
	case S2_MOVE_PIVOT_LEFT: left = 0; right = 1; break;
	case S2_MOVE_STOP:
	default:
		return;
	}

	/* Brief zero-output interval before changing direction. */
	tk_dly_tsk(1);
	UW duty = ((MOTOR_PWM_WRAP + 1U) * power_percent) / 100U;
	motor_set_pair(BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
		       left * MOTOR1_FORWARD_SIGN, duty);
	motor_set_pair(BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO,
		       right * MOTOR2_FORWARD_SIGN, duty);
}

static void motor_task(INT stacd, void *exinf)
{
	(void)stacd;
	(void)exinf;

	motor_init();

#if APP_S2_ONLY
	ER encoder_er = encoder_init();
	CHECK(encoder_er, "S2 encoder init");
	if (encoder_er < E_OK) { motor_stop(); return; }
	tm_printf((UB *)"[S2] motor 2 encoder A=GP4 B=GP5 ready (x1 counts)\n");

#endif
	motor_apply(S2_MOVE_STOP, 0);

	for (;;)
	{
		T_MSG *raw = NULL;
		ER er;

		er = tk_rcv_mbx(
			h_movement_mailbox, &raw, TMO_FEVR);

		if (er < E_OK)
		{
			CHECK(er, "S2 receive");
			motor_apply(S2_MOVE_STOP, 0);
			tk_dly_tsk(10);
			continue;
		}

		s2_movement_msg_t *msg =
			(s2_movement_msg_t *)raw;

		/* Copy before returning the block to the pool. */
		s2_move_command_t command = msg->command;
		UINT power = msg->power_percent;

		er = tk_rel_mpf(h_movement_pool, msg);
		CHECK(er, "S2 release message");

		motor_apply(command, power);
		tm_printf((UB *)"[S2 MOTOR] command=%d power=%u%%\n", command, power);
	}
}
EXPORT ER subsystem2_init(void)
{
	/* USE_PTMR=0 leaves PWM in reset in the kernel startup. Release it
	 * before either S2 or S5 starts; do not reset slices owned by S5. */
	clr_w(RESETS_RESET, RESETS_RESET_PWM);
	UW attempts = 10000;
	while (!(in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM)) {
		if (--attempts == 0) return E_SYS;
	}
	T_CMBX mailbox_config = {0};
	T_CMPF pool_config = {0};

	mailbox_config.mbxatr = TA_TFIFO;

	h_movement_mailbox = tk_cre_mbx(&mailbox_config);
	if (h_movement_mailbox < E_OK)
	{
		return h_movement_mailbox;
	}

	pool_config.mpfatr = TA_TFIFO | TA_RNG3;
	pool_config.mpfcnt = S2_COMMAND_COUNT;
	pool_config.blfsz = sizeof(s2_movement_msg_t);

	h_movement_pool = tk_cre_mpf(&pool_config);
	if (h_movement_pool < E_OK)
	{
		tk_del_mbx(h_movement_mailbox);
		return h_movement_pool;
	}

	return E_OK;
}

EXPORT ER subsystem2_start(void)
{
	ID tid = app_start_task(
		(FP)motor_task,
		S2_PRI_MOTOR,
		TP_PRC1, /* core 0 */
		"motor");

	return (tid < E_OK) ? tid : E_OK;
}

EXPORT ER s2_movement_send(
	s2_move_command_t command,
	UINT power_percent)
{
#if !APP_S2_ONLY
	if (controller_tid == 0 || tk_get_tid() != controller_tid) return E_OACV;
#endif
	void *block = NULL;
	ER er;

	/* Commands persist until replaced by another command. */
	if (power_percent > 100U)
	{
		return E_PAR;
	}

	switch (command)
	{
	case S2_MOVE_STOP:
	case S2_MOVE_FORWARD:
	case S2_MOVE_REVERSE:
	case S2_MOVE_LEFT:
	case S2_MOVE_RIGHT:
	case S2_MOVE_PIVOT_RIGHT:
	case S2_MOVE_PIVOT_LEFT:
		break;

	default:
		return E_PAR;
	}

	er = tk_get_mpf(h_movement_pool, &block, TMO_POL);
	if (er < E_OK)
	{
		return er;
	}

	s2_movement_msg_t *msg =
		(s2_movement_msg_t *)block;

	msg->hdr.msgque[0] = NULL;
	msg->command = command;
	msg->power_percent =
		(command == S2_MOVE_STOP) ? 0 : power_percent;

	er = tk_snd_mbx(h_movement_mailbox, (T_MSG *)msg);
	if (er < E_OK)
	{
		tk_rel_mpf(h_movement_pool, msg);
	}

	return er;
}
