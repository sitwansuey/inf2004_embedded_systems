/*
 * subsystem2.c
 * Buddy 2 -- Open-loop motor command consumer. Student ID: 2500613.
 */

#include "subsystem2.h"
#include "../board_pins.h"
#include <tk/syslib.h>

static ID h_controller_task;
static ID h_movement_mailbox;
static ID h_movement_pool;
static ID h_status_lock;
static BOOL motor_started;
static s2_motion_status_t motion_status;

/* PWM duty = compare / (wrap + 1). */
#define MOTOR_PWM_WRAP 6249U

/* Motor 1 is the left wheel, motor 2 the right wheel.
 * Change either sign if that wheel rotates backwards during FORWARD.
 */
#define MOTOR1_FORWARD_SIGN 1
#define MOTOR2_FORWARD_SIGN (-1)

EXPORT void s2_controller_bind(void)
{
    h_controller_task = tk_get_tid();
}

#if APP_S2_ONLY && S2_ENCODER_ENABLED
/* RP2040 GPIO interrupt registers have eight pins per 32-bit bank. */
#define S2_ENCODER_IRQ 13U
#define S2_ENCODER_A_SHIFT ((BOARD_MOTOR2_ENCODER_A_GPIO % 8U) * 4U)
#define S2_ENCODER_B_SHIFT ((BOARD_MOTOR2_ENCODER_B_GPIO % 8U) * 4U)
#define S2_ENCODER_A_BANK ((BOARD_MOTOR2_ENCODER_A_GPIO / 8U) * 4U)
#define S2_ENCODER_B_BANK ((BOARD_MOTOR2_ENCODER_B_GPIO / 8U) * 4U)
#define S2_ENCODER_A_INTR (IO_BANK0_BASE + 0xf0U + S2_ENCODER_A_BANK)
#define S2_ENCODER_B_INTR (IO_BANK0_BASE + 0xf0U + S2_ENCODER_B_BANK)
#define S2_ENCODER_A_INTE (IO_BANK0_BASE + 0x100U + S2_ENCODER_A_BANK)
#define S2_ENCODER_B_INTE (IO_BANK0_BASE + 0x100U + S2_ENCODER_B_BANK)
#define S2_ENCODER_A_RISING (8U << S2_ENCODER_A_SHIFT)
#define S2_ENCODER_A_EDGES (12U << S2_ENCODER_A_SHIFT)
#define S2_ENCODER_B_EDGES (12U << S2_ENCODER_B_SHIFT)

static volatile UW encoder_positive;
static volatile UW encoder_negative;
static volatile UW encoder_a_edges;
static volatile UW encoder_b_edges;
static BOOL encoder_ready;

static void encoder_interrupt(UINT intno)
{
    UW a_pending = in_w(S2_ENCODER_A_INTR) & S2_ENCODER_A_EDGES;
    UW b_pending = in_w(S2_ENCODER_B_INTR) & S2_ENCODER_B_EDGES;

    (void)intno;
    /* No printing or kernel calls in this interrupt handler. */
    if (a_pending != 0U)
    {
        out_w(S2_ENCODER_A_INTR, a_pending);
        ++encoder_a_edges;
        if ((a_pending & S2_ENCODER_A_RISING) != 0U)
        {
            if (gpio_get_val(BOARD_MOTOR2_ENCODER_B_GPIO) != 0)
            {
                ++encoder_positive;
            }
            else
            {
                ++encoder_negative;
            }
        }
    }
    if (b_pending != 0U)
    {
        out_w(S2_ENCODER_B_INTR, b_pending);
        ++encoder_b_edges;
    }
}

static ER encoder_init(void)
{
    T_DINT config = {0};
    ER er = gpio_set_pin(BOARD_MOTOR2_ENCODER_A_GPIO, GPIO_MODE_IN);

    if (er >= E_OK)
    {
        er = gpio_set_pin(BOARD_MOTOR2_ENCODER_B_GPIO, GPIO_MODE_IN);
    }
    if (er >= E_OK)
    {
        /* Pull up open-collector outputs; input voltage must be 3.3 V. */
        set_w(GPIO(BOARD_MOTOR2_ENCODER_A_GPIO), GPIO_PUE | GPIO_SHEMITT);
        clr_w(GPIO(BOARD_MOTOR2_ENCODER_A_GPIO), GPIO_PDE);
        set_w(GPIO(BOARD_MOTOR2_ENCODER_B_GPIO), GPIO_PUE | GPIO_SHEMITT);
        clr_w(GPIO(BOARD_MOTOR2_ENCODER_B_GPIO), GPIO_PDE);
        config.intatr = TA_HLNG;
        /* T-Kernel uses a generic function pointer for interrupt handlers. */
        config.inthdr = (FP)encoder_interrupt;
        er = tk_def_int(S2_ENCODER_IRQ, &config);
    }
    if (er >= E_OK)
    {
        out_w(S2_ENCODER_A_INTR, S2_ENCODER_A_EDGES);
        out_w(S2_ENCODER_B_INTR, S2_ENCODER_B_EDGES);
        set_w(S2_ENCODER_A_INTE, S2_ENCODER_A_EDGES);
        set_w(S2_ENCODER_B_INTE, S2_ENCODER_B_EDGES);
        ClearInt(S2_ENCODER_IRQ);
        EnableInt(S2_ENCODER_IRQ, 3);
        encoder_ready = TRUE;
        /* tm_printf takes an unsigned-byte format string. */
        tm_printf((UB *)"[S2] motor 2 encoder A=GP%u B=GP%u ready\n",
                  BOARD_MOTOR2_ENCODER_A_GPIO, BOARD_MOTOR2_ENCODER_B_GPIO);
    }
    return er;
}
#endif

EXPORT ER s2_encoder_get_status(s2_encoder_status_t *p_status)
{
    ER er = E_PAR;

    if (p_status != NULL)
    {
        *p_status = (s2_encoder_status_t){0};
#if APP_S2_ONLY && S2_ENCODER_ENABLED
        if (encoder_ready)
        {
            UW pins = in_w(GPIO_IN);

            /* Individual aligned reads are atomic, not a joint snapshot. */
            p_status->positive_pulses = encoder_positive;
            p_status->negative_pulses = encoder_negative;
            p_status->a_edges = encoder_a_edges;
            p_status->b_edges = encoder_b_edges;
            p_status->a_level =
                (pins >> BOARD_MOTOR2_ENCODER_A_GPIO) & 1U;
            p_status->b_level =
                (pins >> BOARD_MOTOR2_ENCODER_B_GPIO) & 1U;
            er = E_OK;
        }
        else
        {
            er = E_OBJ;
        }
#else
        er = E_NOSPT;
#endif
    }
    return er;
}

static void motor_stop(void)
{
    pwm_set_cc(BOARD_MOTOR1_A_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR1_B_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR2_A_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR2_B_GPIO, 0);
}

static ER motor_init(void)
{
    const UINT pins[] =
    {
        BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
        BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO
    };
    ER er = E_OK;

    /* Hold all driver inputs low before connecting them to PWM. */
    for (UINT i = 0; (i < 4U) && (er >= E_OK); ++i)
    {
        gpio_set_val(pins[i], 0);
        er = gpio_set_pin(pins[i], GPIO_MODE_OUT);
    }

    pwm_set_enabled(BOARD_MOTOR1_A_GPIO, FALSE);
    pwm_set_enabled(BOARD_MOTOR2_A_GPIO, FALSE);
    pwm_set_wrap(BOARD_MOTOR1_A_GPIO, MOTOR_PWM_WRAP);
    pwm_set_wrap(BOARD_MOTOR2_A_GPIO, MOTOR_PWM_WRAP);
    motor_stop();
    if (er >= E_OK)
    {
        for (UINT i = 0; i < 4U; ++i)
        {
            pwm_set_pin(pins[i]);
        }
        pwm_set_enabled(BOARD_MOTOR1_A_GPIO, TRUE);
        pwm_set_enabled(BOARD_MOTOR2_A_GPIO, TRUE);
    }
    return er;
}

static void motor_set_pair(UINT pin_a, UINT pin_b, INT direction, UW duty)
{
    if (direction > 0)
    {
        pwm_set_cc(pin_a, duty);
    }
    else if (direction < 0)
    {
        pwm_set_cc(pin_b, duty);
    }
}

static ER motor_apply(s2_move_command_t command, UINT power_percent,
                      UW issued_at_ms)
{
    INT left = 0;
    INT right = 0;
    UW duty = ((MOTOR_PWM_WRAP + 1U) * power_percent) / 100U;
    ER er = E_OK;

    motor_stop();
    switch (command)
    {
        case S2_MOVE_FORWARD:
            left = 1;
            right = 1;
            break;
        case S2_MOVE_REVERSE:
            left = -1;
            right = -1;
            break;
        case S2_MOVE_LEFT:
            left = -1;
            right = 1;
            break;
        case S2_MOVE_RIGHT:
            left = 1;
            right = -1;
            break;
        case S2_MOVE_PIVOT_RIGHT:
            left = 1;
            break;
        case S2_MOVE_PIVOT_LEFT:
            right = 1;
            break;
        case S2_MOVE_STOP:
        default:
            break;
    }
    if ((left != 0) || (right != 0))
    {
        SYSTIM now;

        /* Brief zero-output interval before changing direction. */
        er = tk_dly_tsk(1);
        if (er >= E_OK)
        {
            er = tk_get_tim(&now);
        }
        if ((er >= E_OK) &&
            ((now.lo - issued_at_ms) >= S2_COMMAND_TIMEOUT_MS))
        {
            er = E_TMOUT;
        }
        if (er >= E_OK)
        {
            motor_set_pair(BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
                           left * MOTOR1_FORWARD_SIGN, duty);
            motor_set_pair(BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO,
                           right * MOTOR2_FORWARD_SIGN, duty);
        }
    }
    return er;
}

static BOOL movement_valid(s2_move_command_t command, UINT power_percent)
{
    return (command >= S2_MOVE_STOP) &&
           (command <= S2_MOVE_PIVOT_LEFT) && (power_percent <= 100U);
}

static ER motion_publish(const s2_motion_status_t *p_status)
{
    ER er = tk_wai_sem(h_status_lock, 1, TMO_FEVR);

    if (er >= E_OK)
    {
        /* The semaphore also protects readers on the other core. */
        motion_status = *p_status;
        er = tk_sig_sem(h_status_lock, 1);
    }
    return er;
}

EXPORT ER s2_motion_get_status(s2_motion_status_t *p_status)
{
    ER er = E_PAR;

    if (p_status != NULL)
    {
        er = E_OBJ;
        if (h_status_lock > 0)
        {
            er = tk_wai_sem(h_status_lock, 1, TMO_FEVR);
            if (er >= E_OK)
            {
                *p_status = motion_status;
                er = tk_sig_sem(h_status_lock, 1);
            }
        }
    }
    return er;
}

static ER motion_fault(s2_motion_status_t *p_status, ER error)
{
    motor_stop();
    p_status->state = S2_MOTION_FAULT;
    p_status->command = S2_MOVE_STOP;
    p_status->power_percent = 0;
    p_status->last_error = error;
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *)"[S2 FAULT] error=%d; PWM outputs off\n", error);
    return motion_publish(p_status);
}

static void motor_task(INT stacd, void *p_exinf)
{
    s2_motion_status_t current = {S2_MOTION_STOPPED, S2_MOVE_STOP, 0,
                                  E_OK, 0};
    ER er = motor_init();

    (void)stacd;
    (void)p_exinf;
#if APP_S2_ONLY && S2_ENCODER_ENABLED
    if (er >= E_OK)
    {
        er = encoder_init();
    }
#else
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *)"[S2] open-loop PWM; encoder diagnostics disabled\n");
#endif
    if (er < E_OK)
    {
        CHECK(motion_fault(&current, er), "S2 initialisation fault status");
    }

    while (er >= E_OK)
    {
        T_MSG *p_raw = NULL;
        TMO wait = TMO_FEVR;
        SYSTIM now;

        if (current.state == S2_MOTION_MOVING)
        {
            er = tk_get_tim(&now);
            if (er >= E_OK)
            {
                UW elapsed = now.lo - current.last_command_ms;

                if (elapsed >= S2_COMMAND_TIMEOUT_MS)
                {
                    er = E_TMOUT;
                }
                else
                {
                    /* The configured lease fits a positive TMO value. */
                    wait = (TMO)(S2_COMMAND_TIMEOUT_MS - elapsed);
                }
            }
            if (er < E_OK)
            {
                er = motion_fault(&current, er);
                continue;
            }
        }

        er = tk_rcv_mbx(h_movement_mailbox, &p_raw, wait);
        if (er < E_OK)
        {
            ER receive_error = er;

            er = motion_fault(&current, receive_error);
            if (receive_error != E_TMOUT)
            {
                tk_dly_tsk(10);
            }
            continue;
        }

        /* The mailbox header is the first member of the pooled message. */
        s2_movement_msg_t *p_msg = (s2_movement_msg_t *)p_raw;
        s2_move_command_t command = p_msg->command;
        UINT power = p_msg->power_percent;
        UW issued_at = p_msg->issued_at_ms;

        /* Copy all fields before returning the block to the pool. */
        er = tk_rel_mpf(h_movement_pool, p_msg);
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }
        if (!movement_valid(command, power))
        {
            er = motion_fault(&current, E_PAR);
            continue;
        }

        if ((command == S2_MOVE_STOP) || (power == 0U))
        {
            command = S2_MOVE_STOP;
            power = 0;
        }
        else
        {
            er = tk_get_tim(&now);
            if ((er >= E_OK) &&
                ((now.lo - issued_at) >= S2_COMMAND_TIMEOUT_MS))
            {
                er = E_TMOUT;
            }
            if (er < E_OK)
            {
                er = motion_fault(&current, er);
                continue;
            }
        }

        /* Identical renewals leave PWM running without extra log messages. */
        if ((command != current.command) ||
            (power != current.power_percent))
        {
            er = motor_apply(command, power, issued_at);
            if (er < E_OK)
            {
                er = motion_fault(&current, er);
                continue;
            }
            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *)"[S2 MOTOR] command=%d power=%u%%\n",
                      command, power);
        }
        current.state = (command == S2_MOVE_STOP) ?
                        S2_MOTION_STOPPED : S2_MOTION_MOVING;
        current.command = command;
        current.power_percent = power;
        current.last_error = E_OK;
        current.last_command_ms = issued_at;
        er = motion_publish(&current);
    }
    motor_stop();
    CHECK(er, "S2 motor task stopped");
}

EXPORT ER subsystem2_init(void)
{
    T_CMBX mailbox_config = {0};
    T_CMPF pool_config = {0};
    T_CSEM lock_config = {0};
    UW attempts = 10000U;
    ER er = E_OK;

    if (h_status_lock > 0)
    {
        er = E_OBJ;
    }
    if (er >= E_OK)
    {
        /* Release PWM reset without resetting slices owned by Buddy 5. */
        clr_w(RESETS_RESET, RESETS_RESET_PWM);
        while ((in_w(RESETS_RESET_DONE) & RESETS_RESET_PWM) == 0U)
        {
            if (--attempts == 0U)
            {
                er = E_SYS;
                break;
            }
        }
    }
    if (er >= E_OK)
    {
        mailbox_config.mbxatr = TA_TFIFO;
        h_movement_mailbox = tk_cre_mbx(&mailbox_config);
        er = (h_movement_mailbox > 0) ? E_OK : h_movement_mailbox;
    }
    if (er >= E_OK)
    {
        pool_config.mpfatr = TA_TFIFO | TA_RNG3;
        pool_config.mpfcnt = S2_COMMAND_COUNT;
        pool_config.blfsz = sizeof(s2_movement_msg_t);
        h_movement_pool = tk_cre_mpf(&pool_config);
        er = (h_movement_pool > 0) ? E_OK : h_movement_pool;
        if (er < E_OK)
        {
            tk_del_mbx(h_movement_mailbox);
            h_movement_mailbox = 0;
        }
    }
    if (er >= E_OK)
    {
        lock_config.sematr = TA_TPRI;
        lock_config.isemcnt = 1;
        lock_config.maxsem = 1;
        h_status_lock = tk_cre_sem(&lock_config);
        er = (h_status_lock > 0) ? E_OK : h_status_lock;
        if (er < E_OK)
        {
            tk_del_mpf(h_movement_pool);
            tk_del_mbx(h_movement_mailbox);
            h_movement_pool = 0;
            h_movement_mailbox = 0;
            h_status_lock = 0;
        }
        else
        {
            motion_status = (s2_motion_status_t)
                {S2_MOTION_STOPPED, S2_MOVE_STOP, 0, E_OK, 0};
        }
    }
    return er;
}

EXPORT ER subsystem2_start(void)
{
    ER er = E_OBJ;

    if ((h_status_lock > 0) && !motor_started)
    {
        /* app_start_task takes T-Kernel's generic function pointer. */
        ID h_task = app_start_task((FP)motor_task, S2_PRI_MOTOR,
                                  TP_PRC1, "motor");

        er = (h_task > 0) ? E_OK : h_task;
        if (er >= E_OK)
        {
            motor_started = TRUE;
        }
    }
    return er;
}

EXPORT ER s2_movement_send(s2_move_command_t command, UINT power_percent)
{
    void *p_block = NULL;
    SYSTIM now = {0};
    ER er = E_OK;

#if !APP_S2_ONLY
    if ((h_controller_task == 0) || (tk_get_tid() != h_controller_task))
    {
        er = E_OACV;
    }
#endif
    if ((er >= E_OK) && !movement_valid(command, power_percent))
    {
        er = E_PAR;
    }
    if ((er >= E_OK) && !motor_started)
    {
        er = E_OBJ;
    }
    if (er >= E_OK)
    {
        if ((command == S2_MOVE_STOP) || (power_percent == 0U))
        {
            command = S2_MOVE_STOP;
            power_percent = 0;
            /* A STOP must still work if the clock service fails. */
            (void)tk_get_tim(&now);
        }
        else
        {
            er = tk_get_tim(&now);
        }
    }
    if (er >= E_OK)
    {
        er = tk_get_mpf(h_movement_pool, &p_block, TMO_POL);
    }
    if (er >= E_OK)
    {
        /* Memory pool blocks have room and alignment for this message. */
        s2_movement_msg_t *p_msg = (s2_movement_msg_t *)p_block;

        p_msg->hdr.msgque[0] = NULL;
        p_msg->command = command;
        p_msg->power_percent = power_percent;
        p_msg->issued_at_ms = now.lo;
        /* The mailbox header is the first member of the message. */
        er = tk_snd_mbx(h_movement_mailbox, (T_MSG *)p_msg);
        if (er < E_OK)
        {
            CHECK(tk_rel_mpf(h_movement_pool, p_msg), "S2 send cleanup");
        }
    }
    return er;
}
