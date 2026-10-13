/* Tests run the actual motor task with simulated time and RTOS services. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "subsystem2/subsystem2.c"

static UW now_ms;
static ID caller_id;
static UW duties[32];
static UW registers[0x5000U / 4U];
static s2_movement_msg_t blocks[S2_COMMAND_COUNT];
static BOOL block_used[S2_COMMAND_COUNT];
static T_MSG *p_queue[32];
static UINT queue_count;
static UINT receive_count;
static UINT receive_error_at;
static UINT time_call_count;
static UINT time_error_at;
static UINT pwm_writes;
static UINT nonzero_writes;
static UINT encoder_accesses;
static UINT create_count;
static UINT create_error_at;
static UINT delete_count;
static UINT send_error;
static UINT renewal_writes;
static INT expected_left;
static INT expected_right;
static INT lock_count;
static jmp_buf task_exit;
static void (*p_receive_hook)(void);

static void assert_off(void)
{
    assert(duties[8] == 0U && duties[9] == 0U);
    assert(duties[10] == 0U && duties[11] == 0U);
}

static void assert_status(s2_motion_state_t state, ER error)
{
    s2_motion_status_t status;

    assert(s2_motion_get_status(&status) == E_OK);
    assert(status.state == state);
    assert(status.last_error == error);
    if (state != S2_MOTION_MOVING)
    {
        assert(status.command == S2_MOVE_STOP);
        assert(status.power_percent == 0U);
        assert_off();
    }
}

static void reset_fixture(void)
{
    h_controller_task = 0;
    h_movement_mailbox = 0;
    h_movement_pool = 0;
    h_status_lock = 0;
    motor_started = FALSE;
    motion_status = (s2_motion_status_t){0};
    now_ms = 0;
    caller_id = 7;
    memset(duties, 0, sizeof(duties));
    memset(registers, 0, sizeof(registers));
    memset(block_used, 0, sizeof(block_used));
    queue_count = 0;
    receive_count = 0;
    receive_error_at = 0;
    time_call_count = 0;
    time_error_at = 0;
    pwm_writes = 0;
    nonzero_writes = 0;
    encoder_accesses = 0;
    create_count = 0;
    create_error_at = 0;
    delete_count = 0;
    send_error = 0;
    lock_count = 0;
    p_receive_hook = NULL;
#if APP_S2_ONLY && S2_ENCODER_ENABLED
    encoder_ready = FALSE;
    encoder_positive = 0;
    encoder_negative = 0;
    encoder_a_edges = 0;
    encoder_b_edges = 0;
#endif
}

static void setup(void)
{
    reset_fixture();
    assert(subsystem2_init() == E_OK);
    assert_status(S2_MOTION_STOPPED, E_OK);
    assert(subsystem2_start() == E_OK);
    s2_controller_bind();
}

static void run_task(void)
{
    if (setjmp(task_exit) == 0)
    {
        motor_task(0, NULL);
    }
    for (UINT i = 0; i < S2_COMMAND_COUNT; ++i)
    {
        assert(!block_used[i]);
    }
}

void test_check(ER error, const char *p_label)
{
    (void)error;
    (void)p_label;
}

ID tk_get_tid(void) { return caller_id; }

ER tk_get_tim(SYSTIM *p_time)
{
    ++time_call_count;
    if ((time_error_at != 0U) && (time_call_count >= time_error_at))
    {
        return E_SYS;
    }
    p_time->hi = 0;
    p_time->lo = now_ms;
    return E_OK;
}

ER tk_dly_tsk(UW delay)
{
    now_ms += delay;
    return E_OK;
}

static ID create_object(void)
{
    ++create_count;
    return (create_count == create_error_at) ? E_SYS : (ID)create_count;
}

ID tk_cre_mbx(const T_CMBX *p_config)
{
    assert(p_config->mbxatr == TA_TFIFO);
    return create_object();
}

ID tk_cre_mpf(const T_CMPF *p_config)
{
    assert(p_config->mpfcnt == S2_COMMAND_COUNT);
    assert(p_config->blfsz == sizeof(s2_movement_msg_t));
    return create_object();
}

ID tk_cre_sem(const T_CSEM *p_config)
{
    assert(p_config->isemcnt == 1 && p_config->maxsem == 1);
    lock_count = p_config->isemcnt;
    return create_object();
}

ER tk_del_mbx(ID h_mailbox)
{
    assert(h_mailbox > 0);
    ++delete_count;
    return E_OK;
}

ER tk_del_mpf(ID h_pool)
{
    assert(h_pool > 0);
    ++delete_count;
    return E_OK;
}

ER tk_get_mpf(ID h_pool, void **pp_block, TMO wait)
{
    assert(h_pool > 0 && wait == TMO_POL);
    for (UINT i = 0; i < S2_COMMAND_COUNT; ++i)
    {
        if (!block_used[i])
        {
            block_used[i] = TRUE;
            *pp_block = &blocks[i];
            return E_OK;
        }
    }
    return E_TMOUT;
}

ER tk_rel_mpf(ID h_pool, void *p_block)
{
    assert(h_pool > 0);
    for (UINT i = 0; i < S2_COMMAND_COUNT; ++i)
    {
        if (p_block == &blocks[i])
        {
            assert(block_used[i]);
            block_used[i] = FALSE;
            return E_OK;
        }
    }
    assert(0 && "unknown pool block");
    return E_PAR;
}

ER tk_snd_mbx(ID h_mailbox, T_MSG *p_message)
{
    assert(h_mailbox > 0);
    if (send_error != 0U)
    {
        return E_SYS;
    }
    assert(queue_count < 32U);
    p_queue[queue_count++] = p_message;
    return E_OK;
}

ER tk_rcv_mbx(ID h_mailbox, T_MSG **pp_message, TMO wait)
{
    assert(h_mailbox > 0);
    ++receive_count;
    if (p_receive_hook != NULL)
    {
        p_receive_hook();
    }
    if (receive_count == receive_error_at)
    {
        return E_SYS;
    }
    if (queue_count > 0U)
    {
        *pp_message = p_queue[0];
        --queue_count;
        memmove(p_queue, p_queue + 1, queue_count * sizeof(p_queue[0]));
        return E_OK;
    }
    if (wait == TMO_FEVR)
    {
        longjmp(task_exit, 1);
    }
    assert(wait > 0);
    now_ms += (UW)wait;
    return E_TMOUT;
}

ER tk_wai_sem(ID h_lock, INT count, TMO wait)
{
    assert(h_lock > 0 && count == 1 && wait == TMO_FEVR);
    assert(lock_count == 1);
    --lock_count;
    return E_OK;
}

ER tk_sig_sem(ID h_lock, INT count)
{
    assert(h_lock > 0 && count == 1 && lock_count == 0);
    ++lock_count;
    return E_OK;
}

ID app_start_task(FP p_task, UINT priority, UINT core, const char *p_name)
{
    assert(p_task != NULL && priority == S2_PRI_MOTOR && core == TP_PRC1);
    assert(strcmp(p_name, "motor") == 0);
    return 42;
}

int tm_printf(UB *p_format, ...)
{
    (void)p_format;
    return 0;
}

UW in_w(UW address)
{
    assert(address < sizeof(registers));
    if (address == RESETS_RESET_DONE)
    {
        return RESETS_RESET_PWM;
    }
    if ((address >= IO_BANK0_BASE && address < IO_BANK0_BASE + 0x200U) ||
        address == GPIO_IN)
    {
        ++encoder_accesses;
    }
    return registers[address / 4U];
}

void out_w(UW address, UW value)
{
    assert(address < sizeof(registers));
    if (address >= IO_BANK0_BASE + 0xf0U &&
        address < IO_BANK0_BASE + 0x100U)
    {
        registers[address / 4U] &= ~value;
    }
    else
    {
        registers[address / 4U] = value;
    }
}

void set_w(UW address, UW value) { registers[address / 4U] |= value; }
void clr_w(UW address, UW value) { registers[address / 4U] &= ~value; }

ER gpio_set_pin(UINT pin, UINT mode)
{
    assert(pin < 30U);
    if (mode == GPIO_MODE_IN)
    {
        ++encoder_accesses;
    }
    return E_OK;
}

void gpio_set_val(UINT pin, UINT value) { assert(pin < 30U && value == 0U); }
UINT gpio_get_val(UINT pin) { return (registers[GPIO_IN / 4U] >> pin) & 1U; }

void pwm_set_cc(UINT pin, UW duty)
{
    assert(pin >= 8U && pin <= 11U && duty <= MOTOR_PWM_WRAP + 1U);
    duties[pin] = duty;
    ++pwm_writes;
    if (duty != 0U)
    {
        ++nonzero_writes;
    }
    assert(duties[8] == 0U || duties[9] == 0U);
    assert(duties[10] == 0U || duties[11] == 0U);
}

void pwm_set_enabled(UINT pin, BOOL enabled)
{
    assert(pin == 8U || pin == 10U);
    (void)enabled;
}

void pwm_set_wrap(UINT pin, UW wrap)
{
    assert((pin == 8U || pin == 10U) && wrap == MOTOR_PWM_WRAP);
}

void pwm_set_pin(UINT pin) { assert(pin >= 8U && pin <= 11U); }
void ClearInt(UINT intno) { assert(intno == 13U); }
void EnableInt(UINT intno, UINT priority)
{
    assert(intno == 13U && priority == 3U);
}

ER tk_def_int(UINT intno, const T_DINT *p_config)
{
    assert(intno == 13U && p_config->inthdr != NULL);
    return E_OK;
}

static void renewal_hook(void)
{
    if (receive_count == 2U)
    {
        assert_status(S2_MOTION_MOVING, E_OK);
        renewal_writes = pwm_writes;
        now_ms = 500;
        assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    }
    if (receive_count == 3U)
    {
        assert(pwm_writes == renewal_writes);
        now_ms = 1250;
        assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    }
    if (receive_count == 4U)
    {
        assert(pwm_writes == renewal_writes);
        now_ms = 1300;
        assert(s2_movement_send(S2_MOVE_STOP, 0) == E_OK);
    }
    if (receive_count == 5U)
    {
        assert_status(S2_MOTION_STOPPED, E_OK);
    }
}

static void direction_hook(void)
{
    if (receive_count == 2U)
    {
        UW duty = ((MOTOR_PWM_WRAP + 1U) * 30U) / 100U;

        assert(duties[8] == (expected_left > 0 ? duty : 0U));
        assert(duties[9] == (expected_left < 0 ? duty : 0U));
        assert(duties[10] == (expected_right > 0 ? duty : 0U));
        assert(duties[11] == (expected_right < 0 ? duty : 0U));
    }
}

static void recovery_hook(void)
{
    if (receive_count == 2U)
    {
        assert_status(S2_MOTION_FAULT, E_TMOUT);
        assert(s2_movement_send(S2_MOVE_REVERSE, 30) == E_OK);
    }
    if (receive_count == 3U)
    {
        assert_status(S2_MOTION_MOVING, E_OK);
        assert(s2_movement_send(S2_MOVE_STOP, 0) == E_OK);
    }
}

static void test_lifecycle_and_validation(void)
{
    s2_motion_status_t status;

    reset_fixture();
    assert(subsystem2_start() == E_OBJ);
    assert(s2_motion_get_status(&status) == E_OBJ);
    s2_controller_bind();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OBJ);
    setup();
    assert(subsystem2_init() == E_OBJ);
    assert(subsystem2_start() == E_OBJ);
    assert(s2_movement_send((s2_move_command_t)-1, 30) == E_PAR);
    assert(s2_movement_send((s2_move_command_t)99, 30) == E_PAR);
    assert(s2_movement_send(S2_MOVE_FORWARD, 101) == E_PAR);
    assert(s2_motion_get_status(NULL) == E_PAR);
    assert(s2_encoder_get_status(NULL) == E_PAR);
#if !APP_S2_ONLY
    caller_id = 8;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OACV);
#endif
}

static void test_init_cleanup(void)
{
    for (UINT failure = 1; failure <= 3U; ++failure)
    {
        reset_fixture();
        create_error_at = failure;
        assert(subsystem2_init() == E_SYS);
        assert(delete_count == failure - 1U);
        assert(subsystem2_start() == E_OBJ);
        create_error_at = 0;
        assert(subsystem2_init() == E_OK);
    }
}

static void test_watchdog_and_rollover(void)
{
    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(now_ms == S2_COMMAND_TIMEOUT_MS);
    assert(nonzero_writes == 2U);

    setup();
    now_ms = UINT32_MAX - 100U;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(now_ms == S2_COMMAND_TIMEOUT_MS - 101U);
}

static void test_stale_and_invalid_messages(void)
{
    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    now_ms = S2_COMMAND_TIMEOUT_MS;
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(nonzero_writes == 0U);

    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    now_ms = S2_COMMAND_TIMEOUT_MS - 1U;
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(nonzero_writes == 0U);

    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    blocks[0].command = (s2_move_command_t)99;
    run_task();
    assert_status(S2_MOTION_FAULT, E_PAR);
    assert(nonzero_writes == 0U);
}

static void test_renewal_and_recovery(void)
{
    setup();
    p_receive_hook = renewal_hook;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    run_task();
    assert_status(S2_MOTION_STOPPED, E_OK);
    assert(now_ms == 1300U);

    setup();
    p_receive_hook = recovery_hook;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    now_ms = S2_COMMAND_TIMEOUT_MS;
    run_task();
    assert_status(S2_MOTION_STOPPED, E_OK);
}

static void test_directions(void)
{
    const INT left[] = {0, 1, -1, -1, 1, 1, 0};
    const INT right[] = {0, 1, -1, 1, -1, 0, 1};

    for (UINT i = 0; i < 7U; ++i)
    {
        setup();
        p_receive_hook = direction_hook;
        expected_left = left[i] * MOTOR1_FORWARD_SIGN;
        expected_right = right[i] * MOTOR2_FORWARD_SIGN;
        assert(s2_movement_send((s2_move_command_t)i, 30) == E_OK);
        run_task();
        assert_off();
    }
}

static void test_pool_and_transport_errors(void)
{
    setup();
    for (UINT i = 0; i < S2_COMMAND_COUNT; ++i)
    {
        assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    }
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_TMOUT);
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);

    setup();
    send_error = 1;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_SYS);
    assert(!block_used[0]);

    setup();
    receive_error_at = 2;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    run_task();
    assert_status(S2_MOTION_FAULT, E_SYS);
}

static void test_clock_errors_and_zero_power(void)
{
    setup();
    time_error_at = 1;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_SYS);
    assert(s2_movement_send(S2_MOVE_STOP, 0) == E_OK);
    run_task();
    assert_status(S2_MOTION_STOPPED, E_OK);

    setup();
    time_error_at = 4;
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    run_task();
    assert_status(S2_MOTION_FAULT, E_SYS);
    assert(nonzero_writes == 2U);

    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 0) == E_OK);
    run_task();
    assert_status(S2_MOTION_STOPPED, E_OK);
    assert(nonzero_writes == 0U);
}

static void test_encoder_mode(void)
{
    s2_encoder_status_t status;

    setup();
    run_task();
#if APP_S2_ONLY && S2_ENCODER_ENABLED
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(registers[S2_ENCODER_A_INTE / 4U] & S2_ENCODER_A_EDGES);
    assert(registers[S2_ENCODER_B_INTE / 4U] & S2_ENCODER_B_EDGES);
    registers[S2_ENCODER_A_INTR / 4U] |= S2_ENCODER_A_RISING;
    registers[S2_ENCODER_B_INTR / 4U] |= S2_ENCODER_B_EDGES;
    registers[GPIO_IN / 4U] = 1U << BOARD_MOTOR2_ENCODER_B_GPIO;
    encoder_interrupt(S2_ENCODER_IRQ);
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.positive_pulses == 1U && status.negative_pulses == 0U);
    assert(status.a_edges == 1U && status.b_edges == 1U);
    registers[S2_ENCODER_A_INTR / 4U] |= S2_ENCODER_A_RISING;
    registers[GPIO_IN / 4U] = 0;
    encoder_interrupt(S2_ENCODER_IRQ);
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.positive_pulses == 1U && status.negative_pulses == 1U);
    assert(status.a_edges == 2U && status.b_edges == 1U);
#else
    memset(&status, 0xff, sizeof(status));
    assert(s2_encoder_get_status(&status) == E_NOSPT);
    assert(status.positive_pulses == 0U && status.negative_pulses == 0U);
    assert(status.a_edges == 0U && status.b_edges == 0U);
    assert(status.a_level == 0U && status.b_level == 0U);
    assert(encoder_accesses == 0U);
#endif
}

int main(void)
{
    test_lifecycle_and_validation();
    test_init_cleanup();
    test_watchdog_and_rollover();
    test_stale_and_invalid_messages();
    test_renewal_and_recovery();
    test_directions();
    test_pool_and_transport_errors();
    test_clock_errors_and_zero_power();
    test_encoder_mode();
    puts("Motor-control host checks passed");
    return 0;
}
