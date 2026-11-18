/* Tests run the actual motor task with simulated time and RTOS services. */
#include <assert.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>

#include "subsystem2/subsystem2.c"
#include "subsystem2/s2_data.h"

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
static s2_move_command_t renewal_command;
static INT expected_left;
static INT expected_right;
static INT lock_count;
static jmp_buf task_exit;
static void (*p_receive_hook)(void);
#if S2_ENCODER_ENABLED
static void (*p_gpio_read_hook)(void);
static UINT gpio_input_reads;
#endif

static void assert_off(void)
{
    assert(duties[8] == 0U && duties[9] == 0U);
    assert(duties[10] == 0U && duties[11] == 0U);
}

static void assert_status(s2_motion_state_t state, ER error)
{
    s2_motion_status_t status;
    uint16_t expected;

    assert(s2_motion_get_status(&status) == E_OK);
    assert(status.state == state);
    assert(status.last_error == error);
    assert(edr_motor_pack((uint8_t)status.command,
                           (uint8_t)status.power_percent,
                           (uint8_t)state, &expected));
    assert(motion_status.motor == expected);
    assert(sizeof(motion_status.motor) == 2U);
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
    motion_status = (s2_packed_status_t){0};
    stop_epoch = 0;
    stopped_epoch = 0;
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
    renewal_command = S2_MOVE_FORWARD;
#if S2_ENCODER_ENABLED
    p_gpio_read_hook = NULL;
    gpio_input_reads = 0;
    __atomic_store_n(&encoder_ready, 0U, __ATOMIC_RELEASE);
    encoder_sequence = 0;
    encoder_positive = 0;
    encoder_negative = 0;
    encoder_a_edges = 0;
    encoder_b_edges = 0;
    encoder1_positive = 0;
    encoder1_negative = 0;
    encoder1_a_edges = 0;
    encoder1_b_edges = 0;
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

ER tk_get_otm(SYSTIM *p_time)
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
    if (wait == TMO_FEVR ||
        (stop_epoch == stopped_epoch &&
         ((motion_status.motor >> 10U) & 3U) != S2_MOTION_MOVING))
    {
        longjmp(task_exit, 1);
    }
    assert(wait > 0);
    now_ms += (UW)wait;
    return E_TMOUT;
}

ER tk_wai_sem(ID h_lock, INT count, TMO wait)
{
    assert(h_lock > 0 && count == 1 && wait == 5);
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
#if S2_ENCODER_ENABLED
    if (address == GPIO_IN)
    {
        ++gpio_input_reads;
        if (p_gpio_read_hook != NULL)
        {
            void (*hook)(void) = p_gpio_read_hook;

            p_gpio_read_hook = NULL;
            hook();
        }
    }
#endif
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
        now_ms = S2_COMMAND_TIMEOUT_MS / 2U;
        assert(s2_movement_send(renewal_command, 30) == E_OK);
    }
    if (receive_count == 3U)
    {
        assert(pwm_writes == renewal_writes);
        now_ms = S2_COMMAND_TIMEOUT_MS + S2_COMMAND_TIMEOUT_MS / 4U;
        assert(s2_movement_send(renewal_command, 30) == E_OK);
    }
    if (receive_count == 4U)
    {
        assert(pwm_writes == renewal_writes);
        now_ms = S2_COMMAND_TIMEOUT_MS + S2_COMMAND_TIMEOUT_MS / 3U;
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
        UW duty = (MOTOR_PWM_WRAP + 1U) * 30U;
        UW left = (duty * S2_MOTOR1_TRIM_PERCENT) / 10000U;
        UW right = (duty * S2_MOTOR2_TRIM_PERCENT) / 10000U;

        assert(duties[8] == (expected_left > 0 ? left : 0U));
        assert(duties[9] == (expected_left < 0 ? left : 0U));
        assert(duties[10] == (expected_right > 0 ? right : 0U));
        assert(duties[11] == (expected_right < 0 ? right : 0U));
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
    /* Steered reverse shares the forward steering range and PID requirement. */
    assert(s2_reverse_follow_send(30, 401) == E_PAR);
    assert(s2_reverse_follow_send(30, -401) == E_PAR);
#if !BOARD_PID_ENABLED
    assert(s2_reverse_follow_send(30, 100) == E_NOSPT);
    assert(s2_follow_send(30, -100) == E_NOSPT);
#endif
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
    assert(s2_movement_send(S2_MOVE_RIGHT, 30) == E_OK);
    now_ms = S2_COMMAND_TIMEOUT_MS - 1U;
    /* Fresh at dequeue, expired during the one-millisecond reversal deadtime. */
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(nonzero_writes == 0U);
    assert(now_ms == S2_COMMAND_TIMEOUT_MS);

    setup();
    assert(s2_movement_send(S2_MOVE_RIGHT, 30) == E_OK);
    now_ms = S2_COMMAND_TIMEOUT_MS - 2U;
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    assert(nonzero_writes == 2U);
    assert(now_ms == S2_COMMAND_TIMEOUT_MS);

    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    blocks[0].motor = UINT16_C(0x8000); /* Reserved bit rejects malformed data. */
    run_task();
    assert_status(S2_MOTION_FAULT, E_PAR);
    assert(nonzero_writes == 0U);

    setup();
    assert(s2_movement_send(S2_MOVE_FORWARD, 30) == E_OK);
    /* Queue words represent requests and must not claim an active state. */
    blocks[0].motor |= UINT16_C(0x0400);
    run_task();
    assert_status(S2_MOTION_FAULT, E_PAR);
    assert(nonzero_writes == 0U);
}

static void test_renewal_and_recovery(void)
{
    const s2_move_command_t commands[] =
    {
        S2_MOVE_FORWARD, S2_MOVE_LEFT, S2_MOVE_RIGHT
    };

    for (UINT i = 0; i < sizeof(commands) / sizeof(commands[0]); ++i)
    {
        setup();
        renewal_command = commands[i];
        p_receive_hook = renewal_hook;
        assert(s2_movement_send(renewal_command, 30) == E_OK);
        run_task();
        assert_status(S2_MOTION_STOPPED, E_OK);
        assert(now_ms >= S2_COMMAND_TIMEOUT_MS + S2_COMMAND_TIMEOUT_MS / 3U);
        assert(now_ms <= S2_COMMAND_TIMEOUT_MS + S2_COMMAND_TIMEOUT_MS / 3U +
                        (APP_S2_ONLY ? 0U : 10U));

        setup();
        assert(s2_movement_send(commands[i], 30) == E_OK);
        run_task();
        assert_status(S2_MOTION_FAULT, E_TMOUT);
        assert(now_ms == S2_COMMAND_TIMEOUT_MS);
        assert(nonzero_writes == 2U);
    }

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

#if S2_ENCODER_ENABLED
static void queue_forward_encoder_edges(void)
{
    registers[S2_ENCODER1_A_INTR / 4U] |= S2_ENCODER1_A_RISING;
    registers[S2_ENCODER1_B_INTR / 4U] |= S2_ENCODER1_B_EDGES;
    registers[S2_ENCODER_A_INTR / 4U] |= S2_ENCODER_A_RISING;
    registers[S2_ENCODER_B_INTR / 4U] |= S2_ENCODER_B_EDGES;
    registers[GPIO_IN / 4U] = 1U << BOARD_MOTOR2_ENCODER_B_GPIO;
    encoder_interrupt(S2_ENCODER_IRQ);
}
#endif

static void assert_encoder_zero(const s2_encoder_status_t *p_status)
{
    const s2_encoder_status_t zero = {0};

    assert(memcmp(p_status, &zero, sizeof(zero)) == 0);
}

static void test_encoder_mode(void)
{
    s2_encoder_status_t status;

    setup();
    memset(&status, 0xff, sizeof(status));
#if S2_ENCODER_ENABLED
    assert(s2_encoder_get_status(&status) == E_OK);
#else
    assert(s2_encoder_get_status(&status) == E_NOSPT);
#endif
    assert_encoder_zero(&status);
    run_task();
#if S2_ENCODER_ENABLED
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.valid == 1U);
    assert(status.motor1_forward_counts == 0 && status.motor2_forward_counts == 0);
    assert(registers[S2_ENCODER_A_INTE / 4U] & S2_ENCODER_A_EDGES);
    assert(registers[S2_ENCODER_B_INTE / 4U] & S2_ENCODER_B_EDGES);
    assert(registers[S2_ENCODER1_A_INTE / 4U] & S2_ENCODER1_A_EDGES);
    assert(registers[S2_ENCODER1_B_INTE / 4U] & S2_ENCODER1_B_EDGES);

    /* Leave an unrelated interrupt pending: the handler owns only encoder edges. */
    registers[S2_ENCODER_A_INTR / 4U] |= 1U;
    queue_forward_encoder_edges();
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.positive_pulses == 1U && status.negative_pulses == 0U);
    assert(status.a_edges == 1U && status.b_edges == 1U);
    assert(status.motor1_positive_pulses == 0U && status.motor1_negative_pulses == 1U);
    assert(status.motor1_a_edges == 1U && status.motor1_b_edges == 1U);
    assert(status.motor1_forward_counts == 1 && status.motor2_forward_counts == 1);
    assert(status.a_level == 0U && status.b_level == 1U);
    assert(status.motor1_a_level == 0U && status.motor1_b_level == 0U);
    assert(registers[S2_ENCODER_A_INTR / 4U] & 1U);
    assert((registers[S2_ENCODER_A_INTR / 4U] & S2_ENCODER_A_EDGES) == 0U);
    assert((registers[S2_ENCODER_B_INTR / 4U] & S2_ENCODER_B_EDGES) == 0U);
    assert((registers[S2_ENCODER1_A_INTR / 4U] & S2_ENCODER1_A_EDGES) == 0U);
    assert((registers[S2_ENCODER1_B_INTR / 4U] & S2_ENCODER1_B_EDGES) == 0U);

    /* Reverse both wheels; one A rising edge must cancel each forward count. */
    registers[S2_ENCODER1_A_INTR / 4U] |= S2_ENCODER1_A_RISING;
    registers[S2_ENCODER_A_INTR / 4U] |= S2_ENCODER_A_RISING;
    registers[GPIO_IN / 4U] = (1U << BOARD_MOTOR1_ENCODER_B_GPIO) |
                             (1U << BOARD_MOTOR1_ENCODER_A_GPIO) |
                             (1U << BOARD_MOTOR2_ENCODER_A_GPIO);
    encoder_interrupt(S2_ENCODER_IRQ);
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.positive_pulses == 1U && status.negative_pulses == 1U);
    assert(status.a_edges == 2U && status.b_edges == 1U);
    assert(status.motor1_positive_pulses == 1U && status.motor1_negative_pulses == 1U);
    assert(status.motor1_a_edges == 2U && status.motor1_b_edges == 1U);
    assert(status.motor1_forward_counts == 0 && status.motor2_forward_counts == 0);
    assert(status.a_level == 1U && status.b_level == 0U);
    assert(status.motor1_a_level == 1U && status.motor1_b_level == 1U);

    /* Falling A edges count as diagnostics, never as x1 position pulses. */
    registers[S2_ENCODER1_A_INTR / 4U] |= S2_ENCODER1_A_EDGES & ~S2_ENCODER1_A_RISING;
    registers[S2_ENCODER_A_INTR / 4U] |= S2_ENCODER_A_EDGES & ~S2_ENCODER_A_RISING;
    encoder_interrupt(S2_ENCODER_IRQ);
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.a_edges == 3U && status.motor1_a_edges == 3U);
    assert(status.positive_pulses == 1U && status.negative_pulses == 1U);
    assert(status.motor1_positive_pulses == 1U && status.motor1_negative_pulses == 1U);

    /* Interrupt the snapshot between wheel copies: retry must include both updates. */
    UINT reads_before = gpio_input_reads;

    p_gpio_read_hook = queue_forward_encoder_edges;
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(gpio_input_reads >= reads_before + 2U);
    assert(p_gpio_read_hook == NULL);
    assert(status.positive_pulses == 2U && status.negative_pulses == 1U);
    assert(status.motor1_positive_pulses == 1U && status.motor1_negative_pulses == 2U);
    assert(status.motor1_forward_counts == 1 && status.motor2_forward_counts == 1);
    assert(status.a_edges == 4U && status.motor1_a_edges == 4U);
    assert(status.b_edges == 2U && status.motor1_b_edges == 2U);

    /* Unsigned pulse subtraction preserves signed position across counter rollover. */
    encoder_positive = 0U;
    encoder_negative = UINT32_MAX;
    encoder1_positive = UINT32_MAX;
    encoder1_negative = 0U;
    assert(s2_encoder_get_status(&status) == E_OK);
    assert(status.motor1_forward_counts == 1 && status.motor2_forward_counts == 1);
#else
    memset(&status, 0xff, sizeof(status));
    assert(s2_encoder_get_status(&status) == E_NOSPT);
    assert_encoder_zero(&status);
    assert(encoder_accesses == 0U);
#endif
}

static void assert_payload_fill(const uint8_t *p_payload, size_t length,
                                uint8_t value)
{
    for (size_t index = 0U; index < length; ++index)
    {
        assert(p_payload[index] == value);
    }
}

static void test_data_adapters(void)
{
    uint8_t payload[EDR_ENCODER_SIZE + 1U];
    const uint8_t timeout_payload[EDR_MOTION_SIZE] =
    {
        0x0bU, 0x80U, 0x0cU, 0x02U
    };
    s2_packed_status_t status;

    reset_fixture();
    memset(payload, 0xa5, sizeof(payload));
    assert(s2_motion_encode(payload, sizeof(payload)) == E_OBJ);
    assert_payload_fill(payload, sizeof(payload), 0xa5U);
    setup();
    assert(s2_motion_encode(NULL, sizeof(payload)) == E_PAR);
    assert(s2_motion_encode(payload, EDR_MOTION_SIZE - 1U) == E_PAR);
    assert(s2_encoder_encode(NULL, sizeof(payload)) == E_PAR);
    assert(s2_encoder_encode(payload, EDR_ENCODER_SIZE - 1U) == E_PAR);
    assert_payload_fill(payload, sizeof(payload), 0xa5U);

    /* Startup has no measured count sample, even when encoders are enabled. */
#if S2_ENCODER_ENABLED
    assert(s2_encoder_encode(payload, sizeof(payload)) == E_OBJ);
    assert_payload_fill(payload, EDR_ENCODER_SIZE, 0xa5U);
    assert(payload[EDR_ENCODER_SIZE] == 0xa5U);
#else
    assert(s2_encoder_encode(payload, sizeof(payload)) == E_NOSPT);
    assert_payload_fill(payload, sizeof(payload), 0xa5U);
#endif

    now_ms = UINT32_C(0x12345678);
    assert(s2_movement_send(S2_MOVE_FORWARD, 30U) == E_OK);
    assert(blocks[0].motor == UINT16_C(0x00f1));
    assert(sizeof(blocks[0].motor) == 2U);
    run_task();
    assert_status(S2_MOTION_FAULT, E_TMOUT);
    memset(payload, 0xa5, sizeof(payload));
    assert(s2_motion_encode(payload, sizeof(payload)) == E_OK);
    assert(memcmp(payload, timeout_payload, EDR_MOTION_SIZE) == 0);
    assert_payload_fill(payload + EDR_MOTION_SIZE,
                         sizeof(payload) - EDR_MOTION_SIZE, 0xa5U);

    status = motion_status;
    status.last_error = E_SYS;
    assert(motion_publish(&status) == E_OK);
    assert(s2_motion_encode(payload, sizeof(payload)) == E_OK);
    assert(payload[2] == 0x0cU && payload[3] == 0x04U);

    /* A corrupt stored word cannot become a partial external payload. */
    motion_status.motor |= UINT16_C(0x8000);
    memset(payload, 0xa5, sizeof(payload));
    assert(s2_motion_encode(payload, sizeof(payload)) == E_SYS);
    assert_payload_fill(payload, sizeof(payload), 0xa5U);
    motion_status = status;

#if S2_ENCODER_ENABLED
    {
        const uint8_t encoder_payload[EDR_ENCODER_SIZE] =
        {
            0x11U, 0x00U, 0x12U, 0x00U
        };
        s2_encoder_status_t encoder;

        now_ms = UINT32_C(0x78563412);
        queue_forward_encoder_edges();
        memset(payload, 0xa5, sizeof(payload));
        assert(s2_encoder_encode(payload, sizeof(payload)) == E_OK);
        assert(memcmp(payload, encoder_payload, EDR_ENCODER_SIZE) == 0);
        assert(payload[EDR_ENCODER_SIZE] == 0xa5U);
        assert(s2_encoder_get_status(&encoder) == E_OK);
        assert(encoder.sampled_ms == now_ms);

        /* The adapter propagates clock failure without exposing partial bytes. */
        time_error_at = time_call_count + 1U;
        memset(payload, 0xa5, sizeof(payload));
        assert(s2_encoder_encode(payload, sizeof(payload)) == E_SYS);
        assert_payload_fill(payload, sizeof(payload), 0xa5U);
        memset(&encoder, 0xff, sizeof(encoder));
        assert(s2_encoder_get_status(&encoder) == E_SYS);
        assert_encoder_zero(&encoder);
    }
#else
    memset(payload, 0xa5, sizeof(payload));
    assert(s2_encoder_encode(payload, sizeof(payload)) == E_NOSPT);
    assert_payload_fill(payload, sizeof(payload), 0xa5U);
#endif
}

static void epoch_stop_hook(void)
{
    if (receive_count == 2U) {
        assert_status(S2_MOTION_MOVING, E_OK);
        assert(s2_emergency_stop() == E_OK);
    }
}
static void test_stop_epoch(void)
{
#if !APP_S2_ONLY
    setup();
    /* STOP bypasses exhausted pool capacity and invalidates old requests. */
    for (UINT n=0;n<S2_COMMAND_COUNT;++n)
        assert(s2_movement_send(S2_MOVE_FORWARD,30)==E_OK);
    assert(s2_movement_send(S2_MOVE_FORWARD,30)==E_TMOUT);
    assert(s2_emergency_stop()==E_OK);
    assert(s2_movement_send(S2_MOVE_FORWARD,30)==E_OBJ);
    run_task();assert_off();assert(nonzero_writes==0U);
    setup();p_receive_hook=epoch_stop_hook;
    assert(s2_movement_send(S2_MOVE_FORWARD,30)==E_OK);
    run_task();assert_status(S2_MOTION_STOPPED,E_OK);
    assert(now_ms<=11U); /* one ms deadtime + at most ten ms polling */
    puts("Integrated STOP epoch: saturated pool, old commands, running motor and bounded poll PASS");
#else
    (void)epoch_stop_hook;
#endif
}

static void test_turn_drive_helpers(void)
{
    for (UINT command = 0; command <= (UINT)S2_MOVE_PIVOT_LEFT; ++command)
    {
        /* Only in-place spins; the line follower's pivots keep their tuning. */
        BOOL turn = command == (UINT)S2_MOVE_LEFT || command == (UINT)S2_MOVE_RIGHT;

        assert(s2_turn_command(command) == (turn != 0));
        assert(s2_turn_limit(command, 35U, 55U) == (turn ? 55U : 35U));
        assert(s2_stall_ms(command, 500U, 1200U) == (turn ? 1200U : 500U));
        assert(s2_turn_kick_milli(command, 0U, 150U, 45U, 55U) == (turn ? 45000U : 0U));
        assert(s2_turn_kick_milli(command, 149U, 150U, 45U, 55U) == (turn ? 45000U : 0U));
        assert(s2_turn_kick_milli(command, 150U, 150U, 45U, 55U) == 0U);
    }
    /* No kick when disabled or long expired; never above the ceiling. */
    assert(s2_turn_kick_milli(S2_MOVE_LEFT, 0U, 0U, 45U, 55U) == 0U);
    assert(s2_turn_kick_milli(S2_MOVE_LEFT, UINT32_MAX, 150U, 45U, 55U) == 0U);
    assert(s2_turn_kick_milli(S2_MOVE_RIGHT, 0U, 150U, 70U, 55U) == 55000U);
    /* Per-wheel kick: one backlash count does not end it, 5 real counts do;
     * the window bounds it; a wheel that re-stalls for 100 ms is kicked again. */
    s2_kick_t k;
    s2_kick_start(&k, 1000U, 150U);
    assert(s2_kick_step(&k, 1010U, 1, 10U, 150U, 5, 100U));
    assert(s2_kick_step(&k, 1020U, 3, 10U, 150U, 5, 100U));
    assert(!s2_kick_step(&k, 1030U, 1, 0U, 150U, 5, 100U)); /* 5 counts: moving */
    assert(!s2_kick_step(&k, 1100U, 0, 70U, 150U, 5, 100U));
    assert(s2_kick_step(&k, 1130U, 0, 100U, 150U, 5, 100U)); /* re-stalled */
    assert(s2_kick_step(&k, 1270U, 0, 240U, 150U, 5, 100U));
    assert(s2_kick_step(&k, 1290U, 0, 260U, 150U, 5, 100U)); /* still stuck: kept */
    assert(!s2_kick_step(&k, 1300U, 9, 0U, 150U, 5, 100U)); /* breaks away */
    s2_kick_start(&k, 0U, 150U);
    assert(s2_kick_step(&k, 149U, 0, 40U, 150U, 5, 100U) && !s2_kick_step(&k, 150U, 2, 0U, 150U, 5, 100U));
    s2_kick_start(&k, UINT32_MAX - 20U, 150U); /* clock wrap */
    assert(s2_kick_step(&k, 100U, 0, 0U, 150U, 5, 100U) && !s2_kick_step(&k, 130U, 0, 0U, 150U, 5, 100U));
    s2_kick_start(&k, 0U, 0U); /* disabled */
    assert(!s2_kick_step(&k, 10U, 0, 500U, 0U, 5, 100U));
    puts("Turn drive helpers: spin ceiling, per-wheel kick until real motion, re-kick on re-stall, stall time PASS");
}

#if BOARD_PID_ENABLED
/* Speed-PID profile: the actual motor task with simulated wheel encoders
 * turning at 500 counts/s, except an optional stalled wheel. */
static const INT sim_left[] = {0, 1, -1, -1, 1, 1, 0};
static const INT sim_right[] = {0, 1, -1, 1, -1, 0, 1};
static s2_move_command_t sim_command;
static UINT sim_still; /* 0 left, 1 right, 2 neither */
static UW sim_moved_ms;
static UW sim_stop_ms;
static UW sim_fault_ms;
static UW sim_first[4];
static UW sim_kick_min[4];
static UW sim_after_min[4];
static UW sim_late_max[4];
static UW sim_late[4];

static void sim_wheels_hook(void)
{
    UW counts = (now_ms - sim_moved_ms) / 2U;
    UINT state = (motion_status.motor >> 10U) & 3U;

    sim_moved_ms += counts * 2U;
    if (state == S2_MOTION_FAULT && sim_fault_ms == 0U)
    {
        sim_fault_ms = now_ms;
    }
    if (state != S2_MOTION_MOVING)
    {
        return;
    }
    /* Left forward counts = negative - positive; right = positive - negative. */
    if (sim_still != 0U && sim_left[sim_command] > 0) encoder1_negative += counts;
    if (sim_still != 0U && sim_left[sim_command] < 0) encoder1_positive += counts;
    if (sim_still != 1U && sim_right[sim_command] > 0) encoder_positive += counts;
    if (sim_still != 1U && sim_right[sim_command] < 0) encoder_negative += counts;
    for (UINT pin = 0; pin < 4U; ++pin)
    {
        UW duty = duties[8U + pin];

        if (receive_count == 2U) sim_first[pin] = duty;
        if (now_ms < 150U && duty < sim_kick_min[pin]) sim_kick_min[pin] = duty;
        if (now_ms >= 160U && now_ms < 200U && duty < sim_after_min[pin]) sim_after_min[pin] = duty;
        if (now_ms >= 200U && duty > sim_late_max[pin]) sim_late_max[pin] = duty;
    }
    if (sim_stop_ms != 0U && now_ms >= sim_stop_ms)
    {
        memcpy(sim_late, &duties[8], sizeof(sim_late));
        sim_stop_ms = 0U;
        assert(s2_movement_send(S2_MOVE_STOP, 0U) == E_OK);
    }
    else if (receive_count % 10U == 0U)
    {
        assert(s2_movement_send(sim_command, 25U) == E_OK);
    }
}

static void sim_run(s2_move_command_t command, UINT still, UW stop_ms)
{
    setup();
    sim_command = command;
    sim_still = still;
    sim_moved_ms = 0U;
    sim_stop_ms = stop_ms;
    sim_fault_ms = 0U;
    memset(sim_first, 0, sizeof(sim_first));
    memset(sim_kick_min, 0xff, sizeof(sim_kick_min));
    memset(sim_after_min, 0xff, sizeof(sim_after_min));
    memset(sim_late_max, 0, sizeof(sim_late_max));
    memset(sim_late, 0, sizeof(sim_late));
    p_receive_hook = sim_wheels_hook;
    assert(s2_movement_send(command, 25U) == E_OK);
    run_task();
}

static void test_pid_turn_drive(void)
{
    const UW kick = (MOTOR_PWM_WRAP + 1U) * BOARD_TURN_KICK_POWER / 100U;
    const UW turn_cap = (MOTOR_PWM_WRAP + 1U) * BOARD_PID_TURN_MAX_POWER / 100U;
    const UW straight_cap = (MOTOR_PWM_WRAP + 1U) * BOARD_PID_MAX_POWER / 100U;

    /* Robot log: a LEFT spin whose left wheel does not break away. Both
     * wheels start at the kick; the moving one drops it once it has really
     * moved, the stalled one keeps being kicked (re-kick after 100 ms without
     * progress), may then reach the turn ceiling, and faults only after the
     * longer turn stall time. */
    sim_run(S2_MOVE_LEFT, 0U, 0U);
    assert(sim_first[0] == 0U && sim_first[1] == kick);
    assert(sim_first[2] == 0U && sim_first[3] == kick);
    assert(sim_kick_min[1] >= kick && sim_kick_min[3] < kick);
    assert(sim_after_min[1] >= kick); /* still stalled: kicked again */
    assert(sim_late_max[1] == turn_cap && turn_cap > straight_cap);
    assert(sim_fault_ms > BOARD_TURN_STALL_MS && sim_fault_ms <= BOARD_TURN_STALL_MS + 20U);
    assert_status(S2_MOTION_FAULT, E_TMOUT);

    /* Straight driving is unchanged: no kick, the PID cap, a 500 ms stall. */
    sim_run(S2_MOVE_FORWARD, 0U, 0U);
    assert(sim_first[0] == (MOTOR_PWM_WRAP + 1U) * 25U * S2_MOTOR1_TRIM_PERCENT / 10000U);
    assert(sim_first[3] == (MOTOR_PWM_WRAP + 1U) * 25U * S2_MOTOR2_TRIM_PERCENT / 10000U);
    assert(sim_first[0] < kick && sim_late_max[0] == straight_cap);
    assert(sim_fault_ms > 500U && sim_fault_ms <= 520U);
    assert_status(S2_MOTION_FAULT, E_TMOUT);

    /* Both wheels turning: the kick ends as they move and the PID tracks
     * the 25 % speed target below it. */
    sim_run(S2_MOVE_RIGHT, 2U, 300U);
    assert(sim_first[0] == kick && sim_first[1] == 0U);
    assert(sim_first[2] == kick && sim_first[3] == 0U);
    assert(sim_kick_min[0] < kick && sim_kick_min[2] < kick);
    assert(sim_late[0] > 0U && sim_late[0] < kick);
    assert(sim_late[2] > 0U && sim_late[2] < kick);
    assert(sim_fault_ms == 0U);
    assert_status(S2_MOTION_STOPPED, E_OK);

    /* The line follower's pivots keep their tuning: no kick, the straight
     * PID cap and the 500 ms stall. */
    sim_run(S2_MOVE_PIVOT_LEFT, 1U, 0U);
    assert(sim_first[0] == 0U && sim_first[1] == 0U);
    assert(sim_first[2] == 0U && sim_first[3] == (MOTOR_PWM_WRAP + 1U) * 25U * S2_MOTOR2_TRIM_PERCENT / 10000U);
    assert(sim_first[3] < kick);
    assert(sim_late_max[0] == 0U && sim_late_max[1] == 0U);
    assert(sim_late_max[3] == straight_cap);
    assert(sim_fault_ms > 500U && sim_fault_ms <= 520U);
    puts("PID spins: kick until real motion, re-kick, raised ceiling, longer stall; straight and pivots unchanged PASS");
}
#endif

int main(void)
{
    test_turn_drive_helpers();
#if BOARD_PID_ENABLED
    /* The scenarios below model open-loop wheels; speed regulation would
     * fault their motionless encoders, so this profile checks turns only. */
    test_pid_turn_drive();
    if (BOARD_PID_ENABLED)
    {
        puts("Motor-control host checks passed");
        return 0;
    }
#endif
    test_lifecycle_and_validation();
    test_init_cleanup();
    test_watchdog_and_rollover();
    test_stale_and_invalid_messages();
    test_renewal_and_recovery();
    test_directions();
    test_pool_and_transport_errors();
    test_clock_errors_and_zero_power();
    test_encoder_mode();
    test_data_adapters();
    test_stop_epoch();
    puts("Motor-control host checks passed");
    return 0;
}
