/*
 * subsystem2.c
 * Buddy 2 -- Motor owner with optional calibrated wheel-speed PID. Student ID:
 * 2500613.
 */

#include "subsystem2.h"
#include "../board_pins.h"
#include "../shared/edr.h"
#include "../robot_config.h"
#include <tk/syslib.h>
#include "speed_pid.h"
#include "wheel_steering.h"
#include "turn_drive.h"
#if BOARD_TIMING_STATS && defined(TIMER_TIMERAWL)
/* Motor loop timing (C5, printed by the VC): a run is wake to the next
 * mailbox wait; the direction dead time sleep is left out. Firmware only:
 * host builds have no hardware timer. */
#include "../shared/task_timing.h"
#define S2_TIMING(step)                                                        \
    task_timing_##step(&app_task_timing[TASK_TIMING_MOTOR],                    \
                       (uint32_t) in_w(TIMER_TIMERAWL))
#else
#define S2_TIMING(step) ((void) 0)
#endif
#if defined(_PICO_RP2040_)
/* Watchdog heartbeat (app_watchdog.c): one per loop. Host builds: none. */
#include "../../include/tm/robot_health.h"
#define S2_HEARTBEAT() HEALTH_BEAT(HEALTH_HB_MOTOR)
#else
#define S2_HEARTBEAT() ((void) 0)
#endif
_Static_assert((int) S2_MOVE_LEFT == 3 && (int) S2_MOVE_RIGHT == 4 &&
                   (int) S2_MOVE_PIVOT_RIGHT == 5 &&
                   (int) S2_MOVE_PIVOT_LEFT == 6,
               "turn_drive.h command numbers changed");
static INT requested_steering;
/* Straight-line commands accept differential steering. Positive steering
 * always raises the left wheel's demand, so it turns CW going forward and
 * CCW in reverse. */
static BOOL
s2_steerable (UINT command)
{
    return command == (UINT) S2_MOVE_FORWARD ||
           command == (UINT) S2_MOVE_REVERSE;
}
#if BOARD_PID_ENABLED && !S2_ENCODER_ENABLED
#error Speed regulation requires both encoders
#endif
#if BOARD_PID_ENABLED
static speed_pid_t wheel_pid[2];
static uint32_t    pid_counts[2], pid_time, pid_progress[2];
static int32_t     pid_speed[2];
static BOOL        pid_initialized;
static UINT        pid_command;
static int32_t     filtered_steering;
static s2_kick_t
    turn_kick[2]; /* per-wheel spin breakaway kick (turn_drive.h) */
/* Stall time is driven time without progress, kept per wheel across command
 * changes and short STOPs (stall_carry). pid_dir: each wheel's direction in
 * pid_command, x2 in a spin (0: not driven). stall_dir/stall_held: the
 * direction (0: none) and stall time a wheel keeps while it is not driven;
 * stall_moved: the wheel has turned since that count started. */
static int8_t   pid_dir[2], stall_dir[2];
static uint32_t stall_held[2];
static BOOL     stall_moved[2];
#endif

/* Persistent state uses the same motor word as the external representation.
 * Native enum fields exist only in temporary compatibility snapshots. */
typedef struct
{
    uint16_t motor;
    ER       last_error;
    UW       last_command_ms;
} s2_packed_status_t;

static ID                 h_controller_task;
static UW                 stop_epoch;
static UW                 stopped_epoch;
static ID                 h_movement_mailbox;
static ID                 h_movement_pool;
static ID                 h_status_lock;
static BOOL               motor_started;
static s2_packed_status_t motion_status;
static BOOL status_unpublished; /* the last status copy missed the lock */

/* PWM duty = compare / (wrap + 1). */
#define MOTOR_PWM_WRAP 6249U

/* Motor 1 is the left wheel, motor 2 the right wheel.
 * Change either sign if that wheel rotates backwards during FORWARD.
 */
#define MOTOR1_FORWARD_SIGN 1
#define MOTOR2_FORWARD_SIGN (-1)

EXPORT void
s2_controller_bind (void)
{
    if (h_controller_task == 0)
    {
        h_controller_task = tk_get_tid();
    }
}

#if S2_ENCODER_ENABLED
/* RP2040 GPIO interrupt registers have eight pins per 32-bit bank. */
#define S2_ENCODER_IRQ 13U
#define S2_ENCODER_INTR(pin) (IO_BANK0_BASE + 0xf0U + ((pin) / 8U) * 4U)
#define S2_ENCODER_INTE(pin) (IO_BANK0_BASE + 0x100U + ((pin) / 8U) * 4U)
#define S2_ENCODER_RISING(pin) (8U << (((pin) % 8U) * 4U))
#define S2_ENCODER_EDGES(pin) (12U << (((pin) % 8U) * 4U))
#define S2_ENCODER_A_INTR S2_ENCODER_INTR(BOARD_MOTOR2_ENCODER_A_GPIO)
#define S2_ENCODER_B_INTR S2_ENCODER_INTR(BOARD_MOTOR2_ENCODER_B_GPIO)
#define S2_ENCODER_A_INTE S2_ENCODER_INTE(BOARD_MOTOR2_ENCODER_A_GPIO)
#define S2_ENCODER_B_INTE S2_ENCODER_INTE(BOARD_MOTOR2_ENCODER_B_GPIO)
#define S2_ENCODER_A_RISING S2_ENCODER_RISING(BOARD_MOTOR2_ENCODER_A_GPIO)
#define S2_ENCODER_A_EDGES S2_ENCODER_EDGES(BOARD_MOTOR2_ENCODER_A_GPIO)
#define S2_ENCODER_B_EDGES S2_ENCODER_EDGES(BOARD_MOTOR2_ENCODER_B_GPIO)
#define S2_ENCODER1_A_INTR S2_ENCODER_INTR(BOARD_MOTOR1_ENCODER_A_GPIO)
#define S2_ENCODER1_B_INTR S2_ENCODER_INTR(BOARD_MOTOR1_ENCODER_B_GPIO)
#define S2_ENCODER1_A_INTE S2_ENCODER_INTE(BOARD_MOTOR1_ENCODER_A_GPIO)
#define S2_ENCODER1_B_INTE S2_ENCODER_INTE(BOARD_MOTOR1_ENCODER_B_GPIO)
#define S2_ENCODER1_A_RISING S2_ENCODER_RISING(BOARD_MOTOR1_ENCODER_A_GPIO)
#define S2_ENCODER1_A_EDGES S2_ENCODER_EDGES(BOARD_MOTOR1_ENCODER_A_GPIO)
#define S2_ENCODER1_B_EDGES S2_ENCODER_EDGES(BOARD_MOTOR1_ENCODER_B_GPIO)

static volatile UW encoder_sequence;
static volatile UW encoder_positive;
static volatile UW encoder_negative;
static volatile UW encoder_a_edges;
static volatile UW encoder_b_edges;
static volatile UW encoder1_positive;
static volatile UW encoder1_negative;
static volatile UW encoder1_a_edges;
static volatile UW encoder1_b_edges;
static UW          encoder_ready;

/* Only the core-0 GPIO IRQ writes counters. Loads/stores remain atomic for
 * readers on either core without requiring Cortex-M0+ atomic RMW helpers.
 */
static void
encoder_increment (volatile UW * p_count)
{
    UW count = __atomic_load_n(p_count, __ATOMIC_RELAXED);

    __atomic_store_n(p_count, count + 1U, __ATOMIC_RELAXED);
}

static void
encoder_read_edges (UINT a_pin, UINT b_pin, volatile UW * p_positive,
                    volatile UW * p_negative, volatile UW * p_a_edges,
                    volatile UW * p_b_edges)
{
    UW a_pending = in_w(S2_ENCODER_INTR(a_pin)) & S2_ENCODER_EDGES(a_pin);
    UW b_pending = in_w(S2_ENCODER_INTR(b_pin)) & S2_ENCODER_EDGES(b_pin);

    /* Clear only this channel's pending bits, including shared banks. */
    if (a_pending != 0U)
    {
        out_w(S2_ENCODER_INTR(a_pin), a_pending);
        encoder_increment(p_a_edges);
        if ((a_pending & S2_ENCODER_RISING(a_pin)) != 0U)
        {
            if (gpio_get_val(b_pin) != 0U)
            {
                encoder_increment(p_positive);
            }
            else
            {
                encoder_increment(p_negative);
            }
        }
    }
    if (b_pending != 0U)
    {
        out_w(S2_ENCODER_INTR(b_pin), b_pending);
        encoder_increment(p_b_edges);
    }
}

static void
encoder_interrupt (UINT intno)
{
    UW sequence = __atomic_load_n(&encoder_sequence, __ATOMIC_RELAXED);

    (void) intno;
    /* Mark the snapshot busy before any counter changes. No kernel calls. */
    __atomic_store_n(&encoder_sequence, sequence + 1U, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    encoder_read_edges(BOARD_MOTOR1_ENCODER_A_GPIO, BOARD_MOTOR1_ENCODER_B_GPIO,
                       &encoder1_positive, &encoder1_negative,
                       &encoder1_a_edges, &encoder1_b_edges);
    encoder_read_edges(BOARD_MOTOR2_ENCODER_A_GPIO, BOARD_MOTOR2_ENCODER_B_GPIO,
                       &encoder_positive, &encoder_negative, &encoder_a_edges,
                       &encoder_b_edges);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_store_n(&encoder_sequence, sequence + 2U, __ATOMIC_RELEASE);
}

static ER
encoder_init (void)
{
    const UINT pins[] = {
        BOARD_MOTOR1_ENCODER_A_GPIO, BOARD_MOTOR1_ENCODER_B_GPIO,
        BOARD_MOTOR2_ENCODER_A_GPIO, BOARD_MOTOR2_ENCODER_B_GPIO};
    T_DINT config = {0};
    ER     er     = E_OK;

    /* Reject invalid/overlapping pins before touching encoder registers. */
    for (UINT i = 0; i < 4U; ++i)
    {
        if ((pins[i] >= 30U) || (pins[i] == BOARD_MOTOR1_A_GPIO) ||
            (pins[i] == BOARD_MOTOR1_B_GPIO) ||
            (pins[i] == BOARD_MOTOR2_A_GPIO) ||
            (pins[i] == BOARD_MOTOR2_B_GPIO))
        {
            return E_PAR;
        }
        for (UINT j = 0; j < i; ++j)
        {
            if (pins[i] == pins[j])
            {
                return E_PAR;
            }
        }
    }
    for (UINT i = 0; (i < 4U) && (er >= E_OK); ++i)
    {
        er = gpio_set_pin(pins[i], GPIO_MODE_IN);
        if (er < E_OK)
        {
            break;
        }
        /* Pull up open-collector outputs; input voltage must be 3.3 V. */
        set_w(GPIO(pins[i]), GPIO_PUE | GPIO_SHEMITT);
        clr_w(GPIO(pins[i]), GPIO_PDE);
    }
    if (er >= E_OK)
    {
        config.intatr = TA_HLNG;
        /* T-Kernel uses a generic function pointer for interrupt handlers. */
        config.inthdr = (FP) encoder_interrupt;
        er            = tk_def_int(S2_ENCODER_IRQ, &config);
    }
    if (er >= E_OK)
    {
        for (UINT i = 0; i < 4U; ++i)
        {
            out_w(S2_ENCODER_INTR(pins[i]), S2_ENCODER_EDGES(pins[i]));
            set_w(S2_ENCODER_INTE(pins[i]), S2_ENCODER_EDGES(pins[i]));
        }
        ClearInt(S2_ENCODER_IRQ);
        EnableInt(S2_ENCODER_IRQ, 3);
        __atomic_store_n(&encoder_ready, 1U, __ATOMIC_RELEASE);
        /* tm_printf takes an unsigned-byte format string. */
        tm_printf((UB *) "[S2] encoders motor 1 A=GP%u B=GP%u; "
                         "motor 2 A=GP%u B=GP%u ready (x1 counts)\n",
                  BOARD_MOTOR1_ENCODER_A_GPIO, BOARD_MOTOR1_ENCODER_B_GPIO,
                  BOARD_MOTOR2_ENCODER_A_GPIO, BOARD_MOTOR2_ENCODER_B_GPIO);
    }
    return er;
}
#endif

EXPORT ER
s2_encoder_get_status (s2_encoder_status_t * p_status)
{
    ER er = E_PAR;

    if (p_status != NULL)
    {
        *p_status = (s2_encoder_status_t){0};
#if S2_ENCODER_ENABLED
        er = E_OK;
        if (__atomic_load_n(&encoder_ready, __ATOMIC_ACQUIRE) != 0U)
        {
            UW     sequence;
            UW     pins;
            SYSTIM sampled;

            /* Retry if the IRQ updates either wheel during the copy. */
            UINT retries = 0;
            for (;;)
            {
                if (++retries > 32U)
                {
                    *p_status = (s2_encoder_status_t){0};
                    return E_TMOUT;
                }
                sequence = __atomic_load_n(&encoder_sequence, __ATOMIC_ACQUIRE);
                if ((sequence & 1U) != 0U)
                {
                    continue;
                }
                p_status->positive_pulses =
                    __atomic_load_n(&encoder_positive, __ATOMIC_RELAXED);
                p_status->negative_pulses =
                    __atomic_load_n(&encoder_negative, __ATOMIC_RELAXED);
                p_status->a_edges =
                    __atomic_load_n(&encoder_a_edges, __ATOMIC_RELAXED);
                p_status->b_edges =
                    __atomic_load_n(&encoder_b_edges, __ATOMIC_RELAXED);
                p_status->motor1_positive_pulses =
                    __atomic_load_n(&encoder1_positive, __ATOMIC_RELAXED);
                p_status->motor1_negative_pulses =
                    __atomic_load_n(&encoder1_negative, __ATOMIC_RELAXED);
                p_status->motor1_a_edges =
                    __atomic_load_n(&encoder1_a_edges, __ATOMIC_RELAXED);
                p_status->motor1_b_edges =
                    __atomic_load_n(&encoder1_b_edges, __ATOMIC_RELAXED);
                pins = in_w(GPIO_IN);
                er   = tk_get_otm(&sampled);
                if (er < E_OK)
                {
                    *p_status = (s2_encoder_status_t){0};
                    return er;
                }
                __atomic_thread_fence(__ATOMIC_SEQ_CST);
                if (sequence ==
                    __atomic_load_n(&encoder_sequence, __ATOMIC_ACQUIRE))
                {
                    break;
                }
            }
            p_status->a_level = (pins >> BOARD_MOTOR2_ENCODER_A_GPIO) & 1U;
            p_status->b_level = (pins >> BOARD_MOTOR2_ENCODER_B_GPIO) & 1U;
            p_status->motor1_a_level =
                (pins >> BOARD_MOTOR1_ENCODER_A_GPIO) & 1U;
            p_status->motor1_b_level =
                (pins >> BOARD_MOTOR1_ENCODER_B_GPIO) & 1U;
            p_status->motor1_forward_counts =
                (INT) (p_status->motor1_negative_pulses -
                       p_status->motor1_positive_pulses);
            p_status->motor2_forward_counts =
                (INT) (p_status->positive_pulses - p_status->negative_pulses);
            p_status->valid      = 1U;
            p_status->sampled_ms = sampled.lo;
        }
#else
        er = E_NOSPT;
#endif
    }
    return er;
}

EXPORT ER
s2_emergency_stop (void)
{
    if (h_controller_task == 0 || tk_get_tid() != h_controller_task)
    {
        return E_OACV;
    }
    UW epoch = __atomic_load_n(&stop_epoch, __ATOMIC_RELAXED);
    __atomic_store_n(&stop_epoch, epoch + 1U, __ATOMIC_RELEASE);
    return E_OK;
}

/* Internal result, never published: a STOP arrived while a drive command was
 * being applied. The loop's epoch path then switches PWM off and publishes
 * STOPPED; it is not a motion fault. */
#define S2_STOP_PENDING ((ER) - 1)

static BOOL
stop_pending (void)
{
    return __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE) !=
           __atomic_load_n(&stopped_epoch, __ATOMIC_ACQUIRE);
}

static void
motor_stop (void)
{
    pwm_set_cc(BOARD_MOTOR1_A_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR1_B_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR2_A_GPIO, 0);
    pwm_set_cc(BOARD_MOTOR2_B_GPIO, 0);
}

static ER
motor_init (void)
{
    const UINT pins[] = {BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
                         BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO};
    ER         er     = E_OK;

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
    if ((er >= E_OK) && !BOARD_MOTORS_DISABLED)
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

static void
motor_set_pair (UINT pin_a, UINT pin_b, INT direction, UW duty)
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

static ER
motor_apply (s2_move_command_t command, UINT power_percent, UW issued_at_ms,
             UINT direction_changed)
{
    INT left  = 0;
    INT right = 0;
#if BOARD_PID_ENABLED
    if (power_percent > BOARD_PID_MAX_POWER)
    {
        power_percent = BOARD_PID_MAX_POWER;
    }
#endif
    UW duty       = (MOTOR_PWM_WRAP + 1U) * power_percent;
    UW left_duty  = (duty * S2_MOTOR1_TRIM_PERCENT) / 10000U;
    UW right_duty = (duty * S2_MOTOR2_TRIM_PERCENT) / 10000U;
#if BOARD_PID_ENABLED
    INT initial_steering =
        s2_steerable(command)
            ? (direction_changed ? requested_steering : (INT) filtered_steering)
            : 0;
    unsigned left_request = s2_steered_demand_milli(
        power_percent, initial_steering, 0U, BOARD_PID_MAX_POWER);
    unsigned right_request = s2_steered_demand_milli(
        power_percent, initial_steering, 1U, BOARD_PID_MAX_POWER);
    left_duty  = (UW) ((uint64_t) (MOTOR_PWM_WRAP + 1U) * left_request *
                       S2_MOTOR1_TRIM_PERCENT / 10000000U);
    right_duty = (UW) ((uint64_t) (MOTOR_PWM_WRAP + 1U) * right_request *
                       S2_MOTOR2_TRIM_PERCENT / 10000000U);
#endif
    ER er = E_OK;

    if (direction_changed || command == S2_MOVE_STOP || !power_percent)
    {
        motor_stop();
    }
    /* Enforce the bench lock even if a caller bypasses the command queue. */
    if (BOARD_MOTORS_DISABLED)
    {
        motor_stop();
        return ((command == S2_MOVE_STOP) || (power_percent == 0U)) ? E_OK
                                                                    : E_OACV;
    }
    switch (command)
    {
        case S2_MOVE_FORWARD:
            left  = 1;
            right = 1;
            break;
        case S2_MOVE_REVERSE:
            left  = -1;
            right = -1;
            break;
        case S2_MOVE_LEFT:
            left  = -1;
            right = 1;
            break;
        case S2_MOVE_RIGHT:
            left  = 1;
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

        /* Power-only updates preserve PWM and the PI steering history. */
        if (direction_changed)
        {
            S2_TIMING(pause);
        }
        er = direction_changed ? tk_dly_tsk(1) : E_OK;
        if (direction_changed)
        {
            S2_TIMING(resume);
        }
        /* The VC may STOP on the tick the dead time ends: that STOP wins,
         * and it is not a fault. */
        if (er >= E_OK && stop_pending())
        {
            er = S2_STOP_PENDING;
        }
        if (er >= E_OK)
        {
            er = tk_get_otm(&now);
        }
        if ((er >= E_OK) && ((now.lo - issued_at_ms) >= S2_COMMAND_TIMEOUT_MS))
        {
            er = E_TMOUT;
        }
        if (er >= E_OK)
        {
#if BOARD_PID_ENABLED
            /* A new spin starts both wheels at the breakaway kick;
             * motor_regulate keeps it until each wheel really moves. (A slow
             * straight/reverse/pivot move is kicked only in motor_regulate,
             * and only once a wheel has stopped making progress.) */
            if (direction_changed)
            {
                UW kick = (UW) ((uint64_t) (MOTOR_PWM_WRAP + 1U) *
                                s2_turn_kick_milli(
                                    command, 0U, BOARD_TURN_KICK_MS,
                                    BOARD_TURN_KICK_POWER,
                                    s2_turn_limit(command, BOARD_PID_MAX_POWER,
                                                  BOARD_PID_TURN_MAX_POWER)) /
                                100000U);
                if (left_duty < kick)
                {
                    left_duty = kick;
                }
                if (right_duty < kick)
                {
                    right_duty = kick;
                }
            }
#endif
            motor_set_pair(BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
                           left * MOTOR1_FORWARD_SIGN, left_duty);
            motor_set_pair(BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO,
                           right * MOTOR2_FORWARD_SIGN, right_duty);
        }
    }
    return er;
}

#if BOARD_PID_ENABLED
/* A STOP freezes each driven wheel's stall time. It keeps it only for a
 * wheel that has turned since its count started: a start that has not turned
 * yet (the VC restarts its power ramps after a STOP) starts again, as before.
 * A FAULT forgets it. */
static void
stall_freeze (BOOL forget)
{
    for (unsigned n = 0; n < 2; ++n)
    {
        if (pid_initialized && pid_dir[n])
        {
            stall_dir[n]  = pid_dir[n];
            stall_held[n] = pid_time - pid_progress[n];
        }
        if (forget || !stall_moved[n])
        {
            stall_dir[n] = 0;
        }
        pid_dir[n] = 0;
    }
}
/* Stall time wheel n brings into a new command (key: direction, x2 in a
 * spin). A wheel the last command drove the same way keeps the time since
 * its last progress. A wheel the last command did not drive (pivot, STOP)
 * keeps the time it had then. A new key, a FAULT or a STOP longer than the
 * allowance starts from 0. A wheel that is not driven is never checked. */
static uint32_t
stall_carry (unsigned n, int key, uint32_t count, uint32_t now, uint32_t stall)
{
    if (pid_initialized && pid_dir[n])
    {
        int32_t delta;
        BOOL    moved = edr_count_delta(count, pid_counts[n], &delta) &&
                        (pid_dir[n] > 0 ? delta > 0 : delta < 0);
        stall_dir[n]  = pid_dir[n];
        stall_held[n] = moved ? 0U : now - pid_progress[n];
        if (moved)
        {
            stall_moved[n] = TRUE;
        }
    }
    else if (!pid_initialized && now - pid_time > stall)
    {
        stall_dir[n] = 0;
    }
    pid_dir[n] = (int8_t) key;
    if (!key)
    {
        return 0U;
    }
    if (key != stall_dir[n])
    {
        stall_dir[n]   = (int8_t) key;
        stall_held[n]  = 0U;
        stall_moved[n] = FALSE;
    }
    return stall_held[n];
}
static ER
motor_regulate (uint16_t word)
{
    uint8_t             command, power, state;
    s2_encoder_status_t e;
    if (!edr_motor_unpack(word, &command, &power, &state))
    {
        return E_SYS;
    }
    if (state != S2_MOTION_MOVING || !power)
    {
        stall_freeze(state == S2_MOTION_FAULT);
        pid_initialized    = FALSE;
        filtered_steering  = 0;
        requested_steering = 0;
        return E_OK;
    }
    ER er = s2_encoder_get_status(&e);
    if (er < E_OK || !e.valid)
    {
        return er < E_OK ? er : E_OBJ;
    }
    uint32_t counts[2]    = {(uint32_t) e.motor1_forward_counts,
                             (uint32_t) e.motor2_forward_counts};
    int      direction[2] = {0, 0};
    switch (command)
    {
        case S2_MOVE_FORWARD:
            direction[0] = 1;
            direction[1] = 1;
            break;
        case S2_MOVE_REVERSE:
            direction[0] = -1;
            direction[1] = -1;
            break;
        case S2_MOVE_LEFT:
            direction[0] = -1;
            direction[1] = 1;
            break;
        case S2_MOVE_RIGHT:
            direction[0] = 1;
            direction[1] = -1;
            break;
        case S2_MOVE_PIVOT_RIGHT:
            direction[0] = 1;
            break;
        case S2_MOVE_PIVOT_LEFT:
            direction[1] = 1;
            break;
        default:
            return E_PAR;
    }
    uint32_t stall = s2_stall_ms(command, 500U, BOARD_TURN_STALL_MS);
    /* Spins scrub: more PWM headroom, a breakaway kick for each wheel until
     * it really moves (and again if it re-stalls), and a longer stall
     * allowance (turn_drive.h). A very slow straight, reverse or pivot move
     * is not kicked at its start: only a wheel that has made no progress for
     * REKICK_MS gets the kick (s2_kick_step), so a rolling wheel, a soft
     * start that is already moving and the slow barcode read are untouched,
     * and a wheel that would otherwise reach the 500 ms stall breaks away.
     * Speed targets are unchanged. */
    unsigned limit =
        s2_turn_limit(command, BOARD_PID_MAX_POWER, BOARD_PID_TURN_MAX_POWER);
    unsigned kick = s2_turn_kick_milli(command, 0U, BOARD_TURN_KICK_MS,
                                       BOARD_TURN_KICK_POWER, limit);
#if BOARD_TURN_KICK_MS
    if (!kick)
    {
        kick = s2_low_kick_milli(command, power, BOARD_LOW_KICK_BELOW,
                                 BOARD_LOW_KICK_POWER, limit);
    }
#endif
    if (!pid_initialized || pid_command != command)
    {
        /* The PI restarts, but each wheel's stall time carries over, so a
         * held wheel still faults while the follower switches commands. */
        BOOL stalled = FALSE;
        for (unsigned n = 0; n < 2; ++n)
        {
            uint32_t held = stall_carry(
                n, direction[n] * (s2_turn_command(command) ? 2 : 1), counts[n],
                e.sampled_ms, stall);
            if (held > stall)
            {
                stalled = TRUE;
            }
            wheel_pid[n]    = (speed_pid_t){0};
            pid_counts[n]   = counts[n];
            pid_progress[n] = e.sampled_ms - held;
            pid_speed[n]    = 0;
            s2_kick_start(&turn_kick[n], e.sampled_ms,
                          s2_turn_command(command) ? BOARD_TURN_KICK_MS : 0U);
        }
        pid_time          = e.sampled_ms;
        pid_command       = command;
        pid_initialized   = TRUE;
        filtered_steering = s2_steerable(command) ? requested_steering : 0;
        return stalled ? E_TMOUT : E_OK;
    }
    uint32_t dt = e.sampled_ms - pid_time;
    if (dt < 10U)
    {
        return E_OK;
    }
    if (dt > 100U)
    {
        return E_TMOUT;
    }
    filtered_steering = s2_steering_step(
        filtered_steering, s2_steerable(command) ? requested_steering : 0, dt);
    unsigned output[2] = {0, 0};
    for (unsigned n = 0; n < 2; ++n)
    {
        int32_t delta;
        if (!edr_count_delta(counts[n], pid_counts[n], &delta) ||
            delta > 10000 || delta < -10000)
        {
            return E_SYS;
        }
        if (direction[n])
        {
            int32_t progress = delta * direction[n];
            if (progress > 0)
            {
                pid_progress[n] = e.sampled_ms;
                stall_moved[n]  = TRUE;
            }
            if (e.sampled_ms - pid_progress[n] > stall)
            {
                return E_TMOUT;
            }
            int32_t instant =
                (int32_t) ((int64_t) delta * direction[n] * 1000 / dt);
            pid_speed[n]          = (3 * pid_speed[n] + instant) / 4;
            unsigned wheel_demand = s2_steered_demand_milli(
                power, filtered_steering, n, BOARD_PID_MAX_POWER);
            int32_t  target = (int32_t) ((uint64_t) wheel_demand *
                                         BOARD_PID_MAX_CPS / 100000U);
            unsigned trim =
                n == 0 ? S2_MOTOR1_TRIM_PERCENT : S2_MOTOR2_TRIM_PERCENT;
            output[n] = speed_pid_step_milli(
                &wheel_pid[n], target, pid_speed[n], dt,
                (unsigned) ((uint64_t) wheel_demand * trim / 100U), limit,
                BOARD_PID_KP_MILLI, BOARD_PID_KI_MILLI, BOARD_PID_KD_MILLI);
            if (kick &&
                s2_kick_step(&turn_kick[n], e.sampled_ms, progress,
                             e.sampled_ms - pid_progress[n], BOARD_TURN_KICK_MS,
                             BOARD_TURN_KICK_COUNTS, BOARD_TURN_REKICK_MS) &&
                output[n] < kick)
            {
                output[n] = kick;
            }
        }
        pid_counts[n] = counts[n];
    }
    pid_time = e.sampled_ms;
    if (stop_pending())
    {
        return S2_STOP_PENDING; /* the VC preempted this update with a STOP */
    }
    motor_set_pair(
        BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
        direction[0] * MOTOR1_FORWARD_SIGN,
        (UW) ((uint64_t) (MOTOR_PWM_WRAP + 1U) * output[0] / 100000U));
    motor_set_pair(
        BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO,
        direction[1] * MOTOR2_FORWARD_SIGN,
        (UW) ((uint64_t) (MOTOR_PWM_WRAP + 1U) * output[1] / 100000U));
    return E_OK;
}
#endif

static BOOL
movement_valid (s2_move_command_t command, UINT power_percent)
{
    return ((UINT) command <= (UINT) S2_MOVE_PIVOT_LEFT) &&
           (power_percent <= 100U);
}

static ER
motion_publish (const s2_packed_status_t * p_status)
{
    ER er = tk_wai_sem(h_status_lock, 1, 5);

    if (er >= E_OK)
    {
        /* The semaphore also protects readers on the other core. */
        motion_status = *p_status;
        er            = tk_sig_sem(h_status_lock, 1);
    }
    /* The VC and core-1 telemetry share the lock, so the wait can time
     * out: motor_task retries the copy on its next poll. */
    status_unpublished = er < E_OK;
    return er;
}

EXPORT ER
s2_motion_get_status (s2_motion_status_t * p_status)
{
    uint8_t            command;
    uint8_t            power;
    uint8_t            state;
    s2_packed_status_t packed;
    ER                 er;

    if (NULL == p_status)
    {
        return E_PAR;
    }

    if (h_status_lock <= 0)
    {
        return E_OBJ;
    }

    er = tk_wai_sem(h_status_lock, 1, 5);
    if (er < E_OK)
    {
        return er;
    }

    packed = motion_status;
    er     = tk_sig_sem(h_status_lock, 1);
    if (er < E_OK)
    {
        return er;
    }

    if (!edr_motor_unpack(packed.motor, &command, &power, &state))
    {
        return E_SYS;
    }

    /* The codec validates both legacy enum ranges before these casts. */
    p_status->command         = (s2_move_command_t) command;
    p_status->power_percent   = power;
    p_status->state           = (s2_motion_state_t) state;
    p_status->last_error      = packed.last_error;
    p_status->last_command_ms = packed.last_command_ms;

    return E_OK;
}

static ER
motion_fault (s2_packed_status_t * p_status, ER error)
{
    motor_stop();
    if (!edr_motor_pack(UINT8_C(0), UINT8_C(0), S2_MOTION_FAULT,
                        &p_status->motor))
    {
        return E_SYS;
    }

    p_status->last_error = error;
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *) "[S2 FAULT] error=%d; PWM outputs off\n", error);
    return motion_publish(p_status);
}

/* Compute the watchdog wait from packed state, without keeping decoded
 * command fields alive while the motor task waits for another message. */
static ER
s2_command_wait (const s2_packed_status_t * const p_status, TMO * const p_wait)
{
    uint8_t  command;
    uint8_t  power;
    uint8_t  state;
    SYSTIM   now;
    uint32_t elapsed;
    ER       result;

    if (!edr_motor_unpack(p_status->motor, &command, &power, &state))
    {
        return E_SYS;
    }

    *p_wait = APP_S2_ONLY ? TMO_FEVR : 10;
    if (S2_MOTION_MOVING != state)
    {
        return E_OK;
    }

    result = tk_get_otm(&now);
    if (result < E_OK)
    {
        return result;
    }

    elapsed = now.lo - p_status->last_command_ms;
    if (elapsed >= S2_COMMAND_TIMEOUT_MS)
    {
        return E_TMOUT;
    }

    /* The configured lease fits a positive TMO value. */
    *p_wait = (TMO) (S2_COMMAND_TIMEOUT_MS - elapsed);
    if (!APP_S2_ONLY && *p_wait > 10)
    {
        *p_wait = 10;
    }

    return E_OK;
}

static void
motor_task (INT stacd, void * p_exinf)
{
    s2_packed_status_t current = {0, E_OK, 0};
    ER                 er      = motor_init();

    (void) stacd;
    (void) p_exinf;
#if S2_ENCODER_ENABLED
    if (er >= E_OK)
    {
        er = encoder_init();
    }
#else
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *) "[S2] open-loop PWM; encoder diagnostics disabled\n");
#endif
    if (er < E_OK)
    {
        CHECK(motion_fault(&current, er), "S2 initialisation fault status");
        /* PWM stays off and FAULT stays published (VC fault 7 once armed):
         * commands are dropped, stop epochs never acknowledged. Never
         * return: tasks start with lr 0, so a return HardFaults core 0. */
        for (;;)
        {
#if defined(S2_HOST_TEST_STOP_REQUESTED)
            if (S2_HOST_TEST_STOP_REQUESTED())
            {
                return;
            }
#endif
            T_MSG * p_raw = NULL;

            S2_HEARTBEAT(); /* alive, though faulted: no watchdog reboot loop */
            if (status_unpublished)
            {
                (void) motion_publish(&current);
            }
            er = tk_rcv_mbx(h_movement_mailbox, &p_raw, 10);
            if (er >= E_OK)
            {
                CHECK(tk_rel_mpf(h_movement_pool, p_raw), "S2 drop command");
            }
            else if (er != E_TMOUT)
            {
                tk_dly_tsk(10);
            }
        }
    }

    /* Never leaves: a task return HardFaults core 0 (lr is 0). */
    for (;;)
    {
#if defined(S2_HOST_TEST_STOP_REQUESTED)
        if (S2_HOST_TEST_STOP_REQUESTED())
        {
            return;
        }
#endif
        T_MSG * p_raw = NULL;
        TMO     wait  = 10;
        S2_HEARTBEAT();
        UW epoch = __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE);
        if (epoch != __atomic_load_n(&stopped_epoch, __ATOMIC_RELAXED))
        {
            motor_stop();
            current.motor      = 0; /* STOP / zero power / STOPPED in EDR v1. */
            current.last_error = E_OK;
            __atomic_store_n(&stopped_epoch, epoch, __ATOMIC_RELEASE);
            er = motion_publish(&current);
        }
        /* A status copy that missed the lock is retried every poll. PWM
         * already matches `current` (off in FAULT), so it never stops S2. */
        else if (status_unpublished)
        {
            (void) motion_publish(&current);
        }
        SYSTIM now;

        er = s2_command_wait(&current, &wait);
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }

#if BOARD_PID_ENABLED
        er = motor_regulate(current.motor);
        if (er == S2_STOP_PENDING)
        {
            continue; /* the epoch path publishes STOPPED */
        }
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }
        if (wait > 10 || wait == TMO_FEVR)
        {
            wait = 10;
        }
#endif
        /* Poll again soon to retry a missed status copy. */
        if (status_unpublished && (wait > 10 || wait == TMO_FEVR))
        {
            wait = 10;
        }
        S2_TIMING(done);
        er = tk_rcv_mbx(h_movement_mailbox, &p_raw, wait);
        S2_TIMING(release);
        if (er == E_TMOUT)
        {
            er = E_OK;
            continue;
        }
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
        s2_movement_msg_t * p_msg    = (s2_movement_msg_t *) p_raw;
        uint16_t            motor    = p_msg->motor;
        INT                 steering = p_msg->steering;
        uint8_t             packed_command;
        uint8_t             packed_power;
        uint8_t             packed_state;
        s2_move_command_t   command;
        UINT                power;
        uint8_t             current_command;
        uint8_t             current_power;
        uint8_t             current_state;
        UW                  issued_at     = p_msg->issued_at_ms;
        UW                  message_epoch = p_msg->stop_epoch;

        /* Copy all fields before returning the block to the pool. */
        er = tk_rel_mpf(h_movement_pool, p_msg);
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }
        if (message_epoch != __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE))
        {
            continue;
        }
        if (!edr_motor_unpack(motor, &packed_command, &packed_power,
                              &packed_state) ||
            (packed_state != S2_MOTION_STOPPED))
        {
            er = motion_fault(&current, E_PAR);
            continue;
        }
        /* Unpack validates the command and power before widening. */
        command = (s2_move_command_t) packed_command;
        power   = packed_power;
        if (steering < -400 || steering > 400 ||
            (!s2_steerable(command) && steering != 0))
        {
            er = motion_fault(&current, E_PAR);
            continue;
        }
        requested_steering = steering;

        if ((command == S2_MOVE_STOP) || (power == 0U))
        {
            command = S2_MOVE_STOP;
            power   = 0;
        }
        else
        {
            er = tk_get_otm(&now);
            if ((er >= E_OK) && ((now.lo - issued_at) >= S2_COMMAND_TIMEOUT_MS))
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
        if (!edr_motor_unpack(current.motor, &current_command, &current_power,
                              &current_state))
        {
            er = motion_fault(&current, E_SYS);
            continue;
        }

        if (((UINT) command != current_command) || (power != current_power))
        {
            er = motor_apply(command, power, issued_at,
                             (UINT) command != current_command);
            /* A STOP arrived while applying: the epoch path at the top of
             * the loop switches PWM off and publishes STOPPED, no fault. */
            if (er == S2_STOP_PENDING)
            {
                continue;
            }
            if (er < E_OK)
            {
                er = motion_fault(&current, er);
                continue;
            }
            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *) "[S2 MOTOR] command=%d power=%u%%\n", command,
                      power);
        }
        /* Validated command/power values fit exactly in the packed fields. */
        if (!edr_motor_pack((uint8_t) command, (uint8_t) power,
                            (S2_MOVE_STOP == command) ? S2_MOTION_STOPPED
                                                      : S2_MOTION_MOVING,
                            &current.motor))
        {
            er = motion_fault(&current, E_SYS);
            continue;
        }

        current.last_error      = E_OK;
        current.last_command_ms = issued_at;
        er                      = motion_publish(&current);
    }
}

EXPORT ER
subsystem2_init (void)
{
    T_CMBX mailbox_config = {0};
    T_CMPF pool_config    = {0};
    T_CSEM lock_config    = {0};
    UW     attempts       = 10000U;
    ER     er             = E_OK;

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
        h_movement_mailbox    = tk_cre_mbx(&mailbox_config);
        er = (h_movement_mailbox > 0) ? E_OK : h_movement_mailbox;
    }
    if (er >= E_OK)
    {
        pool_config.mpfatr = TA_TFIFO | TA_RNG3;
        pool_config.mpfcnt = S2_COMMAND_COUNT;
        pool_config.blfsz  = sizeof(s2_movement_msg_t);
        h_movement_pool    = tk_cre_mpf(&pool_config);
        er                 = (h_movement_pool > 0) ? E_OK : h_movement_pool;
        if (er < E_OK)
        {
            tk_del_mbx(h_movement_mailbox);
            h_movement_mailbox = 0;
        }
    }
    if (er >= E_OK)
    {
        lock_config.sematr  = TA_TPRI;
        lock_config.isemcnt = 1;
        lock_config.maxsem  = 1;
        h_status_lock       = tk_cre_sem(&lock_config);
        er                  = (h_status_lock > 0) ? E_OK : h_status_lock;
        if (er < E_OK)
        {
            tk_del_mpf(h_movement_pool);
            tk_del_mbx(h_movement_mailbox);
            h_movement_pool    = 0;
            h_movement_mailbox = 0;
            h_status_lock      = 0;
        }
        else
        {
            /* STOP, zero power and STOPPED are all encoded as zero. */
            motion_status = (s2_packed_status_t){0, E_OK, 0};
        }
    }
    return er;
}

EXPORT ER
subsystem2_start (void)
{
    ER er = E_OBJ;

    if ((h_status_lock > 0) && !motor_started)
    {
        /* app_start_task takes T-Kernel's generic function pointer. */
        ID h_task =
            app_start_task((FP) motor_task, S2_PRI_MOTOR, TP_PRC1, "motor");

        er = (h_task > 0) ? E_OK : h_task;
        if (er >= E_OK)
        {
            motor_started = TRUE;
        }
    }
    return er;
}

static ER
movement_send (s2_move_command_t command, UINT power_percent, INT steering)
{
    void *   p_block = NULL;
    SYSTIM   now     = {0};
    uint16_t motor;
    ER       er = E_OK;
    if (steering < -400 || steering > 400 ||
        (!s2_steerable(command) && steering != 0))
    {
        return E_PAR;
    }
#if !BOARD_PID_ENABLED
    if (steering != 0)
    {
        return E_NOSPT;
    }
#endif

#if !APP_S2_ONLY
    if ((h_controller_task == 0) || (tk_get_tid() != h_controller_task))
    {
        er = E_OACV;
    }
#endif
    if (er >= E_OK && movement_valid(command, power_percent) &&
        (command == S2_MOVE_STOP || power_percent == 0U))
    {
#if !APP_S2_ONLY
        return s2_emergency_stop();
#endif
    }
    if ((er >= E_OK) && !movement_valid(command, power_percent))
    {
        er = E_PAR;
    }
    if ((er >= E_OK) && BOARD_MOTORS_DISABLED && (command != S2_MOVE_STOP) &&
        (power_percent != 0U))
    {
        er = E_OACV;
    }
    if ((er >= E_OK) && !motor_started)
    {
        er = E_OBJ;
    }
    if (er >= E_OK)
    {
        if ((command == S2_MOVE_STOP) || (power_percent == 0U))
        {
            command       = S2_MOVE_STOP;
            power_percent = 0;
            /* A STOP must still work if the clock service fails. */
            (void) tk_get_otm(&now);
        }
        else
        {
            er = tk_get_otm(&now);
        }
    }
    if (er >= E_OK && command != S2_MOVE_STOP &&
        __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE) !=
            __atomic_load_n(&stopped_epoch, __ATOMIC_ACQUIRE))
    {
        er = E_OBJ;
    }
    if (er >= E_OK)
    {
        /* movement_valid() checked both fixed-width value ranges. */
        if (!edr_motor_pack((uint8_t) command, (uint8_t) power_percent,
                            UINT8_C(0), &motor))
        {
            er = E_PAR;
        }
    }
    if (er >= E_OK)
    {
        er = tk_get_mpf(h_movement_pool, &p_block, TMO_POL);
    }
    if (er >= E_OK)
    {
        /* Memory pool blocks have room and alignment for this message. */
        s2_movement_msg_t * p_msg = (s2_movement_msg_t *) p_block;

        p_msg->hdr.msgque[0] = NULL;
        p_msg->motor         = motor;
        p_msg->steering      = (int16_t) steering;
        p_msg->issued_at_ms  = now.lo;
        p_msg->stop_epoch    = __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE);
        /* The mailbox header is the first member of the message. */
        er = tk_snd_mbx(h_movement_mailbox, (T_MSG *) p_msg);
        if (er < E_OK)
        {
            CHECK(tk_rel_mpf(h_movement_pool, p_msg), "S2 send cleanup");
        }
    }
    return er;
}

EXPORT ER
s2_movement_send (s2_move_command_t command, UINT power_percent)
{
    return movement_send(command, power_percent, 0);
}
EXPORT ER
s2_follow_send (UINT power_percent, INT steering)
{
    return movement_send(S2_MOVE_FORWARD, power_percent, steering);
}
EXPORT ER
s2_reverse_follow_send (UINT power_percent, INT steering)
{
    return movement_send(S2_MOVE_REVERSE, power_percent, steering);
}

/*** end of file ***/
