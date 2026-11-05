/*
 * subsystem2.c
 * Buddy 2 -- Motor owner with optional calibrated wheel-speed PID. Student ID: 2500613.
 */

#include "subsystem2.h"
#include "../board_pins.h"
#include "../shared/edr.h"
#include "../robot_config.h"
#ifndef BOARD_DRIVE_POWER_CAP
#define BOARD_DRIVE_POWER_CAP 100U
#endif
#ifndef BOARD_PID_LEFT_FF_OFFSET
#define BOARD_PID_LEFT_FF_OFFSET 0U
#endif
#ifndef BOARD_PID_RIGHT_FF_OFFSET
#define BOARD_PID_RIGHT_FF_OFFSET 0U
#endif
#ifndef BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE
#define BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE 1000U
#endif
#ifndef BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE
#define BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE 1000U
#endif
#if BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE > 1000U || BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE < 500U || BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE > 1000U || BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE < 500U
#error Forward wheel target calibration must be between 500 and 1000 permille
#endif
#include <tk/syslib.h>
#include "speed_pid.h"
#include "wheel_steering.h"
static INT requested_steering;
#if BOARD_PID_ENABLED && !S2_ENCODER_ENABLED
#error Speed regulation requires both encoders
#endif
#if BOARD_PID_ENABLED
static speed_pid_t wheel_pid[2];
static uint32_t pid_counts[2], pid_time, pid_progress[2];
static int32_t pid_speed[2];
static BOOL pid_initialized;
static UINT pid_command;
static int32_t filtered_steering;
#endif

/* Persistent state uses the same motor word as the external representation.
 * Native enum fields exist only in temporary compatibility snapshots. */
typedef struct
{
    uint16_t motor;
    ER last_error;
    UW last_command_ms;
} s2_packed_status_t;

static ID h_controller_task;
static UW stop_epoch;
static UW stopped_epoch;
static ID h_movement_mailbox;
static ID h_movement_pool;
static ID h_status_lock;
static BOOL motor_started;
static s2_packed_status_t motion_status;

/* PWM duty = compare / (wrap + 1). */
#define MOTOR_PWM_WRAP 6249U

/* Motor 1 is the left wheel, motor 2 the right wheel.
 * Change either sign if that wheel rotates backwards during FORWARD.
 */
#define MOTOR1_FORWARD_SIGN 1
#define MOTOR2_FORWARD_SIGN (-1)

EXPORT void s2_controller_bind(void)
{
    if (h_controller_task == 0) h_controller_task = tk_get_tid();
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
static UW encoder_ready;

/* Only the core-0 GPIO IRQ writes counters. Loads/stores remain atomic for
 * readers on either core without requiring Cortex-M0+ atomic RMW helpers.
 */
static void encoder_increment(volatile UW *p_count)
{
    UW count = __atomic_load_n(p_count, __ATOMIC_RELAXED);

    __atomic_store_n(p_count, count + 1U, __ATOMIC_RELAXED);
}

static void encoder_read_edges(UINT a_pin, UINT b_pin,
                               volatile UW *p_positive,
                               volatile UW *p_negative,
                               volatile UW *p_a_edges,
                               volatile UW *p_b_edges)
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

static void encoder_interrupt(UINT intno)
{
    UW sequence = __atomic_load_n(&encoder_sequence, __ATOMIC_RELAXED);

    (void)intno;
    /* Mark the snapshot busy before any counter changes. No kernel calls. */
    __atomic_store_n(&encoder_sequence, sequence + 1U, __ATOMIC_RELAXED);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    encoder_read_edges(BOARD_MOTOR1_ENCODER_A_GPIO,
                       BOARD_MOTOR1_ENCODER_B_GPIO,
                       &encoder1_positive, &encoder1_negative,
                       &encoder1_a_edges, &encoder1_b_edges);
    encoder_read_edges(BOARD_MOTOR2_ENCODER_A_GPIO,
                       BOARD_MOTOR2_ENCODER_B_GPIO,
                       &encoder_positive, &encoder_negative,
                       &encoder_a_edges, &encoder_b_edges);
    __atomic_thread_fence(__ATOMIC_SEQ_CST);
    __atomic_store_n(&encoder_sequence, sequence + 2U, __ATOMIC_RELEASE);
}

static ER encoder_init(void)
{
    const UINT pins[] =
    {
        BOARD_MOTOR1_ENCODER_A_GPIO, BOARD_MOTOR1_ENCODER_B_GPIO,
        BOARD_MOTOR2_ENCODER_A_GPIO, BOARD_MOTOR2_ENCODER_B_GPIO
    };
    T_DINT config = {0};
    ER er = E_OK;

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
        config.inthdr = (FP)encoder_interrupt;
        er = tk_def_int(S2_ENCODER_IRQ, &config);
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
        tm_printf((UB *)"[S2] encoders motor 1 A=GP%u B=GP%u; "
                  "motor 2 A=GP%u B=GP%u ready (x1 counts)\n",
                  BOARD_MOTOR1_ENCODER_A_GPIO, BOARD_MOTOR1_ENCODER_B_GPIO,
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
#if S2_ENCODER_ENABLED
        er = E_OK;
        if (__atomic_load_n(&encoder_ready, __ATOMIC_ACQUIRE) != 0U)
        {
            UW sequence;
            UW pins;
            SYSTIM sampled;

            /* Retry if the IRQ updates either wheel during the copy. */
            UINT retries = 0;
            for (;;)
            {
                if (++retries > 32U) { *p_status = (s2_encoder_status_t){0}; return E_TMOUT; }
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
                er = tk_get_otm(&sampled);
                if (er < E_OK)
                {
                    *p_status = (s2_encoder_status_t){0};
                    return er;
                }
                __atomic_thread_fence(__ATOMIC_SEQ_CST);
                if (sequence == __atomic_load_n(&encoder_sequence,
                                                __ATOMIC_ACQUIRE))
                {
                    break;
                }
            }
            p_status->a_level =
                (pins >> BOARD_MOTOR2_ENCODER_A_GPIO) & 1U;
            p_status->b_level =
                (pins >> BOARD_MOTOR2_ENCODER_B_GPIO) & 1U;
            p_status->motor1_a_level =
                (pins >> BOARD_MOTOR1_ENCODER_A_GPIO) & 1U;
            p_status->motor1_b_level =
                (pins >> BOARD_MOTOR1_ENCODER_B_GPIO) & 1U;
            p_status->motor1_forward_counts =
                (INT)(p_status->motor1_negative_pulses -
                      p_status->motor1_positive_pulses);
            p_status->motor2_forward_counts =
                (INT)(p_status->positive_pulses - p_status->negative_pulses);
            p_status->valid = 1U;
            p_status->sampled_ms = sampled.lo;
        }
#else
        er = E_NOSPT;
#endif
    }
    return er;
}

EXPORT ER s2_emergency_stop(void)
{
    if (h_controller_task == 0 || tk_get_tid() != h_controller_task) return E_OACV;
    UW epoch = __atomic_load_n(&stop_epoch, __ATOMIC_RELAXED);
    __atomic_store_n(&stop_epoch, epoch + 1U, __ATOMIC_RELEASE);
    return E_OK;
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

static void motor_set_pair(UINT pin_a, UINT pin_b, INT direction, UW duty)
{
#if BOARD_IR_LINE_TRACE_BENCH && defined(BOARD_RIGHT_PWM_PERMILLE)
    /* Scale PWM ticks after PID rounding to allow sub-percent trim changes. */
    if (pin_a == BOARD_MOTOR2_A_GPIO)
    {
        duty = (duty * BOARD_RIGHT_PWM_PERMILLE + 500U) / 1000U;
    }
#endif
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
#if BOARD_PID_ENABLED
    if(power_percent>BOARD_PID_MAX_POWER)power_percent=BOARD_PID_MAX_POWER;
#endif
    if(power_percent>BOARD_DRIVE_POWER_CAP)power_percent=BOARD_DRIVE_POWER_CAP;
    UW duty = (MOTOR_PWM_WRAP + 1U) * power_percent;
    UW left_duty = (duty * S2_MOTOR1_TRIM_PERCENT) / 10000U;
    UW right_duty = (duty * S2_MOTOR2_TRIM_PERCENT) / 10000U;
    ER er = E_OK;

    motor_stop();
    /* Enforce the bench lock even if a caller bypasses the command queue. */
    if (BOARD_MOTORS_DISABLED)
    {
        return ((command == S2_MOVE_STOP) || (power_percent == 0U)) ? E_OK : E_OACV;
    }
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
            er = tk_get_otm(&now);
        }
        if ((er >= E_OK) &&
            ((now.lo - issued_at_ms) >= S2_COMMAND_TIMEOUT_MS))
        {
            er = E_TMOUT;
        }
        if (er >= E_OK && __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE) !=
                              __atomic_load_n(&stopped_epoch, __ATOMIC_ACQUIRE)) er = E_OBJ;
        if (er >= E_OK)
        {
            motor_set_pair(BOARD_MOTOR1_A_GPIO, BOARD_MOTOR1_B_GPIO,
                           left * MOTOR1_FORWARD_SIGN, left_duty);
            motor_set_pair(BOARD_MOTOR2_A_GPIO, BOARD_MOTOR2_B_GPIO,
                           right * MOTOR2_FORWARD_SIGN, right_duty);
        }
    }
    return er;
}

#if BOARD_PID_ENABLED
static ER motor_regulate(uint16_t word)
{
    uint8_t command, power, state;
    s2_encoder_status_t e;
    if (!edr_motor_unpack(word,&command,&power,&state)) return E_SYS;
    if (state!=S2_MOTION_MOVING || !power) {
        pid_initialized=FALSE;filtered_steering=0;requested_steering=0;return E_OK;
    }
    ER er=s2_encoder_get_status(&e);
    if (er<E_OK || !e.valid) return er<E_OK?er:E_OBJ;
    uint32_t counts[2]={(uint32_t)e.motor1_forward_counts,(uint32_t)e.motor2_forward_counts};
    int direction[2]={0,0};
    switch(command) {
    case S2_MOVE_FORWARD:direction[0]=1;direction[1]=1;break;
    case S2_MOVE_REVERSE:direction[0]=-1;direction[1]=-1;break;
    case S2_MOVE_LEFT:direction[0]=-1;direction[1]=1;break;
    case S2_MOVE_RIGHT:direction[0]=1;direction[1]=-1;break;
    case S2_MOVE_PIVOT_RIGHT:direction[0]=1;break;
    case S2_MOVE_PIVOT_LEFT:direction[1]=1;break;
    default:return E_PAR;
    }
    if (!pid_initialized || pid_command!=command) {
        for(unsigned n=0;n<2;++n) {
            wheel_pid[n]=(speed_pid_t){0};pid_counts[n]=counts[n];
            pid_progress[n]=e.sampled_ms;pid_speed[n]=0;
        }
        pid_time=e.sampled_ms;pid_command=command;pid_initialized=TRUE;
        filtered_steering=0;return E_OK;
    }
    uint32_t dt=e.sampled_ms-pid_time;
    if(dt<10U)return E_OK;
    if(dt>100U)return E_TMOUT;
    unsigned output_limit=BOARD_PID_MAX_POWER;
    if(output_limit>BOARD_DRIVE_POWER_CAP)output_limit=BOARD_DRIVE_POWER_CAP;
    filtered_steering=s2_steering_step(filtered_steering,
        command==S2_MOVE_FORWARD?requested_steering:0,dt);
    unsigned output[2]={0,0};
#if BOARD_WHEEL_CAL_BENCH
    int32_t targets[2]={0,0};
#endif
    for(unsigned n=0;n<2;++n) {
        int32_t delta;
        if(!edr_count_delta(counts[n],pid_counts[n],&delta) || delta>10000 || delta< -10000)return E_SYS;
        if(direction[n]) {
            if(delta*direction[n]>0)pid_progress[n]=e.sampled_ms;
            if(e.sampled_ms-pid_progress[n]>500U)return E_TMOUT;
            int32_t instant=(int32_t)((int64_t)delta*direction[n]*1000/dt);
            pid_speed[n]=(3*pid_speed[n]+instant)/4;
            unsigned wheel_demand=s2_steered_demand_milli(power,filtered_steering,n,output_limit);
            int32_t target=(int32_t)((uint64_t)wheel_demand*BOARD_PID_MAX_CPS/100000U);
            if(command==S2_MOVE_FORWARD) {
                unsigned balance=n==0?BOARD_PID_FORWARD_LEFT_TARGET_PERMILLE:
                                      BOARD_PID_FORWARD_RIGHT_TARGET_PERMILLE;
                target=(int32_t)((int64_t)target*balance/1000U);
            }
#if BOARD_WHEEL_CAL_BENCH
            targets[n]=target;
#endif
            unsigned trim=n==0?S2_MOTOR1_TRIM_PERCENT:S2_MOTOR2_TRIM_PERCENT;
            unsigned offset=n==0?BOARD_PID_LEFT_FF_OFFSET:BOARD_PID_RIGHT_FF_OFFSET;
            output[n]=speed_pid_step_milli(&wheel_pid[n],target,pid_speed[n],dt,
                speed_pid_feedforward(wheel_demand,trim,offset,output_limit),output_limit,
                BOARD_PID_KP_MILLI,BOARD_PID_KI_MILLI,BOARD_PID_KD_MILLI);
        }
        pid_counts[n]=counts[n];
    }
    pid_time=e.sampled_ms;
    if(__atomic_load_n(&stop_epoch,__ATOMIC_ACQUIRE)!=__atomic_load_n(&stopped_epoch,__ATOMIC_ACQUIRE))return E_OBJ;
    motor_set_pair(BOARD_MOTOR1_A_GPIO,BOARD_MOTOR1_B_GPIO,direction[0]*MOTOR1_FORWARD_SIGN,
                   (MOTOR_PWM_WRAP+1U)*output[0]/100000U);
    motor_set_pair(BOARD_MOTOR2_A_GPIO,BOARD_MOTOR2_B_GPIO,direction[1]*MOTOR2_FORWARD_SIGN,
                   (MOTOR_PWM_WRAP+1U)*output[1]/100000U);
#if BOARD_WHEEL_CAL_BENCH
    /* Dedicated bench diagnostics; production driving emits no regulator log. */
    static uint32_t log_time;
    if(e.sampled_ms-log_time>=40U) {
        log_time=e.sampled_ms;
        tm_printf((UB *)"[WHEEL PI] t=%u target=%d,%d speed=%d,%d pwm_milli=%u,%u\n",
            e.sampled_ms,targets[0],targets[1],pid_speed[0],pid_speed[1],output[0],output[1]);
    }
#endif
    return E_OK;
}
#endif

static BOOL movement_valid(s2_move_command_t command, UINT power_percent)
{
    return ((UINT)command <= (UINT)S2_MOVE_PIVOT_LEFT) &&
           (power_percent <= BOARD_DRIVE_POWER_CAP);
}

static ER motion_publish(const s2_packed_status_t *p_status)
{
    ER er = tk_wai_sem(h_status_lock, 1, 5);

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
    uint8_t command;
    uint8_t power;
    uint8_t state;
    s2_packed_status_t packed;
    ER er;

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
    er = tk_sig_sem(h_status_lock, 1);
    if (er < E_OK)
    {
        return er;
    }

    if (!edr_motor_unpack(packed.motor, &command, &power, &state))
    {
        return E_SYS;
    }

    /* The codec validates both legacy enum ranges before these casts. */
    p_status->command = (s2_move_command_t)command;
    p_status->power_percent = power;
    p_status->state = (s2_motion_state_t)state;
    p_status->last_error = packed.last_error;
    p_status->last_command_ms = packed.last_command_ms;

    return E_OK;
}

static ER motion_fault(s2_packed_status_t *p_status, ER error)
{
    motor_stop();
    if (!edr_motor_pack(UINT8_C(0), UINT8_C(0), S2_MOTION_FAULT,
                        &p_status->motor))
    {
        return E_SYS;
    }

    p_status->last_error = error;
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *)"[S2 FAULT] error=%d; PWM outputs off\n", error);
    return motion_publish(p_status);
}

/* Compute the watchdog wait from packed state, without keeping decoded
 * command fields alive while the motor task waits for another message. */
static ER s2_command_wait(const s2_packed_status_t * const p_status,
                           TMO * const p_wait)
{
    uint8_t command;
    uint8_t power;
    uint8_t state;
    SYSTIM now;
    uint32_t elapsed;
    ER result;

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
    *p_wait = (TMO)(S2_COMMAND_TIMEOUT_MS - elapsed);
    if (!APP_S2_ONLY && *p_wait > 10) *p_wait = 10;

    return E_OK;
}

static void motor_task(INT stacd, void *p_exinf)
{
    s2_packed_status_t current = {0, E_OK, 0};
    ER er = motor_init();

    (void)stacd;
    (void)p_exinf;
#if S2_ENCODER_ENABLED
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
        TMO wait = 10;
        UW epoch = __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE);
        if (epoch != __atomic_load_n(&stopped_epoch, __ATOMIC_RELAXED)) {
            motor_stop();
            current.motor = 0; /* STOP / zero power / STOPPED in EDR v1. */
            current.last_error = E_OK;
            __atomic_store_n(&stopped_epoch, epoch, __ATOMIC_RELEASE);
            er = motion_publish(&current);
            if (er < E_OK) break;
        }
        SYSTIM now;

        er = s2_command_wait(&current, &wait);
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }

#if BOARD_PID_ENABLED
        er=motor_regulate(current.motor);
        if(er<E_OK) {er=motion_fault(&current,er);continue;}
        if(wait>10 || wait==TMO_FEVR)wait=10;
#endif
        er = tk_rcv_mbx(h_movement_mailbox, &p_raw, wait);
        if (er == E_TMOUT) { er = E_OK; continue; }
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
        uint16_t motor = p_msg->motor;
        INT steering=p_msg->steering;
        uint8_t packed_command;
        uint8_t packed_power;
        uint8_t packed_state;
        s2_move_command_t command;
        UINT power;
        uint8_t current_command;
        uint8_t current_power;
        uint8_t current_state;
        UW issued_at = p_msg->issued_at_ms;
        UW message_epoch = p_msg->stop_epoch;

        /* Copy all fields before returning the block to the pool. */
        er = tk_rel_mpf(h_movement_pool, p_msg);
        if (er < E_OK)
        {
            er = motion_fault(&current, er);
            continue;
        }
        if (message_epoch != __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE)) continue;
        if (!edr_motor_unpack(motor, &packed_command, &packed_power,
                              &packed_state) ||
            (packed_state != S2_MOTION_STOPPED))
        {
            er = motion_fault(&current, E_PAR);
            continue;
        }
        /* Unpack validates the command and power before widening. */
        command = (s2_move_command_t)packed_command;
        power = packed_power;
        if(steering< -400 || steering>400 || (command!=S2_MOVE_FORWARD && steering!=0)) {
            er=motion_fault(&current,E_PAR);continue;
        }
        requested_steering=steering;

        if ((command == S2_MOVE_STOP) || (power == 0U))
        {
            command = S2_MOVE_STOP;
            power = 0;
        }
        else
        {
            er = tk_get_otm(&now);
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
        if (!edr_motor_unpack(current.motor, &current_command,
                              &current_power, &current_state))
        {
            er = motion_fault(&current, E_SYS);
            continue;
        }

        if (((UINT)command != current_command) || (power != current_power))
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
        /* Validated command/power values fit exactly in the packed fields. */
        if (!edr_motor_pack((uint8_t)command, (uint8_t)power,
                            (S2_MOVE_STOP == command) ?
                            S2_MOTION_STOPPED : S2_MOTION_MOVING,
                            &current.motor))
        {
            er = motion_fault(&current, E_SYS);
            continue;
        }

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
            /* STOP, zero power and STOPPED are all encoded as zero. */
            motion_status = (s2_packed_status_t){0, E_OK, 0};
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

static ER movement_send(s2_move_command_t command, UINT power_percent, INT steering)
{
    void *p_block = NULL;
    SYSTIM now = {0};
    uint16_t motor;
    ER er = E_OK;
    if(steering< -400 || steering>400 || (command!=S2_MOVE_FORWARD && steering!=0))
        return E_PAR;
#if !BOARD_PID_ENABLED
    if(steering!=0)return E_NOSPT;
#endif

#if !APP_S2_ONLY
    if ((h_controller_task == 0) || (tk_get_tid() != h_controller_task))
    {
        er = E_OACV;
    }
#endif
    if (er >= E_OK && movement_valid(command,power_percent) && (command == S2_MOVE_STOP || power_percent == 0U)) {
#if !APP_S2_ONLY
        return s2_emergency_stop();
#endif
    }
    if ((er >= E_OK) && !movement_valid(command, power_percent))
    {
        er = E_PAR;
    }
    if ((er >= E_OK) && BOARD_MOTORS_DISABLED &&
        (command != S2_MOVE_STOP) && (power_percent != 0U))
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
            command = S2_MOVE_STOP;
            power_percent = 0;
            /* A STOP must still work if the clock service fails. */
            (void)tk_get_otm(&now);
        }
        else
        {
            er = tk_get_otm(&now);
        }
    }
    if (er >= E_OK && command != S2_MOVE_STOP &&
        __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE) !=
        __atomic_load_n(&stopped_epoch, __ATOMIC_ACQUIRE)) er = E_OBJ;
    if (er >= E_OK)
    {
        /* movement_valid() checked both fixed-width value ranges. */
        if (!edr_motor_pack((uint8_t)command, (uint8_t)power_percent,
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
        s2_movement_msg_t *p_msg = (s2_movement_msg_t *)p_block;

        p_msg->hdr.msgque[0] = NULL;
        p_msg->motor = motor;
        p_msg->steering = (int16_t)steering;
        p_msg->issued_at_ms = now.lo;
        p_msg->stop_epoch = __atomic_load_n(&stop_epoch, __ATOMIC_ACQUIRE);
        /* The mailbox header is the first member of the message. */
        er = tk_snd_mbx(h_movement_mailbox, (T_MSG *)p_msg);
        if (er < E_OK)
        {
            CHECK(tk_rel_mpf(h_movement_pool, p_msg), "S2 send cleanup");
        }
    }
    return er;
}

EXPORT ER s2_movement_send(s2_move_command_t command, UINT power_percent)
{
    return movement_send(command,power_percent,0);
}
EXPORT ER s2_follow_send(UINT power_percent, INT steering)
{
    return movement_send(S2_MOVE_FORWARD,power_percent,steering);
}
