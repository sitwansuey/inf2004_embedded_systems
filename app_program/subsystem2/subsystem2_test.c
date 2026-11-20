/*
 * subsystem2_test.c
 * Buddy 2 -- On-target motor test. Student ID: 2500613.
 * Build with S2_ONLY=1 TEST=1 (APP_S2_ONLY=1, APP_TEST=1); wheels must be raised.
 * Each check prints one "[TEST] S2-Uxx PASS|FAIL" line (../test_report.h).
 * The IDs are defined in testing/subsystem2/TEST_LOG.md.
 */
#include "subsystem2.h"

#ifdef S2_HOST_APP_H
/* tests/host/run.py only compiles this file, against its own app.h, which
 * has no tm/tmonitor.h for ../test_report.h. Same macros, same line format. */
#define TEST_INIT() INT test_pass_ = 0, test_fail_ = 0
#define TEST_CHECK(id, cond, desc)                                      \
    do {                                                                \
        if (cond) {                                                     \
            test_pass_++;                                               \
            tm_printf((UB *)"[TEST] %s PASS  %s\n", (id), (desc));      \
        } else {                                                        \
            test_fail_++;                                               \
            tm_printf((UB *)"[TEST] %s FAIL  %s   (%s:%d)\n",          \
                      (id), (desc), __FILE__, __LINE__);                \
        }                                                               \
    } while (0)
#define TEST_SUMMARY(name)                                              \
    tm_printf((UB *)"[TEST] %s SUMMARY %d passed, %d failed\n",        \
              (name), test_pass_, test_fail_)
#else
#include "../test_report.h"
#endif

#if APP_TEST
#define S2_TEST_DURATION_MS 2000U
#define S2_TEST_REFRESH_MS 250U

#if S2_COMMAND_TIMEOUT_MS <= S2_TEST_REFRESH_MS
#error "The motor test needs a command timeout greater than 250 ms"
#endif

/* One move: 30% for 2000 ms, renewed every 250 ms, then STOP.
 * Motor 1 is the left wheel, motor 2 the right wheel. */
typedef struct
{
    s2_move_command_t command;
    const char *p_name;
    const char *p_move_id;  /* command, renewals and STOP */
    const char *p_move_desc;
    const char *p_wheel_id; /* encoder pulses on each driven wheel */
    const char *p_wheel_desc;
} s2_test_move_t;

static const s2_test_move_t s2_test_moves[] =
{
    {S2_MOVE_FORWARD, "straight",
     "S2-U01", "straight 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U02", "straight: left and right encoders count pulses"},
    {S2_MOVE_PIVOT_RIGHT, "right pivot",
     "S2-U03", "right pivot 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U04", "right pivot: left (driven) encoder counts pulses"},
    {S2_MOVE_PIVOT_LEFT, "left pivot",
     "S2-U05", "left pivot 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U06", "left pivot: right (driven) encoder counts pulses"},
    {S2_MOVE_RIGHT, "rotate clockwise",
     "S2-U07", "rotate clockwise 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U08", "rotate clockwise: left and right encoders count pulses"},
    {S2_MOVE_LEFT, "rotate counterclockwise",
     "S2-U09", "rotate counterclockwise 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U10", "rotate counterclockwise: left and right encoders count pulses"},
    {S2_MOVE_REVERSE, "reverse",
     "S2-U11", "reverse 30% 2 s: sends and renewals accepted, STOP reads STOPPED",
     "S2-U12", "reverse: left and right encoders count pulses"},
};

static const char s2_test_watchdog_desc[] =
    "command watchdog: no renewal for timeout + 100 ms gives FAULT, E_TMOUT, STOP, 0%";

static ER s2_test_stop(void)
{
    s2_motion_status_t status;
    ER er = s2_movement_send(S2_MOVE_STOP, 0);

    if (er >= E_OK)
    {
        tk_dly_tsk(10);
        er = s2_motion_get_status(&status);
        if ((er >= E_OK) && (status.state != S2_MOTION_STOPPED))
        {
            er = E_SYS;
        }
    }
    CHECK(er, "S2 test stop");
    return er;
}

/* Runs one move and prints the encoder counts every 250 ms, then sends
 * STOP. Returns the first error. *p_have_encoder is FALSE when the
 * encoders are disabled (E_NOSPT). */
static ER s2_test_run_move(const s2_test_move_t *p_move,
                           s2_encoder_status_t *p_start,
                           s2_encoder_status_t *p_last,
                           BOOL *p_have_encoder)
{
    s2_encoder_status_t last;
    SYSTIM begin;
    SYSTIM previous_time;
    ER er = s2_encoder_get_status(p_start);

    *p_have_encoder = (er == E_OK) && (p_start->valid != 0U);
    if ((er < E_OK) && (er != E_NOSPT))
    {
        return er;
    }
    if ((er == E_OK) && !*p_have_encoder)
    {
        return E_OBJ;
    }
    last = *p_start;
    *p_last = last;
    er = tk_get_otm(&begin);
    if (er < E_OK)
    {
        return er;
    }
    previous_time = begin;
    /* tm_printf takes an unsigned-byte format string. */
    tm_printf((UB *)"[S2 TEST] START %s: 30%% for 2000 ms\n",
              p_move->p_name);
    er = s2_movement_send(p_move->command, 30);
    while (er >= E_OK)
    {
        SYSTIM now;

        er = tk_get_otm(&now);
        if ((er < E_OK) ||
            ((now.lo - begin.lo) >= S2_TEST_DURATION_MS))
        {
            break;
        }
        UW remaining = S2_TEST_DURATION_MS - (now.lo - begin.lo);

        er = tk_dly_tsk((remaining < S2_TEST_REFRESH_MS) ?
                        remaining : S2_TEST_REFRESH_MS);
        if (er >= E_OK)
        {
            er = tk_get_otm(&now);
        }
        if (er < E_OK)
        {
            break;
        }
        if ((now.lo - begin.lo) < S2_TEST_DURATION_MS)
        {
            er = s2_movement_send(p_move->command, 30);
            if (er < E_OK)
            {
                break;
            }
        }

        if (*p_have_encoder)
        {
            s2_encoder_status_t current;

            er = s2_encoder_get_status(&current);
            if (er < E_OK)
            {
                break;
            }
            UW positive = current.positive_pulses - last.positive_pulses;
            UW negative = current.negative_pulses - last.negative_pulses;
            UW motor1_positive = current.motor1_positive_pulses -
                                 last.motor1_positive_pulses;
            UW motor1_negative = current.motor1_negative_pulses -
                                 last.motor1_negative_pulses;
            UW interval = now.lo - previous_time.lo;

            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *)"[S2 ENCODER] %s elapsed=%u ms "
                      "motor1 positive=%u negative=%u forward_delta=%d "
                      "A=%u B=%u A_edges=%u B_edges=%u; "
                      "motor2 positive=%u negative=%u forward_delta=%d "
                      "rate=%u pulses/s A=%u B=%u A_edges=%u B_edges=%u\n",
                      p_move->p_name, now.lo - begin.lo,
                      motor1_positive, motor1_negative,
                      (INT)(motor1_negative - motor1_positive),
                      current.motor1_a_level, current.motor1_b_level,
                      current.motor1_a_edges - last.motor1_a_edges,
                      current.motor1_b_edges - last.motor1_b_edges,
                      positive, negative, (INT)(positive - negative),
                      interval != 0U ?
                      (positive + negative) * 1000U / interval : 0U,
                      current.a_level, current.b_level,
                      current.a_edges - last.a_edges,
                      current.b_edges - last.b_edges);
            last = current;
            *p_last = last;
            previous_time = now;
        }
    }

    ER stop_er = s2_test_stop();

    return (er >= E_OK) ? stop_er : er;
}

EXPORT void subsystem2_test(void)
{
    ER er = E_OK;
    ER stop_er;

    TEST_INIT();
    tk_dly_tsk(2000);
    for (UINT i = 0; i < sizeof(s2_test_moves) / sizeof(s2_test_moves[0]); ++i)
    {
        const s2_test_move_t *p_move = &s2_test_moves[i];
        s2_encoder_status_t start = {0};
        s2_encoder_status_t last = {0};
        BOOL have_encoder = FALSE;
        BOOL wheels_ok = FALSE;

        if (er >= E_OK)
        {
            er = s2_test_run_move(p_move, &start, &last, &have_encoder);
        }
        else
        {
            /* The sequence stops at the first error, as before. */
            tm_printf((UB *)"[S2 TEST] %s not run: an earlier step failed\n",
                      p_move->p_name);
        }
        if (er >= E_OK)
        {
            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *)"[S2 TEST] END %s; PWM outputs off\n",
                      p_move->p_name);
            if (have_encoder)
            {
                UW pulses = last.positive_pulses - start.positive_pulses +
                            last.negative_pulses - start.negative_pulses;

                UW motor1_pulses =
                    last.motor1_positive_pulses - start.motor1_positive_pulses +
                    last.motor1_negative_pulses - start.motor1_negative_pulses;

                /* A pivot drives one wheel only. */
                BOOL left_ok = (p_move->command == S2_MOVE_PIVOT_LEFT) ||
                               (motor1_pulses != 0U);
                BOOL right_ok = (p_move->command == S2_MOVE_PIVOT_RIGHT) ||
                                (pulses != 0U);

                if (!left_ok)
                {
                    tm_printf((UB *)"[S2 TEST] no motor 1 pulses; "
                              "check mounting and board_pins.h wiring\n");
                }
                if (!right_ok)
                {
                    /* tm_printf takes an unsigned-byte format string. */
                    tm_printf((UB *)"[S2 TEST] no motor 2 pulses; "
                              "check mounting and board_pins.h wiring\n");
                }
                wheels_ok = left_ok && right_ok;
            }
        }
        TEST_CHECK(p_move->p_move_id, er >= E_OK, p_move->p_move_desc);
        if (S2_ENCODER_ENABLED)
        {
            TEST_CHECK(p_move->p_wheel_id, (er >= E_OK) && wheels_ok,
                       p_move->p_wheel_desc);
        }
        else
        {
            tm_printf((UB *)"[TEST] %s SKIP  encoders disabled "
                      "(S2_ENCODER_ENABLED=0)\n", p_move->p_wheel_id);
        }
        if (er >= E_OK)
        {
            tk_dly_tsk(1000);
        }
    }

    if (er >= E_OK)
    {
        s2_motion_status_t status;

        /* Deliberately withhold renewals to check the command watchdog. */
        er = s2_movement_send(S2_MOVE_FORWARD, 20);
        if (er >= E_OK)
        {
            tk_dly_tsk(S2_COMMAND_TIMEOUT_MS + 100U);
            er = s2_motion_get_status(&status);
            if ((er >= E_OK) &&
                ((status.state != S2_MOTION_FAULT) ||
                 (status.last_error != E_TMOUT) ||
                 (status.command != S2_MOVE_STOP) ||
                 (status.power_percent != 0U)))
            {
                er = E_SYS;
            }
        }
        if (er >= E_OK)
        {
            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *)"[S2 TEST] command watchdog passed\n");
        }
    }
    else
    {
        tm_printf((UB *)"[S2 TEST] command watchdog not run: "
                  "an earlier step failed\n");
    }
    TEST_CHECK("S2-U13", er >= E_OK, s2_test_watchdog_desc);
    CHECK(er, "S2 test sequence");

    stop_er = s2_test_stop();
    if (stop_er >= E_OK)
    {
        /* tm_printf takes an unsigned-byte format string. */
        tm_printf((UB *)"[S2 TEST] sequence finished; PWM outputs off\n");
    }
    TEST_CHECK("S2-U14", stop_er >= E_OK,
               "final STOP (after the watchdog FAULT) reads STOPPED");
    TEST_SUMMARY("subsystem2");
}
#endif
