/*
 * subsystem2_test.c
 * Buddy 2 -- On-target motor test. Student ID: 2500613.
 * Build with APP_TEST=1 and APP_S2_ONLY=1; wheels must be raised.
 */
#include "subsystem2.h"

#if APP_TEST
#define S2_TEST_DURATION_MS 2000U
#define S2_TEST_REFRESH_MS 250U

#if S2_COMMAND_TIMEOUT_MS <= S2_TEST_REFRESH_MS
#error "The motor test needs a command timeout greater than 250 ms"
#endif

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

EXPORT void subsystem2_test(void)
{
    const s2_move_command_t commands[] =
    {
        S2_MOVE_FORWARD, S2_MOVE_PIVOT_RIGHT, S2_MOVE_PIVOT_LEFT,
        S2_MOVE_RIGHT, S2_MOVE_REVERSE
    };
    const char *p_names[] =
    {
        "straight", "right pivot", "left pivot",
        "rotate clockwise", "reverse"
    };
    ER er = E_OK;

    tk_dly_tsk(2000);
    for (UINT i = 0; (i < 5U) && (er >= E_OK); ++i)
    {
        s2_encoder_status_t start = {0};
        s2_encoder_status_t last = {0};
        SYSTIM begin;
        SYSTIM previous_time;
        BOOL have_encoder;

        er = s2_encoder_get_status(&start);
        have_encoder = (er == E_OK);
        if ((er < E_OK) && (er != E_NOSPT))
        {
            break;
        }
        last = start;
        er = tk_get_tim(&begin);
        if (er < E_OK)
        {
            break;
        }
        previous_time = begin;
        /* tm_printf takes an unsigned-byte format string. */
        tm_printf((UB *)"[S2 TEST] START %s: 30%% for 2000 ms\n",
                  p_names[i]);
        er = s2_movement_send(commands[i], 30);
        while (er >= E_OK)
        {
            SYSTIM now;

            er = tk_get_tim(&now);
            if ((er < E_OK) ||
                ((now.lo - begin.lo) >= S2_TEST_DURATION_MS))
            {
                break;
            }
            UW remaining = S2_TEST_DURATION_MS - (now.lo - begin.lo);

            tk_dly_tsk((remaining < S2_TEST_REFRESH_MS) ?
                       remaining : S2_TEST_REFRESH_MS);
            er = tk_get_tim(&now);
            if (er < E_OK)
            {
                break;
            }
            if ((now.lo - begin.lo) < S2_TEST_DURATION_MS)
            {
                er = s2_movement_send(commands[i], 30);
                if (er < E_OK)
                {
                    break;
                }
            }

            if (have_encoder)
            {
                s2_encoder_status_t current;

                er = s2_encoder_get_status(&current);
                if (er < E_OK)
                {
                    break;
                }
                UW positive = current.positive_pulses - last.positive_pulses;
                UW negative = current.negative_pulses - last.negative_pulses;
                UW interval = now.lo - previous_time.lo;

                /* tm_printf takes an unsigned-byte format string. */
                tm_printf((UB *)"[S2 ENCODER] %s elapsed=%u ms "
                          "positive=%u negative=%u rate=%u pulses/s "
                          "A=%u B=%u A_edges=%u B_edges=%u\n",
                          p_names[i], now.lo - begin.lo, positive, negative,
                          interval != 0U ?
                          (positive + negative) * 1000U / interval : 0U,
                          current.a_level, current.b_level,
                          current.a_edges - last.a_edges,
                          current.b_edges - last.b_edges);
                last = current;
                previous_time = now;
            }
        }

        ER stop_er = s2_test_stop();

        if (er >= E_OK)
        {
            er = stop_er;
        }
        if (er >= E_OK)
        {
            /* tm_printf takes an unsigned-byte format string. */
            tm_printf((UB *)"[S2 TEST] END %s; PWM outputs off\n",
                      p_names[i]);
            if (have_encoder)
            {
                UW pulses = last.positive_pulses - start.positive_pulses +
                            last.negative_pulses - start.negative_pulses;

                if ((i != 1U) && (pulses == 0U))
                {
                    /* tm_printf takes an unsigned-byte format string. */
                    tm_printf((UB *)"[S2 TEST] no motor 2 pulses; "
                              "check mounting and board_pins.h wiring\n");
                }
            }
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
    CHECK(er, "S2 test sequence");
    if (s2_test_stop() >= E_OK)
    {
        /* tm_printf takes an unsigned-byte format string. */
        tm_printf((UB *)"[S2 TEST] sequence finished; PWM outputs off\n");
    }
}
#endif
