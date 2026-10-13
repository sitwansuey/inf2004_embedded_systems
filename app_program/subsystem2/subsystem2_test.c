/* On-target motor/encoder sequence: TEST=1 S2_ONLY=1. */
#include "subsystem2.h"

#if APP_TEST
EXPORT void subsystem2_test(void)
{
    const s2_move_command_t commands[] = {
        S2_MOVE_FORWARD, S2_MOVE_PIVOT_RIGHT, S2_MOVE_PIVOT_LEFT,
        S2_MOVE_RIGHT, S2_MOVE_REVERSE
    };
    const char *names[] = { "straight", "right pivot", "left pivot",
                            "rotate clockwise", "reverse" };
    tk_dly_tsk(2000);
    for (UINT i = 0; i < 5; ++i) {
        s2_encoder_status_t start, last;
        SYSTIM begin, previous_time;
        s2_encoder_get_status(&start);
        last = start;
        tk_get_tim(&begin);
        previous_time = begin;
        tm_printf((UB *)"[S2 TEST] START %s: 30%% for 2000 ms\n", names[i]);
        ER er = s2_movement_send(commands[i], 30);
        if (er < E_OK) {
            CHECK(er, "test movement");
            CHECK(s2_movement_send(S2_MOVE_STOP, 0), "test emergency stop");
            return;
        }
        for (;;) {
            SYSTIM now;
            tk_get_tim(&now);
            UW elapsed = now.lo - begin.lo;
            if (elapsed >= 2000U) break;
            UW wait = 2000U - elapsed;
            tk_dly_tsk(wait < 250U ? wait : 250U);
            s2_encoder_status_t current;
            s2_encoder_get_status(&current);
            tk_get_tim(&now);
            UW positive = current.positive_pulses - last.positive_pulses;
            UW negative = current.negative_pulses - last.negative_pulses;
            UW interval = now.lo - previous_time.lo;
            tm_printf((UB *)"[S2 ENCODER] %s elapsed=%u ms positive=%u negative=%u "
                      "net=%d pulses rate=%u pulses/s A=%u B=%u A_edges=%u B_edges=%u\n", names[i], now.lo - begin.lo,
                      positive, negative, (W)(positive - negative),
                      interval ? (positive + negative) * 1000U / interval : 0,
                      current.a_level, current.b_level,
                      current.a_edges - last.a_edges, current.b_edges - last.b_edges);
            last = current;
            previous_time = now;
        }
        er = s2_movement_send(S2_MOVE_STOP, 0);
        CHECK(er, "test stop");
        if (er < E_OK) return;
        UW positive = last.positive_pulses - start.positive_pulses;
        UW negative = last.negative_pulses - start.negative_pulses;
        tm_printf((UB *)"[S2 TEST] END %s: positive=%u negative=%u net=%d "
                  "total=%u pulses; stopping for 1000 ms\n",
                  names[i], positive, negative, (W)(positive - negative), positive + negative);
        if (i != 1 && positive + negative == 0) {
            tm_printf((UB *)"[S2 TEST] no motor 2 pulses: check encoder power, common "
                      "ground, GP4/GP5 signal wiring, and wheel rotation\n");
        }
        tk_dly_tsk(1000);
    }
    tm_printf((UB *)"[S2 TEST] sequence finished; motors stopped\n");
}
#endif
