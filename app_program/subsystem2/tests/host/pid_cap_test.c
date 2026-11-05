#define main existing_suite
#include "subsystem2_host_test.c"
#undef main

int main(void)
{
    speed_pid_t fine={0},whole={0};
    assert(speed_pid_step_milli(&fine,500,510,20,25,30,20,0,0)==24800);
    assert(speed_pid_step(&whole,500,510,20,25,30,20,0,0)==25);
    fine=(speed_pid_t){0};
    assert(speed_pid_step_milli(&fine,500,490,20,25,30,20,0,0)==25200);
    /* The same error used to produce equal 25% outputs in both directions. */
    assert(6250U*24800U/100000U < 6250U*25000U/100000U);
    assert(6250U*25200U/100000U > 6250U*25000U/100000U);
    assert(speed_pid_feedforward(0,74,3,30)==0);
    assert(speed_pid_feedforward(30000,74,3,30)==25);
    assert(speed_pid_feedforward(30000,78,4,30)==27);
    assert(speed_pid_feedforward(20000,74,3,30)==18);
    assert(speed_pid_feedforward(20000,78,4,30)==20);
    assert(speed_pid_feedforward(100000,100,30,30)==30);
    setup();run_task();stopped_epoch=stop_epoch;
    uint16_t word;
    assert(edr_motor_pack(S2_MOVE_FORWARD,30,S2_MOTION_MOVING,&word));
    now_ms=100;assert(motor_regulate(word)==E_OK);
    now_ms=120;assert(motor_regulate(word)==E_OK);
    /* A 50% PID ceiling must still respect the board's lower 30% drive cap. */
    assert(duties[8]==1875U && duties[11]==1875U);
    assert(duties[9]==0 && duties[10]==0);
    motor_stop();assert_off();
    puts("Actual PID: saturation respects 30% drive cap below 50% PID ceiling; STOP PASS");
    return 0;
}
