#include "app.h"
#if BOARD_WHEEL_CAL_BENCH
#include "wheel_calibration_policy.h"
#include "app_state.h"
#include "barcode_route.h"
#include "subsystem1/control.h"
#include "subsystem2/subsystem2.h"

static void wheel_calibration_task(INT code,void *context) {
    (void)code;(void)context;s2_controller_bind();
    wheel_cal_t bench={0};app_state_t state={0};barcode_route_t map={0};
    uint32_t previous=0;bool have_previous=false;
    for(;;) {
        SYSTIM now;s2_motion_status_t motor={0};s2_encoder_status_t enc={0};
        ER clock=tk_get_otm(&now);
        bool motor_ok=s2_motion_get_status(&motor)==E_OK && motor.state!=S2_MOTION_FAULT;
        bool encoder_ok=s2_encoder_get_status(&enc)==E_OK && enc.valid;
        if(clock<E_OK || tk_get_otm(&now)<E_OK) {
            wheel_cal_stop(&bench);s2_emergency_stop();tk_dly_tsk(10);continue;
        }
        bool ready=motor_ok && encoder_ok && now.lo-enc.sampled_ms<30U;
        if(bench.running && have_previous && now.lo-previous>100U) {
            wheel_cal_stop(&bench);state.vehicle.fault=VC_TRANSPORT;
        }
        previous=now.lo;have_previous=true;
        for(unsigned n=0;n<8;++n) {
            control_message_t c;ER er=s1_command_take(&c);if(er==E_TMOUT)break;
            if(er<E_OK){wheel_cal_stop(&bench);state.vehicle.fault=VC_TRANSPORT;break;}
            if(c.wire[0]==0xC3) {
                unsigned result=now.lo-c.time_ms>=100U?BARCODE_MAP_STALE:
                    barcode_route_configure(&map,c.wire+2,bench.running);
                s1_barcode_map_ack(result,c.wire+2);continue;
            }
            unsigned opcode=c.wire[1];
            if(opcode==0) {
                wheel_cal_stop(&bench);state.vehicle.fault=0;s2_emergency_stop();
                s1_command_ack(c.wire[5],0,0,0,0);break;
            }
            unsigned result=1;
            if(now.lo-c.time_ms>=100U)result=3;
            else if(opcode==2 && !bench.running && motor.state==S2_MOTION_STOPPED) {
                state.vehicle=(vc_state_t){0};result=0;
            } else if(opcode==1 && !state.vehicle.fault) {
                unsigned lease=(unsigned)c.wire[3]|((unsigned)c.wire[4]<<8U);
                if(wheel_cal_arm(&bench,now.lo,c.wire[2],lease,ready)) {
                    result=0;
                    tm_printf((UB *)"[WHEEL CAL ARM] t=%u power=%u lease_ms=%u\n",now.lo,bench.power,lease);
                }
            }
            /* Opcode 3 (unbounded START) is always refused in this image. */
            s1_command_ack(c.wire[5],opcode,result,bench.running?VC_TRACK:VC_DISARMED,state.vehicle.fault);
        }
        bool was_running=bench.running;
        unsigned power=wheel_cal_step(&bench,now.lo,ready);
        if(was_running && !bench.running) {
            if(!ready)state.vehicle.fault=motor_ok?VC_SENSOR:VC_MOTOR;
            tm_printf((UB *)"[WHEEL CAL END] t=%u fault=%u left=%d right=%d\n",now.lo,state.vehicle.fault,
                enc.motor1_forward_counts,enc.motor2_forward_counts);
        }
        if(!power)s2_emergency_stop();
        else {
            ER er=s2_movement_send(S2_MOVE_FORWARD,power);
            if(er<E_OK && er!=E_OBJ){wheel_cal_stop(&bench);power=0;state.vehicle.fault=VC_MOTOR;s2_emergency_stop();}
        }
        state.time_ms=now.lo;++state.sequence;
        state.vehicle.armed=bench.running;state.vehicle.command=power?S2_MOVE_FORWARD:S2_MOVE_STOP;
        state.vehicle.power=(uint8_t)power;
        state.vehicle.phase=bench.running?VC_TRACK:state.vehicle.fault?VC_FAULT:VC_DISARMED;
        (void)app_state_publish(&state);
        tk_dly_tsk(10);
    }
}
ER wheel_calibration_start(void) {
    ID tid=app_start_task((FP)wheel_calibration_task,5,TP_PRC1,"wheel_cal");
    return tid<E_OK?tid:E_OK;
}
#endif
