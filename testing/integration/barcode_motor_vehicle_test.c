/* Run the actual demo controller and actual profile with fake hardware only. */
#define BOARD_BARCODE_DEMO 1
#define TM_MQTT 1
#define S2_ENCODER_ENABLED 1
#define S2_COMMAND_TIMEOUT_MS 150
#include "../../app_program/robot_config.h"
#define main existing_vehicle_suite
#include "buddy4_vehicle_test.c"
#undef main
static unsigned scenario,started_at;
static unsigned junction_stops;
static bool map_sent,stop_sent,reset_sent,event_sent;
static ER demo_command(control_message_t *c) {
    *c=(control_message_t){.time_ms=clock_ms};
    if(iteration==5 && !map_sent) {
        map_sent=true;c->wire[0]=0xC3;memcpy(c->wire+2,"EMQZ",4);return E_OK;
    }
    if(((scenario==1 && iteration==50) || (scenario==3 && iteration==175)) && !stop_sent) {
        stop_sent=true;c->wire[0]=0xC1;return E_OK;
    }
    if(scenario==1 && iteration==60 && !reset_sent) {
        reset_sent=true;c->wire[0]=0xC1;c->wire[1]=2;return E_OK;
    }
    return E_TMOUT;
}
static ER demo_event(s3_event_record_t *e) {
    if((((scenario==0 || scenario==5 || scenario==6 || scenario==7) && iteration==160) ||
        (scenario==8 && iteration==195)) && !event_sent) {
        event_sent=true;*e=(s3_event_record_t){0};
        e->event.kind=S3_EVENT_BARCODE;e->event.symbol=scenario==5?'B':scenario==6?'E':'Q';return E_OK;
    }
    return E_TMOUT;
}
static void demo_publish(const app_state_t *s) {
    if(s->vehicle.armed && !started_at)started_at=clock_ms;
    if(clock_ms<3200)assert(!s->vehicle.armed && !s->vehicle.command);
    if(scenario==0) {
        if(iteration==161)assert(s->barcode==17 && s->vehicle.command==S2_MOVE_FORWARD && !s->navigation);
        if(iteration==165)fixture_mask=3;
        if(iteration==170)assert(s->navigation==S3_NAV_LEFT && s->vehicle.phase==VC_NAV_TURN);
    }
    if(scenario>=4 && scenario<=6) {
        if(iteration==164)junction_stops=stops;
        if(iteration==165)fixture_mask=3;
        if(iteration>=166 && iteration<=174) {
            assert(s->vehicle.armed && s->vehicle.phase==VC_TRACK && !s->vehicle.fault);
            assert(s->vehicle.command==S2_MOVE_FORWARD && s->vehicle.power==30);
            assert(stops==junction_stops); /* No pause at the straight/default action. */
        }
        if(iteration==170)assert(s->navigation==S3_NAV_STRAIGHT);
        if(iteration==175)fixture_mask=0;
        if(iteration==190)fixture_mask=3;
        if(iteration==195) {
            assert(s->navigation==S3_NAV_STRAIGHT && s->vehicle.command==S2_MOVE_FORWARD);
            assert(!s->barcode && !s->vehicle.fault); /* No stale symbol at the next missed scan. */
        }
    }
    if(scenario==7) {
        if(iteration==159){fixture_scanning=true;fixture_mask=3;}
        if(iteration==165){fixture_scanning=false;fixture_capture_result=1;fixture_result_mask=3;}
        if(iteration>=160 && iteration<=180) {
            assert(s->vehicle.phase==VC_TRACK && s->vehicle.command==S2_MOVE_FORWARD);
            assert(s->barcode==17 && !s->navigation && !s->vehicle.fault);
        }
        if(iteration==180)fixture_mask=0;
        if(iteration==190)fixture_mask=3;
        if(iteration==195)assert(s->navigation==S3_NAV_LEFT && s->vehicle.phase==VC_NAV_TURN);
    }
    if(scenario==8) {
        if(iteration==159){fixture_scanning=true;fixture_mask=1;fixture_dark_right=900;fixture_dark_left=100;}
        if(iteration>=160 && iteration<=194) {
            assert(s->vehicle.phase==VC_TRACK && s->vehicle.command==S2_MOVE_FORWARD && !s->vehicle.fault);
        }
        if(iteration==165) {
            if(BOARD_BARCODE_SCAN_STEERING)assert(s->vehicle.steering>0 && s->vehicle.steering<=100);
            else assert(!s->vehicle.steering);
        }
        if(iteration==169){fixture_mask=2;fixture_dark_right=100;fixture_dark_left=900;}
        if(iteration==176 && BOARD_BARCODE_SCAN_STEERING)assert(s->vehicle.steering<0 && s->vehicle.steering>=-100);
        if(iteration==180){fixture_mask=3;fixture_dark_right=fixture_dark_left=900;}
        if(iteration==185)assert(!s->vehicle.steering && !s->navigation);
        if(iteration==190){fixture_mask=0;fixture_dark_right=fixture_dark_left=100;}
        if(iteration==194){fixture_scanning=false;fixture_capture_result=1;fixture_result_mask=0;}
        if(iteration==200)assert(s->barcode==17 && !s->navigation && !s->vehicle.steering);
        if(iteration==210)fixture_mask=3;
        if(iteration==215)assert(s->navigation==S3_NAV_LEFT && s->vehicle.phase==VC_NAV_TURN);
    }
}
int main(void) {
    assert(!BOARD_MOTORS_DISABLED && BOARD_PID_ENABLED==BOARD_BARCODE_WHEEL_PI && BOARD_DRIVE_POWER_CAP==30);
    assert(BOARD_BARCODE_RETRY==1);
    assert(BOARD_BARCODE_DEFAULT_FORWARD==1);
    assert(!APP_S5_ENABLED && !BOARD_TERRAIN_CALIBRATED && !BOARD_BYPASS_VALIDATED);
    for(scenario=0;scenario<9;++scenario) {
        fixture_limit=400;fixture_mask=0;fixture_white=scenario!=2;
        fixture_scanning=false;fixture_capture_result=0;fixture_result_mask=0;
        fixture_dark_left=fixture_dark_right=0;
        fixture_command=demo_command;fixture_event=demo_event;fixture_publish=demo_publish;
        failure_mode=9;iteration=stops=positive_requests=0;clock_ms=100;
        map_sent=stop_sent=reset_sent=event_sent=false;started_at=0;
        if(setjmp(done)==0)vehicle_task(0,NULL);
        if(scenario==1 || scenario==2)assert(!started_at && !positive_requests && !saved.vehicle.armed);
        else assert(started_at==3200 && positive_requests);
        if(scenario==0)assert(saved.barcode==17 && saved.navigation==S3_NAV_LEFT && saved.vehicle.phase==VC_NAV_TURN);
        if(scenario==3)assert(stop_sent && !saved.vehicle.armed && !saved.vehicle.command);
        if(scenario>=4 && scenario<=6)assert(saved.vehicle.armed && !saved.vehicle.fault && saved.vehicle.command==S2_MOVE_FORWARD);
        if(scenario==7 || scenario==8)assert(saved.barcode==17 && saved.navigation==S3_NAV_LEFT && saved.vehicle.phase==VC_NAV_TURN);
    }
    puts("Actual motor-enabled BarcodeDemo: MQTT map -> 3s -> 30% drive, store -> junction -> turn; no PID/terrain/sonar gate; STOP/RESET/no-white no restart PASS");
    return 0;
}
