/* Synthetic pulse/ADC traces through the production decoder, core and route.
 * These checks establish software behaviour, not mounted-sensor accuracy. */
#include "../../app_program/barcode_route.h"
#include "../../app_program/vehicle_policy.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

/* Independent narrow/wide fixtures, black first, from the Code 39 alphabet. */
static const char *widths[26]={
    "wnnnnwnnw","nnwnnwnnw","wnwnnwnnn","nnnnwwnnw","wnnnwwnnn","nnwnwwnnn",
    "nnnnnwwnw","wnnnnwwnn","nnwnnwwnn","nnnnwwwnn","wnnnnnnww","nnwnnnnww",
    "wnwnnnnwn","nnnnwnnww","wnnnwnnwn","nnwnwnnwn","nnnnnnwww","wnnnnnwwn",
    "nnwnnnwwn","nnnnwnwwn","wwnnnnnnw","nwwnnnnnw","wwwnnnnnn","nwnnwnnnw",
    "wwnnwnnnn","nwwnwnnnn"
};
static void make_runs(unsigned letter,unsigned unit,unsigned reverse,uint32_t runs[29]) {
    uint32_t forward[29];
    for(unsigned c=0;c<3;++c) {
        const char *w=c==1?widths[letter]:"nwnnwnwnn";
        for(unsigned n=0;n<9;++n)forward[c*10+n]=(w[n]=='w'?3U:1U)*unit;
        if(c<2)forward[c*10+9]=unit;
    }
    for(unsigned n=0;n<29;++n)runs[n]=forward[reverse?28-n:n];
}
static bool white_layout;
static unsigned feed(s3_core_t *core,barcode_route_t *route,uint32_t *t,bool black) {
    s3_sample_t sample={{white_layout?0U:4095U,0,black?4095U:0U},7,*t};s3_output_t output;
    *t+=10;sample.time_ms=*t;s3_core_step(core,&sample,&output);
    assert(output.steering_allowed && !output.awaiting_controller && !output.error);
    if(output.event.kind==S3_EVENT_BARCODE) {
        assert(!output.event.command && barcode_route_store(route,output.event.symbol));return 1;
    }
    assert(output.event.kind==S3_EVENT_NONE);return 0;
}
static void full_path(unsigned letter,unsigned reverse,bool white_center) {
    s3_config_t config;s3_config_defaults(&config);config.line.line_mask=3;
    config.barcode_sensor=2;config.barcode.min_run_ms=20;
    config.barcode_retry=true;
    config.line.center_white=white_center;white_layout=white_center;
    s3_core_t core;assert(s3_core_init(&core,&config));
    assert(s3_core_control(&core,S3_MODE_TRACK,true));
    barcode_route_t route={0};uint8_t map[4];
    for(unsigned n=0;n<4;++n)map[n]=(uint8_t)('A'+(letter+n)%26);
    assert(barcode_route_configure(&route,map,false)==BARCODE_MAP_OK);
    uint32_t t=0,runs[29];unsigned events=0;make_runs(letter,40,reverse,runs);
    for(unsigned n=0;n<30;++n)events+=feed(&core,&route,&t,false);
    /* An invalid 10 ms pulse does not stop tracking or become a route. */
    events+=feed(&core,&route,&t,true);
    events+=feed(&core,&route,&t,false);
    for(unsigned n=0;n<30;++n)events+=feed(&core,&route,&t,false);
    assert(!events && !route.pending && !route.fault);
    assert(core.barcode_diagnostics.status==S3_BARCODE_NOISE &&
           core.barcode_diagnostics.rejected_count==1);
    events+=feed(&core,&route,&t,true);
    for(unsigned n=0;n<29;++n)for(uint32_t elapsed=10;elapsed<=runs[n];elapsed+=10)
        events+=feed(&core,&route,&t,elapsed==runs[n]?(n%2U)!=0U:(n%2U)==0U);
    for(unsigned n=0;n<70;++n)events+=feed(&core,&route,&t,false);
    assert(events==1 && route.pending==(char)('A'+letter));
    assert(core.barcode_diagnostics.decoded_count==1 && core.barcode_diagnostics.result_id==2);
    assert(core.barcode_diagnostics.result_status==S3_BARCODE_OK &&
           core.barcode_diagnostics.symbol==(char)('A'+letter) &&
           core.barcode_diagnostics.reverse==(reverse!=0) &&
           core.barcode_diagnostics.result_runs==29 && core.barcode_diagnostics.max_gap_ms==10);
    assert(!memcmp(core.barcode_diagnostics.runs,runs,sizeof(runs)));
    uint32_t rt=t;
    for(unsigned n=0;n<100;++n) {
        rt+=10;assert(barcode_route_step(&route,t,true,n%2,false)==S3_NAV_NONE);
        assert(route.pending==(char)('A'+letter));
    }
    /* A short black patch is not a confirmed cross. */
    assert(!barcode_route_step(&route,rt+=10,true,3,true));
    assert(!barcode_route_step(&route,rt+=10,true,0,true));
    for(unsigned n=0;n<3;++n)assert(!barcode_route_step(&route,rt+=10,true,3,true));
    assert(barcode_route_step(&route,rt+=10,true,3,true)==S3_NAV_STRAIGHT);
    assert(!route.pending && !route.fault);
    for(unsigned n=0;n<20;++n)assert(!barcode_route_step(&route,rt+=10,true,3,true));
    /* The next marker is accepted after controller rearm, without a reboot. */
    assert(s3_core_control(&core,S3_MODE_TRACK,true));
    for(unsigned n=0;n<20;++n) {
        s3_sample_t junction={{4095,4095,0},7,t+=10};s3_output_t out;
        s3_core_step(&core,&junction,&out);
        assert(out.steering_allowed && out.event.kind==S3_EVENT_NONE);
        assert(core.barcode.state==S3_CAPTURE_HOLD && core.barcode.rearm_requested);
        assert(!core.barcode.have_sample);
    }
    for(unsigned n=0;n<30;++n)events+=feed(&core,&route,&t,false);
    events+=feed(&core,&route,&t,true);
    for(unsigned n=0;n<29;++n)for(uint32_t elapsed=10;elapsed<=runs[n];elapsed+=10)
        events+=feed(&core,&route,&t,elapsed==runs[n]?(n%2U)!=0U:(n%2U)==0U);
    for(unsigned n=0;n<70;++n)events+=feed(&core,&route,&t,false);
    assert(events==2 && route.pending==(char)('A'+letter));
}
static void mapping_and_junction(void) {
    const char *json=" { \"right\":\"Z\", \"forward\":\"E\", \"left\":\"Q\", \"uturn\":\"M\" } ";
    uint8_t map[4];assert(barcode_map_parse((const uint8_t *)json,(unsigned)strlen(json),map));
    assert(!memcmp(map,"EMQZ",4));
    const char *bad[]={"{}","[]","null","{\"forward\":\"E\",\"uturn\":\"M\",\"left\":\"Q\",\"right\":\"E\"}",
        "{\"forward\":\"E\",\"uturn\":\"M\",\"left\":\"Q\",\"right\":\"z\"}",
        "{\"forward\":\"E\",\"uturn\":\"M\",\"left\":\"Q\",\"forward\":\"Z\"}",
        "{\"forward\":\"EE\",\"uturn\":\"M\",\"left\":\"Q\",\"right\":\"Z\"}"};
    for(unsigned n=0;n<sizeof(bad)/sizeof(bad[0]);++n)
        assert(!barcode_map_parse((const uint8_t *)bad[n],(unsigned)strlen(bad[n]),map));
    for(unsigned cut=0;cut<strlen(json)-2;++cut)
        assert(!barcode_map_parse((const uint8_t *)json,cut,map));
    const unsigned actions[4]={3,4,1,2};
    for(unsigned a=0;a<4;++a) {
        barcode_route_t r={0};assert(barcode_route_configure(&r,(const uint8_t *)"EMQZ",false)==0);
        assert(barcode_route_configure(&r,(const uint8_t *)"ABCD",true)==BARCODE_MAP_BUSY);
        assert(barcode_route_store(&r,"EMQZ"[a]));assert(!barcode_route_store(&r,'A'));
        uint32_t t=UINT32_MAX-20;
        for(unsigned n=0;n<3;++n)assert(!barcode_route_step(&r,t+=10,true,3,true));
        unsigned nav=barcode_route_step(&r,t+=10,true,3,true);assert(nav==actions[a]);
        vc_state_t vehicle={0};assert(vc_start(&vehicle,0,30));
        vc_input_t in={.now_ms=10,.motor_ok=1,.sensor_fresh=1,.terrain_valid=1,.odom_valid=1,
            .transport_ok=1,.line_fresh=1,.steering=1,.front_clear=1,.navigation_event=1,.nav=(uint8_t)nav};
        vc_step(&vehicle,&in);
        if(nav==3)assert(vehicle.navigation_ack && vehicle.phase==VC_TRACK);
        else assert(vehicle.phase==VC_NAV_TURN && vehicle.turn_deg==(nav==1?90:nav==2?-90:180));
        barcode_route_clear(&r);assert(r.configured && !r.pending);
    }
    barcode_route_t r={0};assert(barcode_route_configure(&r,(const uint8_t *)"EMQZ",false)==0);
    assert(barcode_route_store(&r,'B'));
    for(unsigned n=0;n<5;++n)(void)barcode_route_step(&r,n*10,true,3,true);
    assert(r.fault && r.pending=='B');
}
static unsigned confirmed_junction(barcode_route_t *r,uint32_t *t,unsigned mask) {
    unsigned action=0;
    for(unsigned n=0;n<4;++n)action=barcode_route_step(r,*t+=10,true,mask,true);
    return action;
}
static void default_forward_course(void) {
    barcode_route_t route={.default_forward=true};uint32_t t=UINT32_MAX-20U;
    assert(barcode_route_configure(&route,(const uint8_t *)"ABCD",false)==BARCODE_MAP_OK);
    assert(route.default_forward);
    /* Consecutive missed scans pass straight; one action per confirmed cross. */
    for(unsigned junction=0;junction<3;++junction) {
        assert(confirmed_junction(&route,&t,3)==S3_NAV_STRAIGHT);
        assert(!route.pending && !route.fault);
        assert(!confirmed_junction(&route,&t,3));
        assert(!confirmed_junction(&route,&t,0));
    }
    assert(barcode_route_store(&route,'Z'));
    assert(confirmed_junction(&route,&t,3)==S3_NAV_STRAIGHT);
    assert(!route.pending && !route.fault); /* Decoded but unmapped also defaults. */
    assert(!confirmed_junction(&route,&t,0));
    const unsigned actions[]={S3_NAV_STRAIGHT,S3_NAV_UTURN,S3_NAV_LEFT,S3_NAV_RIGHT};
    for(unsigned n=0;n<4;++n) {
        assert(barcode_route_store(&route,(char)('A'+n)));
        assert(confirmed_junction(&route,&t,3)==actions[n]);
        assert(!route.pending && !route.fault);
        assert(!confirmed_junction(&route,&t,0));
    }
    /* Stale/paused observations cannot generate even a default instruction. */
    assert(!barcode_route_step(&route,t+=10,false,3,true));
    assert(!barcode_route_step(&route,t+=10,true,3,false));
    assert(!barcode_route_step(&route,t+=10,true,3,true));
    assert(!barcode_route_step(&route,t+=40,true,3,true));
    assert(!barcode_route_step(&route,t+=10,true,3,true));
    assert(!barcode_route_step(&route,t+=10,true,3,true));
    assert(barcode_route_step(&route,t+=10,true,3,true)==S3_NAV_STRAIGHT);
    barcode_route_clear(&route);assert(route.default_forward && route.configured && !route.pending);
    vc_state_t vehicle={0};assert(vc_start(&vehicle,0,30));
    vc_input_t in={.now_ms=10,.barcode_demo=1,.motor_ok=1,.odom_valid=1,.transport_ok=1,
        .line_fresh=1,.steering=1,.navigation_event=1,.nav=S3_NAV_STRAIGHT};
    vc_step(&vehicle,&in);
    assert(vehicle.phase==VC_TRACK && vehicle.navigation_ack && vehicle.command==1 && vehicle.power==30);
    in.now_ms+=10;in.safety_event=1;vc_step(&vehicle,&in);
    assert(!vehicle.armed && !vehicle.command);
}
static unsigned capture_sample(s3_core_t *core,barcode_route_t *route,uint32_t *time,
                               bool black,unsigned mask,unsigned *events) {
    s3_sample_t sample={{mask&1U?4095U:0U,mask&2U?4095U:0U,black?4095U:0U},7,*time+=10};
    s3_output_t out;s3_core_step(core,&sample,&out);
    assert(out.steering_allowed && !out.awaiting_controller && !out.error);
    if(out.event.kind==S3_EVENT_BARCODE) {
        ++*events;assert(barcode_route_store(route,out.event.symbol));
    } else assert(out.event.kind==S3_EVENT_NONE);
    barcode_route_observe_capture(route,true,mask,out.barcode_scanning,
        core->barcode_diagnostics.result_id,core->barcode_diagnostics.result_line_mask);
    return barcode_route_step(route,*time,true,mask,!out.barcode_scanning);
}
static void junction_during_capture(unsigned letter,unsigned reverse,unsigned overlap_run,unsigned prefix_pairs) {
    s3_config_t config;s3_config_defaults(&config);config.line.line_mask=3;
    config.line.center_white=true;config.barcode_sensor=2;config.barcode_retry=true;
    config.barcode.min_run_ms=0;
    s3_core_t core;assert(s3_core_init(&core,&config));
    assert(s3_core_control(&core,S3_MODE_TRACK,true));
    barcode_route_t route={.default_forward=true};uint8_t map[4];
    unsigned selected=(letter+reverse)%4U;
    for(unsigned n=0;n<4;++n)map[n]=(uint8_t)('A'+(letter+n+26U-selected)%26U);
    assert(barcode_route_configure(&route,map,false)==0);
    uint32_t t=0,runs[29];unsigned events=0;make_runs(letter,40,reverse,runs);
    for(unsigned n=0;n<30;++n)assert(!capture_sample(&core,&route,&t,false,0,&events));
    assert(!capture_sample(&core,&route,&t,true,0,&events));
    for(unsigned pair=0;pair<prefix_pairs;++pair)for(unsigned n=0;n<2;++n) {
        uint32_t duration=n?40U:20U;
        for(uint32_t elapsed=10;elapsed<=duration;elapsed+=10)
            assert(!capture_sample(&core,&route,&t,
                elapsed==duration?(n&1U)!=0U:(n&1U)==0U,0,&events));
    }
    for(unsigned n=0;n<29;++n)for(uint32_t elapsed=10;elapsed<=runs[n];elapsed+=10)
        assert(!capture_sample(&core,&route,&t,elapsed==runs[n]?(n%2U)!=0U:(n%2U)==0U,
                               n>=overlap_run?3U:0U,&events));
    /* Keep the same black patch under the line sensors through completion. */
    for(unsigned n=0;n<70;++n)assert(!capture_sample(&core,&route,&t,false,3,&events));
    assert(events==1 && route.pending==(char)('A'+letter) && route.junction_latched);
    assert(core.barcode_diagnostics.result_status==S3_BARCODE_OK && !core.barcode_diagnostics.rejected_count);
    assert(core.barcode_diagnostics.result_line_mask==3 && core.barcode_diagnostics.result_runs==29);
    assert(!memcmp(core.barcode_diagnostics.runs,runs,sizeof(runs)));
    assert(core.barcode.resync_pairs==prefix_pairs);
    for(unsigned n=0;n<8;++n)assert(!capture_sample(&core,&route,&t,false,0,&events));
    assert(!route.junction_latched && route.pending==(char)('A'+letter));
    const unsigned actions[]={S3_NAV_STRAIGHT,S3_NAV_UTURN,S3_NAV_LEFT,S3_NAV_RIGHT};
    unsigned delivered=0;
    for(unsigned n=0;n<8;++n) {
        unsigned nav=capture_sample(&core,&route,&t,false,3,&events);
        if(nav){assert(!delivered && nav==actions[selected]);delivered=nav;}
    }
    assert(delivered && !route.pending && events==1);
}
static void retained_capture_junction(void) {
    barcode_route_t route={.default_forward=true};uint32_t t=0;
    assert(!barcode_route_configure(&route,(const uint8_t *)"ABCD",false));
    assert(barcode_route_store(&route,'C'));
    /* Controller can miss the terminal sampling tick; retained result still
     * prevents its same black patch from becoming a navigation junction. */
    barcode_route_observe_capture(&route,true,3,false,99,3);
    for(unsigned n=0;n<6;++n)assert(!barcode_route_step(&route,t+=10,true,3,true));
    assert(route.pending=='C');
    for(unsigned n=0;n<6;++n) {
        barcode_route_observe_capture(&route,true,0,false,99,3);
        assert(!barcode_route_step(&route,t+=10,true,0,true));
    }
    assert(confirmed_junction(&route,&t,3)==S3_NAV_LEFT);
    barcode_route_clear(&route);
    barcode_route_observe_capture(&route,true,3,false,99,3);
    assert(!route.junction_latched); /* RESET must not replay an old result. */
}
int main(void) {
    for(unsigned letter=0;letter<26;++letter)for(unsigned reverse=0;reverse<2;++reverse) {
        const unsigned units[]={20,40,100,1000,100000000};
        for(unsigned u=0;u<sizeof(units)/sizeof(units[0]);++u) {
            uint32_t runs[29];s3_symbol_t symbol;make_runs(letter,units[u],reverse,runs);
            assert(s3_barcode_decode(runs,29,&symbol)==S3_BARCODE_OK);
            assert(symbol.symbol==(char)('A'+letter) && symbol.reverse==(reverse!=0));
        }
        full_path(letter,reverse,false);full_path(letter,reverse,true);
        junction_during_capture(letter,reverse,8,0);
        junction_during_capture(letter,reverse,17,0);
        junction_during_capture(letter,reverse,8,1);
        junction_during_capture(letter,reverse,17,S3_BARCODE_MAX_RESYNC_PAIRS);
    }
    mapping_and_junction();
    default_forward_course();
    retained_capture_junction();
    puts("Barcode demo: A-Z both directions/scales, ADC-to-memory, deferred junction, repeat, all MQTT actions, malformed/busy/unmapped/wrap PASS");
    return 0;
}
