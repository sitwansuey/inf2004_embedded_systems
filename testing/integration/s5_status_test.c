/* Deterministic preemption/timeout fixture around the production S5 API. */
#include "s5_hardware_host.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#define TP_PRC2 2U
#define REQUIRE(c) do { if (!(c)) { \
    fprintf(stderr,"S5 status failed line %d: %s\n",__LINE__,#c); exit(2); } } while(0)
static ID caller = 100;
static ER forced_lock, forced_unlock;
static BOOL owner_preempted;
static unsigned handoffs, locks;
ID app_start_task(FP entry, PRI priority, UW core, const char *name);
INT tm_printf(UB *format, ...);
#include "../../app_program/subsystem5/subsystem5.c"

ID tk_cre_mtx(const T_CMTX *config) { REQUIRE(config->mtxatr==TA_INHERIT); return 17; }
ER tk_loc_mtx(ID id, TMO timeout)
{
    REQUIRE(id==17); ++locks;
    if(forced_lock<E_OK)return forced_lock;
    if(owner_preempted) {
        if(timeout==TMO_POL)return E_TMOUT;
        REQUIRE(timeout>0 && timeout<=5); /* finite wait; no busy retry */
        owner_preempted=FALSE; ++handoffs;
    }
    return E_OK;
}
ER tk_unl_mtx(ID id) { REQUIRE(id==17); return forced_unlock; }
ID tk_get_tid(void) { return caller; }
ER tk_dly_tsk(UW delay) { (void)delay; return E_OK; }
void tk_ext_tsk(void) { }
ER tk_get_otm(SYSTIM *time) { time->hi=0;time->lo=100;return E_OK; }
INT tm_printf(UB *format, ...) { (void)format;return 0; }
ID app_start_task(FP entry, PRI priority, UW core, const char *name)
{
    (void)name;REQUIRE(entry==(FP)scan_task && priority==20 && core==TP_PRC2);return 20;
}
ER s5_hw_timer_init(void) { return E_OK; }
void s5_hw_io_init(void) { }
void s5_hw_servo(INT angle) { (void)angle; }
void s5_hw_sample(s5_status_t *sample) { (void)sample; }
INT s5_scan_step(s5_status_t *sample,s5_scan_context_t *context)
{ (void)sample;(void)context;return 0; }

int main(void)
{
    s5_status_t copy={0}, sentinel;
    REQUIRE(s5_get_status(NULL)==E_PAR);
    REQUIRE(s5_get_status(&copy)==E_OBJ && subsystem5_start()==E_OBJ);
    REQUIRE(subsystem5_init()==E_OK && subsystem5_init()==E_OBJ);
    status=(s5_status_t){.state=S5_MONITOR,.scan_id=7,.time_ms=100,.valid=1,.distance_mm=400};
    owner_preempted=TRUE;
    REQUIRE(s5_get_status(&copy)==E_OK && copy.distance_mm==400 && handoffs==1);
    owner_preempted=TRUE;
    REQUIRE(s5_controller_bind()==E_OK && handoffs==2);
    caller=101;
    REQUIRE(s5_controller_bind()==E_OACV);
    REQUIRE(s5_navigation_set(7,S5_APPROACH,0)==E_OACV);
    caller=100;
    REQUIRE(s5_navigation_set(6,S5_APPROACH,0)==E_OBJ);
    REQUIRE(s5_navigation_set(7,S5_SCAN_LEFT,0)==E_PAR);
    REQUIRE(s5_navigation_set(7,S5_APPROACH,91)==E_PAR);
    owner_preempted=TRUE;
    REQUIRE(s5_navigation_set(7,S5_APPROACH,20)==E_OK && handoffs==3);
    REQUIRE(navigation_requested && navigation_look==20 && !status.valid);
    status.state=S5_READY;status.valid=1;owner_preempted=TRUE;
    REQUIRE(s5_avoidance_begin(7)==E_OK && handoffs==4 && turn_requested);
    REQUIRE(status.state==S5_ROTATING && !status.valid);
    owner_preempted=TRUE;
    REQUIRE(s5_avoidance_complete()==E_OK && handoffs==5 && recenter);
    status.state=S5_MONITOR;
    REQUIRE(s5_avoidance_complete()==E_OBJ);
    memset(&copy,0x5a,sizeof(copy));memcpy(&sentinel,&copy,sizeof(copy));
    forced_lock=E_TMOUT;
    REQUIRE(s5_get_status(&copy)==E_TMOUT && memcmp(&copy,&sentinel,sizeof(copy))==0);
    REQUIRE(s5_controller_bind()==E_TMOUT);
    REQUIRE(s5_navigation_set(7,S5_MONITOR,0)==E_TMOUT);
    REQUIRE(s5_avoidance_begin(7)==E_TMOUT && s5_avoidance_complete()==E_TMOUT);
    forced_lock=E_OK;forced_unlock=E_SYS;
    REQUIRE(s5_get_status(&copy)==E_SYS && s5_controller_bind()==E_SYS);
    forced_unlock=E_OK;
    REQUIRE(subsystem5_start()==E_OK && locks>0);
    puts("INT-CR1-09: preempted publisher handoff, ownership, timeout, unchanged failed output, unlock/lifecycle errors PASS");
    return 0;
}
