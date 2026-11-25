/* Run the actual controller task with synthetic owner snapshots. */
#include "s5_hardware_host.h"
#include <assert.h>
#include <setjmp.h>
#include <stdarg.h>
#include <stdio.h>
#include <string.h>
#define BOARD_TICKS_PER_REV 100
#define BOARD_WHEEL_CIRCUMFERENCE_MM 200
#define BOARD_TRACK_WIDTH_MM 100
#define BOARD_TERRAIN_CALIBRATED 1
#define BOARD_MOTORS_DISABLED 0
#define BOARD_BYPASS_VALIDATED 1 /* synthetic policy fixture only */
/* Loop timing report every 1 s here (10 s on the robot), so a mode sees it. */
#define VC_TIMING_REPORT_MS 1000U
ID app_start_task(FP task, PRI priority, UW core, const char *name);
ID app_start_task_stack(FP task, PRI priority, UW core, const char *name, INT stack_bytes);
#include "../../app_program/vehicle.c"
#include "../../app_program/subsystem5/s5_scan.h" /* mode 15 runs S5's real acquisition step */
#if !BOARD_LINE_MASK_PIVOT && !BOARD_LINE_EDGE_RIGHT
#error The edge-trace scenarios below ride the right edge (BOARD_LINE_EDGE_RIGHT 1)
#endif
#if BOARD_TIMING_STATS && !VC_TIMING
#error The vehicle task must build its loop timing hook here (BOARD_TIMING_STATS, TIMER_TIMERAWL)
#endif
/* Mode 0 also offers MAP frames: valid, invalid letter, stale. */
static unsigned acks,ack_log[4][3],map_calls,map_letter,map_nav,map_step;
static unsigned last_ack_seq,last_ack_opcode,last_ack_result; /* any opcode */
/* STOP/ARM/RESET/START ACKs (modes 46-49 send the tick number as the
 * sequence). */
static unsigned arm_acks,arm_ack_seq[16],arm_ack_op[16],arm_ack_result[16];
void s1_command_ack(unsigned seq,unsigned opcode,unsigned result,unsigned phase,unsigned fault)
{
    (void)phase;(void)fault;
    last_ack_seq=seq;last_ack_opcode=opcode;last_ack_result=result;
    if(opcode==4U && acks<4U) {ack_log[acks][0]=seq;ack_log[acks][1]=opcode;ack_log[acks++][2]=result;}
    if(opcode!=4U && arm_acks<16U) {
        arm_ack_seq[arm_acks]=seq;arm_ack_op[arm_acks]=opcode;arm_ack_result[arm_acks++]=result;
    }
}
/* USB console lines the checks look at: the loop timing report and the
 * re-sent TRACK for a marker S3 held with no turn pending. */
static char timing_line[256];static unsigned timing_lines,orphan_lines;
INT tm_printf(UB *format, ...)
{
    char line[256];va_list ap;va_start(ap,format);
    vsnprintf(line,sizeof(line),(const char *)format,ap);va_end(ap);
    if(!strncmp(line,"[TIMING]",8)) {++timing_lines;memcpy(timing_line,line,sizeof(line));}
    if(!strncmp(line,"[VC] S3 held",12))++orphan_lines;
    return 0;
}
static jmp_buf done;
static app_state_t saved;
static uint32_t clock_ms, iteration;
/* The loop timing records (app_main.c on the robot) and the 1 MHz hardware
 * timer: each read in a tick is 250 us after the previous one, so the VC's
 * run (one read at its release, one when it is done) is 250 us. */
#if VC_TIMING
task_timing_t app_task_timing[TASK_TIMING_COUNT];
#endif
static uint32_t timer_tick_ms=~0U,timer_reads;
UW in_w(UW address)
{
    assert(address==TIMER_TIMERAWL);
    if(clock_ms!=timer_tick_ms) {timer_tick_ms=clock_ms;timer_reads=0;}
    return (UW)(clock_ms*1000U+250U*timer_reads++);
}
static unsigned failure_mode, stops, positive_requests;
/* Robot events posted for the dashboard's event log (shared/events.h). */
#define EVENT_LOG 16U
static unsigned event_count,event_id[EVENT_LOG],event_value[EVENT_LOG],event_tick[EVENT_LOG];
void s1_event_post(unsigned id,unsigned value)
{
    if(event_count<EVENT_LOG) {event_id[event_count]=id;event_value[event_count]=value;event_tick[event_count]=iteration;}
    ++event_count;
}
/* Mode 7: the sonar reports an unusable distance from SONAR_GAP_AT on. */
#define SONAR_GAP_AT 8U
#define SONAR_GAP_TICKS (VC_SONAR_GAP_MS/TICK_MS)
/* Mode 10: barcode miss -> retry reverse, with one forward sonar glitch. */
#define BR_MISS_AT 10U
#define BR_GLITCH_AT 25U
#define BR_ITERATIONS 40U
static unsigned reverse_requests, reverse_after_glitch, left_track;
/* A reverse request, then a forward request or a STOP in the same tick
 * (iteration+1 of the last reverse; 0 none). */
static unsigned reverse_tick, after_reverse;
/* Mode 11: S3 starts reading a marker during normal tracking (first pass). */
#define READ_FROM 12U
#define READ_UNTIL 16U
static unsigned read_count, read_max, normal_count, normal_min;
/* The first slow tick of mode 11: the stub reports 2 elements at READ_FROM-1,
 * which counts as reading when BOARD_BARCODE_READ_MIN_ELEMENTS <= 2. */
#define READ_SLOW_FROM (BOARD_BARCODE_READ_MIN_ELEMENTS<=2U?READ_FROM-1U:READ_FROM)
/* Mode 12: one >1.8 g accelerometer sample while armed and tracking.
 * Mode 21: the same impact clipped at +/-2 g (the driver marks it invalid). */
#define SHOCK_AT 9U
static bool shock_mode(void) { return failure_mode==12U || failure_mode==21U; }
static unsigned shock_published, after_shock;
static uint8_t shock_flags, shock_faults; /* S4 result published at SHOCK_AT */
static bool reset_offered;
/* Modes 7, 13..20 and 22..27 START the mission (no lease) so they can run
 * past one second; the others ARM at 30% with a 1000 ms lease. */
static bool renew_mode(void) { return failure_mode==46U || failure_mode==48U || failure_mode==49U; }
static bool starts(void) { return failure_mode==7U || (failure_mode>=13U && failure_mode<=20U) || (failure_mode>=22U && !renew_mode()); }
#define MODE_COUNT 51U
/* Mode 46: ARM with a 500 ms lease at tick 7 (tools/robot_console.py --arm),
 * the same ARM again every RENEW_EVERY ticks (100 ms) up to RENEW_UNTIL, and
 * one START at RENEW_START_AT. Mode 47: START at tick 7, ARM at tick 10.
 * Mode 48: ARM at 7 and 12, then a renewal at another demand (17), one
 * stamped 100 ms old (22), and one at 27 while S3's status reads have
 * failed for 4 ticks (line not configured). Mode 49: renewals every 100 ms
 * with a STOP at 25, RESET at 30, STOP again at 40, then quiet. */
#define RENEW_EVERY 5U
#define RENEW_UNTIL 37U
#define RENEW_START_AT 20U
#define RENEW_LEASE_TICKS 25U /* 500 ms */
#define RENEW_S3_MISS_FROM 24U /* mode 48: S3 status fails 24..27 */
#define HOLD_STOP1 25U
#define HOLD_RESET 30U
#define HOLD_STOP2 40U
#define HOLD_REARM 98U /* 51 ticks (1020 ms) after the last refused ARM at 47 */
static unsigned renew_offered_at;
#define TICK_MS 20U /* tk_dly_tsk below */
/* Per-tick record of what reached S2 and what the task published. */
#define LOG_TICKS 500U
enum { SENT_NONE, SENT_FOLLOW, SENT_MOVE, SENT_REVERSE };
static uint8_t sent_kind[LOG_TICKS], sent_cmd[LOG_TICKS], sent_power[LOG_TICKS], stopped[LOG_TICKS];
static int32_t sent_steer[LOG_TICKS];
static uint8_t pub_seen[LOG_TICKS], pub_follow[LOG_TICKS], pub_mask[LOG_TICKS];
static uint8_t pub_phase[LOG_TICKS], pub_armed[LOG_TICKS];
/* Renewal modes: what a renewal must not reset (entered_ms, origin), and
 * the run's demand and lease start. */
static uint32_t pub_entered[LOG_TICKS], pub_armed_ms[LOG_TICKS];
static int32_t pub_origin_um[LOG_TICKS], pub_origin_yaw[LOG_TICKS];
static uint8_t pub_max_power[LOG_TICKS], pub_fault[LOG_TICKS];
static int16_t pub_steer[LOG_TICKS];
static void log_send(unsigned kind,unsigned command,UINT power,INT steering)
{
    if(iteration>=LOG_TICKS)return;
    sent_kind[iteration]=(uint8_t)kind;sent_cmd[iteration]=(uint8_t)command;
    sent_power[iteration]=(uint8_t)power;sent_steer[iteration]=steering;
}
/* Line under the straddle sensors, as darkness per mille of each sensor's
 * measured white..black range (GP26 right, GP27 left). "Centred" is both
 * WHITE for the mask-pivot follower and the right IR on the line's edge (the
 * set darkness) for edge tracing, so modes 7..13 and 15 drive straight on
 * (steer 0, full power) with either follower. */
#if BOARD_LINE_MASK_PIVOT
#define CENTRED_RIGHT 0
#else
#define CENTRED_RIGHT BOARD_LINE_EDGE_SET
#endif
/* Mode 13: S3 reads a marker (READING, >= 3 elements) while the straddle
 * sensors see right, left, both, neither, then right again. */
#define SR_FROM 12U
#define SR_UNTIL 24U
#define SR_ITERATIONS 28U
static const uint8_t sr_masks[SR_UNTIL-SR_FROM]={1,1,1,2,2,2,3,3,0,0,1,1};
/* Mode 14: the follower, tick by tick from FOLLOW_FROM: what the sensors
 * see, whether the sonar looks aside (a TRACK STOP: front unknown, no
 * obstacle), and what must reach S2 and the published line view. */
#define FOLLOW_FROM 10U
typedef struct {
    int16_t right,left; uint8_t stop;
    uint8_t kind,cmd,power; int16_t steer; uint8_t follow,mask;
} follow_step_t;
#define F_STRAIGHT(r,l,m) {r,l,0,SENT_FOLLOW,S2_MOVE_FORWARD,30,0,0,m}
#define F_PIVOT(r,l,cmd,code,m) {r,l,0,SENT_MOVE,cmd,30,0,code,m}
#define F_STEER(r,l,power,steer,m) {r,l,0,SENT_FOLLOW,S2_MOVE_FORWARD,power,steer,(steer)>0?1:2,m}
#define F_STOP(r,l,m) {r,l,1,SENT_NONE,0,0,0,0,m}
#if BOARD_LINE_MASK_PIVOT
/* Mask-pivot: right IR black pivots right, left IR black pivots left, both
 * or neither drive straight; the current sample only (no timing), so a
 * resume after a STOP acts on the sensors at once. */
static const follow_step_t follow_steps[]={
    F_STRAIGHT(0,0,0),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STRAIGHT(1000,1000,3),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),
    F_STRAIGHT(0,0,0),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),F_STOP(1000,0,1),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),
    F_STRAIGHT(0,0,0),
};
#else
/* Edge tracing (right IR on the right edge, set 500, kp 0.8, max 400): the
 * steer is proportional to the darkness error and the power drops by up to
 * 35% at full steer; the left IR on black pivots left until the right IR is
 * back on the edge; both black is a crossing (straight until the left IR is
 * white); the right IR deep in the line for 250 ms pivots right until it is
 * back at the edge. Masks follow the 750/550 per mille hysteresis. */
#define DEEP_TICKS ((BOARD_LINE_EDGE_DEEP_MS+TICK_MS-1U)/TICK_MS)
static const follow_step_t follow_steps[]={
    F_STRAIGHT(500,0,0),
    F_STEER(750,0,24,200,1),F_STEER(250,0,24,-200,0),
    F_STRAIGHT(500,0,0),
    F_PIVOT(0,1000,S2_MOVE_PIVOT_LEFT,4,2),F_PIVOT(300,0,S2_MOVE_PIVOT_LEFT,4,0),
    F_STEER(600,0,27,80,0),
    F_STRAIGHT(1000,1000,3),F_STRAIGHT(1000,800,3),
    /* Crossing over; the right IR deep in the line: full steer, then a pivot. */
    F_STEER(1000,500,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),F_STEER(1000,0,19,400,1),
    F_PIVOT(1000,0,S2_MOVE_PIVOT_RIGHT,3,1),F_PIVOT(700,0,S2_MOVE_PIVOT_RIGHT,3,1),
    F_STRAIGHT(500,0,0),
    F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),F_STOP(500,0,0),
    F_STEER(750,0,24,200,1),
};
_Static_assert(DEEP_TICKS==13U,"12 deep ticks of full steer, then the pivot");
#endif
#define FOLLOW_STEPS (sizeof(follow_steps)/sizeof(follow_steps[0]))
static const follow_step_t *follow_step(unsigned n)
{ return failure_mode==14U && n>=FOLLOW_FROM && n<FOLLOW_FROM+FOLLOW_STEPS?&follow_steps[n-FOLLOW_FROM]:NULL; }
/* Mode 15: forward sonar samples while tracking a centred line, one per
 * tick from OC_FROM (H: one near echo re-read for several ticks, F far,
 * N near 150 mm, D no-echo, C close 40 mm, I invalidated by S5 re-centring)
 * and what TRACK must do (d drive, s hold still, a VC_ACQUIRE): the held
 * echo counts once; a no-echo holds only right after a near echo and is no
 * reading; C and I hold; acquisition only on the third near echo.
 * Each new sample goes through S5's own acquisition step, as its scan task
 * does (subsystem5.c): S5 counts the same echoes, so the third near echo
 * starts SCAN_LEFT and is published already turning to 20 deg (invalid);
 * S5 then settles the servo for 200 ms, so the VC re-reads that status. The
 * VC acquires on S5's stage with no forward echo in hand, and EV_OBSTACLE
 * detected must still carry the 15 cm. I is S5's status invalidated outside
 * its step (re-centring), published as is. */
#define OC_FROM 12U
static const char oc_sample[]="HHHHHFNNDFDCFINNDNNN";
static const char oc_expect[]="ddddddddsddsdsddsaaa";
#define OC_ITERATIONS (OC_FROM+sizeof(oc_sample)-1U)
#define OC_ACQUIRE_AT (OC_FROM+17U) /* the third near echo */
static s5_status_t oc_status;static s5_scan_context_t oc_scan;static bool oc_sampled;
static void oc_s5_publish(s5_status_t *s,char sample)
{
    if(oc_status.state!=S5_MONITOR || (sample=='H' && oc_sampled)) {*s=oc_status;return;}
    oc_sampled=true;
    if(sample!='I') {
        s->sample_angle_deg=s->angle_deg;
        INT target=s5_scan_step(s,&oc_scan);
        if(target!=s->angle_deg) {s->valid=FALSE;s->angle_deg=target;} /* never valid while panning */
    }
    oc_status=*s;
}
/* Mode 16: lifted from LIFT_AT (both line IRs read past BOARD_LIFT_RAW, GP28
 * black): VC_LIFTED after BOARD_LIFT_FAULT_MS (0: never). */
#define LIFT_AT 10U
#define LIFT_TICKS 100U /* 2000 ms */
/* Mode 17: a hump. The front rises 10 deg from HUMP_FROM (the accelerometer
 * tilts with it), level again from HUMP_LEVEL: with BOARD_RAMP_SPEED the
 * smoothed tilt held 400 ms is the way up (EV_HUMP detected), 1.5 deg below
 * its peak the crest (EV_HUMP crest; power capped at BOARD_RAMP_DOWN_POWER),
 * level for 400 ms the end (EV_HUMP level; normal power). */
#define HUMP_FROM 10U
#define HUMP_LEVEL 70U
#define HUMP_DOWN 140U
#define HUMP_BOTTOM 200U
#define HUMP_ITERATIONS 300U
#define HUMP_COUNTS_PER_TICK 5U /* 10 mm per tick: S4 integrates the height */
static bool scenario_nose_up(unsigned n) { return failure_mode==17U && n>=HUMP_FROM && n<HUMP_LEVEL; }
/* The mask-pivot follower drives the 1.2 m centred (both IRs white): a
 * crossing line (both black, straight on) every 200 mm keeps it within
 * BOARD_LINE_LOST_MM of the line. */
#define HUMP_CROSS_TICKS 20U
_Static_assert(!BOARD_LINE_LOST_MM || HUMP_CROSS_TICKS*HUMP_COUNTS_PER_TICK*2U<BOARD_LINE_LOST_MM,"mode 17 line contact");
static bool hump_cross(unsigned n) { return BOARD_LINE_MASK_PIVOT && (failure_mode==17U || failure_mode==38U || failure_mode==50U) && n%HUMP_CROSS_TICKS==0U; }
/* Mode 50: a low hump, 3.3 cm: 4.8 deg up for 400 mm from LOW_FROM, then
 * straight down at 4.8 deg (4 mm per tick). The ramp handling sees it (4 deg
 * held 400 ms); S4 terrain never does (it needs a 5 deg climb), so before
 * the controller's estimate it had no height, and it ended at the top: the
 * way down was a second hump, at full speed. */
#define LOW_FROM 10U
#define LOW_DOWN 110U
#define LOW_BOTTOM 210U
#define LOW_ITERATIONS 320U
#define LOW_COUNTS_PER_TICK 2U /* 4 mm per tick */
/* Mode 18: two decoded markers (straight on). S3 queues the first turn one
 * tick before its status shows the decode (DEC1_AT, then DEC1_AT+1); the
 * second turn and its decode arrive in the same tick (DEC2_AT). The event log
 * lists each EV_BARCODE attempt before the EV_INSTRUCTION it decoded. */
#define DEC1_AT 10U
#define DEC2_AT 16U
#define DEC_ITERATIONS 20U
static unsigned nav_given; /* iteration whose turn was handed out */
/* Mode 19: as mode 8 (S5 READY at 100 mm: acquisition), then a STOP command
 * at AVOID_STOP_AT ends the avoidance: EV_OBSTACLE back, how 0. */
#define AVOID_STOP_AT 10U
/* Mode 20: S5's status lock is busy (s5_get_status fails) from S5_BUSY_FROM
 * to S5_BUSY_UNTIL, across the START phase change: the last snapshot is
 * reused, but the new sensor mode is requested only once a fresh read
 * succeeds (s5_navigation_set would time out on the busy lock). */
#define S5_BUSY_FROM 7U
#define S5_BUSY_UNTIL 10U
static bool s5_busy(unsigned n) { return failure_mode==20U && n>=S5_BUSY_FROM && n<S5_BUSY_UNTIL; }
static unsigned nav_set_busy, nav_set_after;
/* Modes 22 and 23 run S3's real core (buddy3_core.c) behind its controller
 * API, as subsystem3.c does. A sampler hiccup (a late sample or ADC reply)
 * latches STALE, or SENSOR, with no event; S3 then aborts every barcode
 * capture until the VC sends TRACK again. The VC reads the next, fresh
 * sample with the latch.
 * Mode 22: one STALE latch at LATCH_AT, in the middle of a *B* marker (GP28,
 * narrow 1 tick, wide 3, the robot at 2 mm per tick). With
 * BOARD_BARCODE_FAULT_FAILS (and the re-read drive) S3 counts the cut-short
 * capture (5 elements) as a failed read: EV_BARCODE attempt 1, and the
 * re-read brakes, then reverses; TRACK (marker clear) is re-sent as the
 * reverse starts and the next marker decodes during it (attempt 2). Without
 * it, the VC re-sends TRACK at once and keeps driving, and the next marker
 * decodes.
 * Mode 23: a turn is pending (F, left: no junction yet) and S3 latches
 * SENSOR again on every sample from LATCH_AT (cutting the *B* short as in
 * mode 22, so its re-read brakes first): TRACK (holding the marker) is
 * re-sent at most every VC_S3_REARM_MS, then VC_LINE cause event after
 * VC_S3_LATCH_MS.
 * Mode 24: as mode 10 (synthetic S3), with the status latched STALE from the
 * miss at BR_MISS_AT: no re-send while the re-read brakes; from the reverse
 * on, as mode 23. */
#define LATCH_AT 30U
#define MARKER1_AT 20U
#define MARKER2_AT 80U
#define NAV_AT 20U
#define LATCH_TICKS (VC_S3_LATCH_MS/TICK_MS)
#define REARM_TICKS (VC_S3_REARM_MS/TICK_MS)
#define REVERSE_AT (BR_MISS_AT+BOARD_BARCODE_RETRY_BRAKE_MS/TICK_MS)
#if BOARD_BARCODE_FAULT_FAILS && BOARD_BARCODE_RETRY_DRIVE
#define LATCH_READ_FAILS 1 /* the latch cuts mode 22/23's first *B*: a failed read, re-read */
#define LATCH_REARM_AT (LATCH_AT+BOARD_BARCODE_RETRY_BRAKE_MS/TICK_MS) /* the re-read brakes first */
#else
#define LATCH_READ_FAILS 0
#define LATCH_REARM_AT LATCH_AT
#endif
#define MARKER2_READ_AT (MARKER2_AT+63U) /* the second *B* decodes 63 ticks after it starts */
/* Mode 22 drives 2 mm a tick centred (both IRs white) to the end. */
_Static_assert(!BOARD_LINE_LOST_MM || (MARKER2_AT+70U)*2U<BOARD_LINE_LOST_MM,"mode 22 line contact");
static bool real_s3(void) { return failure_mode==22U || failure_mode==23U; }
/* Modes 25..27 (START) drive by what reached S2 each tick (kin_step).
 * Mode 25: the right IR touches the line at LL_TOUCH_AT, then the line is
 * gone (both IRs white): BOARD_LINE_LOST_MM later the robot searches on the
 * spot, turning right (the line's side) first, and the right IR finds black
 * LL_FOUND_AFTER ticks into the search: tracking resumes. Mode 26: never
 * found, fault 2 cause 3. Edge tracing has no line-lost distance: the lost
 * line is its own full-turn fault (VC_LINE, cause other). Mode 27: centred
 * throughout with a barcode miss at BR_MISS_AT, as mode 10: the re-read (an
 * 800 mm reverse, then a 950 mm slow pass) never triggers it; the distance
 * runs again from the re-read's end. Mode 28: centred throughout, S3 reads a
 * marker (READING, >= 3 elements) from MR_FROM for 700 mm: the same. */
#define LL_TOUCH_AT 20U
#define LL_FOUND_AFTER 3U
#define MR_FROM 20U
#define MR_UNTIL (MR_FROM+70U)
static bool obstacle_mode(void) {return failure_mode>=41U && failure_mode<=45U;}
static bool kinematic(void) { return (failure_mode>=25U && failure_mode<=28U) || obstacle_mode(); }
static s5_state_t nav_mode;static int nav_angle,old_sample_angle;
static unsigned pan_at,probe_at,probe_angle,side_looks;
static bool pan_blocked[LOG_TICKS];
/* Modes 29..35 (START, centred line): short timing faults that must not stop
 * the run, the same faults when they last, and S3 holding a marker.
 * Mode 29: one failed encoder copy (ENC_FAIL_AT) and one late controller
 * period (LATE_AT: the clock jumps LATE_EXTRA_MS more, so two encoder
 * snapshots are 70 ms apart) each leave odometry invalid for two ticks: the
 * VC holds still, then drives on (C1). The motor status copy fails for two
 * ticks (MOTOR_BUSY_AT: the last status is reused, C2); S3's lock is busy
 * for the first mode request and for three renewals (RENEW_BUSY_AT: retried,
 * L7); a MAP frame stamped 10 ms after the VC read its clock is accepted
 * (FUTURE_AT, W6). The timing report shows the late period (C5).
 * Mode 30: the encoder copy fails from FAULT_AT on: fault 1 cause 3 after
 * VC_ODOM_GAP_MS. Mode 31: the motor status copy fails from FAULT_AT on:
 * fault 7 after VC_MOTOR_STATUS_MISSES reused ticks. Mode 32: renewals are
 * busy from FAULT_AT on: fault 4 once the lease would have run out. Mode
 * 33: S2 publishes FAULT at FAULT_AT: fault 7 at once.
 * Mode 34: from ORPHAN_AT S3 shows a decoded marker held (awaiting the
 * controller, no error) with no turn event: TRACK (marker clear) is re-sent
 * VC_S3_REARM_MS later, once (L6). Mode 35: a turn (Z, right) arrives at
 * ORPHAN_AT with S3 latched STALE in the same sample; S3 then holds its
 * marker: the turn stays pending, TRACK re-sent for the latch holds the
 * marker, and the orphan re-send never fires (L1). */
#define ENC_FAIL_AT 12U
#define LATE_AT 20U
#define LATE_EXTRA_MS 50U
#define MOTOR_BUSY_AT 30U
#define RENEW_BUSY_AT 40U
#define FUTURE_AT 50U
#define FAULT_AT 12U
#define ORPHAN_AT 20U
static bool timing_mode(void) { return failure_mode>=29U && failure_mode<=33U; }
static unsigned control_busy; /* mode 29: S3 control requests refused as busy */
static bool future_offered; /* mode 29: the future-stamped MAP frame was taken */
static bool orphan_cleared; /* modes 34, 35: TRACK with marker clear after ORPHAN_AT */
static unsigned lost_tick; /* first VC_REACQUIRE published after START (0 none) */
static uint32_t kin_counts[2];static int32_t kin_mm[LOG_TICKS]; /* odometry each tick */
/* Forward or a follow 10 mm (a follow's steer splits the wheels), a pivot
 * moves its outer wheel 10 mm, a spin 2 mm a wheel each way (2.3 deg), a
 * reverse 10 mm back, a STOP holds (2 mm a count). */
static void kin_step(void)
{
    if(!kinematic() || iteration>=LOG_TICKS || stopped[iteration])return;
    int32_t l=0,r=0,s=sent_steer[iteration];
    if(sent_kind[iteration]==SENT_FOLLOW) {l=(5*(1000+s)+500)/1000;r=(5*(1000-s)+500)/1000;}
    else if(sent_kind[iteration]==SENT_REVERSE) l=r=-5;
    else if(sent_kind[iteration]==SENT_MOVE) switch(sent_cmd[iteration]) {
        case S2_MOVE_FORWARD: l=r=5;break;
        case S2_MOVE_LEFT: l=-1;r=1;break;
        case S2_MOVE_RIGHT: l=1;r=-1;break;
        case S2_MOVE_PIVOT_RIGHT: l=5;break;
        case S2_MOVE_PIVOT_LEFT: r=5;break;
        default: break;
    }
    kin_counts[0]+=(uint32_t)l;kin_counts[1]+=(uint32_t)r;
}
static s3_core_t s3c;static s3_event_t s3_pending;
static unsigned track_calls,track_tick[16],track_clear[16]; /* TRACK requests */
static bool marker_black(unsigned n)
{
    static const uint16_t chars[3]={0x094U,0x049U,0x094U}; /* *B*, B unmapped */
    const unsigned at[2]={MARKER1_AT,MARKER2_AT};
    for(unsigned m=0;m<2U;++m) {
        if(n<at[m])continue;
        unsigned t=n-at[m];
        for(unsigned e=0;e<29U;++e) {
            unsigned i=e%10U,w=i==9U?1U:(((chars[e/10U]>>(8U-i))&1U)?3U:1U);
            if(t<w)return (e&1U)==0U;
            t-=w;
        }
    }
    return false;
}
static void scenario_dark(unsigned n,int32_t *right,int32_t *left)
{
    const follow_step_t *f=follow_step(n);
    *right=CENTRED_RIGHT;*left=0;
    if(f) {*right=f->right;*left=f->left;}
    else if(failure_mode==13U && n>=SR_FROM && n<SR_UNTIL) {
        *right=(sr_masks[n-SR_FROM]&1U)?1000:0;*left=(sr_masks[n-SR_FROM]&2U)?1000:0;
    }
    else if(hump_cross(n)) *right=*left=1000;
    else if(failure_mode==25U || failure_mode==26U) {
        if(n==LL_TOUCH_AT || (failure_mode==25U && lost_tick && n>=lost_tick+LL_FOUND_AFTER)) *right=1000;
        else if(n>LL_TOUCH_AT) *right=0; /* the line is gone */
    }
}
static bool scenario_lifted(unsigned n) { return failure_mode==16U && n>=LIFT_AT; }
static unsigned scenario_mask(unsigned n)
{
    const follow_step_t *f=follow_step(n);
    if(f)return f->mask;
    if(failure_mode==13U)return n>=SR_FROM && n<SR_UNTIL?sr_masks[n-SR_FROM]:0U;
    if(hump_cross(n))return 3U;
    if(failure_mode==25U || failure_mode==26U) {int32_t r,l;scenario_dark(n,&r,&l);return r==1000?1U:0U;}
    return (scenario_lifted(n) || ((failure_mode==39U || failure_mode==40U) && n>=20U))?3U:0U; /* otherwise centred */
}
static bool scenario_reading(unsigned n) { return failure_mode==13U && n>=SR_FROM && n<SR_UNTIL; }
static bool scenario_dropout(unsigned n) { const follow_step_t *f=follow_step(n);return f && f->stop; }
static unsigned iterations(void)
{
    switch(failure_mode) {
    case 7U: return SONAR_GAP_AT+SONAR_GAP_TICKS+2U;
    case 10U: return BR_ITERATIONS;
    case 11U: return 20U;
    case 13U: return SR_ITERATIONS;
    case 14U: return FOLLOW_FROM+(unsigned)FOLLOW_STEPS;
    case 15U: return OC_ITERATIONS;
    case 16U: return LIFT_AT+LIFT_TICKS+3U;
    case 17U: return HUMP_ITERATIONS;
    case 50U: return LOW_ITERATIONS;
    case 18U: return DEC_ITERATIONS;
    case 20U: return 14U;
    case 22U: return MARKER2_AT+70U; /* the decode is 63 ticks after it starts */
    case 23U: return LATCH_REARM_AT+LATCH_TICKS+3U;
    case 24U: return REVERSE_AT+LATCH_TICKS+3U;
    case 25U: return 120U;
    case 26U: case 27U: return LOG_TICKS;
    case 28U: return MR_UNTIL+70U;
    case 29U: return 110U; /* two 1 s timing reports */
    case 30U: case 31U: case 32U: case 33U: return 30U;
    case 34U: case 35U: return 60U;
    case 36U: case 37U: return 16U;
    case 38U: return 160U;
    case 39U: case 40U: return 150U;
    case 41U: case 42U: case 43U: case 44U: case 45U: return 450U;
    case 46U: return RENEW_UNTIL+RENEW_LEASE_TICKS+10U;
    case 47U: return 20U;
    case 48U: return 27U+RENEW_LEASE_TICKS+10U;
    case 49U: return HOLD_REARM+RENEW_LEASE_TICKS+5U;
    default: return 12U;
    }
}

ER tk_get_otm(SYSTIM *t) { t->hi=0;t->lo=clock_ms;return E_OK; }
ER tk_dly_tsk(UW delay)
{
    (void)delay;
    if(failure_mode==10U && iteration>BR_MISS_AT && saved.vehicle.phase!=VC_TRACK)++left_track;
    kin_step();
    if (++iteration >= iterations()) longjmp(done,1);
    clock_ms += TICK_MS;
    if(failure_mode==29U && iteration==LATE_AT)clock_ms+=LATE_EXTRA_MS; /* one late period */
    return E_OK;
}
void tk_ext_tsk(void) { assert(0); }
void s2_controller_bind(void) {}
ER s3_controller_bind(void) { return E_OK; }
ER s5_controller_bind(void) { return E_OK; }
ER s2_emergency_stop(void)
{
    ++stops;if(iteration<LOG_TICKS)stopped[iteration]=1;
    if(reverse_tick==iteration+1U)++after_reverse;
    return E_OK;
}
/* Positive drive request accounting shared by S2_MOVE_* and the PID follow
 * request (forward with steering). */
static void count_move(s2_move_command_t command,UINT power)
{
    ++positive_requests;
    if(command==S2_MOVE_REVERSE)reverse_tick=iteration+1U;
    else if(reverse_tick==iteration+1U)++after_reverse;
    if(shock_mode() && iteration>SHOCK_AT && command==S2_MOVE_FORWARD && power)++after_shock;
    if(failure_mode==11U && command==S2_MOVE_FORWARD) {
        if(iteration>=READ_SLOW_FROM && iteration<READ_UNTIL) {++read_count;if(power>read_max)read_max=power;}
        else {++normal_count;if(power<normal_min)normal_min=power;}
    }
    if(command==S2_MOVE_REVERSE) {++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;}
}
ER s2_movement_send(s2_move_command_t command,UINT power)
{
    count_move(command,power);log_send(SENT_MOVE,command,power,0);return E_OK;
}
ER s2_follow_send(UINT power,INT steering)
{
    count_move(S2_MOVE_FORWARD,power);log_send(SENT_FOLLOW,S2_MOVE_FORWARD,power,steering);return E_OK;
}
ER s2_reverse_follow_send(UINT power,INT steering)
{
    ++positive_requests;log_send(SENT_REVERSE,S2_MOVE_REVERSE,power,steering);reverse_tick=iteration+1U;
    ++reverse_requests;if(iteration>BR_GLITCH_AT)++reverse_after_glitch;return E_OK;
}
ER s2_motion_get_status(s2_motion_status_t *s)
{
    /* A timed-out copy (busy semaphore) leaves nothing behind. */
    if((failure_mode==29U && (iteration==MOTOR_BUSY_AT || iteration==MOTOR_BUSY_AT+1U)) ||
       (failure_mode==31U && iteration>=FAULT_AT)) {*s=(s2_motion_status_t){.state=S2_MOTION_FAULT};return E_TMOUT;}
    *s=(s2_motion_status_t){0};
    if(failure_mode==33U && iteration>=FAULT_AT)s->state=S2_MOTION_FAULT;
    return E_OK;
}
ER s2_encoder_snapshot(uint8_t *p,size_t capacity)
{
    uint32_t stamp=clock_ms;
    if(failure_mode==1U && iteration>=8U)stamp+=100U;
    uint32_t counts=failure_mode==17U?iteration*HUMP_COUNTS_PER_TICK:failure_mode==50U?iteration*LOW_COUNTS_PER_TICK:
                    failure_mode==22U?iteration:0U;
    encoder_snapshot_t e={counts,counts,stamp,ENCODER_SNAPSHOT_VALID};
    if(kinematic()) {
        e.left_counts=kin_counts[0];e.right_counts=kin_counts[1];
        if(iteration<LOG_TICKS)kin_mm[iteration]=(int32_t)(kin_counts[0]+kin_counts[1]-2000U); /* from 1000/1000 */
    }
    assert(encoder_snapshot_encode(&e,p,capacity));
    /* A failed provider deliberately leaves old-looking data behind. */
    return (failure_mode==2U && iteration>=8U) || (failure_mode==29U && iteration==ENC_FAIL_AT) ||
        (failure_mode==30U && iteration>=FAULT_AT)?E_SYS:E_OK;
}
ER s3_status_get(s3_status_t *s) {
    *s=(s3_status_t){0};
    /* Lock busy: zeroed output, as subsystem3.c does. */
    if(failure_mode==48U && iteration>=RENEW_S3_MISS_FROM && iteration<=27U)return E_TMOUT;
    if(failure_mode>=7U) {s->configured=1;s->sample_time_ms=clock_ms;s->output.steering_allowed=1;}
    if((failure_mode==10U || failure_mode==24U || failure_mode==27U) && iteration>=BR_MISS_AT)s->output.barcode_fail_count=1;
    if(failure_mode==24U && iteration>=BR_MISS_AT)s->output.error=S3_ERROR_STALE;
    if(failure_mode==18U) { /* decode counts one tick after the first turn */
        unsigned ok=(iteration>DEC1_AT?1U:0U)+(iteration>=DEC2_AT?1U:0U);
        s->output.barcode_ok_count=(uint8_t)ok;
        if(ok) {
            s->output.barcode_result=S3_BARCODE_RESULT_DECODED;s->output.barcode_elements=29U;
            s->output.last_symbol=ok==1U?'F':'B';
        }
    }
    /* A marker in progress (bar, gap, bar...): with fewer completed elements
     * than BOARD_BARCODE_READ_MIN_ELEMENTS (8f: 3) it must not slow the robot
     * (checked at READ_FROM-1, 2 elements); from the default 0 it does. */
    if(failure_mode==11U && iteration>=READ_FROM-1U && iteration<READ_UNTIL) {
        s->output.barcode_result=S3_BARCODE_RESULT_READING;
        s->output.barcode_elements=(uint8_t)(iteration<READ_FROM?2U:3U+iteration-READ_FROM);
    }
    if(failure_mode==28U && iteration>=MR_FROM && iteration<MR_UNTIL) {
        s->output.barcode_result=S3_BARCODE_RESULT_READING;
        s->output.barcode_elements=(uint8_t)(3U+iteration-MR_FROM);
    }
    /* The straddle observer interprets S3's raw scan itself: a complete,
     * fresh sample against the measured GP26/27 endpoints (GP28 digital). */
    if(failure_mode>=7U) {
        static const uint16_t white[S3_SENSOR_COUNT]=BOARD_IR_ADC_WHITE,black[S3_SENSOR_COUNT]=BOARD_IR_ADC_BLACK;
        int32_t dark[2];scenario_dark(iteration,&dark[0],&dark[1]);
        for(unsigned i=0;i<2U;++i) /* rounded up, so S3 normalises back to exactly dark[i] */
            s->sample.raw[i]=(uint16_t)(white[i]+((uint32_t)(black[i]-white[i])*(uint32_t)dark[i]+999U)/1000U);
        s->sample.raw[2]=0U;s->sample.valid_mask=7U;s->sample.time_ms=clock_ms;
        if(scenario_lifted(iteration) || ((failure_mode==39U || failure_mode==40U) && iteration>=20U)) { /* no surface under the IRs, GP28 black */
            s->sample.raw[0]=s->sample.raw[1]=4000U;s->sample.raw[2]=1U;s->output.barcode_black=1;
        }
        if(scenario_reading(iteration)) {
            s->output.barcode_result=S3_BARCODE_RESULT_READING;
            s->output.barcode_elements=(uint8_t)(3U+iteration-SR_FROM);
        }
    }
    /* Modes 34/35: S3 holds a decoded marker (awaiting the controller) until
     * a TRACK with marker clear; mode 35 latched STALE in the decode's tick. */
    if((failure_mode==34U || failure_mode==35U) && iteration>=ORPHAN_AT && !orphan_cleared) {
        s->output.awaiting_controller=1;
        if(failure_mode==35U && iteration==ORPHAN_AT)s->output.error=S3_ERROR_STALE;
    }
    if(real_s3()) { /* this sample through S3's own core */
        s->sample.raw[2]=(uint16_t)(marker_black(iteration)?BOARD_BARCODE_DIGITAL_BLACK_LEVEL:
            1U-BOARD_BARCODE_DIGITAL_BLACK_LEVEL);
        if(failure_mode==22U?iteration==LATCH_AT:iteration>=LATCH_AT)
            s3_core_fault(&s3c,failure_mode==22U?S3_ERROR_STALE:S3_ERROR_SENSOR);
        s3_output_t out;s3_core_step(&s3c,&s->sample,&out);
        if(out.event.kind!=S3_EVENT_NONE)s3_pending=out.event;
        s->output=out;s->output.event=(s3_event_t){0};
    }
    return E_OK;
}
ER s5_get_status(s5_status_t *s) {
    if(s5_busy(iteration))return E_TMOUT; /* lock wait timed out: no copy */
    *s=(s5_status_t){0};
    if(failure_mode>=7U) {
        s->state=S5_MONITOR;s->valid=1;s->distance_mm=400;s->time_ms=clock_ms;
        if(iteration>=SONAR_GAP_AT && failure_mode==7U)s->distance_mm=UINT32_MAX;
        if(iteration==BR_GLITCH_AT && failure_mode==10U)s->distance_mm=150; /* one spurious echo */
        if(iteration>=8U && (failure_mode==8U || failure_mode==19U)) {
            s->state=S5_READY;s->distance_mm=100;s->approach_plan_valid=1;s->object_width_valid=1;
            s->planned_rotate_deg=65537;s->planned_approach_mm=65537;s->planned_return_mm=65537;
        }
        if(scenario_dropout(iteration)) {s->angle_deg=s->sample_angle_deg=30;} /* fresh, looking aside */
        if(failure_mode==15U && iteration>=OC_FROM && iteration<OC_ITERATIONS) {
            static uint32_t held_ms;
            switch(oc_sample[iteration-OC_FROM]) {
            case 'H':if(iteration==OC_FROM)held_ms=clock_ms;s->distance_mm=150;s->time_ms=held_ms;break;
            case 'N':s->distance_mm=150;break;
            case 'C':s->distance_mm=40;break;
            case 'D':s->valid=0;s->distance_mm=0;s->echo_result=S5_ECHO_NO_RISE;break;
            case 'I':s->valid=0;s->distance_mm=150;break; /* echo OK, distance unusable */
            default:break;
            }
            oc_s5_publish(s,oc_sample[iteration-OC_FROM]);
        }
    }
    if(obstacle_mode()) {
        s->scan_id=7;s->state=nav_mode;s->angle_deg=nav_angle;s->sample_angle_deg=nav_angle;
        s->valid=1;s->distance_mm=4000;s->obstacle_distance_mm=150;
        s->left_bound_valid=s->right_bound_valid=s->object_width_valid=1;
        s->left_bound_deg=30;s->right_bound_deg=-30;
        s->left_bound_distance_mm=s->right_bound_distance_mm=173;
        s->object_width_mm=172;s->object_width_max_mm=181;
        if(iteration>=8U && nav_mode==S5_MONITOR) {
            s->state=S5_READY;s->approach_plan_valid=1;s->planned_rotate_deg=50;
            s->planned_approach_mm=s->planned_return_mm=180;s->distance_mm=150;
            s->angle_deg=s->sample_angle_deg=50;
        } else if(nav_angle==90 || nav_angle==-90) {
            /* Alternating missing and present echoes must still resolve O8. */
            s->distance_mm=150;
            if(iteration%2U) {s->valid=0;s->echo_result=S5_ECHO_NO_RISE;}
        } else if(nav_angle!=0 && failure_mode==42U)s->distance_mm=300;
        bool waiting=pan_at && iteration-pan_at<15U;
        if((failure_mode==43U && probe_at && nav_angle!=0 && abs(nav_angle)<90) ||
           (failure_mode==44U && side_looks && nav_angle==0) ||
           (failure_mode==45U && nav_mode==S5_APPROACH && !probe_at))waiting=true;
        if(waiting) {s->valid=0;s->sample_angle_deg=old_sample_angle;}
        pan_blocked[iteration]=waiting && nav_angle==0;
    }
    return E_OK;
}
ER s4_sensor_get_record(s4_sensor_cache_t *s)
{
    if(failure_mode==37U && iteration==SHOCK_AT+2U)return E_TMOUT; /* VC-local fault */
    static s4_sensor_cache_t cache;
    if(iteration==0U) {
        s4_data_reset(&cache);s4_data_init_result(&cache,LSM303_OK);
        if(failure_mode==36U)s4_cache_set_count(&cache,S4_COUNT_IMPACT,UINT32_MAX-1U);
        if(failure_mode==37U)s4_cache_set_count(&cache,S4_COUNT_FAULT,UINT32_MAX-1U);
    }
    lsm303_frame_t frame={0};frame.accel.fresh=frame.accel.valid=1;
    frame.accel.read_time_us=(uint64_t)(clock_ms-(failure_mode==3U?40U:0U))*1000U;
    /* Standing level on the floor with this robot's mounting (the
     * robot_config.h zeros: sensor -Y front, +X left, ~15 deg nose-up), so
     * terrain reads flat and the ramp handling stays idle. */
    frame.accel.raw[0]=-54;frame.accel.raw[1]=-250;frame.accel.raw[2]=909;
    if(scenario_nose_up(iteration)) {frame.accel.raw[1]=-404;frame.accel.raw[2]=852;} /* 10 deg more nose-up */
    frame.accel.overrun=failure_mode==5U;
    if(failure_mode==6U || (failure_mode==12U && iteration==SHOCK_AT))frame.accel.raw[0]=1900;
    if(failure_mode==21U && iteration==SHOCK_AT) { /* what the driver returns for a +2 g left hit */
        frame.accel.raw[0]=2047;frame.accel.valid=0;frame.accel.status=LSM303_DATA_CLIPPED;
    }
    if(failure_mode==17U && iteration>=HUMP_DOWN && iteration<HUMP_BOTTOM) {
        frame.accel.raw[1]=-88;frame.accel.raw[2]=948; /* 10 deg nose-down */
    }
    if(failure_mode==50U && iteration>=LOW_FROM && iteration<LOW_BOTTOM) {
        if(iteration<LOW_DOWN) {frame.accel.raw[1]=-325;frame.accel.raw[2]=885;} /* 4.8 deg more nose-up */
        else {frame.accel.raw[1]=-173;frame.accel.raw[2]=927;}                    /* 4.8 deg nose-down */
    }
    /* Drop the current height after completion, before ramp DONE (I3). */
    if(failure_mode==17U && iteration>=HUMP_BOTTOM+35U && iteration<HUMP_BOTTOM+40U)frame.accel.overrun=1;
    if(failure_mode==38U && iteration>=20U && iteration<100U) {
        frame.accel.raw[1]=-88;frame.accel.raw[2]=948;
        frame.accel.overrun=iteration>=60U && iteration<65U; /* traversal abandoned */
    }
    if((failure_mode==39U || failure_mode==40U) && iteration>=20U) {
        if(failure_mode==39U) {frame.accel.raw[0]=900;frame.accel.raw[2]=100;}
        else frame.accel.raw[2]=-909;
    }
    if(iteration==SHOCK_AT && failure_mode==36U) {
        lsm303_frame_t hit=frame;hit.accel.raw[0]=1900;
        s4_data_update(&cache,&hit,LSM303_OK);s4_data_update(&cache,&hit,LSM303_OK);
    }
    if(iteration==SHOCK_AT && failure_mode==37U) {
        lsm303_frame_t bad=frame;bad.accel.status=LSM303_DATA_IO_ERROR;bad.accel.fresh=0;
        for(unsigned n=0;n<3U;++n)s4_data_update(&cache,&bad,LSM303_IO_ERROR);
    }
    s4_data_update(&cache,&frame,LSM303_OK);*s=cache;return E_OK;
}

ER s1_command_take(control_message_t *c)
{
    if(failure_mode==0U && iteration==3U && map_step<3U) {
        static const uint8_t frames[3][6]={{0xC1,4,6,1,0,7},{0xC1,4,27,1,0,8},{0xC1,4,1,2,0,9}};
        *c=(control_message_t){.time_ms=map_step==2U?clock_ms-100U:clock_ms};
        for(unsigned i=0;i<6U;++i)c->wire[i]=frames[map_step][i];
        ++map_step;return E_OK;
    }
    if(failure_mode==4U && iteration==9U && !reset_offered) {
        *c=(control_message_t){.time_ms=clock_ms};c->wire[1]=2;
        reset_offered=true;return E_OK;
    }
    if(failure_mode==19U && iteration==AVOID_STOP_AT && reset_offered) {
        *c=(control_message_t){.time_ms=clock_ms};c->wire[1]=0; /* STOP */
        reset_offered=false;return E_OK;
    }
    if(failure_mode==29U && iteration==FUTURE_AT && !future_offered) {
        /* MAP stamped one tick after the VC read its clock: the network task
         * (priority 4) offered it while the VC (5) was running. */
        static const uint8_t frame[6]={0xC1,4,3,2,0,42};
        *c=(control_message_t){.time_ms=clock_ms+10U};
        for(unsigned i=0;i<6U;++i)c->wire[i]=frame[i];
        future_offered=true;return E_OK;
    }
    if(renew_mode() || failure_mode==47U) {
        unsigned opcode=0,power=30U;uint32_t stamp=clock_ms;
        bool renew_tick=iteration>=7U && (iteration-7U)%RENEW_EVERY==0U;
        if(iteration==renew_offered_at) {} /* one command per tick */
        else if(failure_mode==46U) {
            if(renew_tick && iteration<=RENEW_UNTIL)opcode=1;
            else if(iteration==RENEW_START_AT)opcode=3;
        } else if(failure_mode==48U) {
            if(renew_tick && iteration<=27U)opcode=1;
            if(iteration==17U)power=40U;
            if(iteration==22U)stamp=clock_ms-100U;
        } else if(failure_mode==49U) {
            if(iteration==HOLD_STOP1 || iteration==HOLD_STOP2)opcode=0x100U; /* STOP */
            else if(iteration==HOLD_RESET)opcode=2;
            else if((renew_tick && iteration<=47U) || iteration==HOLD_REARM)opcode=1;
        } else opcode=iteration==7U?3U:iteration==10U?1U:0U;
        if(!opcode)return E_TMOUT;
        renew_offered_at=iteration;
        /* ARM: 30%, 500 ms lease, as USB 'a' (usb_control.c). START: 30%. */
        *c=(control_message_t){.time_ms=stamp};
        c->wire[1]=(uint8_t)opcode;c->wire[2]=(uint8_t)power;c->wire[3]=0xF4;c->wire[4]=1;c->wire[5]=(uint8_t)iteration;
        return E_OK;
    }
    if(failure_mode>=7U && iteration==7U && !reset_offered) {
        /* START (mission, no lease) or ARM, both at 30%; ARM's lease is 1000 ms. */
        *c=(control_message_t){.time_ms=clock_ms};
        c->wire[1]=starts()?3:1;c->wire[2]=30;c->wire[3]=0xe8;c->wire[4]=3;
        reset_offered=true;return E_OK;
    }
    return E_TMOUT;
}
ER s3_controller_control(s3_mode_t mode,bool clear,UW lease)
{
    (void)lease;
    /* Mode 29: the first request finds S3's status lock busy (nothing changes). */
    if(failure_mode==29U && !control_busy) {++control_busy;return E_TMOUT;}
    if(mode==S3_MODE_TRACK && track_calls<16U) {track_tick[track_calls]=iteration;track_clear[track_calls]=clear;}
    if(mode==S3_MODE_TRACK)++track_calls;
    if(mode==S3_MODE_TRACK && clear && iteration>=ORPHAN_AT)orphan_cleared=true;
    if(real_s3())(void)s3_core_control(&s3c,mode,clear); /* S3 applies it before its next sample */
    return E_OK;
}
ER s3_controller_renew(UW lease)
{
    (void)lease;
    if((failure_mode==29U && iteration>=RENEW_BUSY_AT && iteration<RENEW_BUSY_AT+3U) ||
       (failure_mode==32U && iteration>=FAULT_AT))return E_TMOUT; /* lock busy past its wait */
    return E_OK;
}
ER s3_controller_barcode_end(void) { if(real_s3())s3_core_barcode_end(&s3c);return E_OK; }
ER s3_controller_set_map(UINT letter,UINT nav)
{
    ++map_calls;map_letter=letter;map_nav=nav;
    return letter>26U || nav>4U || (letter==0U && nav)?E_PAR:E_OK;
}
ER s5_navigation_set(uint32_t id,s5_state_t mode,INT angle)
{
    (void)id;
    if(obstacle_mode()) {
        if(angle!=nav_angle || (nav_mode==S5_MONITOR && iteration>=8U)) {
            old_sample_angle=nav_mode==S5_MONITOR?50:nav_angle;pan_at=iteration;
            if(angle && abs(angle)<90) {probe_at=iteration;probe_angle=(unsigned)abs(angle);}
            if(abs(angle)==90)++side_looks;
        }
        nav_mode=mode;nav_angle=angle;
    }
    if(s5_busy(iteration)) {++nav_set_busy;return E_TMOUT;} /* the same lock */
    if(failure_mode==20U && iteration>=S5_BUSY_UNTIL)++nav_set_after;
    return E_OK;
}
ER s3_event_receive(s3_event_record_t *event)
{
    if(failure_mode==18U && (iteration==DEC1_AT || iteration==DEC2_AT) && nav_given!=iteration) {
        nav_given=iteration;*event=(s3_event_record_t){0};
        event->event.kind=S3_EVENT_NAVIGATION;event->event.command=S3_NAV_STRAIGHT;
        event->event.symbol=iteration==DEC1_AT?'F':'B';event->event.time_ms=clock_ms;
        return E_OK;
    }
    if(failure_mode==35U && iteration==ORPHAN_AT && nav_given!=iteration) { /* Z: turn right */
        nav_given=iteration;*event=(s3_event_record_t){0};
        event->event.kind=S3_EVENT_NAVIGATION;event->event.command=S3_NAV_RIGHT;
        event->event.symbol='Z';event->event.time_ms=clock_ms;
        return E_OK;
    }
    if(failure_mode==23U && iteration==NAV_AT && nav_given!=iteration) { /* F: turn left */
        nav_given=iteration;*event=(s3_event_record_t){0};
        event->event.kind=S3_EVENT_NAVIGATION;event->event.command=S3_NAV_LEFT;
        event->event.symbol='F';event->event.time_ms=clock_ms;
        return E_OK;
    }
    if(real_s3() && s3_pending.kind!=S3_EVENT_NONE) {
        *event=(s3_event_record_t){0};event->event=s3_pending;s3_pending=(s3_event_t){0};
        return E_OK;
    }
    return E_TMOUT;
}
ER app_state_publish(const app_state_t *s)
{
    saved=*s;
    if((failure_mode==36U || failure_mode==37U) && iteration==SHOCK_AT) {
        s4_result_t r;assert(s4_result_decode(s->s4_result,sizeof(s->s4_result),&r));
        assert(failure_mode==36U?(r.flags&S4_RESULT_SHOCK)!=0U:(r.faults&EDR_FAULT_IMU)!=0U);
    }
    if(kinematic() && !lost_tick && iteration>7U && s->vehicle.phase==VC_REACQUIRE)lost_tick=iteration;
    if(iteration<LOG_TICKS) {
        pub_seen[iteration]=1;pub_follow[iteration]=s->line_follow;pub_mask[iteration]=s->line_mask;
        pub_steer[iteration]=s->line_steer;pub_phase[iteration]=s->vehicle.phase;pub_armed[iteration]=s->vehicle.armed;
        pub_entered[iteration]=s->vehicle.entered_ms;pub_armed_ms[iteration]=s->vehicle.armed_ms;
        pub_origin_um[iteration]=s->vehicle.origin_um;pub_origin_yaw[iteration]=s->vehicle.origin_yaw;
        pub_max_power[iteration]=(uint8_t)s->vehicle.maximum_power;pub_fault[iteration]=s->vehicle.fault;
    }
    s4_result_t r;
    if(shock_mode() && s4_result_decode(s->s4_result,sizeof(s->s4_result),&r)) {
        if(r.flags&S4_RESULT_SHOCK)++shock_published;
        if(iteration==SHOCK_AT) {shock_flags=r.flags;shock_faults=r.faults;}
    }
    return E_OK;
}
ID app_start_task(FP task,PRI priority,UW core,const char *name)
{ (void)task;(void)priority;(void)core;(void)name;return 1; }
ID app_start_task_stack(FP task,PRI priority,UW core,const char *name,INT stack_bytes)
{ assert(stack_bytes==VC_STACK_SIZE);return app_start_task(task,priority,core,name); }
static unsigned codes_seen; /* line_follow values published across the modes */
static unsigned shock_failures; /* modes 12 and 21, reported at the end */
/* (iii) Every tick publishes the follower request that actually reached S2:
 * 1/2 with the steer for a steered follow, 3/4 for a pivot right/left, 0
 * otherwise (straight follow, STOP, reverse, spin), plus the observed mask. */
static void check_published(void)
{
    for(unsigned n=0;n<iterations();++n) {
        unsigned follow=0U;int32_t steer=0;
        if(pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && sent_steer[n]) {follow=sent_steer[n]>0?1U:2U;steer=sent_steer[n];}
        else if(sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_RIGHT)follow=3U;
        else if(sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_LEFT)follow=4U;
        assert(pub_seen[n] && pub_follow[n]==follow && pub_steer[n]==steer);
        assert(pub_mask[n]==scenario_mask(n));
        codes_seen|=1U<<pub_follow[n];
    }
}
/* (i) Mode 13: while S3 reads a marker the follower is off, but a single
 * straddle sensor on black steers toward it: right (mask 1) positive, left
 * (mask 2) negative, capped at BOARD_BARCODE_READ_LINE_STEER and never a
 * pivot; both or neither on black is straight. (The Makefile's _read150
 * build sets the cap below BOARD_LINE_STEER_MIN.) */
static void check_read_steer(void)
{
    const int32_t cap=BOARD_BARCODE_READ_LINE_STEER;
    assert(cap>=0); /* 0 (the default): straight on while reading */
    unsigned right=0,left=0,straight=0;
    for(unsigned n=8U;n<SR_ITERATIONS;++n) {
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW);
        int32_t steer=sent_steer[n];
        if(!scenario_reading(n)) {assert(steer==0 && sent_power[n]==30U);continue;}
#if BOARD_BARCODE_RETRY_DRIVE && BOARD_BARCODE_SLOW_WHILE_READING
        assert(sent_power[n]<30U); /* the marker is read at the slow speed */
#endif
        switch(scenario_mask(n)) {
        case 1U: if(cap){assert(steer>0 && steer<=cap);}else assert(steer==0);++right;break;
        case 2U: if(cap){assert(steer<0 && steer>=-cap);}else assert(steer==0);++left;break;
        default: assert(steer==0);++straight;break;
        }
    }
    assert(right==5U && left==3U && straight==4U);
}
/* (ii) Mode 14: the follower's request every tick, a TRACK STOP inside a
 * touch, and the resume. */
static void check_follower(void)
{
    for(unsigned n=8U;n<iterations();++n) {
        const follow_step_t *f=follow_step(n);
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
        if(!f) {assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW && sent_steer[n]==0 && sent_power[n]==30U);continue;}
        if(f->stop) {assert(stopped[n] && sent_kind[n]==SENT_NONE && pub_mask[n]==f->mask);continue;}
        assert(!stopped[n] && sent_kind[n]==f->kind && sent_cmd[n]==f->cmd);
        assert(sent_power[n]==f->power && sent_steer[n]==f->steer);
        assert(pub_follow[n]==f->follow && pub_mask[n]==f->mask);
    }
}
/* (iv) Modes 22..24: a silent S3 latch while tracking. The START sends
 * TRACK at tick 7 (marker clear); the VC then re-sends it for the latch. */
static bool reversing(unsigned n)
{
    return sent_kind[n]==SENT_REVERSE || (sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_REVERSE);
}
/* Modes 22, 23 tick n: following until the latch; with LATCH_READ_FAILS the
 * cut-short marker's re-read then brakes (stopped) and reverses. */
static void check_latch_drive(unsigned n)
{
    if(!LATCH_READ_FAILS || n<LATCH_AT)assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW);
    else if(n<LATCH_REARM_AT)assert(stopped[n] && sent_kind[n]==SENT_NONE);
    else assert(!stopped[n] && reversing(n));
}
static void check_latch(void)
{
    unsigned from=failure_mode==24U?REVERSE_AT:LATCH_REARM_AT;
    assert(track_calls>=2U && track_tick[0]==7U && track_clear[0]);
    if(failure_mode==22U) {
        /* Once, clearing the marker (at once, or as the cut marker's re-read
         * starts reversing): S3 re-arms and the next *B* decodes; the robot
         * never left TRACK. */
        assert(track_calls==2U && track_tick[1]==LATCH_REARM_AT && track_clear[1]);
        assert(s3c.barcode_ok_count==1U && s3c.latched_error==S3_ERROR_NONE && saved.barcode==2U);
        assert(s3c.barcode_fail_count==LATCH_READ_FAILS && s3c.barcode_abort_count==LATCH_READ_FAILS);
        for(unsigned n=8U;n<iterations();++n) {
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
            if(n<MARKER2_READ_AT)check_latch_drive(n);
        }
        assert(saved.vehicle.armed && saved.vehicle.fault==VC_OK);
        return;
    }
    /* From the first tracking tick that sees the latch (mode 24, and mode 23
     * with LATCH_READ_FAILS: not while the re-read brakes), every
     * VC_S3_REARM_MS, holding mode 23's pending turn (no marker clear);
     * VC_S3_LATCH_MS after it, VC_LINE cause event. */
    assert(track_calls==1U+LATCH_TICKS/REARM_TICKS);
    for(unsigned k=1U;k<track_calls;++k)
        assert(track_tick[k]==from+(k-1U)*REARM_TICKS && track_clear[k]==(failure_mode==24U));
    for(unsigned n=8U;n<from+LATCH_TICKS;++n) {
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
        if(failure_mode==23U)check_latch_drive(n);
    }
    assert(!pub_armed[from+LATCH_TICKS] && pub_phase[from+LATCH_TICKS]==VC_FAULT);
    assert(saved.vehicle.fault==VC_LINE && saved.vehicle.fault_cause==VC_LINE_CAUSE_EVENT);
}
/* (v) Modes 25..27: the line lost while tracking. The search starts on the
 * first tick BOARD_LINE_LOST_MM of odometry past the last straddle black (25,
 * 26: LL_TOUCH_AT) or past the barcode re-read's last tick (27). */
static void check_line_lost(void)
{
#if BOARD_LINE_MASK_PIVOT
    unsigned from=LL_TOUCH_AT,due=0;
#if BOARD_BARCODE_RETRY_DRIVE
    if(failure_mode==27U) {
        /* The re-read brakes, reverses, then passes slowly (30% of 30) for
         * longer than BOARD_LINE_LOST_MM: none of it counts. */
        unsigned slow=30U*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U,reverse=0;int32_t back=kin_mm[BR_MISS_AT];
        for(unsigned n=BR_MISS_AT;n<iterations();++n) {
            if(sent_kind[n]==SENT_REVERSE)++reverse;
            if(kin_mm[n]<back)back=kin_mm[n];
            if(sent_kind[n]==SENT_FOLLOW && sent_power[n]==slow)from=n;
        }
        assert(reverse>0U && from>BR_MISS_AT && kin_mm[from]-back>(int32_t)BOARD_LINE_LOST_MM);
    }
#endif
    if(failure_mode==28U) { /* driven on through the 700 mm read, the follower off */
        from=MR_UNTIL-1U;
        assert(kin_mm[from]-kin_mm[MR_FROM]>(int32_t)BOARD_LINE_LOST_MM);
        for(unsigned n=MR_FROM;n<MR_UNTIL;++n)assert(sent_kind[n]==SENT_FOLLOW && !sent_steer[n]);
    }
    for(unsigned n=from+1U;n<iterations() && !due;++n) if(kin_mm[n]-kin_mm[from]>=(int32_t)BOARD_LINE_LOST_MM)due=n;
    assert(due && lost_tick==due);
    for(unsigned n=9U;n<due;++n)assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
    /* Stop, then spin slowly toward the line's side first: right after the
     * right IR's touch, left when no single IR has touched (modes 27, 28). */
    assert(pub_phase[due]==VC_REACQUIRE && stopped[due]);
    assert(sent_kind[due+1U]==SENT_MOVE && sent_power[due+1U]==15U &&
           sent_cmd[due+1U]==(failure_mode>=27U?S2_MOVE_LEFT:S2_MOVE_RIGHT));
    if(failure_mode==25U) {
        /* The right IR on black: the spin settles one tick, then tracking
         * resumes and the follower pivots toward it. */
        unsigned found=due+LL_FOUND_AFTER;bool pivot=false;
        for(unsigned n=due+1U;n<=found;++n)
            assert(pub_phase[n]==VC_REACQUIRE && sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_RIGHT && sent_power[n]==15U);
        for(unsigned n=found+1U;n<iterations();++n) {
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
            pivot=pivot || (sent_kind[n]==SENT_MOVE && sent_cmd[n]==S2_MOVE_PIVOT_RIGHT);
        }
        assert(pivot && saved.vehicle.fault==VC_OK);
    } else if(failure_mode==26U) {
        /* Never found: the bounded sweep both ways at 15%, then fault 2 cause 3. */
        unsigned fault=0,left=0,right=0;
        for(unsigned n=due+1U;n<iterations() && !fault;++n) {
            if(!pub_armed[n]) {fault=n;break;}
            assert(pub_phase[n]==VC_REACQUIRE);
            if(stopped[n])continue;
            assert(sent_kind[n]==SENT_MOVE && sent_power[n]==15U &&
                   (sent_cmd[n]==S2_MOVE_LEFT || sent_cmd[n]==S2_MOVE_RIGHT));
            if(sent_cmd[n]==S2_MOVE_LEFT)++left; else ++right;
        }
        assert(fault && fault-due<=4000U/TICK_MS+1U && pub_phase[fault]==VC_FAULT && left && right);
        assert(saved.vehicle.fault==VC_LINE && saved.vehicle.fault_cause==VC_LINE_CAUSE_NOT_FOUND);
    }
#else
    /* Edge tracing has no line-lost distance (its IR rides the edge, below
     * black): never a search. A lost line is its own full turn with both IRs
     * white (VC_LINE, cause other); the barcode modes track on. */
    assert(!lost_tick);
    if(failure_mode>=27U) for(unsigned n=9U;n<iterations();++n)assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
    else assert(!saved.vehicle.armed && saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_LINE &&
                saved.vehicle.fault_cause==VC_LINE_CAUSE_OTHER);
#endif
}
/* (vi) Modes 29..33: short timing faults ride through, lasting ones stop. */
static void check_timing_faults(void)
{
#if VC_TIMING
    const task_timing_t *vc=&app_task_timing[TASK_TIMING_VC];
#endif
    if(failure_mode==29U) {
        /* Each odometry gap (a failed encoder copy, a 70 ms period) holds
         * still for its two invalid ticks; nothing else stops the run. */
        for(unsigned n=10U;n<iterations();++n) {
            bool hold=n==ENC_FAIL_AT || n==ENC_FAIL_AT+1U || n==LATE_AT || n==LATE_AT+1U;
            assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
            if(hold)assert(stopped[n] && sent_kind[n]==SENT_NONE);
            else assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW && sent_power[n]==30U);
        }
        assert(saved.vehicle.armed && saved.vehicle.fault==VC_OK && saved.odometry_valid);
        /* The busy first mode request was retried next tick. */
        assert(control_busy==1U && track_calls==1U && track_tick[0]==8U);
        /* The MAP frame stamped 10 ms ahead was applied (result 0, not 3 expired). */
        assert(future_offered && last_ack_seq==42U && last_ack_opcode==4U && last_ack_result==0U &&
               map_letter==3U && map_nav==2U);
#if VC_TIMING
        /* Loop timing: every run 250 us; one release 70 ms after the last, late. */
        assert(vc->runs==iterations() && vc->late==1U && vc->max_run_us==250U &&
               vc->max_gap_us==(TICK_MS+LATE_EXTRA_MS)*1000U);
        unsigned v[12];
        assert(timing_lines==2U && sscanf(timing_line,
            "[TIMING] max run us, max gap ms, late/runs: vc %u %u %u/%u, motor %u %u %u/%u, net %u %u %u/%u",
            &v[0],&v[1],&v[2],&v[3],&v[4],&v[5],&v[6],&v[7],&v[8],&v[9],&v[10],&v[11])==12);
        assert(v[0]==250U && v[1]==TICK_MS+LATE_EXTRA_MS && v[2]==1U && v[3]==99U); /* report at tick 98 */
        for(unsigned k=4U;k<12U;++k)assert(v[k]==0U); /* no motor or network loop on the host */
#else
        assert(!timing_lines); /* BOARD_TIMING_STATS 0: no hooks, no report */
#endif
        return;
    }
    /* 30..33: driving until FAULT_AT, then the fault after its allowance. */
    unsigned at=failure_mode==30U?FAULT_AT+VC_ODOM_GAP_MS/TICK_MS:
        failure_mode==31U?FAULT_AT+VC_MOTOR_STATUS_MISSES:
        failure_mode==32U?FAULT_AT+VC_S3_LEASE_MS/TICK_MS-1U:FAULT_AT;
    unsigned fault=failure_mode==30U?VC_SENSOR:failure_mode==32U?VC_TRANSPORT:VC_MOTOR;
    for(unsigned n=10U;n<at;++n) {
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
        /* Odometry missing: hold still. A reused motor status or a busy
         * renewal: drive on. */
        if(failure_mode==30U && n>=FAULT_AT)assert(stopped[n] && sent_kind[n]==SENT_NONE);
        else assert(!stopped[n] && sent_kind[n]==SENT_FOLLOW);
    }
    assert(!pub_armed[at] && pub_phase[at]==VC_FAULT && stopped[at] && sent_kind[at]==SENT_NONE);
    assert(saved.vehicle.fault==fault && saved.vehicle.fault_cause==(failure_mode==30U?VC_CAUSE_ODOM:0U));
#if VC_TIMING
    assert(vc->late==0U && vc->max_gap_us==TICK_MS*1000U);
#endif
}
/* (vii) Modes 34, 35: S3 holding a decoded marker. */
static void check_marker_hold(void)
{
    for(unsigned n=10U;n<iterations();++n)
        assert(pub_armed[n] && pub_phase[n]==VC_TRACK && !stopped[n] && sent_kind[n]==SENT_FOLLOW);
    assert(saved.vehicle.armed && saved.vehicle.fault==VC_OK && track_calls==2U && track_tick[0]==7U);
    if(failure_mode==34U) {
        /* No turn here: TRACK with marker clear VC_S3_REARM_MS later, once,
         * with one console line; S3 then reads markers again. */
        assert(track_tick[1]==ORPHAN_AT+VC_S3_REARM_MS/TICK_MS && track_clear[1] && orphan_cleared);
        assert(orphan_lines==1U && !saved.vehicle.pending_nav);
    } else {
        /* The turn delivered with the latch stays pending; the latch's TRACK
         * holds its marker; no orphan re-send while it is pending. */
        assert(track_tick[1]==ORPHAN_AT && !track_clear[1] && !orphan_cleared);
        assert(orphan_lines==0U && saved.vehicle.pending_nav==S3_NAV_RIGHT);
    }
}
int main(void)
{
    for(failure_mode=0;failure_mode<MODE_COUNT;++failure_mode) {
        memset(sent_kind,0,sizeof(sent_kind));memset(sent_cmd,0,sizeof(sent_cmd));
        memset(sent_power,0,sizeof(sent_power));memset(sent_steer,0,sizeof(sent_steer));
        memset(stopped,0,sizeof(stopped));memset(pub_seen,0,sizeof(pub_seen));
        assert(iterations()<=LOG_TICKS);
        iteration=stops=positive_requests=0;clock_ms=100;reset_offered=false;
        shock_published=after_shock=0;shock_flags=shock_faults=0;event_count=0;nav_given=0;
        reverse_requests=reverse_after_glitch=left_track=0;saved=(app_state_t){0};
        reverse_tick=after_reverse=0;nav_set_busy=nav_set_after=0;
        read_count=read_max=normal_count=0;normal_min=~0U;
        track_calls=0;s3_pending=(s3_event_t){0};
        lost_tick=0;kin_counts[0]=kin_counts[1]=1000U;memset(kin_mm,0,sizeof(kin_mm));
        control_busy=0;future_offered=orphan_cleared=false;timing_lines=orphan_lines=0;
        last_ack_seq=last_ack_opcode=last_ack_result=0;
        arm_acks=0;renew_offered_at=~0U;
#if VC_TIMING
        memset(app_task_timing,0,sizeof(app_task_timing));
#endif
        nav_mode=S5_MONITOR;nav_angle=old_sample_angle=0;pan_at=probe_at=probe_angle=side_looks=0;
        memset(pan_blocked,0,sizeof(pan_blocked));
        timer_tick_ms=~0U;
        if(real_s3()) { /* configured as app_main.c configures S3 */
            s3_config_t c;s3_config_defaults(&c);robot_ir_apply_layout(&c);
            assert(robot_ir_apply_adc_calibration(&c) && s3_core_init(&s3c,&c));
        }
        if(setjmp(done)==0)vehicle_task(0,NULL);
        s4_result_t result={0};assert(s4_result_decode(saved.s4_result,sizeof(saved.s4_result),&result));
        if(failure_mode==0)assert((result.flags&S4_RESULT_TILT) && saved.odometry_valid);
        /* MAP: accepted while disarmed, invalid rejected, stale never applied. */
        if(failure_mode==0)assert(acks==3U && map_calls==2U && map_letter==27U && map_nav==1U &&
            ack_log[0][0]==7U && ack_log[0][2]==0U && ack_log[1][0]==8U && ack_log[1][2]==1U &&
            ack_log[2][0]==9U && ack_log[2][2]==3U && !saved.vehicle.armed);
        if(failure_mode==46U) {
            /* Every ARM is ACK 0 (the first arms, the rest renew the lease);
             * the START during the leased run is refused (ACK 1). */
            assert(arm_acks==(RENEW_UNTIL-7U)/RENEW_EVERY+2U);
            for(unsigned k=0;k<arm_acks;++k)
                assert(arm_ack_op[k]==1U?arm_ack_result[k]==0U:
                       arm_ack_seq[k]==RENEW_START_AT && arm_ack_result[k]==1U);
            /* Armed and tracking long past the first lease (tick 7+25),
             * until one lease after the last renewal: then fault 5. */
            for(unsigned n=8U;n<RENEW_UNTIL+RENEW_LEASE_TICKS;++n)
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK);
            for(unsigned n=RENEW_UNTIL+RENEW_LEASE_TICKS;n<iterations();++n)
                assert(!pub_armed[n] && pub_phase[n]==VC_FAULT);
            assert(saved.vehicle.fault==VC_TIMEOUT);
            /* A renewal only moves the lease start: the phase timer, the
             * origin and the demand stay those of the first ARM. */
            for(unsigned n=9U;n<RENEW_UNTIL+RENEW_LEASE_TICKS;++n) {
                assert(pub_entered[n]==pub_entered[8] && pub_origin_um[n]==pub_origin_um[8] &&
                       pub_origin_yaw[n]==pub_origin_yaw[8] && pub_max_power[n]==30U);
                bool renewed=n<=RENEW_UNTIL && (n-7U)%RENEW_EVERY==0U;
                assert(renewed?pub_armed_ms[n]>pub_armed_ms[n-1U]:pub_armed_ms[n]==pub_armed_ms[n-1U]);
            }
        }
        if(failure_mode==48U) {
            /* Renewals at 7 and 12 taken; another demand refused (ACK 1),
             * a 100 ms old one refused (ACK 3); the one at 27 is taken
             * though S3's status was missing (no fault 6). */
            static const unsigned seq[5]={7U,12U,17U,22U,27U},res[5]={0U,0U,1U,3U,0U};
            assert(arm_acks==5U);
            for(unsigned k=0;k<5U;++k)assert(arm_ack_op[k]==1U && arm_ack_seq[k]==seq[k] && arm_ack_result[k]==res[k]);
            for(unsigned n=8U;n<27U+RENEW_LEASE_TICKS;++n)
                assert(pub_armed[n] && pub_max_power[n]==30U && pub_fault[n]==VC_OK);
            assert(pub_armed_ms[17]==pub_armed_ms[16] && pub_armed_ms[22]==pub_armed_ms[21]);
            for(unsigned n=27U+RENEW_LEASE_TICKS;n<iterations();++n)
                assert(!pub_armed[n] && pub_phase[n]==VC_FAULT);
            assert(saved.vehicle.fault==VC_TIMEOUT);
        }
        if(failure_mode==49U) {
            /* STOP ends the renewed run and holds ARM off (ACK 1) while the
             * client keeps sending it; RESET lets ARM in again; after the
             * second STOP the hold ends only after 1 s with no ARM. */
            static const unsigned seq[13]={7U,12U,17U,22U,HOLD_STOP1,27U,HOLD_RESET,32U,37U,HOLD_STOP2,42U,47U,HOLD_REARM};
            static const unsigned op[13]={1U,1U,1U,1U,0U,1U,2U,1U,1U,0U,1U,1U,1U};
            static const unsigned res[13]={0U,0U,0U,0U,0U,1U,0U,0U,0U,0U,1U,1U,0U};
            assert(arm_acks==13U);
            for(unsigned k=0;k<13U;++k)
                assert(arm_ack_seq[k]==seq[k] && arm_ack_op[k]==op[k] && arm_ack_result[k]==res[k]);
            for(unsigned n=8U;n<iterations();++n) {
                bool armed=(n<HOLD_STOP1) || (n>=32U && n<HOLD_STOP2) ||
                    (n>=HOLD_REARM && n<HOLD_REARM+RENEW_LEASE_TICKS);
                assert(pub_armed[n]==(armed?1U:0U));
                if(!armed && n<HOLD_REARM)assert(pub_phase[n]==VC_DISARMED && pub_fault[n]==VC_OK);
            }
            assert(saved.vehicle.fault==VC_TIMEOUT);
        }
        if(failure_mode==47U) {
            /* ARM during a START run is refused; the run keeps no lease. */
            assert(arm_acks==2U && arm_ack_op[0]==3U && arm_ack_result[0]==0U &&
                   arm_ack_op[1]==1U && arm_ack_seq[1]==10U && arm_ack_result[1]==1U);
            assert(saved.vehicle.armed && saved.vehicle.lease_ms==0U && saved.vehicle.fault==VC_OK);
        }
        if(failure_mode==1 || failure_mode==2)
            assert(!(result.flags&S4_RESULT_TILT) && !saved.odometry_valid &&
                   (result.faults&EDR_FAULT_ENCODER));
        if(failure_mode==3)assert(!(result.flags&S4_RESULT_TILT) && saved.odometry_valid &&
                                  (result.faults&EDR_FAULT_IMU));
        if(failure_mode==4)assert(reset_offered && !result.maximum_mm);
        if(failure_mode==5)assert(!(result.flags&S4_RESULT_TILT) && saved.odometry_valid &&
                                  (result.faults&EDR_FAULT_IMU));
        /* An unclipped impact is reported as a shock, not as an IMU fault. */
        if(failure_mode==6)assert((result.flags&S4_RESULT_SHOCK) &&
                                  !(result.faults&EDR_FAULT_IMU) && !(result.flags&S4_RESULT_TILT));
        assert(stops);
        if(failure_mode>=9U && failure_mode!=19U && failure_mode!=45U)assert(positive_requests>0);
        else assert(!positive_requests);
        if(failure_mode==7U) {
            /* No usable sonar sample: tracking holds still (armed, stopped)
             * for VC_SONAR_GAP_MS, then a sonar sensor fault. */
            for(unsigned n=SONAR_GAP_AT;n<SONAR_GAP_AT+SONAR_GAP_TICKS;++n)
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE);
            assert(!pub_armed[SONAR_GAP_AT+SONAR_GAP_TICKS] && pub_phase[SONAR_GAP_AT+SONAR_GAP_TICKS]==VC_FAULT);
            assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SENSOR &&
                   saved.vehicle.fault_cause==VC_CAUSE_SONAR);
        }
        if(shock_mode()) {
            /* The shock is always detected and published (dashboard impact),
             * and counted once so telemetry sends it even after it clears.
             * A clipped sample (mode 21) is the same impact, and it is also
             * an IMU fault; an unclipped one (mode 12) is not. Neither ever
             * gives tilt. Checked without aborting, so the later modes still run. */
            bool clipped=failure_mode==21U;
            bool ok=shock_published==1U && saved.shock_count==1U && (shock_flags&S4_RESULT_SHOCK) &&
                !(shock_flags&S4_RESULT_TILT) && ((shock_faults&EDR_FAULT_IMU)!=0U)==clipped;
#if BOARD_SHOCK_STOP
            ok=ok && !saved.vehicle.armed && saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_SHOCK &&
                !after_shock;
#else
            /* Not a stop: tracking continues, and the one-sample tilt gap is
             * inside the grace, so no VC_SENSOR either. */
            ok=ok && saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && saved.vehicle.fault==VC_OK &&
                after_shock>0U;
#endif
            if(!ok) {
                ++shock_failures;
                printf("FAIL %s shock while tracking: published %u, counted %u, flags %u, faults %u, armed %u, phase %u, fault %u, driving after it %u\n",
                       clipped?"clipped":"unclipped",shock_published,(unsigned)saved.shock_count,
                       (unsigned)shock_flags,(unsigned)shock_faults,(unsigned)saved.vehicle.armed,
                       (unsigned)saved.vehicle.phase,(unsigned)saved.vehicle.fault,after_shock);
            }
        }
#if BOARD_BARCODE_RETRY_DRIVE
        /* The glitch arrives while reversing: TRACK holds and the reverse continues. */
        if(failure_mode==10U)assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK && !left_track &&
            reverse_requests>reverse_after_glitch && reverse_after_glitch>=BR_ITERATIONS-BR_GLITCH_AT-2U);
#endif
        /* Nothing overrides a reverse request in its tick: no forward request
         * (the policy's TRACK command) and no STOP after it. */
        assert(!after_reverse);
        /* Event log: one EV_BARCODE for the failed first read (attempt 1, no
         * letter), one EV_OBSTACLE with its distance when an obstacle is
         * acquired (mode 8: S5 already READY at 100 mm); none else (a lost
         * line, modes 25 and 26, posts nothing). */
        if(failure_mode==8U)
            assert(event_count==1U && event_id[0]==EV_OBSTACLE && event_tick[0]==8U &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,10U) && saved.vehicle.phase==VC_ACQUIRE);
        else if(failure_mode==10U || failure_mode==27U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,0U,0U) &&
                   event_tick[0]==BR_MISS_AT);
        else if(failure_mode==15U) {
            /* S5 published the third echo as it really does (SCAN_LEFT,
             * turning to 20 deg, invalid); the event has its forward range. */
            assert(oc_status.state==S5_SCAN_LEFT && !oc_status.valid && oc_status.angle_deg==S5_SCAN_STEP_DEG &&
                   oc_status.sample_angle_deg==0 && oc_status.obstacle_distance_mm==150U);
            assert(event_count==1U && event_id[0]==EV_OBSTACLE && event_tick[0]==OC_ACQUIRE_AT &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,15U) && pub_phase[event_tick[0]]==VC_ACQUIRE &&
                   pub_phase[event_tick[0]-1U]==VC_TRACK);
        }
        else if(failure_mode==19U)
            /* Acquired at tick 8, then STOP: the avoidance ended (how 0, after 0 s). */
            assert(event_count==2U && event_id[0]==EV_OBSTACLE && event_tick[0]==8U &&
                   event_value[0]==ev_obstacle(EV_OBSTACLE_DETECTED,0U,10U) &&
                   event_id[1]==EV_OBSTACLE && event_tick[1]==AVOID_STOP_AT &&
                   event_value[1]==ev_obstacle(EV_OBSTACLE_BACK,0U,0U) && !saved.vehicle.armed);
        else if(failure_mode==18U) {
            /* Each attempt (1: the normal pass, 29 elements, F/B) before its instruction. */
            unsigned n=0;
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK);
#if BOARD_BARCODE_RETRY_DRIVE
            assert(event_count==4U);
            assert(event_id[n]==EV_BARCODE && event_value[n]==ev_barcode(1U,29U,6U) && event_tick[n]==DEC1_AT+1U);++n;
#else
            assert(event_count==2U);
#endif
            assert(event_id[n]==EV_INSTRUCTION && event_value[n]==ev_instruction(6U,S3_NAV_STRAIGHT) &&
                   event_tick[n]==DEC1_AT+1U);++n;
#if BOARD_BARCODE_RETRY_DRIVE
            assert(event_id[n]==EV_BARCODE && event_value[n]==ev_barcode(1U,29U,2U) && event_tick[n]==DEC2_AT);++n;
#endif
            assert(event_id[n]==EV_INSTRUCTION && event_value[n]==ev_instruction(2U,S3_NAV_STRAIGHT) &&
                   event_tick[n]==DEC2_AT);
        }
        else if(failure_mode==23U) {
            /* The pending turn's instruction, then (LATCH_READ_FAILS) the
             * *B* cut short at 5 elements: a failed first read. */
            assert(event_count==1U+LATCH_READ_FAILS && event_id[0]==EV_INSTRUCTION &&
                   event_value[0]==ev_instruction(6U,S3_NAV_LEFT));
            if(LATCH_READ_FAILS)
                assert(event_id[1]==EV_BARCODE && event_value[1]==ev_barcode(1U,5U,0U) && event_tick[1]==LATCH_AT);
        }
        else if(failure_mode==35U) /* the delivered turn is logged: Z, right */
            assert(event_count==1U && event_id[0]==EV_INSTRUCTION && event_value[0]==ev_instruction(26U,S3_NAV_RIGHT) &&
                   event_tick[0]==ORPHAN_AT+VC_INSTRUCTION_WAIT_TICKS-1U);
#if BOARD_BARCODE_RETRY_DRIVE
        /* Mode 22: with LATCH_READ_FAILS the first *B*, cut short at 5
         * elements, is a failed first read, and the second is read during
         * the re-read (attempt 2); otherwise only the second marker is read
         * (the normal pass). Each 29 elements, B, 30 mm after its last edge.
         * Mode 24: the miss. */
        else if(failure_mode==22U) {
            assert(event_count==1U+LATCH_READ_FAILS);
            if(LATCH_READ_FAILS)
                assert(event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,5U,0U) && event_tick[0]==LATCH_AT);
            assert(event_id[LATCH_READ_FAILS]==EV_BARCODE &&
                   event_value[LATCH_READ_FAILS]==ev_barcode(1U+LATCH_READ_FAILS,29U,2U) &&
                   event_tick[LATCH_READ_FAILS]==MARKER2_READ_AT);
        }
        else if(failure_mode==24U)
            assert(event_count==1U && event_id[0]==EV_BARCODE && event_value[0]==ev_barcode(1U,0U,0U) &&
                   event_tick[0]==BR_MISS_AT);
#endif
#if BOARD_RAMP_SPEED
        else if(failure_mode==17U) {
            assert(event_count==3U && event_id[0]==EV_HUMP && event_id[1]==EV_HUMP && event_id[2]==EV_HUMP);
            assert((event_value[0]&7U)==EV_HUMP_DETECTED && (event_value[1]&7U)==EV_HUMP_CREST &&
                   (event_value[2]&7U)==EV_HUMP_LEVEL);
            /* Detected only after 400 ms of tilt; a ~10 deg climb is reported in 0.1 deg. */
            assert(event_tick[0]>=HUMP_FROM+BOARD_RAMP_HOLD_MS/TICK_MS && event_tick[0]<HUMP_LEVEL);
            assert((event_value[0]>>3U)>=(unsigned)BOARD_RAMP_TILT_CDEG/10U && (event_value[0]>>3U)<=100U);
            assert(event_tick[1]>=HUMP_LEVEL && event_tick[2]>event_tick[1]+BOARD_RAMP_LEVEL_MS/TICK_MS &&
                   event_tick[2]<HUMP_ITERATIONS);
            /* Heights in 2 mm, the controller's estimate (the robot climbs
             * 10 deg at 10 mm per tick, so 104 mm by HUMP_LEVEL): the crest
             * its height so far, the level its highest point. */
            unsigned crest=event_value[1]>>3U,level=event_value[2]>>3U;
            assert(crest>=40U && crest<=level && level>=48U && level<=53U);
            assert(event_tick[2]>HUMP_BOTTOM); /* no DONE on the flat top */
        }
        else if(failure_mode==50U) {
            /* One hump: detected on the way up, crest at the top (the
             * controller's height so far), level only after the way down
             * with its highest point, 33.5 mm. */
            assert(event_count==3U && event_id[0]==EV_HUMP && event_id[1]==EV_HUMP && event_id[2]==EV_HUMP);
            assert((event_value[0]&7U)==EV_HUMP_DETECTED && (event_value[1]&7U)==EV_HUMP_CREST_EST &&
                   (event_value[2]&7U)==EV_HUMP_LEVEL_EST);
            assert(event_tick[0]>LOW_FROM+BOARD_RAMP_HOLD_MS/TICK_MS && event_tick[0]<LOW_DOWN);
            assert(event_tick[1]>=LOW_DOWN && event_tick[1]<LOW_DOWN+10U);
            assert(event_tick[2]>LOW_BOTTOM && event_tick[2]<LOW_ITERATIONS);
            unsigned crest=event_value[1]>>3U,level=event_value[2]>>3U;
            assert(crest>=13U && crest<=level && level>=15U && level<=17U);
        }
#endif
        else if(failure_mode==38U) {
#if BOARD_RAMP_SPEED
            assert(event_count==3U && (event_value[2]&7U)==EV_HUMP_LEVEL && (event_value[2]>>3U)==0U);
#else
            assert(event_count==0U);
#endif
        }
        else if(obstacle_mode()) {
            unsigned bypasses=0;
            for(unsigned e=0;e<event_count && e<EVENT_LOG;++e)if(event_id[e]==EV_OBSTACLE && (event_value[e]&3U)==EV_OBSTACLE_BYPASS) {
                ++bypasses;
                unsigned heading=event_value[e]>>4U;
                assert(heading==((failure_mode==41U || failure_mode==44U)?probe_angle:50U));
            }
            assert(bypasses==(failure_mode==45U?0U:1U));
        }
        else assert(event_count==0U);
        if(failure_mode==11U) {
            unsigned slow=30U;
#if BOARD_BARCODE_RETRY_DRIVE && BOARD_BARCODE_SLOW_WHILE_READING
            /* A marker started at full speed is read on at the re-read speed. */
            slow=30U*BOARD_BARCODE_RETRY_SPEED_PERCENT/100U;
            if(slow<BOARD_BARCODE_RETRY_MIN_POWER)slow=BOARD_BARCODE_RETRY_MIN_POWER;
            assert(slow<30U);
#endif
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK);
            assert(read_count==READ_UNTIL-READ_SLOW_FROM && read_max==slow);
            assert(normal_count>=6U && normal_min==30U); /* full speed before and after */
        }
        if(failure_mode==20U)
            /* No request on the busy lock (no VC_TRANSPORT); sent once a read succeeds. */
            assert(!nav_set_busy && nav_set_after>0U && saved.vehicle.armed &&
                   saved.vehicle.phase==VC_TRACK && saved.vehicle.fault==VC_OK);
        if(failure_mode>=7U)check_published();
        if(failure_mode==13U)check_read_steer();
        if(failure_mode==14U)check_follower();
        if(failure_mode>=22U && failure_mode<=24U)check_latch();
        if(kinematic() && !obstacle_mode())check_line_lost();
        if(timing_mode())check_timing_faults();
        if(failure_mode==34U || failure_mode==35U)check_marker_hold();
        if(failure_mode==15U) {
            _Static_assert(sizeof(oc_expect)==sizeof(oc_sample),"mode 15 table");
            for(unsigned n=10U;n<OC_ITERATIONS;++n) {
                char e=n<OC_FROM?'d':oc_expect[n-OC_FROM];
                assert(pub_armed[n]);
                if(e=='a')assert(pub_phase[n]==VC_ACQUIRE);
                else if(e=='s')assert(pub_phase[n]==VC_TRACK && stopped[n] && sent_kind[n]==SENT_NONE);
                else assert(pub_phase[n]==VC_TRACK && !stopped[n] && sent_kind[n]==SENT_FOLLOW);
            }
        }
        if(failure_mode==17U) {
            /* The policy caps a slope (S4 terrain ascent/descent) at 20%;
             * BOARD_RAMP_UP_POWER 0 adds no cap on the way up; from the crest
             * to the level the ramp caps at BOARD_RAMP_DOWN_POWER; level
             * again: normal power. */
            unsigned slope_ticks=0;
            for(unsigned n=8U;n<HUMP_ITERATIONS;++n) {
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && !sent_steer[n]);
#if BOARD_RAMP_SPEED
                if(n>=event_tick[1] && n<event_tick[2]) {assert(sent_power[n]==BOARD_RAMP_DOWN_POWER);continue;}
#endif
                assert(sent_power[n]==30U || sent_power[n]==20U);
                if(sent_power[n]==20U) {++slope_ticks;assert(n>=HUMP_FROM);}
            }
            assert(slope_ticks>0U && sent_power[HUMP_ITERATIONS-1U]==30U);
        }
        if(failure_mode==50U) {
            /* S4 never sees a slope (no 20% cap); the way down from the crest
             * to the level is capped at BOARD_RAMP_DOWN_POWER (with
             * BOARD_RAMP_SPEED 0: full power all the way). */
            for(unsigned n=8U;n<LOW_ITERATIONS;++n) {
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && !sent_steer[n]);
#if BOARD_RAMP_SPEED
                if(n>=event_tick[1] && n<event_tick[2]) {assert(sent_power[n]==BOARD_RAMP_DOWN_POWER);continue;}
#endif
                assert(sent_power[n]==30U);
            }
        }
        if(failure_mode==36U) {
            assert(saved.shock_count==2U);
#if BOARD_SHOCK_STOP
            assert(saved.vehicle.fault==VC_SHOCK);
#else
            assert(saved.vehicle.armed);
#endif
        }
        if(failure_mode==37U) {assert(saved.imu_fault_count==4U && saved.vehicle.armed);}
        if(failure_mode==3U || failure_mode==5U || failure_mode==21U)assert(saved.imu_fault_count>0U);
        if(failure_mode==38U) {
            assert(saved.vehicle.armed && saved.imu_fault_count>=5U);
#if BOARD_RAMP_SPEED
            for(unsigned n=55U;n<90U;++n)if(sent_kind[n]!=SENT_NONE)assert(sent_power[n]<=20U);
#endif
        }
        if(failure_mode==39U || failure_mode==40U) {
            assert(saved.vehicle.fault==VC_SENSOR && saved.vehicle.fault_cause==VC_CAUSE_TILT);
            unsigned first=0;for(unsigned n=20U;n<iterations();++n)if(!pub_armed[n]){first=n;break;}
            assert(first>20U && first<20U+LIFT_TICKS);
        }
        if(obstacle_mode()) {
            for(unsigned n=9U;n<iterations();++n)if(pan_blocked[n])assert(stopped[n] && sent_kind[n]==SENT_NONE);
            if(failure_mode==45U || failure_mode==44U)assert(saved.vehicle.fault==VC_SENSOR && saved.vehicle.fault_cause==VC_CAUSE_SONAR);
            else assert(saved.vehicle.armed && probe_at && probe_angle<50U && side_looks);
        }
        if(failure_mode==16U) {
            /* Lifted: still driving (straight, both IRs read black) until it
             * has lasted BOARD_LIFT_FAULT_MS, then VC_LIFTED with a STOP. */
            for(unsigned n=8U;n<LIFT_AT+LIFT_TICKS;++n)
                assert(pub_armed[n] && pub_phase[n]==VC_TRACK && sent_kind[n]==SENT_FOLLOW && !sent_steer[n] && sent_power[n]==30U);
#if BOARD_LIFT_FAULT_MS
            _Static_assert(BOARD_LIFT_FAULT_MS==LIFT_TICKS*TICK_MS,"mode 16 timing");
            for(unsigned n=LIFT_AT+LIFT_TICKS;n<iterations();++n)
                assert(!pub_armed[n] && pub_phase[n]==VC_FAULT && stopped[n] && sent_kind[n]==SENT_NONE);
            assert(saved.vehicle.phase==VC_FAULT && saved.vehicle.fault==VC_LIFTED && !saved.vehicle.fault_cause);
#else
            assert(saved.vehicle.armed && saved.vehicle.phase==VC_TRACK); /* lift fault off */
#endif
        }
    }
    assert(codes_seen==0x1FU); /* straight, steer right/left and pivot right/left all published */
    puts("Straddle barcode read: one straddle sensor on black steers toward it (capped, no pivot), both/neither straight PASS");
#if BOARD_LINE_MASK_PIVOT
    puts("Mask-pivot follower: right/left IR black pivots that way at once, both/neither straight; a TRACK STOP resumes on the current sample PASS");
#else
    puts("Edge tracing: proportional steer with power drop, far-side pivot, crossing straight, deep-edge pivot after 250 ms PASS");
#endif
    puts("Straddle app_state line_follow/line_mask/line_steer match the S2 request every tick PASS");
    puts("Obstacle while tracking: re-reads count once, no-echo is no reading, 5 cm/invalidated/no-echo-after-near hold; 3 consecutive new near echoes acquire PASS");
    puts("Obstacle detected through S5's real step (third echo published turning, invalid): EV_OBSTACLE carries its 15 cm PASS");
    puts("Sonar gap: tracking holds still, a sonar fault only after VC_SONAR_GAP_MS PASS");
    puts("S5 lock busy across a phase change: the last snapshot is reused, the sensor mode is requested after a fresh read PASS");
    puts("Avoidance ended by a STOP: EV_OBSTACLE back with how 0 PASS");
    printf("Lifted (no surface, GP28 black): %s PASS\n",
           BOARD_LIFT_FAULT_MS?"VC_LIFTED after BOARD_LIFT_FAULT_MS":"no fault (BOARD_LIFT_FAULT_MS 0)");
    puts("Event log: EV_BARCODE for a failed first read, EV_OBSTACLE on acquisition, each EV_BARCODE attempt before its EV_INSTRUCTION PASS");
#if BOARD_RAMP_SPEED
    puts("8c: skipped impacts/faults and counter wrap, dropped-traversal cap, tipped lift, fresh forward pans and checked reverse replans PASS");
    puts("Hump: EV_HUMP detected after the 400 ms hold, crest caps the power to BOARD_RAMP_DOWN_POWER (this hump's height so far), level restores it (its height) PASS");
    puts("Low hump (3.3 cm at 4.8 deg, below S4's 5 deg climb): one detected/crest/level cycle with the controller's height, slowed all the way down PASS");
#else
    puts("Hump with BOARD_RAMP_SPEED 0: no events, no power cap PASS");
#endif
    puts("INT-11 actual vehicle path: encoded getter errors, future samples, IMU skew, reset and motor STOP PASS");
    puts("Barcode retry: a one-sample forward sonar glitch does not abort the reverse re-read; no forward request or STOP follows a reverse in its tick PASS");
    puts("Barcode first pass: S3 READING caps the forward power to the re-read speed PASS");
    puts("MAP command: applied in any phase, invalid rejected, stale ACK 3 PASS");
    puts("ARM lease renewal: ARM re-sent every 100 ms keeps a 500 ms leased run going (ACK 0) with its phase timer, origin and demand, it times out 500 ms after the last one; START while armed and ARM during a START run refused (ACK 1) PASS");
    puts("ARM renewal limits: another demand refused (ACK 1), a 100 ms old one refused (ACK 3) and the old lease still ends the run; an S3 status miss does not turn a renewal into fault 6 PASS");
    puts("STOP during renewals: the run stays stopped while ARM keeps coming (ACK 1); RESET, or 1 s with no ARM, lets ARM in again PASS");
    puts("Silent S3 latch (STALE/SENSOR) while tracking: TRACK re-sent at once, at most every 100 ms, keeping a pending turn's hold; the robot drives on and the next marker decodes; a latch still returning after 1 s is VC_LINE cause event; no re-send while a re-read brakes PASS");
    printf("Marker cut short by the latch: %s PASS\n",LATCH_READ_FAILS?
           "a failed read (EV_BARCODE attempt 1, 5 elements), re-read, TRACK re-sent as it reverses":
           "skipped (BOARD_BARCODE_FAULT_FAILS 0 or no re-read drive)");
    printf("Short timing faults ride through: an odometry gap holds still (fault 1 cause 3 after %u ms), a failed motor status copy reuses the last (fault 7 after %u ticks; a published FAULT at once), a busy S3 lock is retried (fault 4 once the %u ms lease would run out), a command stamped a tick ahead is fresh PASS\n",
           (unsigned)VC_ODOM_GAP_MS,(unsigned)VC_MOTOR_STATUS_MISSES,(unsigned)VC_S3_LEASE_MS);
#if VC_TIMING
    puts("Loop timing: VC run and release gap measured on the hardware timer, a late period counted, one [TIMING] console line per report interval PASS");
#endif
    puts("S3 holding a decoded marker with no turn pending: TRACK (marker clear) re-sent after 100 ms, once; a turn delivered with a latch stays pending and keeps its marker PASS");
#if BOARD_LINE_MASK_PIVOT
    printf("Line lost while tracking: %u mm with both IRs white spot-searches toward the line's side; found -> tracking, never -> fault 2 cause 3; a barcode read or re-read never counts PASS\n",
           (unsigned)BOARD_LINE_LOST_MM);
#else
    puts("Line lost with edge tracing: no line-lost distance, its own full-turn VC_LINE PASS");
#endif
    if(shock_failures) {
        puts("FAIL Shock while tracking: a >1.8 g or clipped (+/-2 g) sample is not detected/published/counted in this build");
        return 1;
    }
    printf("Shock while tracking, unclipped and clipped at +/-2 g (also an IMU fault): published, counted, %s PASS\n",
           BOARD_SHOCK_STOP?"stops the run (VC_SHOCK)":"run continues (BOARD_SHOCK_STOP 0)");
    return 0;
}
