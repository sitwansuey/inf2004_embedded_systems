/* Run-time and release counters for the periodic loops (C5): the vehicle
 * controller, the S2 motor task and the CYW43/MQTT network poll. Each loop
 * reads the 1 MHz hardware timer a few times per period and writes only its
 * own record (no locks); vehicle.c prints all three on the USB console every
 * VC_TIMING_REPORT_MS. Times are wall-clock microseconds: a run includes
 * preemption by higher-priority tasks, so max_run_us is the measured
 * response time (the WCET bound seen so far). A sleep inside a run (S2's
 * direction dead time) is left out with pause/resume. BOARD_TIMING_STATS 0
 * (robot_config.h) removes every hook and the report. */
#ifndef CSC02_TASK_TIMING_H
#define CSC02_TASK_TIMING_H
#include <stdint.h>
#include "../robot_config.h"
/* These loops run every 20 ms (10 ms asked, plus the kernel's guard tick).
 * A release more than this after the previous one is late: the period
 * slipped by a whole 10 ms tick. */
#ifndef TASK_TIMING_LATE_US
#define TASK_TIMING_LATE_US 25000U
#endif
typedef struct {
    uint32_t runs;       /* completed runs since boot */
    uint32_t late;       /* releases more than TASK_TIMING_LATE_US after the previous one */
    uint32_t max_run_us; /* longest run since boot: release to done, pauses left out */
    uint32_t max_gap_us; /* longest time between two releases since boot */
    uint32_t release_us, start_us, run_us; /* the current run */
    uint32_t released;   /* a run is open (release seen, done not yet) */
    uint32_t seen;       /* any release seen: the first one has no gap */
} task_timing_t;
enum { TASK_TIMING_VC, TASK_TIMING_MOTOR, TASK_TIMING_NET, TASK_TIMING_COUNT };
#if BOARD_TIMING_STATS
/* Defined in app_main.c, which every firmware build links. */
extern task_timing_t app_task_timing[TASK_TIMING_COUNT];
#endif
/* The loop woke for new work (its period or a message). */
static inline void task_timing_release(task_timing_t *t,uint32_t now_us)
{
    if (t->seen) {
        uint32_t gap=now_us-t->release_us;
        if (gap>t->max_gap_us) t->max_gap_us=gap;
        if (gap>TASK_TIMING_LATE_US) ++t->late;
    }
    t->seen=1U;t->released=1U;
    t->release_us=t->start_us=now_us;t->run_us=0U;
}
/* About to sleep inside the run / back from that sleep. */
static inline void task_timing_pause(task_timing_t *t,uint32_t now_us)
{
    if (t->released) t->run_us+=now_us-t->start_us;
}
static inline void task_timing_resume(task_timing_t *t,uint32_t now_us)
{
    t->start_us=now_us;
}
/* The run is over: the loop is about to wait for its next release. */
static inline void task_timing_done(task_timing_t *t,uint32_t now_us)
{
    if (!t->released) return;
    uint32_t run=t->run_us+(now_us-t->start_us);
    if (run>t->max_run_us) t->max_run_us=run;
    ++t->runs;t->released=0U;
}
#endif
