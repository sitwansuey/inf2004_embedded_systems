/* Private S5 bound-search context; main uses only subsystem5.h. */
#ifndef S5_SCAN_H
#define S5_SCAN_H
#include "subsystem5.h"
#include "s5_width.h"
typedef struct
{
    UINT acquisition_samples;
    UINT motion_restarts;
    UINT echo_failures;
    INT  echo_angle;
    BOOL echo_angle_valid;
    INT  fine_clear;
    INT  fine_object;  /* inner magnitude known (or, after the mirror or a
                          retreat, expected) to hit the object */
    INT coarse_object; /* last coarse magnitude on this side that hit the object
                          (0: forward) */
    UINT fine_checks;
    BOOL averaging_bound;
    UINT edge_retreats; /* per side: edge moved 1 deg inward after a missed
                           sample */
    UINT    bound_samples;
    UW      bound_sum_mm;
    INT     origin_left, origin_right;
    UW      width_hold_ms; /* sample time the widths were first published */
    uint8_t position_recorded : 1;
    uint8_t plan_pending : 1;
    uint8_t width_hold : 1; /* VERIFY_CLEAR publishes widths before the plan */
    uint8_t right_coarse
        : 1; /* left blocked through 60 deg: right searched coarsely */
    uint8_t right_outward
        : 1; /* the mirror check hit the object: right searched outward */
    INT forced_side; /* plan side: 0 smaller half, +1 LEFT, -1 RIGHT */
    /* Edge confirmation: the edge needs two clear angles in a row outward of
     * it, so one stray clear probe cannot move it inward. */
    INT outer_clear; /* outermost clear magnitude known on this side (where its
                        walk began) */
    INT prev_clear;  /* clear magnitude before fine_clear last moved inward (0:
                        none) */
    INT edge_deg;    /* averaged edge magnitude waiting for its outward check */
    UW  edge_mm;
    uint8_t edge_confirm : 1;
    /* Side corridor: a side is out once a check failed. */
    uint8_t left_rejected : 1;
    uint8_t right_rejected : 1;
    /* Motion restarts: the first one of an acquisition, at the first sample
     * after the origin was taken, is the robot's own braking and is free. */
    uint8_t origin_fresh
        : 1; /* no sample checked since scan_begin took the origin */
    uint8_t brake_restart
        : 1;              /* the free restart was used in this acquisition */
    UINT corridor_probes; /* angles probed between the edge and the heading */
    UINT corridor_index;  /* next corridor probe (1..corridor_probes), then the
                             heading */
    UW near_mm[2]; /* nearest clear echo inside the corridor window: lateral mm,
                      [0] left, [1] right (0: none) */
} s5_scan_context_t;
#ifndef S5_EDGE_RETREATS
#define S5_EDGE_RETREATS 3U
#endif
/* Unsigned subtraction preserves encoder wrap; compare before abs(INT_MIN). */
static inline BOOL
s5_encoder_displaced (INT a, INT b)
{
    int32_t delta = (int32_t) ((uint32_t) a - (uint32_t) b);
    return delta > S5_SCAN_MOTION_TOLERANCE_COUNTS ||
           delta < -S5_SCAN_MOTION_TOLERANCE_COUNTS;
}
IMPORT INT s5_scan_step (s5_status_t * next, s5_scan_context_t * scan);
#endif

/*** end of file ***/
