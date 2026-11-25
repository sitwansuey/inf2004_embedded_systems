#include <assert.h>
#include <stdio.h>
#include "../../app_program/vehicle_barcode_retry.h"

enum
{
    GAP   = 30,
    RUNUP = 50,
    MAX   = 800
};

static unsigned
step (br_reverse_input_t in, bool * full, bool * miss)
{
    return br_reverse_next(&in, full, miss, GAP, RUNUP, MAX);
}
static br_reverse_input_t
moving (uint32_t moved_mm, uint32_t since_edge_mm)
{
    return (br_reverse_input_t){.odom_valid    = true,
                                .moved_um      = moved_mm * 1000U,
                                .since_edge_um = since_edge_mm * 1000U};
}

int
main (void)
{
    bool               full, miss;
    br_reverse_input_t in;

    /* Clean reverse read: 29 elements counted while reading does not stop the
     * reverse; it continues until the read ends and 30+50 mm past the last
     * edge. */
    full = miss = false;
    in          = moving(150, 0);
    in.reading  = true;
    in.full     = true;
    assert(step(in, &full, &miss) == BR_NEXT_STAY && full);
    in = moving(180, 30); /* S3 ended the read; no result counted yet */
    assert(step(in, &full, &miss) == BR_NEXT_STAY);
    in = moving(229, 79);
    assert(step(in, &full, &miss) == BR_NEXT_STAY);
    in = moving(230, 80);
    assert(step(in, &full, &miss) == BR_NEXT_WAIT);

    /* The reverse read decodes: resume at once (no run-up needed). */
    full = miss = false;
    in          = moving(180, 30);
    in.hit      = true;
    assert(step(in, &full, &miss) == BR_NEXT_RESUME);

    /* The reverse read fails after the whole marker: run-up, then slow pass. */
    full = miss = false;
    in          = moving(150, 0);
    in.reading  = true;
    in.full     = true;
    (void) step(in, &full, &miss);
    in      = moving(180, 30);
    in.miss = true;
    assert(step(in, &full, &miss) == BR_NEXT_STAY && miss);
    in = moving(230, 80);
    assert(step(in, &full, &miss) == BR_NEXT_SLOW);

    /* Noise made the count reach 29 before the first bar: the read keeps
     * running, so the robot keeps reversing however far it has gone. */
    full = miss = false;
    in          = moving(120, 0);
    in.reading  = true;
    in.full     = true;
    (void) step(in, &full, &miss);
    in         = moving(200, 200);
    in.reading = true;
    assert(step(in, &full, &miss) == BR_NEXT_STAY);

    /* Partial reverse read (< 29) fails early: no decision before 5 s, then
     * still the run-up beyond its last edge. */
    full = miss = false;
    in          = moving(100, 30);
    in.miss     = true;
    assert(step(in, &full, &miss) == BR_NEXT_STAY);
    in = moving(200, 120);
    assert(step(in, &full, &miss) == BR_NEXT_STAY); /* min not done */
    in          = moving(260, 180);
    in.min_done = true;
    assert(step(in, &full, &miss) == BR_NEXT_SLOW);

    /* The sensor never saw the marker: give up only after 5 s and 3 s quiet. */
    full = miss = false;
    in          = moving(150, 150);
    in.quiet    = true;
    assert(step(in, &full, &miss) == BR_NEXT_STAY);
    in.min_done = true;
    assert(step(in, &full, &miss) == BR_NEXT_SLOW);

    /* Bounds: the distance limit always stops; no odometry, no run-up. */
    full = miss = false;
    in          = moving(MAX, 0);
    in.reading  = true;
    assert(step(in, &full, &miss) == BR_NEXT_SLOW);
    full          = true;
    miss          = false;
    in            = moving(300, 300);
    in.odom_valid = false;
    assert(step(in, &full, &miss) == BR_NEXT_STAY);

    puts("barcode_retry: reverse runs past the first bar with run-up; "
         "noise-early count does not stop it PASS");
    return 0;
}

/*** end of file ***/
