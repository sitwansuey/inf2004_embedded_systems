#include "../../app_program/vehicle_policy.h"
#include "../../app_program/subsystem2/odometry.h"
#include "../../app_program/subsystem4/terrain.h"
#include "../../app_program/subsystem4/imu_mount.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
static vc_input_t
healthy (void)
{
    return (vc_input_t){.now_ms            = 100,
                        .line_fresh        = 1,
                        .steering          = 1,
                        .front_clear       = 1,
                        .sensor_fresh      = 1,
                        .backup_ready      = 1,
                        .odom_valid        = 1,
                        .terrain_valid     = 1,
                        .motor_ok          = 1,
                        .transport_ok      = 1,
                        .width_valid       = 1,
                        .obstacle_width_mm = 200,
                        .bypass_steering   = 376,
                        .nav_offset_mm     = 90,
                        .nav_search_mm     = 600};
}
static vc_state_t
armed (void)
{
    vc_state_t s = {0};
    assert(vc_arm(&s, 100, 1000, 30));
    return s;
}
static vc_state_t
started (void)
{
    vc_state_t s = {0};
    assert(vc_start(&s, 100, 30));
    return s;
}
static void
controller (void)
{
    vc_input_t i = healthy();
    vc_state_t s = armed();
    vc_step(&s, &i);
    assert(s.command == 1 && s.power == 30);
    /* Either straddle sensor on black keeps FORWARD (the vehicle task's
     * follower steers or pivots); the last single side is kept for a search. */
    i.line_black_mask = 1;
    vc_step(&s, &i);
    assert(s.command == 1 && s.last_line_side == 1);
    i.line_black_mask = 3;
    vc_step(&s, &i);
    assert(s.command == 1 && s.last_line_side == 1);
    i.line_black_mask = 2;
    vc_step(&s, &i);
    assert(s.command == 1 && s.last_line_side == 2);
    i.line_black_mask = 0;
    i.slope           = 1;
    vc_step(&s, &i);
    assert(s.power == 20);
    for (unsigned n = 0; n < 8; ++n)
    {
        bool gap =
            n == 0 || n == 2 ||
            n >= 6; /* sonar (0), odometry (2) or line data (6, 7) missing */
        s = gap ? started() : armed();
        i = healthy();
        switch (n)
        {
            case 0:
                i.sensor_fresh = 0;
                break;
            case 1:
                i.terrain_valid = 0;
                break;
            case 2:
                i.odom_valid = 0;
                break;
            case 3:
                i.motor_ok = 0;
                break;
            case 4:
                i.transport_ok = 0;
                break;
            case 5:
                i.shock = 1;
                break;
            case 6:
                i.line_fresh = 0;
                break;
            case 7:
                i.steering = 0;
                break;
        }
        vc_step(&s, &i);
        if (gap)
        {
            /* A data gap holds still; it faults only once it lasts
             * VC_SONAR_GAP_MS (sonar), VC_ODOM_GAP_MS (odometry) or
             * VC_LINE_GAP_MS (line data). */
            uint32_t limit = n == 0   ? VC_SONAR_GAP_MS
                             : n == 2 ? VC_ODOM_GAP_MS
                                      : VC_LINE_GAP_MS;
            assert(s.command == 0 && s.power == 0 && s.armed &&
                   s.phase == VC_TRACK);
            i.now_ms += limit - 1U;
            vc_step(&s, &i);
            assert(s.command == 0 && s.armed && s.phase == VC_TRACK);
            i.now_ms += 1U;
            vc_step(&s, &i);
        }
        assert(s.command == 0 && s.power == 0 && !s.armed &&
               s.phase == VC_FAULT);
        /* VC_SENSOR: sonar 1, tilt 2, odometry 3; VC_LINE: line data 1; others
         * none */
        unsigned cause = n < 3 ? n + 1U : gap ? VC_LINE_CAUSE_DATA : 0U;
        assert(s.fault_cause == cause && s.fault == (n < 3 ? VC_SENSOR
                                                     : gap ? VC_LINE
                                                           : s.fault));
        i.now_ms += 20;
        vc_step(&s, &i);
        assert(s.fault_cause == cause); /* latched */
        assert(!vc_arm(&s, i.now_ms, 1000, 30));
    }
    /* Check order: tilt faults at once, before the odometry and sonar gaps
     * (which hold, both timed); the shorter odometry gap faults first. */
    s              = armed();
    i              = healthy();
    i.sensor_fresh = i.odom_valid = 0;
    vc_step(&s, &i);
    assert(s.armed && !s.command && s.odom_gap && s.sonar_gap);
    i.now_ms += VC_ODOM_GAP_MS;
    vc_step(&s, &i);
    assert(s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_ODOM);
    s               = armed();
    i               = healthy();
    i.terrain_valid = i.odom_valid = 0;
    vc_step(&s, &i);
    assert(s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_TILT);
    /* C1: one late controller period (or a failed encoder copy) leaves
     * odometry invalid for two ticks: hold still, keep a decoded turn that
     * arrives meanwhile, drive on; a new gap is timed from its own start. */
    s = started();
    i = healthy();
    vc_step(&s, &i);
    assert(s.command == 1);
    i.odom_valid = 0;
    i.now_ms += 20;
    i.navigation_event = 1;
    i.nav              = 2;
    vc_step(&s, &i);
    assert(s.armed && s.phase == VC_TRACK && !s.command && s.pending_nav == 2 &&
           s.odom_gap);
    i.navigation_event = 0;
    i.nav              = 0;
    i.now_ms += VC_ODOM_GAP_MS - 1U;
    vc_step(&s, &i);
    assert(s.armed && !s.command);
    i.odom_valid = 1;
    i.now_ms += 1;
    vc_step(&s, &i);
    assert(s.armed && s.command == 1 && !s.odom_gap && s.pending_nav == 2);
    i.odom_valid = 0;
    i.now_ms += 20;
    vc_step(&s, &i);
    i.now_ms += VC_ODOM_GAP_MS - 1U;
    vc_step(&s, &i);
    assert(s.armed);
    i.now_ms += 1U;
    vc_step(&s, &i);
    assert(!s.armed && s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_ODOM);
    /* STOP or RESET clears it; a new start never inherits a stale cause. */
    vc_stop(&s, VC_OK);
    assert(s.phase == VC_DISARMED && !s.fault_cause);
    s.fault_cause = VC_CAUSE_ODOM;
    assert(vc_start(&s, 200, 30) && !s.fault_cause);
    s.fault_cause = VC_CAUSE_TILT;
    s.armed       = 0;
    assert(vc_arm(&s, 300, 1000, 30) && !s.fault_cause);
    s        = armed();
    i        = healthy();
    i.now_ms = 1100;
    vc_step(&s, &i);
    assert(s.fault == VC_TIMEOUT);
    s = (vc_state_t){0};
    assert(vc_arm(&s, UINT32_MAX - 10, 100, 30));
    i        = healthy();
    i.now_ms = 20;
    vc_step(&s, &i);
    assert(s.command == 1);
    i.now_ms = 100;
    vc_step(&s, &i);
    assert(s.fault == VC_TIMEOUT);
    assert(!vc_arm(&s, 0, 1001, 30));
    assert(!vc_arm(&s, 0, 1, 51));
    /* ARM lease renewal (vc_renew): a leased ARM run gets a new lease from
     * now and keeps its phase; a START run, a disarmed or faulted controller
     * and a bad lease/power are refused with nothing changed. */
    s = armed();
    i = healthy();
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK);
    for (uint32_t t = 500; t <= 3000; t += 500)
    {
        assert(vc_renew(&s, t, 1000, 30) && s.armed_ms == t &&
               s.lease_ms == 1000 && s.phase == VC_TRACK);
        i.now_ms = t + 999;
        vc_step(&s, &i);
        assert(s.armed && s.fault == VC_OK);
    }
    assert(!vc_renew(&s, 3100, 1001, 30) && !vc_renew(&s, 3100, 0, 30) &&
           !vc_renew(&s, 3100, 500, 51));
    assert(s.armed_ms == 3000 && s.lease_ms == 1000 && s.maximum_power == 30);
    /* A different demand is refused (no speed step mid-run); a shorter
     * lease at the same demand is taken. */
    assert(!vc_renew(&s, 3100, 500, 20) && !vc_renew(&s, 3100, 500, 50) &&
           s.maximum_power == 30 && s.armed_ms == 3000 && s.lease_ms == 1000);
    assert(vc_renew(&s, 3100, 500, 30) && s.maximum_power == 30 &&
           s.lease_ms == 500);
    i.now_ms = 3600;
    vc_step(&s, &i);
    assert(!s.armed && s.fault == VC_TIMEOUT);
    assert(!vc_renew(&s, 3700, 1000, 30) && s.phase == VC_FAULT); /* faulted */
    s = (vc_state_t){0};
    assert(!vc_renew(&s, 100, 1000, 30) && !s.armed); /* disarmed */
    s = started();
    assert(!vc_renew(&s, 200, 1000, 30) && s.lease_ms == 0 &&
           s.armed_ms == 100); /* START run */
    s                  = armed();
    i                  = healthy();
    i.navigation_event = 1;
    i.nav              = 4;
    vc_step(&s, &i);
    /* A decoded turn waits for the junction, then the 90 mm sensor-to-wheel
     * offset. */
    assert(s.phase == VC_TRACK && s.pending_nav == 4 && s.command == 1);
    i.navigation_event = 0;
    i.junction         = 1;
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK && s.nav_offset && s.command == 1);
    i.junction    = 0;
    i.distance_um = 90000;
    vc_step(&s, &i);
    /* No turn settings (power 0, no angle window): armed power, exactly 180 deg
     * by yaw. (vehicle.c always sends the robot_config.h power and angle
     * window.) */
    assert(s.phase == VC_NAV_TURN);
    i.yaw_mdeg   = 179000;
    i.line_touch = 1;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.command == 3 && s.power == 30);
    i.yaw_mdeg = 181000;
    vc_step(&s, &i);
    assert(s.phase == VC_REACQUIRE && !s.command && s.spot_scan);
    /* Turn power never exceeds the armed (Start) power. */
    s                  = armed();
    s.maximum_power    = 10;
    i                  = healthy();
    i.nav_turn_power   = 25;
    i.nav_seek_power   = 15;
    i.navigation_event = 1;
    i.nav              = 2;
    vc_step(&s, &i);
    i.navigation_event = 0;
    i.junction         = 1;
    vc_step(&s, &i);
    i.junction    = 0;
    i.distance_um = 90000;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN);
    vc_step(&s, &i);
    assert(s.command == 4 && s.power == 10);
    /* Straddle settings: 25 % to 60 deg, a touch there is ignored, then 15 %
     * until a sensor touches the new line and releases it into the gap
     * (centred), which ends a LEFT (+90, S2 LEFT 3) turn. */
    s                   = armed();
    i                   = healthy();
    i.nav_turn_power    = 25;
    i.nav_seek_power    = 15;
    i.nav_turn_min_deg  = 60;
    i.nav_turn_max_deg  = 120;
    i.nav_uturn_min_deg = 150;
    i.nav_uturn_max_deg = 210;
    i.spot_scan_all     = 1;
    i.navigation_event  = 1;
    i.nav               = 1;
    vc_step(&s, &i);
    i.navigation_event = 0;
    i.junction         = 1;
    vc_step(&s, &i);
    i.junction    = 0;
    i.distance_um = 90000;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.turn_deg == 90);
    i.line_touch      = 1;
    i.line_black_mask = 2;
    i.yaw_mdeg        = 45000;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.command == 3 && s.power == 25);
    i.line_touch      = 0;
    i.line_black_mask = 0;
    i.yaw_mdeg        = 70000;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.command == 3 && s.power == 15);
    i.line_touch      = 1;
    i.line_black_mask = 2;
    i.yaw_mdeg        = 82000;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.command == 3 && s.nav_touched);
    i.line_touch      = 0;
    i.line_black_mask = 0;
    i.yaw_mdeg        = 88000;
    vc_step(&s, &i);
    assert(s.phase == VC_REACQUIRE && !s.spot_scan && !s.command);
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK);
    /* Any close obstacle overrides current steering in the same iteration. */
    s             = armed();
    i             = healthy();
    i.obstacle    = 1;
    i.front_clear = 0;
    vc_step(&s, &i);
    assert(s.phase == VC_ACQUIRE && !s.command);
    i.scan_ready  = 1;
    i.turn_deg    = 30;
    i.approach_mm = 200;
    i.return_mm   = 100;
    vc_step(&s, &i);
    assert(s.phase == VC_FAULT && s.fault == VC_UNCALIBRATED);
    s                  = armed();
    i                  = healthy();
    i.obstacle         = 1;
    i.front_clear      = 0;
    i.navigation_event = 1;
    i.nav              = 2;
    vc_step(&s, &i);
    assert(s.pending_nav == 2);
    i.navigation_event = 0;
    i.scan_ready       = 1;
    i.bypass_enabled   = 1;
    i.turn_deg         = 30;
    i.approach_mm      = 200;
    i.return_mm        = 100;
    vc_step(&s, &i);
    assert(s.phase == VC_BACKUP && !s.command);
    /* front_clear=0 does not gate it; the reverse ramps up from START_POWER */
    vc_step(&s, &i);
    assert(s.phase == VC_BACKUP && s.command == 2 &&
           s.power == AVOID_BACKUP_START_POWER);
    /* Distance bound: stand still before the forward turn-out. */
    i.distance_um = -300000;
    vc_step(&s, &i);
    assert(s.phase == VC_BACKUP && !s.command && s.backup_braking);
    i.now_ms = 100 + AVOID_BACKUP_BRAKE_MS;
    vc_step(&s, &i);
    assert(s.phase == VC_TURN_OUT && !s.command && s.approach_mm == 200 &&
           s.return_mm == 200);
    vc_step(&s, &i);
    assert(!s.command); /* needs a fresh, forward-facing reading */
    i.front_clear = 1;
    vc_step(&s, &i);
    assert(s.command == 1 && s.steering == -376);
    i.yaw_mdeg = 30000;
    vc_step(&s, &i);
    assert(s.phase == VC_DIAGONAL && s.command == 1);
    /* End of the out diagonal: stop and look to the obstacle side. */
    i.distance_um = -100000;
    vc_step(&s, &i);
    assert(s.phase == VC_DIAGONAL && !s.command && s.look_side &&
           s.return_mm == 200);
    i.front_clear   = 0;
    i.side_observed = i.side_ready = 1;
    i.side_clear                   = 0;
    for (unsigned n = 0; n < 3; ++n)
    {
        i.now_ms += 20;
        i.side_sample_ms = i.now_ms;
        vc_step(&s, &i);
        assert(!s.command);
    }
    /* Still beside it: align, then pass it looking every AVOID_PASS_STEP_MM. */
    assert(s.phase == VC_ALIGN && !s.look_side && !s.direct_return);
    i.front_clear   = 1;
    i.side_observed = 0;
    i.now_ms += 20;
    vc_step(&s, &i);
    assert(s.command == 1 && s.steering == 376);
    i.yaw_mdeg = 0;
    vc_step(&s, &i);
    assert(s.phase == VC_PASS && !s.look_side && s.command == 1);
    i.distance_um += AVOID_PASS_STEP_MM * 1000 - 1;
    vc_step(&s, &i);
    assert(s.command == 1 && !s.look_side);
    i.distance_um += 1;
    vc_step(&s, &i);
    assert(!s.command && s.look_side);
    i.front_clear   = 0;
    i.side_observed = i.side_ready = i.side_clear = 1;
    for (unsigned n = 0; n < 3; ++n)
    {
        i.now_ms += 20;
        i.side_sample_ms = i.now_ms;
        vc_step(&s, &i);
        assert(!s.command);
    }
    /* Clear: AVOID_CLEAR_RUN_MM further on, then turn back. */
    assert(s.phase == VC_PASS && s.clear_run && !s.look_side);
    i.front_clear   = 1;
    i.side_observed = 0;
    i.now_ms += 20;
    vc_step(&s, &i);
    assert(s.phase == VC_PASS && s.command == 1);
    i.distance_um += AVOID_CLEAR_RUN_MM * 1000;
    vc_step(&s, &i);
    assert(s.phase == VC_PASS && s.command == 1);
    /* ... and AVOID_CLEAR_RUN_MS of driving from its first drive tick. */
    i.now_ms += AVOID_CLEAR_RUN_MS;
    vc_step(&s, &i);
    assert(s.phase == VC_TURN_IN && s.command == 1 && s.steering == 376);
    /* Back toward the line at the 45 deg return heading (AVOID_RETURN_MIN_DEG).
     */
    i.yaw_mdeg = -44000;
    vc_step(&s, &i);
    assert(s.phase == VC_TURN_IN && s.command == 1);
    i.yaw_mdeg = -45000;
    vc_step(&s, &i);
    assert(s.phase == VC_RETURN && s.command == 1);
    /* The line search is off for AVOID_TURN_IN_LINE_DELAY_MS of driving from
     * the turn-in's first drive tick: a touch that early is ignored. */
    i.line_black_mask = 1;
    vc_step(&s, &i);
    assert(s.phase == VC_RETURN && s.command == 1 && !s.nav_touched);
    i.line_black_mask = 0;
    for (unsigned n = 0; n < AVOID_TURN_IN_LINE_DELAY_MS / 20U; ++n)
    {
        i.now_ms += 20;
        assert(vc_renew(&s, i.now_ms, 1000, 30));
        vc_step(&s, &i);
    }
    assert(s.phase == VC_RETURN &&
           s.turn_in_driven_ms >= AVOID_TURN_IN_LINE_DELAY_MS);
    /* The lead (right) sensor touches the line, then releases it into the
     * gap: the line is under the robot. Arc back to the heading held before
     * the obstacle, then track. */
    i.line_black_mask = 1;
    i.distance_um += 50000;
    vc_step(&s, &i);
    assert(s.phase == VC_RETURN && s.command == 1 && s.nav_touched &&
           !s.line_found);
    i.line_black_mask = 0;
    i.distance_um += 10000;
    vc_step(&s, &i);
    assert(s.phase == VC_RESTORE && s.line_found && s.restore_mdeg == 45000 &&
           s.command == 1 && s.steering == -376);
    i.yaw_mdeg = 0;
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK && !s.command);
    assert(s.pending_nav == 2);
    i.obstacle = 0;
    vc_track_requested(&s, i.now_ms);
    i.steering = 0;
    vc_step(&s, &i);
    assert(!s.command && s.armed && s.pending_nav == 2);
    i.steering = 1;
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK && s.pending_nav == 2 && s.command == 1 &&
           !s.nav_offset);
    i.junction = 1;
    vc_step(&s, &i);
    i.junction = 0;
    i.distance_um += 89999;
    vc_step(&s, &i);
    assert(s.phase == VC_TRACK && s.nav_offset && s.command == 1 &&
           !s.steering && s.pending_nav == 2);
    i.distance_um += 1;
    vc_step(&s, &i);
    assert(s.phase == VC_NAV_TURN && s.turn_deg == -90 && !s.pending_nav &&
           !s.command);
    /* Never resume expired or interrupted open-loop maneuvers. */
    s            = armed();
    s.phase      = VC_TURN_OUT;
    s.entered_ms = 100;
    i            = healthy();
    i.now_ms     = 10101;
    s.armed_ms   = 10101;
    vc_step(&s, &i);
    assert(s.phase == VC_FAULT);
    puts("INT-01 controller: priority, expiry, stale data, complete bounded "
         "bypass, STOP handoffs PASS");
}
static void
odometry (void)
{
    odom_config_t c = {100, 200, 100};
    odom_t        s = {0};
    odom_step(&s, &c, 0, UINT32_MAX - 1, UINT32_MAX - 1, true);
    assert(!s.valid);
    odom_step(&s, &c, 20, 3, 3, true);
    assert(s.valid && odom_delta_um(&s) == 10000 && odom_speed_mm_s(&s) == 500);
    odom_step(&s, &c, 40, 3, 8, true);
    assert(s.valid && odom_turn_mdeg(&s) > 5700 && odom_turn_mdeg(&s) < 5740);
    odom_step(&s, &c, 200, 4, 9, true);
    assert(!s.valid);
    odom_step(&s, &c, 220, 4, 9, false);
    assert(!s.valid);
    s = (odom_t){0};
    odom_step(&s, &c, UINT32_MAX - 10, 0, 0, true);
    odom_step(&s, &c, 9, 1, 1, true);
    assert(s.valid && odom_speed_mm_s(&s) == 100);
    odom_step(&s, &c, 29, INT_MAX, INT_MAX, true);
    assert(!s.valid);
    puts("INT-02 odometry: wrap, scale, speed, yaw, gaps and discontinuities "
         "PASS");
}
static void
terrain (void)
{
    terrain_t s    = {0};
    int32_t   a[3] = {0, 0, 1000};
    terrain_step(&s, 0, a, true, 0, true, false, 0, 0);
    assert(!s.tilt_valid && !s.height_valid);
    terrain_step(&s, 0, a, true, 0, true, true, 0, 0);
    assert(s.tilt_valid);
    for (uint32_t ms = 20; ms <= 80; ms += 20)
    {
        terrain_step(&s, ms, a, true, 0, true, true, 0, 0);
    }
    assert(s.state == TERRAIN_FLAT);
    int      distance = 0;
    uint32_t time     = 80;
    /* Robot frame, +1 g up at rest: nose up 30 deg puts +0.5 g on front. */
    a[0] = 500;
    a[2] = 866;
    for (unsigned n = 0; n < 5; ++n)
    {
        distance += 20000;
        time += 20;
        terrain_step(&s, time, a, true, distance, true, true, 0, 0);
    }
    assert(terrain_pitch_cdeg(&s) == 3000 && terrain_roll_cdeg(&s) == 0);
    assert(s.state == TERRAIN_ASCENT && s.peak_um > 49000);
    a[0] = 0;
    a[2] = 1000;
    time += 20;
    terrain_step(&s, time, a, true, distance, true, true, 0, 0);
    assert(s.state == TERRAIN_CREST);
    a[0] = -500;
    a[2] = 866;
    for (unsigned n = 0; n < 5; ++n)
    {
        distance += 20000;
        time += 20;
        terrain_step(&s, time, a, true, distance, true, true, 0, 0);
    }
    assert(s.state == TERRAIN_DESCENT && !s.height_valid);
    a[0] = 0;
    a[2] = 1000;
    for (unsigned n = 0; n < 5; ++n)
    {
        time += 20;
        terrain_step(&s, time, a, true, distance, true, true, 0, 0);
    }
    assert(s.state == TERRAIN_FLAT && s.height_valid && s.height_mm == 50 &&
           s.maximum_mm == 50);
    time += 20;
    a[0] = 1900;
    terrain_step(&s, time, a, true, distance, true, true, 0, 0);
    assert(s.shock && s.state == TERRAIN_SHOCK && s.maximum_mm == 50 &&
           !s.height_valid);
    time += 20;
    a[0] = 0;
    terrain_step(&s, time, a, false, distance, true, true, 0, 0);
    assert(!s.tilt_valid);
    puts("INT-03 terrain: calibration gate, ordered hump, maximum, impact and "
         "invalid samples PASS");
}
static void
mount (void)
{
    /* This robot: sensor -Y to the front, +X to the left, +Z up. */
    int32_t raw[3] = {-48, -252, 911}, r[3];
    s4_imu_to_robot(raw, r, -2, 1, 3);
    assert(r[0] == 252 && r[1] == -48 && r[2] == 911);
    s4_imu_to_robot(raw, r, 1, 2, 3);
    assert(r[0] == -48 && r[1] == -252 && r[2] == 911);
    /* Saturated -2048 on a negated axis stays in the accepted range. */
    raw[1] = -2048;
    s4_imu_to_robot(raw, r, -2, 1, 3);
    assert(r[0] == 2047);
    raw[1] = 2047;
    s4_imu_to_robot(raw, r, -2, 1, 3);
    assert(r[0] == -2047);
    assert(s4_imu_axis(raw, 0) == 0 && s4_imu_axis(raw, 4) == 0 &&
           s4_imu_axis(raw, -4) == 0);
    /* Level, the mounting offset becomes the zeros; lifting the front
     * ~10 deg then reads positive pitch, lifting the left positive roll. */
    terrain_t t      = {0};
    int32_t   lvl[3] = {-48, -252, 911};
    s4_imu_to_robot(lvl, r, -2, 1, 3);
    terrain_step(&t, 0, r, true, 0, false, true, 1544, -302);
    assert(t.tilt_valid && abs(terrain_pitch_cdeg(&t)) <= 2 &&
           abs(terrain_roll_cdeg(&t)) <= 2);
    int32_t nose[3] = {-48, -406, 867};
    s4_imu_to_robot(nose, r, -2, 1, 3);
    terrain_step(&t, 20, r, true, 0, false, true, 1544, -302);
    assert(t.tilt_valid && terrain_pitch_cdeg(&t) > 800 &&
           terrain_pitch_cdeg(&t) < 1200);
    int32_t side[3] = {110, -252, 905};
    s4_imu_to_robot(side, r, -2, 1, 3);
    terrain_step(&t, 40, r, true, 0, false, true, 1544, -302);
    assert(t.tilt_valid && terrain_roll_cdeg(&t) > 800 &&
           terrain_roll_cdeg(&t) < 1300);
    /* Side tilt past 15 deg drops tilt; the same nose-up does not. */
    int32_t steep[3] = {300, -252, 870};
    s4_imu_to_robot(steep, r, -2, 1, 3);
    terrain_step(&t, 60, r, true, 0, false, true, 1544, -302);
    assert(!t.tilt_valid);
    int32_t climb[3] = {-48, -560, 800};
    s4_imu_to_robot(climb, r, -2, 1, 3);
    terrain_step(&t, 80, r, true, 0, false, true, 1544, -302);
    assert(t.tilt_valid && terrain_pitch_cdeg(&t) > 1800);
    puts("IMU mount: axis remap, saturation, nose-up pitch and side roll PASS");
}
/* Avoidance arc smoothing runs on elapsed ms, not VC ticks (the VC asks for
 * 10 ms but runs every 20 ms): at 5, 10 or 20 ms ticks the power reaches
 * AVOID_POWER, and the steering full scale, on the first tick at or after
 * AVOID_RAMP_POWER_MS / AVOID_STEER_RAMP_MS. At 20 ms ticks the steps are the
 * ones the robot was tested with: 625 milli-% (25% x 10/400 ms) and 13 steer
 * units (400 x 10/300 ms) a tick. */
static void
avoid_ramp (void)
{
    static const uint32_t ticks[] = {5U, 10U, 20U};
    const int32_t         full    = (int32_t) AVOID_POWER * 1000;
    for (unsigned k = 0; k < 3U; ++k)
    {
        uint32_t        tick = ticks[k], power_at = 0, steer_at = 0;
        vc_avoid_ramp_t r;
        vc_avoid_ramp_reset(&r, 5000U); /* last tick outside the arc */
        for (uint32_t t = tick; t <= 1000U; t += tick)
        {
            vc_avoid_ramp_step(&r, 5000U + t, AVOID_POWER, 400);
            int32_t p  = (int32_t) ((uint64_t) full * t / AVOID_RAMP_POWER_MS),
                    st = (int32_t) (400U * t / AVOID_STEER_RAMP_MS);
            assert(r.power_m == (p < full ? p : full) &&
                   r.steer == (st < 400 ? st : 400));
            if (!power_at && r.power_m == full)
            {
                power_at = t;
            }
            if (!steer_at && r.steer == 400)
            {
                steer_at = t;
            }
            if (tick == 20U)
            {
                int32_t n = (int32_t) (t / tick);
                assert(r.power_m == (625 * n < 25000 ? 625 * n : 25000) &&
                       r.steer == (13 * n < 400 ? 13 * n : 400));
            }
        }
        assert(power_at == (AVOID_RAMP_POWER_MS + tick - 1U) / tick * tick);
        assert(steer_at == (AVOID_STEER_RAMP_MS + tick - 1U) / tick * tick);
    }
    assert(AVOID_POWER == 25U && AVOID_RAMP_POWER_MS == 800U &&
           AVOID_STEER_RAMP_MS == 615U);
    /* A drop is immediate and the power rises again at the ramp rate; the
     * steering slews toward a reversed target at the same rate; a late tick
     * moves by its real elapsed ms. */
    vc_avoid_ramp_t r;
    vc_avoid_ramp_reset(&r, 0);
    for (uint32_t t = 20U; t <= 1000U; t += 20U)
    {
        vc_avoid_ramp_step(&r, t, 25U, 400);
    }
    assert(r.power_m == 25000 && r.steer == 400);
    vc_avoid_ramp_step(&r, 1020U, 20U, 400);
    assert(r.power_m == 20000);
    vc_avoid_ramp_step(&r, 1040U, 25U, -376);
    assert(r.power_m == 20625 && r.steer == 387);
    vc_avoid_ramp_step(&r, 1080U, 25U, -376);
    assert(r.power_m == 21875 && r.steer == 361);
    vc_avoid_ramp_reset(&r, 1100U);
    assert(r.power_m == 0 && r.steer == 0);
    vc_avoid_ramp_step(&r, 1120U, 25U, -376);
    assert(r.power_m == 625 && r.steer == -13);
    puts("Avoidance ramps: real ms at 5/10/20 ms ticks, tested 20 ms steps, "
         "drops and slews PASS");
}
static void
sensor_waits (void)
{
    vc_input_t i       = healthy();
    vc_state_t s       = started();
    s.phase            = VC_BACKUP;
    s.entered_ms       = 100;
    i.backup_ready     = 0;
    i.sensor_acquiring = 0;
    i.now_ms           = 120;
    vc_step(&s, &i);
    assert(!s.command && !s.backup_moving);
    i.now_ms = 100 + AVOID_BACKUP_SETTLE_MS - 1U;
    vc_step(&s, &i);
    assert(s.armed && !s.command);
    i.backup_ready = 1;
    vc_step(&s, &i);
    assert(s.command == 2 && s.backup_started_ms == i.now_ms);
    i.now_ms += AVOID_BACKUP_MS - 1U;
    vc_step(&s, &i);
    assert(s.command == 2);
    i.now_ms += 1U;
    vc_step(&s, &i);
    assert(s.backup_braking && !s.command);
    s              = started();
    s.phase        = VC_BACKUP;
    s.entered_ms   = 100;
    i              = healthy();
    i.backup_ready = 0;
    i.now_ms       = 100 + AVOID_BACKUP_SETTLE_MS;
    vc_step(&s, &i);
    assert(!s.armed && s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_SONAR);
    for (unsigned phase = VC_TURN_OUT; phase <= VC_RESTORE; ++phase)
    {
        s             = started();
        s.phase       = (uint8_t) phase;
        s.entered_ms  = 100;
        s.turn_deg    = 40;
        s.approach_mm = s.return_mm = 200;
        i                           = healthy();
        i.front_clear               = 0;
        i.front_pending             = 1;
        i.now_ms                    = 100;
        vc_step(&s, &i);
        assert(s.armed && !s.command && s.front_wait);
        i.now_ms += AVOID_FRONT_WAIT_MS - 1U;
        vc_step(&s, &i);
        assert(s.armed && !s.command);
        i.now_ms += 1U;
        vc_step(&s, &i);
        assert(s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_SONAR);
    }
    /* DIAGONAL and ALIGN deadlines also apply during pan and stale-data waits.
     */
    for (unsigned phase = VC_DIAGONAL; phase <= VC_ALIGN; ++phase)
    {
        s               = started();
        s.phase         = (uint8_t) phase;
        s.entered_ms    = 100;
        i               = healthy();
        i.now_ms        = 100 + 10000U;
        i.front_clear   = 0;
        i.front_pending = 1;
        i.sensor_fresh  = 0;
        vc_step(&s, &i);
        assert(s.armed && !s.command);
        i.now_ms += 1U;
        vc_step(&s, &i);
        assert(s.fault == VC_TIMEOUT && !s.command);
    }
    /* Not-ready side echoes preserve both kinds of streak, bounded by 4 s. */
    for (unsigned clear = 0; clear < 2U; ++clear)
    {
        s                       = started();
        s.phase                 = VC_PASS;
        s.look_side             = 1;
        s.side_check_started_ms = 100;
        i                       = healthy();
        i.front_clear           = 0;
        i.side_observed         = 1;
        i.side_clear            = clear;
        for (unsigned n = 0; n < 5U; ++n)
        {
            i.now_ms = i.side_sample_ms = 120 + n * 20U;
            i.side_ready                = (n % 2U) == 0U;
            vc_step(&s, &i);
            assert(!s.command);
        }
        assert(!s.look_side && s.clear_run == clear);
    }
    s                       = started();
    s.phase                 = VC_PASS;
    s.look_side             = 1;
    s.side_check_started_ms = 100;
    i                       = healthy();
    i.now_ms                = 100 + AVOID_SIDE_CHECK_TIMEOUT_MS;
    vc_step(&s, &i);
    assert(s.fault == VC_SENSOR && s.fault_cause == VC_CAUSE_SONAR);
    puts("Sensor waits: APPROACH sample, full reverse, forward pan bound, "
         "phase deadlines and intermittent side echoes PASS");
}
int
main (void)
{
    sensor_waits();
    controller();
    odometry();
    terrain();
    mount();
    avoid_ramp();
    return 0;
}

/*** end of file ***/
