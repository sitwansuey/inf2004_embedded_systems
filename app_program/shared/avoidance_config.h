#ifndef AVOIDANCE_CONFIG_H
#define AVOIDANCE_CONFIG_H
#define AVOID_MAX_WIDTH_MM 500U
#define AVOID_CLEARANCE_MM 75U
#define AVOID_POWER 25U
#define AVOID_SIDE_CLEAR_MM 200U
#define AVOID_SIDE_CONFIRM_SAMPLES 3U
#define AVOID_SIDE_CHECK_TIMEOUT_MS 4000U
/* VC_PASS drives this far between side looks (the first look is at the end
 * of the out diagonal, before aligning). */
#define AVOID_PASS_STEP_MM 100U
/* A clear side look (end of the diagonal or in VC_PASS): the servo returns
 * to the front, the robot drives on straight at least this far and for at
 * least AVOID_CLEAR_RUN_MS of driving (timed from its first drive tick, so
 * the stop for the forward check does not count), then turns back. */
#define AVOID_CLEAR_RUN_MM 20U
#ifndef AVOID_CLEAR_RUN_MS
#define AVOID_CLEAR_RUN_MS 500U
#endif
/* Smooth avoidance driving (TURN_OUT..RESTORE, vehicle.c): power rises from 0
 * to the avoidance power over AVOID_RAMP_POWER_MS (drops stay immediate) and
 * the arc steering moves at most full scale (400) per AVOID_STEER_RAMP_MS,
 * in real ms (vehicle_policy.c vc_avoid_ramp_step). These are what the robot
 * was tested with when the ramps stepped once per 20 ms VC tick as if it were
 * 10 ms (400/300 then): 625 milli-% a tick = 25% in 800 ms; 13 steer units a
 * tick = 400 in 615 ms (8000/13 = 615.4; 615 keeps 13 a tick, 616 gives 12). */
#define AVOID_RAMP_POWER_MS 800U
/* After the reverse the bypass heading is re-planned (vehicle.c): the
 * shallowest heading, not below this, that still clears the obstacle edge. */
#define AVOID_MIN_TURN_DEG 20U
#define AVOID_STEER_RAMP_MS 615U
/* VC_RETURN heads back toward the line until a sensor finds it. The mirrored
 * diagonal is only where the line is expected: drive on up to this much
 * further before restoring the heading for an on-the-spot search. */
#define AVOID_RETURN_EXTRA_MM 250U
/* VC_RETURN heads back toward the line at least this steeply (deg from the
 * pre-obstacle heading), so it rejoins nearer the obstacle. A steeper
 * diagonal keeps its own angle. The return distance is the diagonal's
 * lateral offset at this heading. 0: mirror the diagonal (before 8c-2). */
#ifndef AVOID_RETURN_MIN_DEG
#define AVOID_RETURN_MIN_DEG 45U
#endif
#if AVOID_RETURN_MIN_DEG > 80
#error AVOID_RETURN_MIN_DEG must be 0..80
#endif
/* VC_BACKUP: straight reverse after the plan is latched, ended by time or
 * by the odometry distance bound, whichever comes first. */
#define AVOID_BACKUP_MS 1000U
#define AVOID_BACKUP_MAX_MM 300U
#define AVOID_BACKUP_POWER 25U
/* Stay stopped until a new APPROACH sample at 0 deg follows the pan.
 * Fault with sonar cause if it has not arrived within this bound. The pan
 * back from an 80 deg plan alone settles 800 ms (10 ms/deg), after S5 ends
 * a wait of up to ~160 ms, so 1.5 s leaves margin over the ~1.05 s worst case. */
#ifndef AVOID_BACKUP_SETTLE_MS
#define AVOID_BACKUP_SETTLE_MS 1500U
#endif
/* Return from a side look: wait for a clear forward sample before driving.
 * A 90 deg pan back settles ~910 ms; in open space S5 then needs 3 no-echo
 * pings (~160 ms each) to call it clear: ~1.6 s worst case, so 3 s. */
#ifndef AVOID_FRONT_WAIT_MS
#define AVOID_FRONT_WAIT_MS 3000U
#endif
/* Ray check of a shallower heading after reversing; timeout keeps S5's plan. */
#ifndef AVOID_REPLAN_WAIT_MS
#define AVOID_REPLAN_WAIT_MS 1000U
#endif
/* Odometry moving forward by more than this while reversing is a motor fault. */
#define AVOID_BACKUP_WRONG_WAY_MM 20U
/* The reverse ramps START_POWER -> POWER over RAMP_MS and back down over its
 * last RAMP_MS: a step into or out of 25% rocks the robot, and the gravity-only
 * tilt drops out (VC_SENSOR tilt) or reads above 1.8 g. */
#define AVOID_BACKUP_START_POWER 10U
#define AVOID_BACKUP_RAMP_MS 300U
/* Then stand still before the forward turn-out (a direct reverse to forward
 * switch measured 2.3 g): at least BRAKE_MS, until odometry has stayed within
 * STILL_UM for STILL_MS, at most BRAKE_MAX_MS. */
#define AVOID_BACKUP_BRAKE_MS 300U
#define AVOID_BACKUP_STILL_MS 150U
#define AVOID_BACKUP_STILL_UM 1000
#define AVOID_BACKUP_BRAKE_MAX_MS 1000U
#endif
