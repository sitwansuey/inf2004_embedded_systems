/* Main vehicle controller and application entry point. */
#include "app.h"
#include "subsystem2/subsystem2.h"
#include "subsystem5/subsystem5.h"
#include "app_state.h"
#include "app_logging.h"
/*
 * Create and start one task (declared in app.h for every subsystem).
 *   prc = TP_PRC1 (core 0), TP_PRC2 (core 1), or 0 for "any core".
 */
EXPORT ID app_start_task(FP entry, PRI pri, UW prc, const char *name)
{
	T_CTSK ctsk = {0};
	ID tid;
	ER er;

	ctsk.task = entry;			  /* function the task runs */
	ctsk.itskpri = pri;			  /* initial priority (1 = highest) */
	ctsk.stksz = TASK_STACK_SIZE; /* kernel allocates the stack */
	ctsk.tskatr = TA_HLNG		  /* written in a high-level language (C) */
				  | TA_RNG3;	  /* runs at user protection level */
#if TK_SUPPORT_SMP
	if (prc != 0)
	{
		ctsk.tskatr |= TA_ASSPRC; /* honour the affinity below */
		ctsk.assprc = prc;		  /* which core(s) it may run on */
	}
#else
	(void)prc; /* single-core build: no affinity */
#endif

	/* tk_cre_tsk: create the task.  It starts in the DORMANT state. */
	tid = tk_cre_tsk(&ctsk);
	if (tid < E_OK)
	{
		CHECK(tid, name);
		return tid;
	}

	/* tk_sta_tsk: DORMANT -> READY.  The int is passed as stacd. */
	er = tk_sta_tsk(tid, 0);
	CHECK(er, name);
	if (er < E_OK) { tk_del_tsk(tid); return er; }
	return tid;
}

#if !APP_S2_ONLY
static ID vehicle_mutex;
static struct {
	s2_move_command_t command;
	UINT power;
	UW expires;
	BOOL active;
} requests[5];

EXPORT ER app_vehicle_request(UINT subsystem, s2_move_command_t command,
                              UINT power, UW lease_ms)
{
	if (subsystem < 1 || subsystem > 4 || command > S2_MOVE_PIVOT_LEFT ||
	    command < S2_MOVE_STOP || power > 100 || lease_ms == 0 ||
	    lease_ms > 60000) return E_PAR;
	ER er = tk_loc_mtx(vehicle_mutex, TMO_POL);
	if (er < E_OK) return er;
	SYSTIM now;
	tk_get_tim(&now);
	requests[subsystem].command = command;
	requests[subsystem].power = power;
	requests[subsystem].expires = now.lo + lease_ms;
	requests[subsystem].active = TRUE;
	tk_unl_mtx(vehicle_mutex);
	return E_OK;
}

static ER vehicle_init(void)
{
	T_CMTX cfg = {0};
	cfg.mtxatr = TA_INHERIT;
	vehicle_mutex = tk_cre_mtx(&cfg);
	return vehicle_mutex < E_OK ? vehicle_mutex : E_OK;
}

static UW vehicle_cruise_ms(UW millimetres)
{
    return (millimetres * 1000U + S5_CRUISE_MM_PER_SECOND - 1U) /
        S5_CRUISE_MM_PER_SECOND;
}

static BOOL vehicle_enter(vehicle_fsm_t *drive, UW now_ms,
    vehicle_phase_t phase, s5_state_t state, INT look, UW duration)
{
    ER er = s5_navigation_set(drive->scan_id, state, look);
    if (er < E_OK) return FALSE;
    drive->phase = phase;
    drive->remaining_ms = duration;
    drive->motion_started = FALSE;
    drive->last_ms = now_ms;
    drive->deadline_ms = now_ms + S5_ROTATE_WAIT_MS;
    return TRUE;
}

static void vehicle_fsm_step(vehicle_fsm_t *drive, UW now_ms,
    const s5_status_t *sensor, BOOL sensor_ok, BOOL external_request,
    s2_move_command_t *command, UINT *power)
{
    *command = S2_MOVE_STOP;
    *power = 0;
    s5_path_t path = s5_assess_path(sensor, sensor_ok, now_ms);
    if (drive->phase == VEHICLE_CRUISE) {
        if (external_request || now_ms-drive->boot_ms < VEHICLE_START_DELAY_MS) return;
        if (path.state == S5_PATH_FORWARD) {
            *command = S2_MOVE_FORWARD;
            *power = VEHICLE_FORWARD_POWER;
            return;
        }
        if (path.state == S5_PATH_IDENTIFY || path.state == S5_PATH_READY) {
            drive->phase = VEHICLE_ACQUISITION;
            tm_printf((UB *)"[AVOID] acquisition: identify target and clearance angle\n");
        } else return;
    }
    if (drive->phase == VEHICLE_ACQUISITION) {
        if (external_request) return;
        if (path.state == S5_PATH_FORWARD) {
            drive->phase = VEHICLE_CRUISE;
            return;
        }
        if (path.state != S5_PATH_READY) return;
        drive->scan_id = sensor->scan_id;
        drive->side = path.turn_deg < 0 ? -1 : 1;
        drive->turn_ms = path.turn_ms;
        drive->approach_mm = path.approach_mm;
        drive->return_mm = path.return_mm;
        if (vehicle_enter(drive, now_ms, VEHICLE_APPROACH_ROTATE, S5_APPROACH, 0, path.turn_ms))
            tm_printf((UB *)"[AVOID] approach: rotate %d deg; cruise %u mm; rotate back\n",
                      path.turn_deg, path.approach_mm);
        return;
    }
    if (drive->phase == VEHICLE_ABORT) {
        /* Do not resume a partially completed open-loop route. */
        if (!external_request && sensor_ok &&
            s5_navigation_set(sensor->scan_id, S5_MONITOR, 0) >= E_OK)
            drive->phase = VEHICLE_CRUISE;
        return;
    }
    /* A 90-degree servo move can take longer than the sample stale limit.
     * Wait only while stopped and S5 explicitly marks its sample invalid;
     * motion still requires a fresh sample, within the settling timeout. */
    BOOL settling = sensor_ok && !drive->motion_started && !sensor->valid;
    if (external_request || (path.state == S5_PATH_UNAVAILABLE && !settling) ||
        path.state == S5_PATH_FAULT) {
        drive->phase = VEHICLE_ABORT;
        drive->motion_started = FALSE;
        return;
    }
    BOOL side_view = drive->phase == VEHICLE_OVERCOME_CRUISE;
    s5_state_t expected = vehicle_avoidance_stage(drive->phase) == S5_STAGE_APPROACH ? S5_APPROACH :
        vehicle_avoidance_stage(drive->phase) == S5_STAGE_OVERCOME ? S5_OVERCOME : S5_REORG;
    INT look = side_view ? -90 * drive->side : 0;
    if (drive->phase == VEHICLE_RESTORE) expected = S5_MONITOR;
    BOOL sample_ready = sensor->valid && sensor->state == expected &&
        sensor->angle_deg == look && sensor->sample_angle_deg == look;
    if (!sample_ready) {
        if (drive->motion_started || (W)(now_ms-drive->deadline_ms) >= 0)
            drive->phase = VEHICLE_ABORT;
        return;
    }
    if (!side_view && sensor->distance_mm <= S5_GUIDANCE_OBSTACLE_MM) {
        drive->phase = VEHICLE_ABORT;
        return;
    }
    if (drive->phase == VEHICLE_RESTORE) {
        drive->phase = VEHICLE_CRUISE;
        tm_printf((UB *)"[AVOID] reorg complete: original heading restored\n");
        return;
    }
    if (!drive->motion_started) {
        drive->motion_started = TRUE;
        drive->last_ms = now_ms;
        drive->deadline_ms = now_ms + S5_NAVIGATION_TIMEOUT_MS;
    } else {
        UW elapsed = now_ms-drive->last_ms;
        drive->remaining_ms = elapsed >= drive->remaining_ms ? 0 : drive->remaining_ms-elapsed;
        drive->last_ms = now_ms;
    }
    if ((W)(now_ms-drive->deadline_ms) >= 0) {
        drive->phase = VEHICLE_ABORT;
        return;
    }
    /* Every handoff emits STOP before changing the sensor direction. */
    if (side_view && sensor->distance_mm > S5_OBJECT_RANGE_MM) {
        vehicle_enter(drive, now_ms, VEHICLE_OVERCOME_CLEARANCE, S5_OVERCOME, 0,
                      vehicle_cruise_ms(S5_OVERCOME_CLEARANCE_MM));
        return;
    }
    if (!side_view && drive->remaining_ms == 0) {
        switch (drive->phase) {
        case VEHICLE_APPROACH_ROTATE:
            vehicle_enter(drive, now_ms, VEHICLE_APPROACH_CRUISE, S5_APPROACH, 0,
                          vehicle_cruise_ms(drive->approach_mm)); break;
        case VEHICLE_APPROACH_CRUISE:
            vehicle_enter(drive, now_ms, VEHICLE_APPROACH_ROTATE_BACK, S5_APPROACH, 0,
                          drive->turn_ms); break;
        case VEHICLE_APPROACH_ROTATE_BACK:
            if (vehicle_enter(drive, now_ms, VEHICLE_OVERCOME_CRUISE, S5_OVERCOME,
                              -90*drive->side, 0))
                tm_printf((UB *)"[AVOID] overcome: look toward object; then cruise another 200 mm\n");
            break;
        case VEHICLE_OVERCOME_CLEARANCE:
            if (vehicle_enter(drive, now_ms, VEHICLE_REORG_ROTATE90, S5_REORG, 0,
                              90U*S5_ROTATE_MS_PER_DEG))
                tm_printf((UB *)"[AVOID] reorg: rotate 90 toward line; cruise %u mm; rotate -90\n",
                          drive->return_mm);
            break;
        case VEHICLE_REORG_ROTATE90:
            vehicle_enter(drive, now_ms, VEHICLE_REORG_CRUISE, S5_REORG, 0,
                          vehicle_cruise_ms(drive->return_mm)); break;
        case VEHICLE_REORG_CRUISE:
            vehicle_enter(drive, now_ms, VEHICLE_REORG_ROTATE_MINUS90, S5_REORG, 0,
                          90U*S5_ROTATE_MS_PER_DEG); break;
        case VEHICLE_REORG_ROTATE_MINUS90:
            vehicle_enter(drive, now_ms, VEHICLE_RESTORE, S5_MONITOR, 0, 0); break;
        default: drive->phase = VEHICLE_ABORT; break;
        }
        return;
    }
    switch (drive->phase) {
    case VEHICLE_APPROACH_ROTATE: case VEHICLE_REORG_ROTATE_MINUS90:
        *command = drive->side > 0 ? S2_MOVE_LEFT : S2_MOVE_RIGHT;
        *power = VEHICLE_TURN_POWER; break;
    case VEHICLE_APPROACH_ROTATE_BACK: case VEHICLE_REORG_ROTATE90:
        *command = drive->side > 0 ? S2_MOVE_RIGHT : S2_MOVE_LEFT;
        *power = VEHICLE_TURN_POWER; break;
    default:
        *command = S2_MOVE_FORWARD;
        *power = VEHICLE_FORWARD_POWER; break;
    }
}

static void vehicle_task(INT stacd, void *exinf)
{
	(void)stacd; (void)exinf;
	s2_controller_bind();
	s2_move_command_t previous = S2_MOVE_STOP;
	UINT previous_power = 0;
	SYSTIM boot;
	tk_get_tim(&boot);
	vehicle_fsm_t drive = { .phase = VEHICLE_CRUISE, .boot_ms = boot.lo };
	tm_printf((UB *)"[AUTO] forward at %u%% after %u ms; obstacle threshold %u mm\n",
	          VEHICLE_FORWARD_POWER, VEHICLE_START_DELAY_MS, S5_OBSTACLE_MM);
	/* Larger value wins; S5 safety overrides every ordinary request. */
	const UINT importance[5] = {0, 10, 20, 30, 40};
	app_shared_state_t next = {0};
	UW next_tick = boot.lo;
	for (;;) {
		SYSTIM now;
		tk_get_tim(&now);
		s2_move_command_t command = S2_MOVE_STOP;
		UINT power = 0, best = 0;
		if (tk_loc_mtx(vehicle_mutex, TMO_POL) >= E_OK) {
			for (UINT i = 1; i <= 4; ++i) {
				if (requests[i].active && (W)(now.lo - requests[i].expires) >= 0)
					requests[i].active = FALSE;
				if (requests[i].active && importance[i] > best) {
					best = importance[i]; command = requests[i].command;
					power = requests[i].power;
				}
			}
			tk_unl_mtx(vehicle_mutex);
		}
        app_state_collect(&next);
        s5_status_t sensor = next.s5;
        BOOL sensor_ok = next.s5_available;
        now.lo = next.time_ms;
		s2_move_command_t auto_command;
		UINT auto_power;
		vehicle_fsm_step(&drive, now.lo, &sensor, sensor_ok, best != 0,
		                      &auto_command, &auto_power);
		if (best == 0) {
			command = auto_command;
			power = auto_power;
		} else if (now.lo - drive.boot_ms < VEHICLE_START_DELAY_MS ||
                   drive.phase != VEHICLE_CRUISE ||
                   s5_assess_path(&sensor, sensor_ok, now.lo).state != S5_PATH_FORWARD) {
			command = S2_MOVE_STOP; power = 0;
		}
		if (command == S2_MOVE_STOP) power = 0;
		if (command != previous || power != previous_power) {
			ER er = s2_movement_send(command, power);
			if (er >= E_OK) {
				previous = command; previous_power = power;
				tm_printf((UB *)"[AUTO] motor command %d, power %u%%\n", command, power);
			}
			else CHECK(er, "vehicle movement");
		}
        next.phase = drive.phase;
        next.avoidance_stage = vehicle_avoidance_stage(drive.phase);
        next.movement = previous;
        next.power_percent = previous_power;
        CHECK(app_state_publish(&next), "publish vehicle state");
        app_log_state(&next);
        /* Deadline scheduling accounts for work time instead of adding it
         * to the 10 ms period. Skip missed ticks rather than busy catching up. */
        next_tick += APP_STATE_PERIOD_MS;
        tk_get_tim(&now);
        W remaining = (W)(next_tick - now.lo);
        if (remaining <= 0) {
            next_tick = now.lo + APP_STATE_PERIOD_MS;
            remaining = APP_STATE_PERIOD_MS;
        }
        tk_dly_tsk((UW)remaining);
	}
}
#endif

EXPORT INT usermain(void)
{
	T_RVER ver;

	/* tk_ref_ver: ask the kernel who it is */
	tk_ref_ver(&ver);
	tm_printf((UB *)"\n=== micro T-Kernel on Raspberry Pi Pico W"
					" (spec 0x%04x, product 0x%04x) ===\n",
			  ver.spver, ver.prver);
#if TK_SUPPORT_SMP
	tm_printf((UB *)"dual-core (SMP) build, usermain on core %d\n",
			  tk_get_prc() - 1); /* tk_get_prc() counts from 1 */
#else
	tm_printf((UB *)"single-core build\n");
#endif

#if APP_S2_ONLY
	/* Isolated movement test; no example producer/monitor tasks. */
	tm_printf((UB *)"[S2 ONLY] movement test firmware\n");
	ER er = subsystem2_init();
	CHECK(er, "subsystem2_init");
	if (er >= E_OK) {
		er = subsystem2_start();
		CHECK(er, "subsystem2_start");
		if (er >= E_OK) {
			subsystem2_test();
		}
	}
#else
	ER er = subsystem2_init();
	CHECK(er, "subsystem2_init");
	if (er >= E_OK) er = subsystem5_init();
	CHECK(er, "subsystem5_init");
	if (er >= E_OK) er = app_state_init();
	CHECK(er, "app_state_init");
	if (er >= E_OK) er = vehicle_init();
	CHECK(er, "vehicle_init");
	if (er >= E_OK) er = subsystem2_start();
	if (er >= E_OK) er = subsystem5_start();
    if (er >= E_OK) {
#if APP_S5_MASTER_TEST
        tm_printf((UB *)"[S5 MASTER TEST] acquisition then approach\n");
#elif APP_ACQUISITION_TEST
        tm_printf((UB *)"[ACQUISITION TEST] acquire left then right bound; GP22 double beep\n");
#elif APP_ACQUISITION_LEFT_TEST
        tm_printf((UB *)"[ACQUISITION LEFT TEST] forward to 15 cm, find left bound, beep GP22\n");
#elif APP_TEST
        tm_printf((UB *)"[S5 TEST] sensor sweep firmware; motors remain stopped\n");
#else
        ID tid = app_start_task((FP)vehicle_task, 10, TP_PRC1, "vehicle");
        CHECK(tid, "vehicle start");
#endif
    } else CHECK(er, "vehicle startup");
#endif /* APP_S2_ONLY */

	/* Sleep forever.  Returning from usermain() would shut down. */
	tk_slp_tsk(TMO_FEVR);
	return 0;
}
