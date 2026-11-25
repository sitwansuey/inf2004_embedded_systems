#include "vehicle_policy.h"
#include <stdlib.h>
/* Commands match S2: STOP=0 FORWARD=1 REVERSE=2 LEFT=3 RIGHT=4 PIVOT_R=5 PIVOT_L=6. */
/* sin of whole degrees 0..90, x1000. */
static const uint16_t sin_milli[91]={
    0,17,35,52,70,87,105,122,139,156,174,191,208,
    225,242,259,276,292,309,326,342,358,375,391,407,423,
    438,454,469,485,500,515,530,545,559,574,588,602,616,
    629,643,656,669,682,695,707,719,731,743,755,766,777,
    788,799,809,819,829,839,848,857,866,875,883,891,899,
    906,914,921,927,934,940,946,951,956,961,966,970,974,
    978,982,985,988,990,993,995,996,998,999,999,1000,1000};
/* Bypass return heading toward the line (deg, > 0): the diagonal's angle,
 * raised to AVOID_RETURN_MIN_DEG. */
static int return_deg(const vc_state_t *s) {
    int a=abs(s->turn_deg);
    return a<(int)AVOID_RETURN_MIN_DEG?(int)AVOID_RETURN_MIN_DEG:a;
}
/* Clear run after a clear side look: done once it has driven at least
 * AVOID_CLEAR_RUN_MM and AVOID_CLEAR_RUN_MS, timed from its first drive tick
 * (this call starts the timer). */
static bool clear_run_done(vc_state_t *s,const vc_input_t *i) {
    if (!s->clear_run_timing) {s->clear_run_timing=1;s->clear_run_ms=i->now_ms;return false;}
    return (int64_t)i->distance_um-s->origin_um>=(int64_t)AVOID_CLEAR_RUN_MM*1000 &&
        i->now_ms-s->clear_run_ms>=AVOID_CLEAR_RUN_MS;
}
/* Distance (mm) back to the line at the return heading: the diagonal's
 * lateral offset (return_mm along turn_deg) divided by sin(return heading). */
static uint32_t return_limit_mm(const vc_state_t *s) {
    int a=abs(s->turn_deg),r=return_deg(s);
    if (r==a || a>90) return s->return_mm;
    return (uint32_t)(((uint32_t)s->return_mm*sin_milli[a]+sin_milli[r]/2U)/sin_milli[r]);
}
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power) {
    if (!s || lease==0 || lease>1000 || power<1 || power>50) return false;
    if (s->phase==VC_FAULT) return false;
    s->armed_ms=now;s->lease_ms=lease;s->maximum_power=(uint8_t)power;s->armed=1;s->fault_cause=0;
    if (s->phase==VC_DISARMED) { s->phase=VC_TRACK; s->entered_ms=now; }
    return true;
}
bool vc_renew(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power) {
    /* Only a leased ARM run: not disarmed, not a START run (lease 0), not
     * FAULT; and the run's own demand, so a renewal never changes speed
     * mid-run (soft start only ramps from a stop). */
    if (!s || !s->armed || s->lease_ms==0 || s->phase==VC_FAULT || power!=(unsigned)s->maximum_power) return false;
    return vc_arm(s,now,lease,power);
}
bool vc_start(vc_state_t *s, uint32_t now, unsigned power) {
    if (!vc_arm(s,now,1000U,power)) return false;
    s->lease_ms=0; /* Mission runs on onboard decisions until STOP/fault. */
    return true;
}
void vc_stop(vc_state_t *s,unsigned reason) {
    if (!s) return;
    s->command=0;s->power=0;s->steering=0;s->armed=0;s->look_side=0;
    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
    s->pending_nav=0;s->navigation_ack=0;s->track_wait=0;
    s->nav_search=0;s->nav_offset=0;s->spot_scan=0;s->uturn_guard=0;
    s->last_line_side=0;s->search_right_first=0;
    s->search_command=0;s->line_settling=0;s->line_found_ms=0;
    s->clear_run=0;s->clear_run_timing=0;s->line_gap=0;s->sonar_gap=0;s->odom_gap=0;s->front_wait=0;s->line_found=0;s->line_seen=0;
    s->rescans=0;s->rescan_wait=0;s->rescan_request=0;
    s->fault=(uint8_t)(reason&15U);s->phase=reason?VC_FAULT:VC_DISARMED;
    s->fault_cause=0; /* sensor_fault() sets it after this */
}
/* VC_SENSOR with the reason kept for telemetry while the fault is latched. */
static void sensor_fault(vc_state_t *s,unsigned cause) {
    vc_stop(s,VC_SENSOR);s->fault_cause=(uint8_t)(cause&3U);
}
/* VC_ACQUIRE, no way round: while rescans remain (and the bypass is enabled)
 * stay stopped in ACQUIRE and have vehicle.c reset S5 for a fresh scan from
 * the same spot; otherwise the sonar fault, as before. */
static void no_way_around(vc_state_t *s,const vc_input_t *i) {
#if AVOID_RESCAN_RETRIES
    if (s->rescans<AVOID_RESCAN_RETRIES && i->bypass_enabled && i->bypass_steering &&
        i->bypass_steering<=400U) {
        s->rescans=(uint8_t)(s->rescans+1U)&3U;
        s->rescan_id=i->scan_id;s->rescan_ms=i->now_ms;
        s->rescan_wait=1;s->rescan_request=1;
        return;
    }
#endif
    (void)i; /* unused when AVOID_RESCAN_RETRIES is 0 */
    sensor_fault(s,VC_CAUSE_SONAR);
}
/* VC_LINE with its reason (VC_LINE_CAUSE_*), likewise. */
static void line_fault(vc_state_t *s,unsigned cause) {
    vc_stop(s,VC_LINE);s->fault_cause=(uint8_t)(cause&3U);
}
/* Line data missing: hold still (no command); VC_LINE (line data) only once
 * the gap has lasted VC_LINE_GAP_MS. vc_step clears the gap on fresh data. */
static void line_gap_hold(vc_state_t *s,const vc_input_t *i) {
    if (!s->line_gap) {s->line_gap=1;s->line_gap_ms=i->now_ms;}
    else if (i->now_ms-s->line_gap_ms>=VC_LINE_GAP_MS) line_fault(s,VC_LINE_CAUSE_DATA);
}
void vc_track_requested(vc_state_t *s,uint32_t now) {
    if (!s) return;
    s->track_requested_ms=now;s->track_wait=1;
    s->command=0;s->power=0;s->steering=0;
}
static unsigned soft_start(vc_state_t *s,uint32_t now,bool driving,unsigned demand);
static int yaw_delta(int32_t now,int32_t origin);
static void enter(vc_state_t *s,const vc_input_t *i,unsigned phase) {
    bool continuous=s->armed && s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE &&
        phase>=VC_TURN_OUT && phase<=VC_RESTORE && !s->look_side && i->front_clear;
    /* A line touch on the way back carries over TURN_IN -> RETURN -> RESTORE. */
    bool keep_touch=s->phase>=VC_TURN_IN && s->phase<=VC_RESTORE &&
        phase>=VC_TURN_IN && phase<=VC_RESTORE;
    s->phase=(uint8_t)phase;s->entered_ms=i->now_ms;
    if (phase==VC_REACQUIRE) s->search_right_first=s->last_line_side==1U;
    s->origin_um=i->distance_um;s->origin_yaw=i->yaw_mdeg;
    s->turn_progress=0;
    if (phase==VC_RESTORE) s->restore_mdeg=yaw_delta(s->avoid_yaw,i->yaw_mdeg);
    s->command=0;s->power=0;s->steering=0;
    s->track_wait=0;
    s->search_command=0;s->line_settling=0;s->line_found_ms=0;
    s->nav_search=0;s->nav_offset=0;s->spot_scan=0; /* pending_nav itself is kept */
    if(!keep_touch) s->nav_touched=0;
    s->clear_run=0;s->clear_run_timing=0;s->line_gap=0;s->line_seen=0; /* TRACK measures line-lost from its entry */
    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
    if(continuous) {
        s->command=1;s->power=s->maximum_power>AVOID_POWER?AVOID_POWER:s->maximum_power;
        s->power=(uint8_t)soft_start(s,i->now_ms,s->drive_prev,s->power);
        if(i->slope && s->power>20U)s->power=20U;
        if(phase==VC_ALIGN || phase==VC_TURN_IN)s->steering=(int16_t)(s->turn_deg>0?(int)s->bypass_steering:-(int)s->bypass_steering);
        else if(phase==VC_RESTORE)s->steering=(int16_t)(s->restore_mdeg>0?-(int)s->bypass_steering:(int)s->bypass_steering);
    }
}
/* Power cap t ms into a ramp: AVOID_BACKUP_START_POWER rising to top over
 * AVOID_BACKUP_RAMP_MS. */
static unsigned soft_cap(uint32_t t,unsigned top) {
    if (t>=AVOID_BACKUP_RAMP_MS || top<=AVOID_BACKUP_START_POWER) return top;
    return AVOID_BACKUP_START_POWER+(unsigned)((top-AVOID_BACKUP_START_POWER)*t/AVOID_BACKUP_RAMP_MS);
}
/* VC_BACKUP power cap t ms into the reverse: up over the first RAMP_MS, back
 * down over the last RAMP_MS of AVOID_BACKUP_MS. */
static unsigned backup_ramp(uint32_t t) {
    uint32_t down=t<AVOID_BACKUP_MS?AVOID_BACKUP_MS-t:0U;
    return soft_cap(t<down?t:down,AVOID_BACKUP_POWER);
}
/* An avoidance arc (TURN_OUT..RESTORE) that starts from standstill - after the
 * reverse's stand-still, a side-look stop or a front-clear hold - ramps up the
 * same way: a step from 0 puts the outer wheel at the PWM cap at once (S2
 * applies the full steering on a fresh start) and rocks the robot. */
static unsigned soft_start(vc_state_t *s,uint32_t now,bool driving,unsigned demand) {
    if (!driving) {s->soft_start=1;s->soft_start_ms=now;}
    if (!s->soft_start) return demand;
    uint32_t t=now-s->soft_start_ms;
    if (t>=AVOID_BACKUP_RAMP_MS) {s->soft_start=0;return demand;}
    unsigned cap=soft_cap(t,AVOID_POWER);
    return demand>cap?cap:demand;
}
/* One step of a stopped side look (VC_DIAGONAL end, VC_PASS): 0 waiting,
 * 1 clear and 2 obstacle (AVOID_SIDE_CONFIRM_SAMPLES new samples in a row),
 * -1 no answer in AVOID_SIDE_CHECK_TIMEOUT_MS (stopped with VC_SENSOR). */
static int side_look(vc_state_t *s,const vc_input_t *i) {
    if(i->now_ms-s->side_check_started_ms>=AVOID_SIDE_CHECK_TIMEOUT_MS){sensor_fault(s,VC_CAUSE_SONAR);return -1;}
    if(!i->side_observed || (s->side_have_sample && i->side_sample_ms==s->side_last_ms))return 0;
    s->side_have_sample=1;s->side_last_ms=i->side_sample_ms;
    if(!i->side_ready)return 0; /* no answer: keep the last ready-sample streak */
    if(i->side_clear){++s->side_clear_count;s->side_present_count=0;}
    else {++s->side_present_count;s->side_clear_count=0;}
    if(s->side_clear_count>=AVOID_SIDE_CONFIRM_SAMPLES) return 1;
    if(s->side_present_count>=AVOID_SIDE_CONFIRM_SAMPLES) return 2;
    return 0;
}
static void start_side_look(vc_state_t *s,const vc_input_t *i) {
    s->look_side=1;s->side_check_started_ms=i->now_ms;
    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
}
static int yaw_delta(int32_t now,int32_t origin) {
    int d=now-origin; if(d>180000)d-=360000;if(d< -180000)d+=360000;return d;
}
void vc_step(vc_state_t *s,const vc_input_t *i) {
    if (!s || !i) return;
    unsigned was_driving=s->command==1 && s->power!=0;
    s->drive_prev=was_driving!=0;
    unsigned turn_power=0; /* VC_NAV_TURN demand, 0: maximum_power */
    unsigned backup_cap=0; /* VC_BACKUP ramp cap, 0: none */
    s->command=0;s->power=0;s->steering=0;
    s->navigation_ack=0;
    if (!s->armed) return;
    if (i->line_fresh && i->steering) s->line_gap=0;
    /* Both-black is straight; retain the last unambiguous search direction. */
    if (i->line_fresh && (i->line_black_mask==1U || i->line_black_mask==2U))
        s->last_line_side=i->line_black_mask;
    if (s->lease_ms && i->now_ms-s->armed_ms>=s->lease_ms) { vc_stop(s,VC_TIMEOUT);return; }
    if (i->shock) { vc_stop(s,VC_SHOCK);return; }
    if (i->safety_event) { line_fault(s,VC_LINE_CAUSE_EVENT);return; }
    if (!i->transport_ok) { vc_stop(s,VC_TRANSPORT);return; }
    if (!i->motor_ok) { vc_stop(s,VC_MOTOR);return; }
    /* Tilt drops when side roll exceeds 15 deg or |a| leaves 0.8..1.2 g.
     * Before the GY-511 axis remap the obstacle reverse's fore-aft rocking
     * tripped the roll limit; now only jolts (the norm gate) can. VC_BACKUP
     * (straight, slow, odometry-bounded) still does not need tilt; TURN_OUT
     * does, after the stand-still has let the robot settle. */
    bool tilt_ok=i->terrain_valid || s->phase==VC_BACKUP;
    if (!tilt_ok) {sensor_fault(s,VC_CAUSE_TILT);return;}
    /* S3 removes events on receive. Preserve navigation before obstacle or
     * clearance gates can defer it; never silently overwrite another marker. */
    if (s->uturn_guard &&
        (int64_t)i->distance_um-s->uturn_end_um>=(int64_t)i->uturn_ignore_mm*1000) s->uturn_guard=0;
    if (i->navigation_event && i->nav==4 && s->uturn_guard && !s->pending_nav) {
        /* The U-turn marker just passed, read again on the way back: release
         * S3 (as for STRAIGHT) and keep going instead of turning again. */
        s->navigation_ack=1;
    } else if (i->navigation_event && i->nav) {
        if (i->nav>4 || (s->pending_nav && s->pending_nav!=i->nav)) {
            vc_stop(s,VC_TRANSPORT);return;
        }
        s->pending_nav=i->nav;
    }
    /* No valid odometry (a late controller period or a failed encoder copy):
     * hold still, after keeping any decoded turn above; an odometry fault
     * only once the gap lasts VC_ODOM_GAP_MS. The sonar gap below is still
     * timed meanwhile. */
    bool odom_hold=!i->odom_valid;
    if (odom_hold) {
        if (!s->odom_gap) {s->odom_gap=1;s->odom_gap_ms=i->now_ms;}
        else if (i->now_ms-s->odom_gap_ms>=VC_ODOM_GAP_MS) {sensor_fault(s,VC_CAUSE_ODOM);return;}
    } else s->odom_gap=0;
    if (s->phase!=VC_TRACK && i->now_ms-s->entered_ms>
        ((s->phase==VC_ACQUIRE || s->phase==VC_PASS)?30000U:(s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE)?10000U:5000U)) { vc_stop(s,VC_TIMEOUT);return; }
    /* Pan waits never extend a phase deadline (DIAGONAL and ALIGN: 10 s). */
    if (s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE && !s->look_side && i->front_pending) {
        if (!s->front_wait) {s->front_wait=1;s->front_wait_ms=i->now_ms;}
        else if (i->now_ms-s->front_wait_ms>=AVOID_FRONT_WAIT_MS) {sensor_fault(s,VC_CAUSE_SONAR);return;}
    } else s->front_wait=0;
    /* No fresh sonar sample (its task shares a core with the radio): hold
     * still, after keeping any decoded turn above; a sonar fault only once
     * the gap lasts VC_SONAR_GAP_MS. */
    if (!i->sensor_fresh) {
        if (!s->sonar_gap) {s->sonar_gap=1;s->sonar_gap_ms=i->now_ms;}
        else if (i->now_ms-s->sonar_gap_ms>=VC_SONAR_GAP_MS) sensor_fault(s,VC_CAUSE_SONAR);
        return;
    }
    s->sonar_gap=0;
    if (odom_hold) return;

    if (s->phase==VC_TRACK) {
        if (i->obstacle) {
            enter(s,i,VC_ACQUIRE);s->avoid_yaw=i->yaw_mdeg;
            s->rescans=0;s->rescan_wait=0;s->rescan_request=0; /* a new obstacle */
            return;
        }
        /* Line-lost distance (checked below) starts again on TRACK entry, on a
         * straddle sensor on black, while the check is off and while a
         * decoded turn is pending (its junction search has its own bound).
         * Reversing never adds to it: it runs from the furthest point back. */
        if (!s->line_seen || i->line_touch || !i->line_lost_mm || s->pending_nav ||
            i->distance_um<s->line_seen_um) {s->line_seen=1;s->line_seen_um=i->distance_um;}
        if (s->track_wait) {
            if (i->line_fresh && i->steering) s->track_wait=0;
            else {
                if (i->now_ms-s->track_requested_ms>=VC_TRACK_HANDOFF_MS) line_fault(s,VC_LINE_CAUSE_DATA);
                return; /* stay stopped while S3 applies TRACK and samples */
            }
        }
        if (!i->front_clear) return;
        if (s->pending_nav && !i->nav_hold) {
            unsigned nav=s->pending_nav;
            if (nav==3) {s->pending_nav=0;s->navigation_ack=1;return;}
            /* Keep following the line to the next junction, then straight on
             * by the sensor-to-wheel offset so the wheels pivot on the crossing
             * (vehicle.c drives it without follower pivots). No junction within
             * nav_search_mm: turn there anyway. */
            if (!s->nav_search) {s->nav_search=1;s->nav_origin_um=i->distance_um;}
            if (!s->nav_offset && i->junction) {s->nav_offset=1;s->nav_origin_um=i->distance_um;}
            if ((int64_t)i->distance_um-s->nav_origin_um>=
                (int64_t)(s->nav_offset?i->nav_offset_mm:i->nav_search_mm)*1000) {
                s->pending_nav=0;
                s->turn_deg=nav==1?90:nav==2?-90:180;
                enter(s,i,VC_NAV_TURN);return;
            }
        } else {s->nav_search=0;s->nav_offset=0;} /* a re-read reposition restarts the search */
        if (!i->line_fresh || !i->steering) { line_gap_hold(s,i);return; }
        /* Line lost: both straddle sensors white is also "centred", so after
         * line_lost_mm with neither on black search on the spot (toward the
         * side the line was last seen first); only a sensor on black ends
         * it, else VC_LINE (not found). */
        if (i->line_lost_mm &&
            (int64_t)i->distance_um-s->line_seen_um>=(int64_t)i->line_lost_mm*1000) {
            enter(s,i,VC_REACQUIRE);s->spot_scan=1;return;
        }
        /* Either front sensor on black is sufficient to keep following forward. */
        s->command=1;
    } else if (s->phase==VC_ACQUIRE) {
        /* After a rescan request nothing S5 shows counts until it has
         * started a new scan (stale BLOCKED or READY included); none within
         * AVOID_RESCAN_WAIT_MS: the sonar fault. */
        if (s->rescan_wait) {
            if (i->scan_id==s->rescan_id) {
                if (i->now_ms-s->rescan_ms>=AVOID_RESCAN_WAIT_MS) sensor_fault(s,VC_CAUSE_SONAR);
                return;
            }
            /* The new scan gets the full ACQUIRE time (30 s): a whole scan
             * takes ~12 s on the robot, so three would not fit in one. */
            s->rescan_wait=0;s->rescan_request=0;s->entered_ms=i->now_ms;
        }
        if(i->scan_blocked){no_way_around(s,i);return;}
        if (i->front_clear && !i->obstacle && !i->scan_ready) { enter(s,i,VC_REACQUIRE);return; }
        if (!i->scan_ready) return;
        if(!i->width_valid || i->obstacle_width_mm>AVOID_MAX_WIDTH_MM){no_way_around(s,i);return;}
        if (!i->bypass_enabled || !i->bypass_steering || i->bypass_steering>400U) { vc_stop(s,VC_UNCALIBRATED);return; }
        if (!i->turn_deg || abs(i->turn_deg)>80 || !i->approach_mm || i->return_mm>1000) {
            sensor_fault(s,VC_CAUSE_SONAR);return;
        }
        s->turn_deg=i->turn_deg;s->side=i->turn_deg<0;s->direct_return=0;s->line_found=0;
        /* Mirror the outbound diagonal and arcs to recover the original lateral area. */
        s->approach_mm=i->approach_mm;s->return_mm=i->approach_mm;
        s->bypass_steering=i->bypass_steering;s->backup_moving=0;s->backup_braking=0;
        s->backup_started_ms=i->now_ms;
        enter(s,i,VC_BACKUP);return;
    } else if (s->phase==VC_BACKUP) {
        /* Straight reverse away from the obstacle before turning out. The sonar
         * looks forward, so front_clear does not gate it; time or odometry ends it. */
        if (!s->backup_moving) {
            if (!i->backup_ready) {
                if (i->now_ms-s->entered_ms>=AVOID_BACKUP_SETTLE_MS) sensor_fault(s,VC_CAUSE_SONAR);
                return;
            }
            s->backup_started_ms=i->now_ms;
            s->backup_moving=1;
        }
        int64_t back=(int64_t)s->origin_um-i->distance_um;
        if (back<-(int64_t)AVOID_BACKUP_WRONG_WAY_MM*1000) {vc_stop(s,VC_MOTOR);return;}
        if(back>(int64_t)AVOID_BACKUP_MAX_MM*2000) {vc_stop(s,VC_SENSOR);return;}
        if (s->backup_braking) {
            /* Stopped (command 0) until the wheels have really stopped; the
             * robot never switches straight from reverse to forward. */
            int64_t moved=(int64_t)i->distance_um-s->brake_last_um;
            if (moved>AVOID_BACKUP_STILL_UM || moved< -AVOID_BACKUP_STILL_UM) {
                s->brake_last_um=i->distance_um;s->brake_moved_ms=i->now_ms;
            }
            uint32_t braking=i->now_ms-s->brake_started_ms;
            if (braking<AVOID_BACKUP_BRAKE_MAX_MS && (braking<AVOID_BACKUP_BRAKE_MS ||
                i->now_ms-s->brake_moved_ms<AVOID_BACKUP_STILL_MS)) return;
            s->backup_braking=0;
            enter(s,i,VC_TURN_OUT);return;
        }
        uint32_t reversing=i->now_ms-s->backup_started_ms;
        if (reversing>=AVOID_BACKUP_MS || back>=(int64_t)AVOID_BACKUP_MAX_MM*1000) {
            s->backup_braking=1;s->brake_started_ms=s->brake_moved_ms=i->now_ms;
            s->brake_last_um=i->distance_um;return;
        }
        s->command=2;backup_cap=backup_ramp(reversing);
    } else if (s->phase==VC_REACQUIRE) {
        /* The search bound also ends a search the sonar or late line data
         * holds: VC_LINE (not found), never the 5 s phase limit. A line
         * already found and settling is not cut short here. */
        if ((!i->front_clear || !i->line_fresh) && !s->line_settling &&
            i->now_ms-s->entered_ms>=VC_SEARCH_MS) {
            line_fault(s,VC_LINE_CAUSE_NOT_FOUND);return;
        }
        if (!i->front_clear) return;
        if (!i->line_fresh) { line_gap_hold(s,i);return; }
        /* After a U-turn a straddle sample is "fresh" even off the line: the
         * spot scan only ends when a sensor is actually on black. */
        bool found=s->spot_scan?i->line_touch:(i->line_detected || i->steering);
        if (s->line_settling) {
            if (i->now_ms-s->line_found_ms<20U) {
                s->command=s->search_command;
                s->power=s->maximum_power>15?15:s->maximum_power;
                return;
            }
            s->line_settling=0;
            if (found) { enter(s,i,VC_TRACK);return; }
        } else if (found) {
            /* No extra turn when starting already on the line. */
            if (!s->search_command) { enter(s,i,VC_TRACK);return; }
            s->line_settling=1;s->line_found_ms=i->now_ms;
            s->command=s->search_command;
            s->power=s->maximum_power>15?15:s->maximum_power;
            return;
        }
        /* Bounded encoder-controlled sweep: +30, -30, +60, -60 degrees.
         * Fresh forward clearance and sensor checks above gate every step. */
        const int targets[4]={30000,-30000,60000,-60000};
        if (i->now_ms-s->entered_ms>=VC_SEARCH_MS || s->turn_progress>=4) {
            line_fault(s,VC_LINE_CAUSE_NOT_FOUND);return;
        }
        int progress=yaw_delta(i->yaw_mdeg,s->origin_yaw);
        int target=targets[s->turn_progress];
        if (s->search_right_first) target=-target;
        if (abs(target-progress)<=3000) { ++s->turn_progress;return; }
        s->command=target>progress?3:4;
        s->search_command=s->command;
        s->power=s->maximum_power>15?15:s->maximum_power;
        return;
    } else {
        if (!i->front_clear && !s->look_side) return;
        /* Line search on the way back from a bypass: in TURN_IN, RETURN and
         * RESTORE the sensor on the line's side touches it first (the touch is
         * kept across those phases). Once it releases into the gap between the
         * sensors, or the line reaches the far sensor, or both are on it, the
         * line is under the robot: arc back to the heading held before the
         * obstacle (VC_RESTORE), then track. A first touch by the lead sensor
         * alone does not count. */
        if ((s->phase==VC_TURN_IN || s->phase==VC_RETURN || s->phase==VC_RESTORE) &&
            i->line_fresh && !s->line_found) {
            unsigned mask=i->line_black_mask&3U, lead=s->turn_deg>0?1U:2U;
            if (mask==3U || (s->nav_touched && mask!=lead)) {
                s->line_found=1;
                if (s->phase!=VC_RESTORE) {enter(s,i,VC_RESTORE);return;}
            } else if (mask&lead) s->nav_touched=1;
        }
        int target=0;unsigned next=VC_FAULT;
        switch(s->phase) {
        case VC_TURN_OUT: target=s->turn_deg*1000;next=VC_DIAGONAL;break;
        case VC_ALIGN: target=-s->turn_deg*1000;next=VC_PASS;break;
        case VC_TURN_IN: {
            /* To the return heading toward the line: from the diagonal (side
             * clear) through turn_deg plus it, after ALIGN/PASS through it. */
            int r=s->turn_deg>0?return_deg(s):-return_deg(s);
            target=-(s->direct_return?s->turn_deg+r:r)*1000;next=VC_RETURN;break;
        }
        case VC_RESTORE: /* back to the heading before the obstacle */
            target=s->restore_mdeg;next=s->line_found?VC_TRACK:VC_REACQUIRE;
            if (abs(target)<2000) {
                /* Already on it: track the found line, else search for it. */
                enter(s,i,next);
                if (next==VC_REACQUIRE) {s->spot_scan=1;s->search_right_first=s->turn_deg>0;}
                return;
            }
            break;
        case VC_NAV_TURN: target=s->turn_deg*1000;next=VC_REACQUIRE;break;
        case VC_DIAGONAL: {
            /* End of the out diagonal: stop and look to the obstacle side.
             * Clear: turn straight back toward the line (no align and no
             * straight pass). Still there: align and pass it in steps. */
            if(s->look_side) {
                int r=side_look(s,i);
                if(r==1) {s->look_side=0;s->clear_run=1;s->clear_run_timing=0;s->origin_um=i->distance_um;}
                else if(r==2) {s->look_side=0;enter(s,i,VC_ALIGN);}
                return;
            }
            if(s->clear_run) {
                /* Clear: a little further on (it adds to the diagonal), then
                 * straight back. */
                if (clear_run_done(s,i)) {
                    unsigned back=s->return_mm+(unsigned)((i->distance_um-s->origin_um)/1000);
                    s->return_mm=(uint16_t)(back>UINT16_MAX?UINT16_MAX:back);
                    s->direct_return=1;enter(s,i,VC_TURN_IN);return;
                }
                s->command=1;break;
            }
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)s->approach_mm*1000) {
                int64_t travel=(int64_t)i->distance_um-s->origin_um;
                if(travel<=0 || travel>1000000) {vc_stop(s,VC_SENSOR);return;}
                s->return_mm=(uint16_t)((travel+999)/1000);
                start_side_look(s,i);return;
            }
            s->command=1;break;
        }
        case VC_RETURN: {
            /* Head back straight until the line is found (handed over above).
             * Only after AVOID_RETURN_MAX_MM with no line (or the expected
             * distance plus AVOID_RETURN_EXTRA_MM, if further) straighten up
             * and search on the spot. */
            uint32_t limit=return_limit_mm(s)+AVOID_RETURN_EXTRA_MM;
            if (limit<AVOID_RETURN_MAX_MM) limit=AVOID_RETURN_MAX_MM;
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)limit*1000) {
                enter(s,i,VC_RESTORE);return;
            }
            s->command=1;break;
        }
        case VC_PASS:
            /* Alongside the obstacle: look to its side every AVOID_PASS_STEP_MM. */
            if(s->look_side) {
                int r=side_look(s,i);
                if(r==1) {s->look_side=0;s->clear_run=1;s->clear_run_timing=0;s->origin_um=i->distance_um;}
                else if(r==2) {
                    s->look_side=0;s->origin_um=i->distance_um;
                    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
                }
                return;
            }
            if(s->clear_run) {
                /* Clear: a little further on, then turn back. */
                if (clear_run_done(s,i)) {
                    enter(s,i,VC_TURN_IN);return;
                }
                s->command=1;break;
            }
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)AVOID_PASS_STEP_MM*1000) {
                start_side_look(s,i);return;
            }
            s->command=1;break;
        default:vc_stop(s,VC_SENSOR);return;
        }
        if(target) {
            s->turn_progress+=yaw_delta(i->yaw_mdeg,s->origin_yaw);
            s->origin_yaw=i->yaw_mdeg;
            int d=s->turn_progress;
            bool done=(target>0 && d>=target) || (target<0 && d<=target),touched=false,guided=false;
            if (s->phase==VC_NAV_TURN) {
                /* Line-guided: past the min angle a straddle touch is the new
                 * line (earlier ones are the old line or the crossing). The
                 * first touch is only the leading sensor reaching its near
                 * edge (~76 deg with the sensors 90 mm ahead of the axle), so
                 * keep turning until that sensor releases it into the gap
                 * between the sensors (centred, ~88 deg) or both sensors are on
                 * it. By the max angle the turn ends regardless. */
                bool u=s->turn_deg==180;
                int reached=target>0?d:-d;
                int lo=(u?i->nav_uturn_min_deg:i->nav_turn_min_deg)*1000;
                int hi=(u?i->nav_uturn_max_deg:i->nav_turn_max_deg)*1000;
                turn_power=i->nav_turn_power;
                if (hi) {
                    guided=true;
                    if (reached>=lo && i->line_fresh) {
                        unsigned mask=i->line_black_mask&3U;
                        if (mask==3U || (s->nav_touched && mask==0U)) touched=true;
                        else if (mask) s->nav_touched=1;
                    }
                    done=touched || reached>=hi;
                    if (reached>=lo && i->nav_seek_power) turn_power=i->nav_seek_power;
                }
            }
            if (done) {
                /* A line-guided turn that ran to its max angle without a
                 * touch has overshot the arm: always search for it. */
                bool scan=s->phase==VC_NAV_TURN && !touched &&
                    (guided || s->turn_deg==180 || i->spot_scan_all);
                bool uturn=s->phase==VC_NAV_TURN && s->turn_deg==180;
                /* A bypass that restored its heading without centring on the
                 * line searches for it (only a sensor on black ends the
                 * search), toward the line's side first. */
                bool restored=s->phase==VC_RESTORE && next==VC_REACQUIRE;
                enter(s,i,next);s->spot_scan=scan || restored;
                if (restored) s->search_right_first=s->turn_deg>0;
                if (uturn && i->uturn_ignore_mm) {s->uturn_guard=1;s->uturn_end_um=i->distance_um;}
                return;
            }
            if (s->phase==VC_NAV_TURN) s->command=target>0?3:4;
            else {
                s->command=1;
                s->steering=(int16_t)(target>0?-(int)s->bypass_steering:(int)s->bypass_steering);
            }
        }
    }
    if (s->command) {
        /* Turn/seek power never exceeds the armed (Start) power. */
        unsigned demand=turn_power && turn_power<s->maximum_power?turn_power:s->maximum_power;
        if(s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE) {
            if(demand>AVOID_POWER)demand=AVOID_POWER;
            demand=soft_start(s,i->now_ms,was_driving!=0,demand);
        }
        if(s->phase==VC_BACKUP && demand>AVOID_BACKUP_POWER)demand=AVOID_BACKUP_POWER;
        if(backup_cap && demand>backup_cap)demand=backup_cap;
        s->power=(uint8_t)(i->slope && demand>20U?20U:demand);
    }
}
/* A ramp's allowance t ms after its start: full*t/ramp_ms. Steps take its
 * differences, so they add up exactly whatever the tick length. */
static uint64_t ramp_allowance(uint32_t full,uint32_t ramp_ms,uint32_t t) {
    return (uint64_t)full*t/ramp_ms;
}
void vc_avoid_ramp_reset(vc_avoid_ramp_t *r,uint32_t now) {
    if (!r) return;
    r->start_ms=now;r->elapsed_ms=0;r->power_m=0;r->steer=0;
}
void vc_avoid_ramp_step(vc_avoid_ramp_t *r,uint32_t now,unsigned power,int32_t steering) {
    if (!r) return;
    uint32_t was=r->elapsed_ms,t=now-r->start_ms;
    if (t<was) t=was;
    int32_t want=(int32_t)power*1000;
    r->power_m+=(int32_t)(ramp_allowance(AVOID_POWER*1000U,AVOID_RAMP_POWER_MS,t)-
        ramp_allowance(AVOID_POWER*1000U,AVOID_RAMP_POWER_MS,was));
    if (r->power_m>want) r->power_m=want;
    int32_t ds=steering-r->steer,step=(int32_t)(ramp_allowance(400U,AVOID_STEER_RAMP_MS,t)-
        ramp_allowance(400U,AVOID_STEER_RAMP_MS,was));
    if (ds>step) ds=step; else if (ds< -step) ds=-step;
    r->steer+=ds;r->elapsed_ms=t;
}
