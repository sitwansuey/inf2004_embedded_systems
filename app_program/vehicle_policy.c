#include "vehicle_policy.h"
#include <stdlib.h>
#include <string.h>
/* Commands match S2: STOP=0 FORWARD=1 REVERSE=2 LEFT=3 RIGHT=4 PIVOT_R=5 PIVOT_L=6. */
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power) {
    if (!s || lease==0 || lease>1000 || power<1 || power>50) return false;
    if (s->phase==VC_FAULT) return false;
    s->armed_ms=now;s->lease_ms=lease;s->maximum_power=(uint8_t)power;s->armed=1;s->fault_cause=0;
    if (s->phase==VC_DISARMED) { s->phase=VC_TRACK; s->entered_ms=now; }
    return true;
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
    s->fault=(uint8_t)(reason&15U);s->phase=reason?VC_FAULT:VC_DISARMED;
    s->fault_cause=0; /* sensor_fault() sets it after this */
}
/* VC_SENSOR with the reason kept for telemetry while the fault is latched. */
static void sensor_fault(vc_state_t *s,unsigned cause) {
    vc_stop(s,VC_SENSOR);s->fault_cause=(uint8_t)(cause&3U);
}
void vc_track_requested(vc_state_t *s,uint32_t now) {
    if (!s) return;
    s->track_requested_ms=now;s->track_wait=1;
    s->command=0;s->power=0;s->steering=0;
}
static unsigned soft_start(vc_state_t *s,uint32_t now,bool driving,unsigned demand);
static void enter(vc_state_t *s,const vc_input_t *i,unsigned phase) {
    bool continuous=s->armed && s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE &&
        phase>=VC_TURN_OUT && phase<=VC_RESTORE && !s->look_side && i->front_clear;
    s->phase=(uint8_t)phase;s->entered_ms=i->now_ms;
    if (phase==VC_REACQUIRE) s->search_right_first=s->last_line_side==1U;
    s->origin_um=i->distance_um;s->origin_yaw=i->yaw_mdeg;
    s->turn_progress=0;s->bypass_drive_ms=0;s->last_step_ms=i->now_ms;
    s->command=0;s->power=0;s->steering=0;
    s->track_wait=0;
    s->search_command=0;s->line_settling=0;s->line_found_ms=0;
    s->nav_search=0;s->nav_offset=0;s->spot_scan=0;s->nav_touched=0; /* pending_nav itself is kept */
    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
    if(continuous) {
        s->command=1;s->power=s->maximum_power>AVOID_POWER?AVOID_POWER:s->maximum_power;
        s->power=(uint8_t)soft_start(s,i->now_ms,s->drive_prev,s->power);
        if(i->slope && s->power>20U)s->power=20U;
        if(phase==VC_ALIGN || phase==VC_TURN_IN)s->steering=(int16_t)(s->turn_deg>0?(int)s->bypass_steering:-(int)s->bypass_steering);
        else if(phase==VC_RESTORE)s->steering=(int16_t)(s->turn_deg>0?-(int)s->bypass_steering:(int)s->bypass_steering);
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
static int yaw_delta(int32_t now,int32_t origin) {
    int d=now-origin; if(d>180000)d-=360000;if(d< -180000)d+=360000;return d;
}
void vc_step(vc_state_t *s,const vc_input_t *i) {
    if (!s || !i) return;
    unsigned was_driving=s->command==1 && s->power!=0;
    s->drive_prev=was_driving!=0;
    unsigned turn_power=0; /* VC_NAV_TURN demand, 0: maximum_power */
    unsigned backup_cap=0; /* VC_BACKUP ramp cap, 0: none */
    uint32_t elapsed=i->now_ms-s->last_step_ms;
    s->last_step_ms=i->now_ms;
    s->command=0;s->power=0;s->steering=0;
    s->navigation_ack=0;
    if(s->phase==VC_DIAGONAL && was_driving && elapsed<=100U)
        s->bypass_drive_ms+=elapsed;
    if (!s->armed) return;
    /* Both-black is straight; retain the last unambiguous search direction. */
    if (i->line_fresh && (i->line_black_mask==1U || i->line_black_mask==2U))
        s->last_line_side=i->line_black_mask;
    if (s->lease_ms && i->now_ms-s->armed_ms>=s->lease_ms) { vc_stop(s,VC_TIMEOUT);return; }
    if (i->shock || i->safety_event) { vc_stop(s,i->shock?VC_SHOCK:VC_LINE);return; }
    if (!i->transport_ok) { vc_stop(s,VC_TRANSPORT);return; }
    if (!i->motor_ok) { vc_stop(s,VC_MOTOR);return; }
    /* Tilt drops when side roll exceeds 15 deg or |a| leaves 0.8..1.2 g.
     * Before the GY-511 axis remap the obstacle reverse's fore-aft rocking
     * tripped the roll limit; now only jolts (the norm gate) can. VC_BACKUP
     * (straight, slow, odometry-bounded) still does not need tilt; TURN_OUT
     * does, after the stand-still has let the robot settle. */
    bool tilt_ok=i->terrain_valid || s->phase==VC_BACKUP;
    if (!i->sensor_fresh || !tilt_ok || !i->odom_valid) {
        sensor_fault(s,!i->sensor_fresh?VC_CAUSE_SONAR:!tilt_ok?VC_CAUSE_TILT:VC_CAUSE_ODOM);return;
    }
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
    if (s->phase!=VC_TRACK && i->now_ms-s->entered_ms>
        ((s->phase==VC_ACQUIRE || s->phase==VC_PASS)?30000U:(s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE)?10000U:5000U)) { vc_stop(s,VC_TIMEOUT);return; }
    if (s->phase==VC_TRACK) {
        if (i->obstacle) { enter(s,i,VC_ACQUIRE);return; }
        if (s->track_wait) {
            if (i->line_fresh && i->steering) s->track_wait=0;
            else {
                if (i->now_ms-s->track_requested_ms>=VC_TRACK_HANDOFF_MS) vc_stop(s,VC_LINE);
                return; /* stay stopped while S3 applies TRACK and samples */
            }
        }
        if (!i->front_clear) return;
        if (i->recovery_event && !i->curve_hold) { enter(s,i,VC_REACQUIRE);return; }
        if (s->pending_nav && !i->nav_hold) {
            unsigned nav=s->pending_nav;
            if (nav==3) {s->pending_nav=0;s->navigation_ack=1;return;}
            /* Keep following the line to the next junction, then straight on
             * by the sensor-to-wheel offset so the wheels pivot on the crossing
             * (vehicle.c drives it without course pivots). No junction within
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
        /* S3 reports SEARCHING (2) before LOST (3); begin recovery then,
         * rather than faulting before its recovery event can be delivered. */
        if (i->line_fresh && i->curve_hold) {
            s->command=1;s->power=s->maximum_power;return;
        }
        if (i->line_fresh && !i->steering && i->line_state==2U) {
            /* SEARCHING is the short pre-LOST window; bridge brief dropouts. */
            s->command=1;s->power=s->maximum_power;
            return;
        }
        if (i->line_fresh && !i->steering && i->line_state==3U) {
            enter(s,i,VC_REACQUIRE);return;
        }
        if (!i->line_fresh || !i->steering) { vc_stop(s,VC_LINE);return; }
        /* Either front sensor on black is sufficient to keep following forward. */
        s->command=1;
    } else if (s->phase==VC_ACQUIRE) {
        if(i->scan_blocked){sensor_fault(s,VC_CAUSE_SONAR);return;}
        if (i->front_clear && !i->obstacle && !i->scan_ready) { enter(s,i,VC_REACQUIRE);return; }
        if (!i->scan_ready) return;
        if(!i->width_valid || i->obstacle_width_mm>AVOID_MAX_WIDTH_MM){sensor_fault(s,VC_CAUSE_SONAR);return;}
        if (!i->bypass_enabled || !i->bypass_steering || i->bypass_steering>400U) { vc_stop(s,VC_UNCALIBRATED);return; }
        if (!i->turn_deg || abs(i->turn_deg)>80 || !i->approach_mm || i->return_mm>1000) {
            sensor_fault(s,VC_CAUSE_SONAR);return;
        }
        s->scan_id=i->scan_id;s->turn_deg=i->turn_deg;s->side=i->turn_deg<0;
        s->pass_check_mm=(uint16_t)((i->obstacle_width_mm+1U)/2U+AVOID_CLEARANCE_MM);
        if(s->pass_check_mm<AVOID_PASS_CHECK_MIN_MM)s->pass_check_mm=AVOID_PASS_CHECK_MIN_MM;
        if(s->pass_check_mm>AVOID_PASS_CHECK_MAX_MM)s->pass_check_mm=AVOID_PASS_CHECK_MAX_MM;
        /* Mirror the outbound diagonal and arcs to recover the original lateral area. */
        s->approach_mm=i->approach_mm;s->return_mm=i->approach_mm;
        s->bypass_steering=i->bypass_steering;s->backup_mm=0;s->backup_moving=0;s->backup_braking=0;
        s->backup_started_ms=i->now_ms;
        enter(s,i,VC_BACKUP);return;
    } else if (s->phase==VC_BACKUP) {
        /* Straight reverse away from the obstacle before turning out. The sonar
         * looks forward, so front_clear does not gate it; time or odometry ends it. */
        if (!s->backup_moving) {
            if (i->sensor_acquiring && i->now_ms-s->entered_ms<AVOID_BACKUP_SETTLE_MS) {
                /* Stopped; the 1 s reverse starts when S5 has let go. */
                s->backup_started_ms=i->now_ms;s->origin_um=i->distance_um;return;
            }
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
            /* Coasting counts as reversed distance. */
            uint32_t mm=back>0?(uint32_t)((back+999)/1000):0U;
            s->backup_mm=(uint16_t)mm;s->backup_braking=0;
            /* The obstacle is now further ahead: delay the side look by as much. */
            mm+=s->pass_check_mm;s->pass_check_mm=(uint16_t)(mm>UINT16_MAX?UINT16_MAX:mm);
            enter(s,i,VC_TURN_OUT);return;
        }
        uint32_t reversing=i->now_ms-s->backup_started_ms;
        if (reversing>=AVOID_BACKUP_MS || back>=(int64_t)AVOID_BACKUP_MAX_MM*1000) {
            s->backup_braking=1;s->brake_started_ms=s->brake_moved_ms=i->now_ms;
            s->brake_last_um=i->distance_um;return;
        }
        s->command=2;backup_cap=backup_ramp(reversing);
    } else if (s->phase==VC_REACQUIRE) {
        if (!i->front_clear) return;
        if (!i->line_fresh) { vc_stop(s,VC_LINE);return; }
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
        if (i->now_ms-s->entered_ms>=4000U || s->turn_progress>=4) {
            vc_stop(s,VC_LINE);return;
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
         * RESTORE the sensor on the line's side touches it first; once it
         * releases into the gap between the sensors (or both are on it) the
         * line is centred, so track it from there rather than finishing the
         * fixed angle/distance moves past it. */
        if ((s->phase==VC_TURN_IN || s->phase==VC_RETURN || s->phase==VC_RESTORE) && i->line_fresh) {
            unsigned mask=i->line_black_mask&3U, lead=s->turn_deg>0?1U:2U;
            if (mask==3U || (s->nav_touched && mask==0U)) {enter(s,i,VC_TRACK);return;}
            if (mask&lead) s->nav_touched=1;
        }
        int target=0;unsigned next=VC_FAULT;
        switch(s->phase) {
        case VC_TURN_OUT: target=s->turn_deg*1000;next=VC_DIAGONAL;break;
        case VC_ALIGN: target=-s->turn_deg*1000;next=VC_PASS;break;
        case VC_TURN_IN: target=-s->turn_deg*1000;next=VC_RETURN;break;
        case VC_RESTORE: target=s->turn_deg*1000;next=VC_REACQUIRE;break;
        case VC_NAV_TURN: target=s->turn_deg*1000;next=VC_REACQUIRE;break;
        case VC_DIAGONAL: {
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)s->approach_mm*1000) {
                int64_t travel=(int64_t)i->distance_um-s->origin_um;
                if(travel<=0 || travel>1000000) {vc_stop(s,VC_SENSOR);return;}
                s->return_mm=(uint16_t)((travel+999)/1000);
                enter(s,i,VC_ALIGN);return;
            }
            s->command=1;break;
        }
        case VC_RETURN: {
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)s->return_mm*1000) {
                enter(s,i,VC_RESTORE);return;
            }
            s->command=1;break;
        }
        case VC_PASS:
            if(s->look_side) {
                if(i->now_ms-s->side_check_started_ms>=AVOID_SIDE_CHECK_TIMEOUT_MS){vc_stop(s,VC_SENSOR);return;}
                if(!i->side_observed || (s->side_have_sample && i->side_sample_ms==s->side_last_ms))return;
                s->side_have_sample=1;s->side_last_ms=i->side_sample_ms;
                if(!i->side_ready){s->side_clear_count=s->side_present_count=0;return;}
                if(i->side_clear){++s->side_clear_count;s->side_present_count=0;}
                else {++s->side_present_count;s->side_clear_count=0;}
                if(s->side_clear_count>=AVOID_SIDE_CONFIRM_SAMPLES) {
                    s->look_side=0;enter(s,i,VC_TURN_IN);return;
                }
                if(s->side_present_count>=AVOID_SIDE_CONFIRM_SAMPLES) {
                    /* The reverse offset only delays the first look. */
                    if(s->backup_mm) {
                        s->pass_check_mm=(uint16_t)(s->pass_check_mm>s->backup_mm?
                            (unsigned)(s->pass_check_mm-s->backup_mm):AVOID_PASS_CHECK_MIN_MM);
                        s->backup_mm=0;
                    }
                    s->look_side=0;s->origin_um=i->distance_um;
                    s->side_clear_count=s->side_present_count=s->side_have_sample=0;
                }
                return;
            }
            if ((int64_t)i->distance_um-s->origin_um>=(int64_t)s->pass_check_mm*1000) {
                s->look_side=1;s->side_check_started_ms=i->now_ms;
                s->side_clear_count=s->side_present_count=s->side_have_sample=0;return;
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
                bool restored=s->phase==VC_RESTORE;
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
