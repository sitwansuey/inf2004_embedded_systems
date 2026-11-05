#include "vehicle_policy.h"
#include <stdlib.h>
#include <string.h>
/* Commands match S2: STOP=0 FORWARD=1 LEFT=3 RIGHT=4 PIVOT_R=5 PIVOT_L=6. */
bool vc_arm(vc_state_t *s,uint32_t now,uint32_t lease,unsigned power) {
    if (!s || lease==0 || lease>1000 || power<1 || power>50) return false;
    if (s->phase==VC_FAULT) return false;
    s->armed_ms=now;s->lease_ms=lease;s->maximum_power=(uint8_t)power;s->armed=1;
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
    s->pending_nav=0;s->navigation_ack=0;s->track_wait=0;
    s->last_line_side=0;s->search_right_first=0;
    s->search_command=0;s->line_settling=0;s->line_found_ms=0;
    s->fault=(uint8_t)(reason&15U);s->phase=reason?VC_FAULT:VC_DISARMED;
}
void vc_track_requested(vc_state_t *s,uint32_t now) {
    if (!s) return;
    s->track_requested_ms=now;s->track_wait=1;
    s->command=0;s->power=0;s->steering=0;
}
static void enter(vc_state_t *s,const vc_input_t *i,unsigned phase) {
    s->phase=(uint8_t)phase;s->entered_ms=i->now_ms;
    if (phase==VC_REACQUIRE) s->search_right_first=s->last_line_side==1U;
    s->origin_um=i->distance_um;s->origin_yaw=i->yaw_mdeg;
    s->turn_progress=0;s->bypass_drive_ms=0;s->last_step_ms=i->now_ms;
    s->command=0;s->power=0;s->steering=0;
    s->track_wait=0;
    s->search_command=0;s->line_settling=0;s->line_found_ms=0;
}
static int yaw_delta(int32_t now,int32_t origin) {
    int d=now-origin; if(d>180000)d-=360000;if(d< -180000)d+=360000;return d;
}
void vc_step(vc_state_t *s,const vc_input_t *i) {
    if (!s || !i) return;
    bool front_clear=i->barcode_demo || i->front_clear;
    unsigned was_driving=s->command==1 && s->power!=0;
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
    if (i->barcode_demo && s->maximum_power>30U) { vc_stop(s,VC_MOTOR);return; }
    if (i->barcode_demo && !i->line_fresh) { vc_stop(s,VC_LINE);return; }
    if ((!i->barcode_demo && (!i->sensor_fresh || !i->terrain_valid)) || !i->odom_valid) { vc_stop(s,VC_SENSOR);return; }
    /* S3 removes events on receive. Preserve navigation before obstacle or
     * clearance gates can defer it; never silently overwrite another marker. */
    if (i->navigation_event && i->nav) {
        if (i->nav>4 || (s->pending_nav && s->pending_nav!=i->nav)) {
            vc_stop(s,VC_TRANSPORT);return;
        }
        s->pending_nav=i->nav;
    }
    if (s->phase!=VC_TRACK && i->now_ms-s->entered_ms>
        ((s->phase==VC_ACQUIRE)?30000U:(s->phase>=VC_TURN_OUT && s->phase<=VC_RESTORE)?10000U:5000U)) { vc_stop(s,VC_TIMEOUT);return; }
    if (s->phase==VC_TRACK) {
        if (!i->barcode_demo && i->obstacle) { enter(s,i,VC_ACQUIRE);return; }
        if (s->track_wait) {
            if (i->line_fresh && i->steering) s->track_wait=0;
            else {
                if (i->now_ms-s->track_requested_ms>=VC_TRACK_HANDOFF_MS) vc_stop(s,VC_LINE);
                return; /* stay stopped while S3 applies TRACK and samples */
            }
        }
        if (!front_clear) return;
        if (i->recovery_event && !i->curve_hold) { enter(s,i,VC_REACQUIRE);return; }
        if (s->pending_nav) {
            unsigned nav=s->pending_nav;s->pending_nav=0;
            if (nav==3) {
                s->navigation_ack=1;
                if(i->barcode_demo && i->line_fresh && i->steering) {
                    s->command=1;s->power=s->maximum_power;
                }
                return;
            }
            s->turn_deg=nav==1?90:nav==2?-90:180;
            enter(s,i,VC_NAV_TURN);return;
        }
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
        s->command=i->barcode_demo && !i->barcode_scanning?
            (i->line_black_mask==1U?5U:i->line_black_mask==2U?6U:1U):1U;
    } else if (s->phase==VC_ACQUIRE) {
        if (i->front_clear && !i->obstacle && !i->scan_ready) { enter(s,i,VC_REACQUIRE);return; }
        if (!i->scan_ready) return;
        if (!i->bypass_enabled || !i->bypass_steering || i->bypass_steering>400U) { vc_stop(s,VC_UNCALIBRATED);return; }
        if (!i->turn_deg || abs(i->turn_deg)>80 || !i->approach_mm || i->return_mm>1000) {
            vc_stop(s,VC_SENSOR);return;
        }
        s->scan_id=i->scan_id;s->turn_deg=i->turn_deg;s->side=i->turn_deg<0;
        /* Mirror the outbound diagonal and arcs to recover the original lateral area. */
        s->approach_mm=i->approach_mm;s->return_mm=i->approach_mm;
        s->bypass_steering=i->bypass_steering;
        enter(s,i,VC_TURN_OUT);return;
    } else if (s->phase==VC_REACQUIRE) {
        if (!front_clear) return;
        if (!i->line_fresh) { vc_stop(s,VC_LINE);return; }
        if (s->line_settling) {
            if (i->now_ms-s->line_found_ms<20U) {
                s->command=s->search_command;
                s->power=s->maximum_power>15?15:s->maximum_power;
                return;
            }
            s->line_settling=0;
            if (i->line_detected || i->steering) { enter(s,i,VC_TRACK);return; }
        } else if (i->line_detected || i->steering) {
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
        if (!front_clear && !s->look_side) return;
        int target=0;unsigned next=VC_FAULT;
        switch(s->phase) {
        case VC_TURN_OUT: target=s->turn_deg*1000;next=VC_DIAGONAL;break;
        case VC_ALIGN: target=-s->turn_deg*2000;next=VC_RETURN;break;
        case VC_TURN_IN: target=-s->turn_deg*1000;next=VC_RETURN;break;
        case VC_RESTORE: target=s->turn_deg*1000;next=VC_REACQUIRE;break;
        case VC_NAV_TURN: target=s->turn_deg*1000;next=VC_REACQUIRE;break;
        case VC_DIAGONAL: {
            if (s->bypass_drive_ms>=3000U) {
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
            /* A side sample cannot establish that the forward path is clear.
             * Alternate stopped side observations with fresh forward samples. */
            if (s->look_side) {
                if (!i->side_ready)return;
                if (i->side_clear) {s->look_side=0;enter(s,i,VC_TURN_IN);return;}
                s->look_side=0;return;
            }
            if ((int64_t)i->distance_um-s->origin_um>=50000) {
                s->look_side=1;s->origin_um=i->distance_um;return;
            }
            s->command=1;break;
        default:vc_stop(s,VC_SENSOR);return;
        }
        if(target) {
            s->turn_progress+=yaw_delta(i->yaw_mdeg,s->origin_yaw);
            s->origin_yaw=i->yaw_mdeg;
            int d=s->turn_progress;
            if ((target>0 && d>=target) || (target<0 && d<=target)) {
                enter(s,i,next);if(next==VC_PASS)s->look_side=1;return;
            }
            if (s->phase==VC_NAV_TURN) s->command=target>0?3:4;
            else {
                s->command=1;
                s->steering=(int16_t)(target>0?-(int)s->bypass_steering:(int)s->bypass_steering);
            }
        }
    }
    if (s->command) {
        unsigned demand=s->maximum_power;
        /* A small three-second boost after the outward bend; hardware PWM cap remains. */
        if (s->phase==VC_DIAGONAL && s->bypass_drive_ms<3000U)
            demand=demand<=48U?demand+2U:50U;
        s->power=(uint8_t)(i->slope && demand>20U?20U:demand);
    }
}
