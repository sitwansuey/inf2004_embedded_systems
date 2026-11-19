/* GY-511 has acceleration and magnetic field, no angular rate sensor.
 * Height is a tilt/encoder geometric estimate, gated by mounting calibration.
 * No vertical acceleration double integration or unsupported gyro fusion. */
#include "terrain.h"
#include <math.h>
#include <stdlib.h>
#define CDEG_PER_RAD 5729.577951308232
void terrain_invalidate_height(terrain_t *s) {
    if (!s) return;
    s->state=TERRAIN_INVALID; s->height_code=20000; s->peak_um=0;
    s->height_valid=0; s->flat_samples=0; s->initialized=0;
    s->bridge=0; s->plain=0;
}
static void abandon(terrain_t *s) {
    s->tilt_valid=0; terrain_invalidate_height(s);
}
static bool traversal(const terrain_t *s) {
    return s->state==TERRAIN_ASCENT || s->state==TERRAIN_CREST || s->state==TERRAIN_DESCENT;
}
/* A hump traversal ends (DESCENT to FLAT, or below its base): latch its
 * height, or 0 when it is not a valid hump, and count it. An invalid end
 * also clears the running peak, so it is never reported as a height. */
static void complete(terrain_t *s,bool valid) {
    if (valid) {
        s->height_mm=((s->peak_um+500U)/1000U)&1023U; s->height_valid=1;
        if (s->height_mm>s->maximum_mm) s->maximum_mm=s->height_mm;
        s->hump_mm=s->height_mm;
    } else {
        s->peak_um=0; s->hump_mm=0;
    }
    s->humps=(uint8_t)(s->humps+1U);
}
/* Height change over du at pitch pc. False: the traversal was abandoned. */
static bool integrate(terrain_t *s,int64_t du,int32_t pc) {
    if (!traversal(s) || s->plain) return true;
    int64_t h=(int64_t)terrain_height_um(s)+(int64_t)lround((double)du*sin((double)pc/CDEG_PER_RAD));
    if (h < -20000 && s->state==TERRAIN_DESCENT) {
        /* Well below where the climb started: not a hump, still a descent.
         * Keep DESCENT (the slope cap), without a height. */
        complete(s,false); s->plain=1; s->height_code=20000;
        return true;
    }
    if (h < -20000 || h > 1000000) { abandon(s); return false; }
    s->height_code=(uint32_t)(h+20000)&0xfffffU;
    if (h>(int64_t)s->peak_um) s->peak_um=(uint32_t)h&0xfffffU;
    return true;
}
/* An unusable sample on FLAT or in a traversal: keep the state and height,
 * add its distance at the last good pitch, and take the next sample as if
 * this one were good. False when it cannot be bridged: no baseline, no
 * usable distance, or the run of unusable samples is too long. */
static bool bridge(terrain_t *s,uint32_t ms,bool av,int32_t distance,bool ov) {
    int64_t du=(int64_t)distance-s->previous_distance_um;
    /* SHOCK with a baseline is a bridged shock on FLAT (see below). */
    if (!s->initialized || !ov || (int)s->bridge>=(int)TERRAIN_BRIDGE_STEPS ||
        !(s->state==TERRAIN_FLAT || s->state==TERRAIN_SHOCK || traversal(s)) ||
        du < -TERRAIN_BACKWARD_UM || du > 120000 ||
        (av && ms-s->sample_ms>TERRAIN_MAX_GAP_MS+TERRAIN_BRIDGE_MS)) return false;
    if (!integrate(s,du,terrain_pitch_cdeg(s))) return false;
    s->previous_distance_um=distance;
    if (av) s->sample_ms=ms; /* a later copy of this sample is a duplicate */
    s->bridge=(uint8_t)(s->bridge+1U)&7U;
    s->tilt_valid=0; s->height_valid=0;
    return true;
}
void terrain_step(terrain_t *s,uint32_t ms,const int32_t a[3],bool av,
                  int32_t distance,bool ov,bool calibrated,int pzero,int rzero) {
    if (!s || !a) return;
    s->shock=0; /* describes this sample only */
    if (!calibrated || pzero < -9000 || pzero > 9000 ||
        rzero < -18000 || rzero > 18000) { abandon(s); return; }
    bool tilt=false,shock=false;
    int pc=0,rc=0;
    if (av) {
        for(unsigned n=0;n<3;++n)if(a[n]<-2048 || a[n]>2047) {abandon(s);return;}
        if (s->initialized && ms==s->sample_ms) return;
        int64_t norm=(int64_t)a[0]*a[0]+(int64_t)a[1]*a[1]+(int64_t)a[2]*a[2];
        /* A sample clipped at +/-2 g (an axis at 2047+ mg, S4 accel_clipped)
         * always lands here: an impact, never tilt or height. */
        shock=norm>1800LL*1800;
        /* Dynamic acceleration makes a gravity-derived tilt unreliable. */
        if (!shock && norm>=800LL*800 && norm<=1200LL*1200 && a[2]>0) {
            /* Robot frame, X front, Y left, Z up, reading +1 g up at rest
             * (imu_mount.h). Nose-up (climbing) pitch and left-up roll are positive. */
            double pitch=atan2((double)a[0],sqrt((double)a[1]*a[1]+(double)a[2]*a[2]));
            double roll=atan2((double)a[1],(double)a[2]);
            pc=(int)lround(pitch*CDEG_PER_RAD)-pzero;
            rc=(int)lround(roll*CDEG_PER_RAD)-rzero;
            tilt=pc >= -9000 && pc <= 9000 && rc >= -18000 && rc <= 18000;
        }
    }
    if (!tilt || abs(rc)>TERRAIN_MAX_ROLL_CDEG) {
        if (bridge(s,ms,av,distance,ov)) {
            /* Still an impact. On FLAT the state shows it for this sample. */
            s->shock=shock;
            if (s->state==TERRAIN_FLAT || s->state==TERRAIN_SHOCK)
                s->state=shock?TERRAIN_SHOCK:TERRAIN_FLAT;
            return;
        }
        if (tilt) { /* roll beyond the limit: shown, never used */
            s->pitch_code=(uint32_t)(pc+9000)&32767U; s->roll_code=(uint32_t)(rc+18000)&65535U;
        }
        abandon(s);
        if (shock) { s->shock=1; s->state=TERRAIN_SHOCK; }
        return;
    }
    s->pitch_code=(uint32_t)(pc+9000)&32767U; s->roll_code=(uint32_t)(rc+18000)&65535U; s->tilt_valid=1;
    uint32_t dt=ms-s->sample_ms;
    int64_t du=(int64_t)distance-s->previous_distance_um;
    uint32_t gap=TERRAIN_MAX_GAP_MS+(s->bridge?TERRAIN_BRIDGE_MS:0U);
    s->previous_distance_um=distance; s->sample_ms=ms; s->bridge=0;
    if (s->initialized && s->state==TERRAIN_SHOCK) s->state=TERRAIN_FLAT; /* bridged shock on FLAT */
    if (!ov) { terrain_invalidate_height(s); return; }
    if (!s->initialized) {
        s->initialized=1; s->state=TERRAIN_INVALID;
        s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?1U:0U;
        if(s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
            s->state=TERRAIN_FLAT;s->flat_samples=0;
        }
        return;
    }
    if (dt>gap || du<-TERRAIN_BACKWARD_UM || du>120000) { terrain_invalidate_height(s); return; }
    if (s->state==TERRAIN_INVALID) {
        /* Recover only on a confirmed flat baseline. Restarting mid-ascent
         * after a long loss must never complete a partial hump. */
        s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0U;
        if (s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
            s->state=TERRAIN_FLAT; s->flat_samples=0;
        }
        return;
    }
    if (s->state==TERRAIN_FLAT && pc>=TERRAIN_SLOPE_CDEG) {
        s->state=TERRAIN_ASCENT; s->height_code=20000; s->peak_um=0; s->height_valid=0;
    }
#if TERRAIN_PLAIN_DESCENT
    else if (s->state==TERRAIN_FLAT && pc<=-TERRAIN_DESCENT_CDEG) {
        /* Down without a climb first (a down-ramp, or a climb S4 missed). */
        s->state=TERRAIN_DESCENT; s->plain=1; s->height_code=20000; s->peak_um=0;
        s->height_valid=0; s->flat_samples=0;
    }
#endif
    if (traversal(s)) {
        if (!integrate(s,du,pc)) return;
        if (s->state==TERRAIN_ASCENT && pc<=TERRAIN_FLAT_CDEG) s->state=TERRAIN_CREST;
        if (s->state==TERRAIN_CREST && pc<=-TERRAIN_SLOPE_CDEG) s->state=TERRAIN_DESCENT;
        if (s->state==TERRAIN_DESCENT) {
            s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0;
            if (s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
                /* A completed hump returns approximately to its base level. */
                if (!s->plain)
                    complete(s,abs(terrain_height_um(s))<=TERRAIN_BASE_TOLERANCE_UM &&
                               s->peak_um>=TERRAIN_MIN_HUMP_UM);
                s->state=TERRAIN_FLAT; s->flat_samples=0; s->plain=0;
            }
        }
    }
}
