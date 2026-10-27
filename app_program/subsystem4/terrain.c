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
}
static void abandon(terrain_t *s) {
    s->tilt_valid=0; terrain_invalidate_height(s);
}
void terrain_step(terrain_t *s,uint32_t ms,const int32_t a[3],bool av,
                  int32_t distance,bool ov,bool calibrated,int pzero,int rzero) {
    if (!s || !a) return;
    if (!av || !calibrated || pzero < -9000 || pzero > 9000 ||
        rzero < -18000 || rzero > 18000) { abandon(s); return; }
    for(unsigned n=0;n<3;++n)if(a[n]<-2048 || a[n]>2047) {abandon(s);s->initialized=0;return;}
    if (s->initialized && ms==s->sample_ms) return;
    int64_t norm=(int64_t)a[0]*a[0]+(int64_t)a[1]*a[1]+(int64_t)a[2]*a[2];
    s->shock=norm>1800LL*1800;
    if (s->shock) { abandon(s); s->state=TERRAIN_SHOCK; s->initialized=0; return; }
    /* Dynamic acceleration makes a gravity-derived tilt unreliable. */
    if (norm<800LL*800 || norm>1200LL*1200 || a[2]<=0) {
        abandon(s); s->initialized=0; return;
    }
    double pitch=atan2(-(double)a[0],sqrt((double)a[1]*a[1]+(double)a[2]*a[2]));
    double roll=atan2((double)a[1],(double)a[2]);
    int pc=(int)lround(pitch*CDEG_PER_RAD)-pzero;
    int rc=(int)lround(roll*CDEG_PER_RAD)-rzero;
    if (pc < -9000 || pc > 9000 || rc < -18000 || rc > 18000) { abandon(s); return; }
    s->pitch_code=(uint32_t)(pc+9000)&32767U; s->roll_code=(uint32_t)(rc+18000)&65535U; s->tilt_valid=1;
    uint32_t dt=ms-s->sample_ms;
    int64_t du=(int64_t)distance-s->previous_distance_um;
    s->previous_distance_um=distance; s->sample_ms=ms;
    if (abs(rc)>TERRAIN_MAX_ROLL_CDEG) { abandon(s); return; }
    if (!ov) { terrain_invalidate_height(s); return; }
    if (!s->initialized) {
        s->initialized=1; s->state=TERRAIN_INVALID;
        s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?1U:0U;
        if(s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
            s->state=TERRAIN_FLAT;s->flat_samples=0;
        }
        return;
    }
    if (dt>TERRAIN_MAX_GAP_MS || du<0 || du>120000) { terrain_invalidate_height(s); return; }
    if (s->state==TERRAIN_INVALID) {
        /* Recover only on a confirmed flat baseline. Restarting mid-ascent
         * after a shock/gap must never complete a partial hump. */
        s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0U;
        if (s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
            s->state=TERRAIN_FLAT; s->flat_samples=0;
        }
        return;
    }
    if (s->state==TERRAIN_FLAT && pc>=TERRAIN_SLOPE_CDEG) {
        s->state=TERRAIN_ASCENT; s->height_code=20000; s->peak_um=0; s->height_valid=0;
    }
    if (s->state==TERRAIN_ASCENT || s->state==TERRAIN_CREST || s->state==TERRAIN_DESCENT) {
        int64_t h=(int64_t)terrain_height_um(s)+(int64_t)lround((double)du*sin((double)pc/CDEG_PER_RAD));
        if (h < -20000 || h > 1000000) { abandon(s); return; }
        s->height_code=(uint32_t)(h+20000)&0xfffffU;
        if (h>(int64_t)s->peak_um) s->peak_um=(uint32_t)h&0xfffffU;
        if (s->state==TERRAIN_ASCENT && pc<=TERRAIN_FLAT_CDEG) s->state=TERRAIN_CREST;
        if (s->state==TERRAIN_CREST && pc<=-TERRAIN_SLOPE_CDEG) s->state=TERRAIN_DESCENT;
        if (s->state==TERRAIN_DESCENT) {
            s->flat_samples=abs(pc)<=TERRAIN_FLAT_CDEG?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0;
            if (s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
                /* A completed hump returns approximately to its base level. */
                if (abs(terrain_height_um(s))<=TERRAIN_BASE_TOLERANCE_UM && s->peak_um>=TERRAIN_MIN_HUMP_UM) {
                    s->height_mm=((s->peak_um+500U)/1000U)&1023U; s->height_valid=1;
                    if (s->height_mm>s->maximum_mm) s->maximum_mm=s->height_mm;
                }
                s->state=TERRAIN_FLAT; s->flat_samples=0;
            }
        }
    }
}
