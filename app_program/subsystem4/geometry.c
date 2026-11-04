#include "terrain.h"
#include "mounting.h"
#include <stdlib.h>

bool terrain_geometry_valid(const terrain_geometry_t *g) {
    return g && g->configured && g->track_mm>=40 && g->track_mm<=1000 &&
        g->forward_mm>=-500 && g->forward_mm<=500 && g->left_mm>=-500 &&
        g->left_mm<=500 && g->up_mm>=-500 && g->up_mm<=500;
}
void terrain_step_model(terrain_t *s,uint32_t ms,const int32_t a[3],bool av,
    int32_t distance,bool ov,bool calibrated,int pzero,int rzero,
    const terrain_geometry_t *g) {
    if(!s || !a)return;
    if(!g || !g->one_sided) {
        terrain_step(s,ms,a,av,distance,ov,calibrated,pzero,rzero);return;
    }
    /* Reuse the same gravity/angle/roll gates without touching traversal. */
    terrain_t tilt={0};
    terrain_step(&tilt,ms,a,av,0,false,calibrated,pzero,rzero);
    s->tilt_valid=tilt.tilt_valid;s->pitch_code=tilt.pitch_code;
    s->roll_code=tilt.roll_code;s->shock=tilt.shock;
    if(!s->tilt_valid || !ov || !terrain_geometry_valid(g)) {
        terrain_invalidate_height(s);
        if(s->shock)s->state=TERRAIN_SHOCK;
        return;
    }
    if(s->initialized && ms==s->sample_ms)return;
    uint32_t dt=ms-s->sample_ms;
    int64_t du=(int64_t)distance-s->previous_distance_um;
    s->sample_ms=ms;s->previous_distance_um=distance;
    double p=terrain_pitch_cdeg(s)/S4_CDEG_PER_RAD;
    double r=terrain_roll_cdeg(s)/S4_CDEG_PER_RAD;
    /* R = Ry(-pitch) Rx(roll). Grounded wheel fixes drive-axle centre
     * elevation; transform the measured IMU lever arm relative to flat. */
    double cp=cos(p),sr=sin(r);
    long h=lround(1000.0*(0.5*g->track_mm*fabs(sr)*cp+
        g->forward_mm*sin(p)+g->left_mm*sr*cp+g->up_mm*(cp*cos(r)-1.0)));
    if(h< -20000L || h>1000000L || (s->initialized && (dt>60U || du<0 || du>120000))) {
        terrain_invalidate_height(s);return;
    }
    if(!s->initialized) {
        s->initialized=1;s->state=TERRAIN_INVALID;s->flat_samples=0;
    }
    if(s->state==TERRAIN_INVALID) {
        bool flat=labs(h)<=TERRAIN_ONE_SIDE_FLAT_UM && abs(terrain_pitch_cdeg(s))<=TERRAIN_FLAT_CDEG &&
            abs(terrain_roll_cdeg(s))<=TERRAIN_FLAT_CDEG;
        s->flat_samples=flat?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0U;
        if(s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
            s->state=TERRAIN_FLAT;s->flat_samples=0;
        }
        return; /* never start halfway through a traversal */
    }
    if(s->state==TERRAIN_FLAT && h>=TERRAIN_ONE_SIDE_ENTER_UM) {
        s->state=TERRAIN_ASCENT;s->peak_um=0;s->height_valid=0;s->traversal_um=0;
    }
    if(s->state==TERRAIN_ASCENT || s->state==TERRAIN_CREST || s->state==TERRAIN_DESCENT) {
        int32_t previous=terrain_height_um(s);
        s->height_code=(uint32_t)(h+20000L)&0xfffffU;
        uint64_t travel=(uint64_t)s->traversal_um+(du>0?(uint64_t)du:0U);
        s->traversal_um=(uint32_t)(travel>1000000U?1000000U:travel)&0xfffffU;
        if(h>(long)s->peak_um)s->peak_um=(uint32_t)h&0xfffffU;
        if(s->state==TERRAIN_ASCENT && labs(h-previous)<=500L)s->state=TERRAIN_CREST;
        if((s->state==TERRAIN_ASCENT || s->state==TERRAIN_CREST) &&
           (long)s->peak_um-h>=TERRAIN_ONE_SIDE_DESCENT_UM)s->state=TERRAIN_DESCENT;
        if(s->state==TERRAIN_DESCENT) {
            s->flat_samples=labs(h)<=TERRAIN_ONE_SIDE_FLAT_UM?((unsigned)s->flat_samples+1U)&((1U<<TERRAIN_FLAT_COUNT_BITS)-1U):0U;
            if(s->flat_samples>=TERRAIN_FLAT_CONFIRM_SAMPLES) {
                if(s->peak_um>=TERRAIN_ONE_SIDE_ENTER_UM && s->traversal_um>=TERRAIN_ONE_SIDE_MIN_TRAVEL_UM) {
                    s->height_mm=((s->peak_um+500U)/1000U)&1023U;s->height_valid=1;
                    if(s->height_mm>s->maximum_mm)s->maximum_mm=s->height_mm;
                }
                s->state=TERRAIN_FLAT;s->flat_samples=0;
            }
        }
    }
}
