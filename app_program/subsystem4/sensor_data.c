/* SPDX-License-Identifier: Apache-2.0 */
#include "sensor_data.h"
#include "packed_bits.h"
/* init(3),poll(3),configured(1); accel XYZ(36),ms(55),flags/status(6);
 * mag XYZ(48),ms(55),flags/status(5); nine modulo-32 counters. */
enum { ACCEL_BIT=7, MAG_BIT=104, COUNTER_BIT=212 };
_Static_assert(S4_CACHE_BITS==500U && S4_CACHE_BYTES==63U,"cache schema");
uint32_t s4_cache_count(const s4_sensor_cache_t *s,unsigned n) {
    return s && n<S4_COUNT_TOTAL?(uint32_t)s4_bits_get(s->bytes,COUNTER_BIT+n*32U,32):0;
}
void s4_cache_set_count(s4_sensor_cache_t *s,unsigned n,uint32_t value) {
    if(s && n<S4_COUNT_TOTAL)s4_bits_put(s->bytes,COUNTER_BIT+n*32U,32,value);
}
void s4_cache_increment(s4_sensor_cache_t *s,unsigned n) {
    s4_cache_set_count(s,n,s4_cache_count(s,n)+1U);
}
void s4_data_reset(s4_sensor_cache_t *s) {
    *s=(s4_sensor_cache_t){0};
    s4_bits_put(s->bytes,0,3,LSM303_NOT_INITIALIZED);
    s4_bits_put(s->bytes,3,3,LSM303_NOT_INITIALIZED);
}
void s4_data_invalidate(s4_sensor_cache_t *s) {
    s4_bits_put(s->bytes,6,1,0);s4_bits_put(s->bytes,0,3,LSM303_IO_ERROR);
    s4_bits_put(s->bytes,ACCEL_BIT+36U+55U+1U,1,0);
    s4_bits_put(s->bytes,MAG_BIT+48U+55U+1U,1,0);
}
void s4_data_init_result(s4_sensor_cache_t *s,lsm303_result_t result) {
    unsigned code=(unsigned)result;
    if(code>LSM303_CONFIG_ERROR)code=LSM303_BAD_ARGUMENT;
    for(unsigned bit=0;bit<COUNTER_BIT;++bit)s4_bits_put(s->bytes,bit,1,0);
    s4_bits_put(s->bytes,0,3,code);s4_bits_put(s->bytes,3,3,LSM303_NOT_INITIALIZED);
    s4_bits_put(s->bytes,6,1,code==LSM303_OK);s4_cache_increment(s,S4_COUNT_INIT);
}
static void update_sample(s4_sensor_cache_t *s,const lsm303_sample_t *sample,bool mag) {
    unsigned base=mag?MAG_BIT:ACCEL_BIT, rawbits=mag?16U:12U;
    unsigned stamp=base+3U*rawbits,flags=stamp+55U,status=sample->status;
    bool fits=sample->read_time_us!=UINT64_MAX && sample->read_time_us%1000U==0U;
    if(status>LSM303_LOCK_DRAINED) {status=LSM303_DATA_IO_ERROR;fits=false;}
    for(unsigned n=0;n<3;++n)
        if(!mag && (sample->raw[n]<-2048 || sample->raw[n]>2047))fits=false;
    if(sample->fresh && fits) {
        for(unsigned n=0;n<3;++n) {
            int32_t minimum=mag?-32768:-2048;
            s4_bits_put(s->bytes,base+n*rawbits,rawbits,(uint32_t)((int32_t)sample->raw[n]-minimum));
        }
        s4_bits_put(s->bytes,stamp,55,sample->read_time_us/1000U);
        s4_bits_put(s->bytes,flags,1,1);s4_bits_put(s->bytes,flags+1U,1,sample->valid);
        if(!mag)s4_bits_put(s->bytes,flags+2U,1,sample->overrun);
    } else if(status==LSM303_DATA_IO_ERROR || (sample->fresh && !fits)) {
        s4_bits_put(s->bytes,flags+1U,1,0);status=LSM303_DATA_IO_ERROR;
    }
    s4_bits_put(s->bytes,flags+(mag?2U:3U),3,status);
}
void s4_data_update(s4_sensor_cache_t *s,const lsm303_frame_t *f,lsm303_result_t result) {
    unsigned code=(unsigned)result;
    if(code>LSM303_CONFIG_ERROR)code=LSM303_BAD_ARGUMENT;
    s4_bits_put(s->bytes,3,3,code);
    if(f->accel.fresh)s4_cache_increment(s,S4_COUNT_ACCEL);
    if(f->mag.fresh)s4_cache_increment(s,S4_COUNT_MAG);
    if(f->accel.status==LSM303_DATA_IO_ERROR)s4_cache_increment(s,S4_COUNT_ACCEL_ERROR);
    if(f->mag.status==LSM303_DATA_IO_ERROR)s4_cache_increment(s,S4_COUNT_MAG_ERROR);
    if(f->accel.overrun)s4_cache_increment(s,S4_COUNT_OVERRUN);
    update_sample(s,&f->accel,false);update_sample(s,&f->mag,true);
}
static lsm303_sample_t decode_sample(const s4_sensor_cache_t *s,bool mag) {
    lsm303_sample_t v={0};
    unsigned base=mag?MAG_BIT:ACCEL_BIT,rawbits=mag?16U:12U;
    unsigned stamp=base+3U*rawbits,flags=stamp+55U;
    v.fresh=s4_bits_get(s->bytes,flags,1)!=0;
    v.valid=s4_bits_get(s->bytes,flags+1U,1)!=0;
    v.overrun=!mag && s4_bits_get(s->bytes,flags+2U,1)!=0;
    switch(s4_bits_get(s->bytes,flags+(mag?2U:3U),3)) {
    case 0:v.status=LSM303_NO_DATA;break;
    case 1:v.status=LSM303_DATA_OK;break;
    case 2:v.status=LSM303_DATA_CLIPPED;break;
    case 4:v.status=LSM303_LOCK_DRAINED;break;
    default:v.status=LSM303_DATA_IO_ERROR;break;
    }
    v.read_time_us=s4_bits_get(s->bytes,stamp,55)*1000U;
    for(unsigned n=0;n<3;++n) {
        int32_t minimum=mag?-32768:-2048;
        v.raw[n]=(int16_t)((int32_t)s4_bits_get(s->bytes,base+n*rawbits,rawbits)+minimum);
        /* Fixed driver scale, preserving full magnetic diagnostic range. */
        v.milli_units[n]=mag?(int32_t)v.raw[n]*1000/(n==2U?980:1100):v.raw[n];
    }
    return v;
}
static uint32_t age(const lsm303_sample_t *v,uint64_t now) {
    if(!v->fresh || now==UINT64_MAX || now<v->read_time_us)return S4_NO_SAMPLE_AGE;
    uint64_t ms=(now-v->read_time_us)/1000U;
    return ms>=S4_NO_SAMPLE_AGE?S4_NO_SAMPLE_AGE-1U:(uint32_t)ms;
}
static bool usable(const lsm303_sample_t *v,uint64_t now) {
    return v->fresh && v->valid && now!=UINT64_MAX && now>=v->read_time_us &&
           now-v->read_time_us<=S4_STALE_AFTER_US;
}
void s4_data_snapshot(const s4_sensor_cache_t *s,uint64_t now,s4_sensor_status_t *out) {
    *out=(s4_sensor_status_t){0};out->latest.accel=decode_sample(s,false);out->latest.mag=decode_sample(s,true);
    out->init_result=(uint8_t)s4_bits_get(s->bytes,0,3)&7U;
    out->poll_result=(uint8_t)s4_bits_get(s->bytes,3,3)&7U;
    out->configured=s4_bits_get(s->bytes,6,1)!=0;out->observed_time_us=now;
    out->accel_age_ms=age(&out->latest.accel,now);out->mag_age_ms=age(&out->latest.mag,now);
    out->accel_usable=out->configured && usable(&out->latest.accel,now);
    out->mag_usable=out->configured && usable(&out->latest.mag,now);
    out->init_attempts=s4_cache_count(s,S4_COUNT_INIT);out->task_heartbeat=s4_cache_count(s,S4_COUNT_HEARTBEAT);
    out->accel_samples=s4_cache_count(s,S4_COUNT_ACCEL);out->mag_samples=s4_cache_count(s,S4_COUNT_MAG);
    out->accel_io_errors=s4_cache_count(s,S4_COUNT_ACCEL_ERROR);out->mag_io_errors=s4_cache_count(s,S4_COUNT_MAG_ERROR);
    out->accel_overruns=s4_cache_count(s,S4_COUNT_OVERRUN);out->publish_errors=s4_cache_count(s,S4_COUNT_PUBLISH_ERROR);
    out->legacy_lines_received=s4_cache_count(s,S4_COUNT_LEGACY);
}
