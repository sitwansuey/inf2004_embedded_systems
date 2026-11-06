#ifndef VEHICLE_POSE_H
#define VEHICLE_POSE_H
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
typedef struct {
 bool compass_calibrated;
 int32_t mag_offset[3],mag_scale[3];
} vehicle_pose_config_t;
typedef struct {
 float x_um,y_um,yaw,ref_x,ref_y,ref_yaw,mag_reference,field;
 int32_t previous_distance,previous_yaw;
 uint32_t previous_ms;
 uint64_t mag_time;
 bool initialized,valid,anchored,mag_initialized,imu_assisted;
} vehicle_pose_t;
static inline float vp_wrap(float a) {
 while(a>3.141592654f)a-=6.283185307f;
 while(a< -3.141592654f)a+=6.283185307f;
 return a;
}
static inline int32_t vp_yaw(const vehicle_pose_t *p) {
 return (int32_t)(p->yaw*57295.77951f);
}
static inline void vp_anchor(vehicle_pose_t *p) {
 p->ref_x=p->x_um;p->ref_y=p->y_um;p->ref_yaw=p->yaw;p->anchored=p->valid;
}
static inline int32_t vp_along_track(const vehicle_pose_t *p) {
 return p->anchored?(int32_t)((p->x_um-p->ref_x)*cosf(p->ref_yaw)+
     (p->y_um-p->ref_y)*sinf(p->ref_yaw)):0;
}
static inline int32_t vp_cross_track(const vehicle_pose_t *p) {
 return p->anchored?(int32_t)(-(p->x_um-p->ref_x)*sinf(p->ref_yaw)+
     (p->y_um-p->ref_y)*cosf(p->ref_yaw)):0;
}
static inline void vp_step(vehicle_pose_t *p,const vehicle_pose_config_t *c,
 uint32_t now,int32_t distance,int32_t yaw,bool odom_valid,
 const int32_t accel[3],const int32_t mag[3],uint64_t mag_time,bool imu_fresh)
{
 p->imu_assisted=false;
 if(!odom_valid) {p->valid=false;p->initialized=false;return;}
 float enc=(float)yaw/57295.77951f;
 if(!p->initialized) {
  p->previous_distance=distance;p->previous_yaw=yaw;p->previous_ms=now;
  p->yaw=vp_wrap(enc);p->initialized=true;p->valid=true;return;
 }
 uint32_t dt=now-p->previous_ms;
 if(!dt)return;
 int64_t delta=(int64_t)distance-p->previous_distance;
 if(dt>100U || delta>2000LL*dt || delta< -2000LL*dt) {
  p->valid=false;p->initialized=false;return;
 }
 float before=p->yaw;
 float predicted=vp_wrap(before+vp_wrap(enc-(float)p->previous_yaw/57295.77951f));
 if(c->compass_calibrated && imu_fresh && mag_time!=p->mag_time) {
  p->mag_time=mag_time;
  float a[3],m[3],norm=0;
  for(unsigned n=0;n<3;++n){a[n]=(float)accel[n];norm+=a[n]*a[n];
   m[n]=((float)mag[n]-(float)c->mag_offset[n])*(float)c->mag_scale[n]/1000.0f;}
  norm=sqrtf(norm);
  if(norm>800.0f && norm<1200.0f) {
   for(unsigned n=0;n<3;++n)a[n]/=norm;
   /* Project forward onto the gravity-normal plane, then derive left. */
   float f[3]={1.0f-a[0]*a[0],-a[0]*a[1],-a[0]*a[2]};
   float fn=sqrtf(f[0]*f[0]+f[1]*f[1]+f[2]*f[2]);
   if(fn>0.5f) {
    for(unsigned n=0;n<3;++n)f[n]/=fn;
    float l[3]={a[1]*f[2]-a[2]*f[1],a[2]*f[0]-a[0]*f[2],a[0]*f[1]-a[1]*f[0]};
    float forward=0,left=0,field=0;
    for(unsigned n=0;n<3;++n){forward+=m[n]*f[n];left+=m[n]*l[n];field+=m[n]*m[n];}
    field=sqrtf(field);
    if(field>150.0f && field<900.0f && forward*forward+left*left>10000.0f) {
     float heading=-atan2f(left,forward);
     if(!p->mag_initialized){p->mag_reference=vp_wrap(heading-predicted);p->field=field;p->mag_initialized=true;}
     float error=vp_wrap(heading-p->mag_reference-predicted);
     if(field>p->field*0.8f && field<p->field*1.2f && fabsf(error)<0.261799388f) {
      predicted=vp_wrap(predicted+0.02f*error);p->imu_assisted=true;
     }
    }
   }
  }
 }
 float mid=vp_wrap(before+vp_wrap(predicted-before)*0.5f);
 p->x_um+=(float)delta*cosf(mid);p->y_um+=(float)delta*sinf(mid);p->yaw=predicted;
 p->previous_distance=distance;p->previous_yaw=yaw;p->previous_ms=now;p->valid=true;
}
#endif
