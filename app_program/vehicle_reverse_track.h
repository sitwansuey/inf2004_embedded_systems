#ifndef VEHICLE_REVERSE_TRACK_H
#define VEHICLE_REVERSE_TRACK_H
#include <stdint.h>
#include <stdbool.h>
/* Line-tracked reverse for the barcode re-read; with the line term off
 * (line_fresh false) it also holds the obstacle reverse (VC_BACKUP) heading.
 *
 * Copying the forward bang-bang rule while reversing is unstable: the straddle
 * sensors trail the axle, so the heading diverges. Instead hold the heading
 * the robot had while tracking forward (odometry yaw, +CCW) and add a small
 * line term. In reverse, positive S2 steering (left wheel faster backwards)
 * turns the robot CCW, which moves the trailing sensors toward the robot's
 * right, so a line under the right sensor (mask 1) asks for positive steering.
 * testing/integration/reverse_track_test.c closes the loop over 60 cm with
 * up to 5 deg of heading-reference error: with gains 40/200 and sensors
 * 30-60 mm ahead of the axle the sensors stay within 1.5 cm of the line. */

/* Odometry yaw wraps at +/-360000 mdeg; differences use the short way round. */
static inline int32_t reverse_track_wrap_mdeg(int32_t mdeg)
{
    mdeg %= 360000;
    if(mdeg>=180000) mdeg-=360000;
    else if(mdeg< -180000) mdeg+=360000;
    return mdeg;
}

/* Exponential moving average of yaw; seed with the first sample. */
static inline int32_t reverse_track_yaw_ref(int32_t ref,int32_t yaw_mdeg,bool seeded,int32_t divisor)
{
    if(!seeded || divisor<=1) return reverse_track_wrap_mdeg(yaw_mdeg);
    return reverse_track_wrap_mdeg(ref+reverse_track_wrap_mdeg(yaw_mdeg-ref)/divisor);
}

/* mask bit 0 = right sensor (GP26) black, bit 1 = left (GP27) black. */
static inline int32_t reverse_track_steering(int32_t yaw_mdeg,int32_t ref_mdeg,
    unsigned black_mask,bool line_fresh,int32_t yaw_gain,int32_t line_gain,int32_t limit)
{
    int32_t theta=reverse_track_wrap_mdeg(yaw_mdeg-ref_mdeg);
    int32_t e=0;
    if(line_fresh) {
        black_mask&=3U;
        e=black_mask==1U?1:black_mask==2U?-1:0;
    }
    int64_t steer=-(int64_t)yaw_gain*theta/1000+(int64_t)line_gain*e;
    if(steer>limit) steer=limit;
    if(steer< -limit) steer=-limit;
    return (int32_t)steer;
}
#endif
