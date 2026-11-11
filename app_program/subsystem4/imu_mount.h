#ifndef S4_IMU_MOUNT_H
#define S4_IMU_MOUNT_H
#include <stdint.h>
/* Signed GY-511 axis code: 1=+X 2=+Y 3=+Z, negative for the minus axis. */
static inline int32_t s4_imu_axis(const int32_t a[3],int axis)
{
    unsigned n=(unsigned)(axis<0?-axis:axis)-1U;
    if(n>2U)return 0;
    /* Negating -2048 would leave the 12-bit range terrain accepts. */
    return axis>0?a[n]:a[n]==-2048?2047:-a[n];
}
/* Sensor reading to the robot frame: X front, Y left, Z up (BOARD_IMU_*). */
static inline void s4_imu_to_robot(const int32_t sensor[3],int32_t robot[3],
                                   int front,int left,int up)
{
    robot[0]=s4_imu_axis(sensor,front);
    robot[1]=s4_imu_axis(sensor,left);
    robot[2]=s4_imu_axis(sensor,up);
}
#endif
