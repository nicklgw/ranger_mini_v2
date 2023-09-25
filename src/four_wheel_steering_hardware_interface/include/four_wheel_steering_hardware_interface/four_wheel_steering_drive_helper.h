
#ifndef __FOUR_WHEEL_STEERING_DRIVE_HELPER_H__
#define __FOUR_WHEEL_STEERING_DRIVE_HELPER_H__

namespace FourWheelSteeringDriveHelper
{

int    Init(); // 电机初始化
int    Exit(); // 电机退出

double GetVelocity(int node_id);              // 获取指定轮子角速度, 单位rad/s
int    SetVelocity(int node_id, double vel);  // 设置指定轮子角速度，单位rad/s
double GetPosition(int node_id);              // 获取指定轮子位置, 单位rad
int    SetPosition(int node_id, double pos);  // 设置指定轮子位置, 单位rad

}

#endif // __FOUR_WHEEL_STEERING_DRIVE_HELPER_H__
