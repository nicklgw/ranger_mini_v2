
#ifndef __DIFF_DRIVE_HELPER_H__
#define __DIFF_DRIVE_HELPER_H__

namespace DiffDriveHelper
{

int    Init(); // 电机初始化
int    Exit(); // 电机退出
double GetVelocity(int index); // 获取指定轮子角速度, 单位rad/s
int    SetVelocity(int index, double vel); // 设置指定轮子角速度，单位rad/s
double GetPosition(int index); // 获取指定轮子位置, 单位rad
}

#endif // __DIFF_DRIVE_HELPER_H__
