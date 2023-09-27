
仿真四舵轮ranger_mini_v2
nick@nick-vmware:~/ranger_mini_v2$ colcon build
nick@nick-vmware:~/ranger_mini_v2$ source install/setup.bash
nick@nick-vmware:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo display_xacro.launch.py                    # 在rviz中显示车体模型
nick@nick-vmware:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo ranger_mini_v2_empty_world.launch.py		# 在gazebo中显示车体模型

手动遥控
nick@nick-vmware:~/ranger_mini_v2$ ros2 run teleop_twist_keyboard teleop_twist_keyboard

加载实际驱动，并启动显示rviz2. 修改宏 PHYSICAL_MACHINE 为 1
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo display_real.launch.py

将xacro转为URDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ xacro ranger_mini_gazebo.xacro > ranger_mini_gazebo.urdf

将URDF转换为SDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ gz sdf -p ranger_mini_gazebo.urdf > ranger_mini_gazebo.sdf

将sdf重命名为ranger_mini_v2.sdf，放置到ranger_mini_v2_gazebo/models/ranger_mini_v2

实体车上编译时，需要将库安装
sudo dpkg -i libmotor-driver-2.0.20230801_amd64.deb

设置程序执行链接库环境
``````
export LD_LIBRARY_PATH=/usr/local/bzl_robot/lib:$LD_LIBRARY_PATH
``````
