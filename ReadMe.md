
仿真四舵轮ranger_mini_v2
nvidia@nvidia-pc:~/ranger_mini_v2$ colcon build
nvidia@nvidia-pc:~/ranger_mini_v2$ source install/setup.bash
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2 display.launch.py

显示gazebo的模型
nvidia@nvidia-pc:~/ranger_mini_v2$ source install/setup.bash
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo display_xacro.launch.py



显示另一个四舵轮模型
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo robot_sim.launch.py
