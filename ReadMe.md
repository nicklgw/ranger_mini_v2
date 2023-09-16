
仿真四舵轮ranger_mini_v2
nvidia@nvidia-pc:~/ranger_mini_v2$ colcon build
nvidia@nvidia-pc:~/ranger_mini_v2$ source install/setup.bash
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2 display.launch.py

显示gazebo的模型
nvidia@nvidia-pc:~/ranger_mini_v2$ source install/setup.bash
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo display_xacro.launch.py



显示另一个四舵轮模型
nvidia@nvidia-pc:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo robot_sim.launch.py


将xacro转为URDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ xacro ranger_mini_gazebo.xacro > ranger_mini_gazebo.urdf

将URDF转换为SDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ gz sdf -p ranger_mini_gazebo.urdf > ranger_mini_gazebo.sdf

将sdf重命名为ranger_mini_v2.sdf，放置到ranger_mini_v2_gazebo/models/ranger_mini_v2

显示ranger_mini_v2仿真模型
nick@nick-vmware:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo ranger_mini_v2_sim.launch.py 


git大文件传输（LFS）及超限解决
git lfs install （安装git lfs，一个账号只用安装一次）
git lfs track “*.gif” （eg:追踪记录.gif类型的文件；根据自己需要想要上传什么类型的文件，就追踪什么类型的文件）
git lfs track “demo/*.mp4”（可以添加多个追踪配置，冒号里表示demo文件夹下的mp4文件）
git add .gitattributes（运行完后当前文件夹下会生成一个.gitattributes文件，存储第二步的相关配
git add 添加的文件
git commit -m “describe info”
