
仿真四舵轮ranger_mini_v2
nick@nick-vmware:~/ranger_mini_v2$ colcon build
nick@nick-vmware:~/ranger_mini_v2$ source install/setup.bash
nick@nick-vmware:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo display_xacro.launch.py                    # 在rviz中显示车体模型
nick@nick-vmware:~/ranger_mini_v2$ ros2 launch ranger_mini_v2_gazebo ranger_mini_v2_empty_world.launch.py		# 在gazebo中显示车体模型

手动遥控
nick@nick-vmware:~/ranger_mini_v2$ ros2 run teleop_twist_keyboard teleop_twist_keyboard

将xacro转为URDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ xacro ranger_mini_gazebo.xacro > ranger_mini_gazebo.urdf

将URDF转换为SDF
nick@nick-vmware:~/ranger_mini_v2/src/ranger_mini_v2_gazebo/xacro$ gz sdf -p ranger_mini_gazebo.urdf > ranger_mini_gazebo.sdf

将sdf重命名为ranger_mini_v2.sdf，放置到ranger_mini_v2_gazebo/models/ranger_mini_v2


four_wheel_steering_msgs来自ros2分支
https://github.com/ros-drivers/four_wheel_steering_msgs.git


git大文件传输（LFS）及超限解决
git lfs install （安装git lfs，一个账号只用安装一次）
git lfs track “*.gif” （eg:追踪记录.gif类型的文件；根据自己需要想要上传什么类型的文件，就追踪什么类型的文件）
git lfs track “demo/*.mp4”（可以添加多个追踪配置，冒号里表示demo文件夹下的mp4文件）
git add .gitattributes（运行完后当前文件夹下会生成一个.gitattributes文件，存储第二步的相关配
git add 添加的文件
git commit -m “describe info”



    <static>0</static>

    <plugin name="ranger_mini_joint_state" filename="libgazebo_ros_joint_state_publisher.so">
      <ros>
          <remapping>~/out:=joint_states</remapping>
      </ros>
      <update_rate>30</update_rate>
      <joint_name>fl_wheel</joint_name>
      <joint_name>fr_wheel</joint_name>
      <joint_name>rl_wheel</joint_name>
      <joint_name>rr_wheel</joint_name>
      <joint_name>fl_steering_wheel</joint_name>
      <joint_name>fr_steering_wheel</joint_name>
      <joint_name>rl_steering_wheel</joint_name>
      <joint_name>rr_steering_wheel</joint_name>
    </plugin>

    <ros2_control name="GazeboSystem" type="system">
        <hardware>
            <plugin>gazebo_ros2_control/GazeboSystem</plugin>
        </hardware>

        <joint name="fl_wheel">
            <command_interface name="velocity">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="fr_wheel">
            <command_interface name="velocity">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="rl_wheel">
            <command_interface name="velocity">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="rr_wheel">
            <command_interface name="velocity">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="fl_steering_wheel">
            <command_interface name="position">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="position">
            <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="fr_steering_wheel">
            <command_interface name="position">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="position">
            <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="rl_steering_wheel">
            <command_interface name="position">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="position">
            <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>

        <joint name="rr_steering_wheel">
            <command_interface name="position">
            <param name="min">-3.14</param>
            <param name="max">3.14</param>
            </command_interface>
            <state_interface name="position">
            <param name="initial_value">0</param>
            </state_interface>
            <state_interface name="velocity"/>
            <state_interface name="effort"/>
        </joint>
    </ros2_control>

    <plugin name="gazebo_ros2_control" filename="libgazebo_ros2_control.so">
        <parameters>/home/nick/ranger_mini_v2/install/ranger_mini_v2_gazebo/share/ranger_mini_v2_gazebo/models/ranger_mini_v2/config/controllers.yaml</parameters>
    </plugin>