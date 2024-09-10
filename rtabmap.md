
ros2 launch --show-args turtlebot3_gazebo turtlebot3_world.launch.py


# 启动 Turtlebot3 模拟器
export TURTLEBOT3_MODEL=waffle
source /usr/share/gazebo/setup.sh
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
 
export TURTLEBOT3_MODEL=waffle
ros2 run turtlebot3_teleop teleop_keyboard

# 启动 RTAB 地图
ros2 launch rtabmap_demos turtlebot3_scan.launch.py
 

# 启动导航（nav2_bringup应安装软件包）：
ros2 launch nav2_bringup navigation_launch.py use_sim_time:=True
ros2 launch nav2_bringup rviz_launch.py




# bringup - vslam
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
clear
ros2 launch agrobot_bringup simulation_vslam_launch.py

source install/setup.sh
clear
ros2 launch agrobot_vslam rtabmap_launch.py