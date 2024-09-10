
# bringup - 模拟建图
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_bringup simulation_mapping_launch.py 

# bringup - 模拟定位
colcon build
source /usr/share/gazebo/setup.sh  
source install/setup.sh
ros2 launch agrobot_bringup simulation_localization_launch.py

# bringup - 模拟导航 amcl
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_bringup simulation_nav_launch.py

# bringup - 模拟导航 slam
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_bringup simulation_nav_slam_launch.py

# bringup - 模拟导航 & 建图
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_bringup simulation_nav_slam_mapping_launch.py

# bringup - 模拟导航 & 建图 & KeepoutFilter
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_bringup simulation_nav_slam_KFilter_launch.py

# bringup - 模拟导航 & KeepoutFilter 
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
clear
ros2 launch agrobot_bringup simulation_nav_filter.py

# bringup - 校验KeepoutFilter 
colcon build
source install/setup.sh
clear
ros2 launch agrobot_bringup visual_map_launch.py

# bringup - 融合数据
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
clear
ros2 launch agrobot_bringup simulation_fusion_launch.py

# localization debug
colcon build
source install/setup.sh
ros2 launch agrobot_localization debug_localization_launch.py 

# bringup - vslam
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
clear
ros2 launch agrobot_bringup simulation_vslam_launch.py

# 启动仿真gazebo
colcon build
source /usr/share/gazebo/setup.sh
source install/setup.sh
ros2 launch agrobot_description gazebo_launch.py

# 仿真环境GitHub地址
https://github.com/aws-robotics
https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps

# 下载环境创库
cd src
wstool update
cd ..

# 启动键盘控制
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# 打开动态调参窗口
ros2 run rqt_reconfigure rqt_reconfigure

# 查看launch参数
ros2 launch --show-args nav2_bringup bringup_launch.py
ros2 launch --show-args nav2_bringup navigation_launch.py
ros2 launch --show-args slam_toolbox online_async_launch.py
ros2 launch --show-args slam_toolbox localization_launch.py
ros2 launch --show-args slam_toolbox lifelong_launch.py
ros2 launch --show-args robot_localization ukf.launch.py
ros2 launch --show-args rtabmap_launch rtabmap.launch.py


# GitHub地址 - 库地址
https://github.com/ros-navigation/navigation2
https://github.com/SteveMacenski/slam_toolbox
https://github.com/introlab/rtabmap_ros

# GitHub地址 - 第三方gazebo仿真环境
https://github.com/mlherd/Dataset-of-Gazebo-Worlds-Models-and-Maps
https://github.com/aws-robotics


# 保存地图
ros2 run nav2_map_server map_saver_cli -t map -f '/ros2_projects/agrobot_ws/src/agrobot_mapping/temp_mapping_dir/map'


# 使用命令行进行slam
ros2 launch nav2_bringup navigation_launch.py
ros2 launch slam_toolbox online_async_launch.py
ros2 topic pub /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 0.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"


# slam_toolbox:保存序列化地图文件
ros2 service call /slam_toolbox/serialize_map slam_toolbox/srv/SerializePoseGraph "filename: '/root/ros2_projects/simulation_agrobot_ws/src/agrobot_mapping/st_maps/map'"

