import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True"
    )

    nav_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_navigation"),"/launch","/nav2_keepout_filter_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time,
            }.items()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_description"),"/launch","/gazebo_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time,
            }.items()
    )
    
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_bringup"),"/launch","/static_tf_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time
            }.items()
    )

    return LaunchDescription([arg_sim,
                              gazebo_launch,
                              nav_launch,
                              tf_launch,
                              ])
    
    
    