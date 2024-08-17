import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        name="use_sim_time",
        default_value="True"
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_description"),"/launch","/gazebo_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time,
            }.items()
    )

    # localization_mode = "/amcl_localization_launch.py"
    localization_mode = "/slam_localization_launch.py"
    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_localization"),"/launch",localization_mode]),
        launch_arguments={
            "use_sim_time":use_sim_time
            }.items()
    )
    
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_bringup"),"/launch","/static_tf_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time
            }.items()
    )

    return LaunchDescription([arg_sim,
                              tf_launch,
                              gazebo_launch,
                              localization_launch,
                              ])
