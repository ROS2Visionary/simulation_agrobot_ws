import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument,TimerAction
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

    vslam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_vslam"),"/launch","/rtabmap_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time,
            }.items()
    )

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_description"),"/launch","/gazebo_launch.py"]),
    )
    
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_bringup"),"/launch","/static_tf_launch.py"]),
    )

    return LaunchDescription([arg_sim,
                              gazebo_launch,
                              tf_launch,
                            #   TimerAction(
                            #       period=10.0,
                            #       actions=[vslam_launch]
                            #   )
                              ])
    
    
    