import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument,IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

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

    
    mapping_launch_name = "/toolbox_mapping_launch.py" # slam建图
    # mapping_launch_name = "/custom_mapping_launch.py" # custom建图
    # mapping_launch_name = "/cartographer_mapping_launch.py" # cartographer建图

    mapping_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_mapping"),"/launch",mapping_launch_name]),
        launch_arguments={
            "use_sim_time":use_sim_time
            }.items()
    )
    
    tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_description"),"/launch","/simulation_tf_launch.py"]),
        launch_arguments={
            "use_sim_time":use_sim_time
            }.items()
    )

    return LaunchDescription([arg_sim,
                              tf_launch,
                              gazebo_launch,
                              mapping_node
                              ])