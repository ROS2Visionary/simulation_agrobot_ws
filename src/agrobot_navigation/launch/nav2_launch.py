import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
        
    map_yaml_path = os.path.join(get_package_share_directory("agrobot_mapping"),"maps","map_56.yaml")
    nav2_param_path = os.path.join(get_package_share_directory("agrobot_navigation"),"config","nav2_params.yaml")
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="True"
    )
    
    nav2_node =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/bringup_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'map': map_yaml_path,
                'use_sim_time': use_sim_time,
                'params_file': nav2_param_path}.items(),
    )
    
    
    lifecycle_nodes = ["map_server","amcl"]
    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager",
        parameters=[
            {"node_names": lifecycle_nodes},
            {"use_sim_time":use_sim_time},
            {"autostart": True}
        ],
    )
    
    return launch.LaunchDescription([arg_sim,
                                     nav2_node,
                                     nav2_lifecycle_manager_node,
                                     ])
