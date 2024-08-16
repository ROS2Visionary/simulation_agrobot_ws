
import os
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    package_name = 'agrobot_description'
    urdf_name = "robot_display.urdf"

    pkg_share = FindPackageShare(package=package_name).find(package_name) 
    urdf_model_path = os.path.join(pkg_share, f'urdf/{urdf_name}')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path]
    )
    
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
        'use_sim_time',
        default_value="True"
    )
    
    tf_base_footprint_to_base_link = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',"1.0",'base_footprint', 'base_link'] 
    )
    
    tf_odom_to_base_footprint = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_odom',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',"1.0",'odom', 'base_footprint'] 
    )
    
    tf_odom_to_map = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher_map',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0',"1.0",'map', 'odom'] 
    )

    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
                output="screen",
        arguments=["-d", os.path.join(pkg_share, "rviz", "display.rviz")],
    )

    return LaunchDescription([robot_state_publisher_node,
                              arg_sim,
                              tf_base_footprint_to_base_link,
                              tf_odom_to_base_footprint,
                              tf_odom_to_map,
                              rviz2_node])

