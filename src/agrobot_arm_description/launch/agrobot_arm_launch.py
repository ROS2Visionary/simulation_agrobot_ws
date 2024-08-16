import os
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share_dir = get_package_share_directory("agrobot_arm_description")
    urdf_model_path = os.path.join(pkg_share_dir,"urdf","agrobot.urdf")
    # urdf_model_path = os.path.join(pkg_share_dir,"urdf","panda.urdf")

    # 读取 URDF 文件内容
    with open(urdf_model_path, 'r') as urdf_file:
        urdf_content = urdf_file.read()


    joint_state_publisher_node = Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            name='joint_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
    )

    robot_state_publisher_node = Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': urdf_content}]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', os.path.join(pkg_share_dir, 'rviz', 'agrobot_arm.rviz')]
    )

    return LaunchDescription([robot_state_publisher_node,joint_state_publisher_node,rviz_node])


