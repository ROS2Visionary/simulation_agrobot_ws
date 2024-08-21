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

    # 定义地图服务器节点，用于提供已知地图的服务
    nav2_map_server_node = Node(
        package="nav2_map_server",  # 节点所在的功能包
        executable="map_server",  # 要执行的可执行文件名
        name="nav2_map_server_node",  # 节点的名称
        output="screen",  # 将节点的输出打印到屏幕
        parameters=[
            {"yaml_filename": os.path.join(get_package_share_directory("agrobot_mapping"), "maps", "laboratory.yaml")},  # 地图文件的路径
        ]
    )

    rviz2_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_node"
    )

    static_tf_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([get_package_share_directory("agrobot_localization"),"/launch","/static_tf_launch.py"]),
    )

    # 定义需要管理生命周期的节点列表
    lifecycle_nodes = ["nav2_map_server_node"]
    # 定义生命周期管理器节点，用于管理其他节点的生命周期
    nav2_lifecycle_manager_node = Node(
        package="nav2_lifecycle_manager",  # 节点所在的功能包
        executable="lifecycle_manager",  # 要执行的可执行文件名
        name="lifecycle_manager",  # 节点的名称
        parameters=[
            {"node_names": lifecycle_nodes},  # 需要管理的节点列表
            {"autostart": True}  # 自动启动管理的节点
        ],
    )

    nav_launch =  IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                [get_package_share_directory('nav2_bringup'), '/launch', '/navigation_launch.py']),
            # 使用 Launch 参数替换原有参数
            launch_arguments={
                'params_file': os.path.join(get_package_share_directory("agrobot_navigation"),"config","navigation_params.yaml"),
                }.items(),
    )

    return LaunchDescription([
        rviz2_node,
        static_tf_node,
        nav2_map_server_node,
        nav_launch,
        nav2_lifecycle_manager_node,
    ])