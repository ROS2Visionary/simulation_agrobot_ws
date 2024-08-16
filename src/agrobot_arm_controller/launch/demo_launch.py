from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import os
import xacro

def generate_launch_description():
    """
    启动一个自包含的示例

    包括:
     * static_virtual_joint_tfs
     * robot_state_publisher
     * move_group
     * moveit_rviz
     * warehouse_db (可选)
     * ros2_control_node + 控制器加载
    """
    ld = LaunchDescription()

    moveit_config_pkg_dir = get_package_share_directory("agrobot_arm_config")
    robot_description_path = os.path.join(moveit_config_pkg_dir,"config","agrobot_arm.urdf.xacro")


    # 加载xacro文件(机器人描述文件)
    robot_description_doc = xacro.process_file(robot_description_path)
    robot_description_content = robot_description_doc.toprettyxml(indent='  ')


    # 声明启动参数
    ld.add_action(DeclareLaunchArgument("db", default_value="false", description="默认情况下，我们不启动数据库（它可能很大）"))
    ld.add_action(DeclareLaunchArgument("debug", default_value="false", description="默认情况下，我们不在调试模式"))
    ld.add_action(DeclareLaunchArgument("use_rviz", default_value="true"))

    # 根据已发布的关节状态，发布机器人的链接的 tf
    # rsp_launch = moveit_config_pkg_dir + "/launch/rsp.launch.py"
    # ld.add_action(
    #     IncludeLaunchDescription(
    #         PythonLaunchDescriptionSource(str(rsp_launch)),
    #     )
    # )
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        respawn=True,
        output="screen",
        parameters=[{
            "robot_description": robot_description_content,
            "publish_frequency": 15.0,
        }],
    )
    ld.add_action(rsp_node)


    # 启动 move_group 节点
    move_group_launch = moveit_config_pkg_dir + "/launch/move_group.launch.py"
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(move_group_launch)),
        )
    )

    # 启动 Rviz，并加载默认配置以查看 move_group 节点的状态
    moveit_rviz_launch = moveit_config_pkg_dir + "/launch/moveit_rviz.launch.py"
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(moveit_rviz_launch)),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # 如果启用了数据库加载，也启动 mongodb
    warehouse_db_launch = moveit_config_pkg_dir + "/launch/warehouse_db.launch.py"
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(warehouse_db_launch)),
            condition=IfCondition(LaunchConfiguration("db")),
        )
    )

    # 启动假关节驱动节点
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            {'robot_description':robot_description_content},
            str(moveit_config_pkg_dir + "/config/ros2_controllers.yaml"),
        ],
    )
    ld.add_action(ros2_control_node)

    # 启动控制器加载文件
    spawn_controllers_launch = moveit_config_pkg_dir + "/launch/spawn_controllers.launch.py"
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(spawn_controllers_launch)),
        )
    )

    return ld
