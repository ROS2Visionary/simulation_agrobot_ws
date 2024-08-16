
import os
from launch.substitutions import LaunchConfiguration
import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 定义世界文件的名称和其所在的软件包目录
    world_file_name = 'small_house.world'
    world_pkg_dir = get_package_share_directory('aws_robomaker_small_house_world')
    # 获取gazebo_ros软件包的目录
    gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # 声明启动参数：世界文件路径
    arg_world = DeclareLaunchArgument(
            name='world',
            default_value=[os.path.join(world_pkg_dir, 'worlds', world_file_name), ''],
            description='SDF world file'
    )
    # 声明启动参数：是否启动GUI
    arg_gui = DeclareLaunchArgument(
            name='gui',
            default_value='True'
    )
    # 声明启动参数：是否使用仿真时间
    use_sim_time = LaunchConfiguration("use_sim_time")
    arg_sim = DeclareLaunchArgument(
            name='use_sim_time',
            default_value='True'
    )
    # 声明启动参数：加载'libgazebo_ros_state.so'状态插件
    arg_state = DeclareLaunchArgument(
            name='state',
            default_value='True',
            description='Set "true" to load "libgazebo_ros_state.so"'
    )
    
    # 定义启动Gazebo客户端的动作
    '''
    Gazebo客户端 (gazebo_client):
        作用: 启动Gazebo仿真客户端,提供用户界面(GUI)以交互和可视化仿真过程。
        重要性: 客户端使得用户可以实时查看和操作仿真环境,调整参数和监控机器人行为。
    '''
    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        # 条件：如果启动参数'gui'为真
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui'))
    )
    
    # 定义启动Gazebo服务器的动作
    '''
    Gazebo服务器 (gazebo_server):
        作用: 启动Gazebo仿真服务器,负责加载和运行仿真环境。
        重要性: Gazebo服务器是整个仿真系统的核心组件,负责物理仿真和环境建模。
    '''
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'))
    )
    
    # 定义URDF文件的名称和其所在软件包的共享目录
    urdf_name = "robot_display.urdf"
    urdf_pkg_share = get_package_share_directory('agrobot_description') 
    urdf_model_path = os.path.join(urdf_pkg_share, f'urdf/{urdf_name}')
    
    # 创建一个发布机器人状态的节点
    '''
    机器人状态发布节点 (robot_state_publisher_node):
        作用: 将机器人的运动学模型(通常使用URDF或SRDF描述)发布为ROS中的TF数据,使得其他节点可以了解机器人的姿态和关节状态。
        重要性: 在使用ROS进行机器人控制和导航时,TF数据对于正确的定位和运动规划至关重要。
    '''
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        arguments=[urdf_model_path],
        parameters=[{"use_sim_time":use_sim_time}],
    )
    
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
                output="screen",
        arguments=["-d", os.path.join(urdf_pkg_share, "rviz", "display.rviz")],
        parameters=[{"use_sim_time":use_sim_time}],
    )

    # 创建一个在Gazebo中生成机器人实体的节点
    '''
    生成机器人实体节点 (spawn_robot):
        作用: 在Gazebo仿真环境中生成一个机器人实体,基于提供的URDF描述文件。
        重要性: 通过在仿真环境中生成机器人实体,可以在仿真中模拟和测试机器人的行为和感知。
    '''
    spawn_robot = Node(
        package='gazebo_ros', 
        executable='spawn_entity.py',
        arguments=['-entity', "robot",  '-file', urdf_model_path ], 
        output='screen',
        parameters=[{"use_sim_time":use_sim_time}],
    )
    
    # 返回整个启动描述
    return LaunchDescription([
        arg_world, # 启动参数：世界文件路径
        arg_gui, # 启动参数：是否启动GUI
        arg_sim, # 启动参数：是否使用仿真时间
        arg_state, # 启动参数：加载'libgazebo_ros_state.so'状态插件
        gazebo_server, # 启动Gazebo服务器
        gazebo_client, # 启动Gazebo客户端
        robot_state_publisher_node, # 启动机器人状态发布节点
        spawn_robot, # 在Gazebo中生成机器人实体节点
        rviz2_node,
    ])
