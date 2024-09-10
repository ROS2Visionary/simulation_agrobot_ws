# custom_rtabmap.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml

def load_yaml(package_name, file_path):
    package_share_directory = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_share_directory, file_path)
    with open(absolute_file_path, 'r') as file:
        return yaml.safe_load(file)

def generate_launch_description():
    # 读取 YAML 文件路径
    param_file_path = os.path.join(
        get_package_share_directory('agrobot_vslam'),  # 替换为你的package名
        'config',
        'rtab_param.yaml'
    )
    
    # 加载 YAML 参数
    params = load_yaml('agrobot_vslam', 'config/rtab_param.yaml') 

    # 创建 LaunchConfiguration 以传递参数
    launch_args = []
    for param_name, param_value in params.items():
        launch_args.append(DeclareLaunchArgument(param_name, default_value=str(param_value)))

    # 使用参数调用 rtabmap.launch.py
    rtabmap_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('rtabmap_launch'),
            'launch',
            'rtabmap.launch.py'
        )]),
        launch_arguments={key: str(value) for key, value in params.items()}.items(),
    )

    return LaunchDescription(launch_args + [rtabmap_launch])
