"""
启动 STM32 串口桥接节点

用法:
  ros2 launch stm32_bridge stm32_bridge.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('stm32_bridge')
    default_params = os.path.join(pkg_dir, 'config', 'stm32_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='STM32 bridge 参数文件路径'),

        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),
    ])
