"""
底盘启动 — 启动 STM32 桥接 + 键盘遥控 (可选)

用法:
  ros2 launch stm32_bridge chassis_bringup.launch.py
  ros2 launch stm32_bridge chassis_bringup.launch.py teleop:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_dir = get_package_share_directory('stm32_bridge')
    default_params = os.path.join(pkg_dir, 'config', 'stm32_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='STM32 bridge 参数文件'),

        DeclareLaunchArgument(
            'teleop',
            default_value='false',
            description='是否同时启动键盘遥控节点'),

        # ── STM32 串口桥接 ──
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[LaunchConfiguration('params_file')],
        ),

        # ── 键盘遥控 (可选) ──
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            condition=IfCondition(LaunchConfiguration('teleop')),
            remappings=[('/cmd_vel', '/cmd_vel')],
            parameters=[{'speed': 0.2, 'turn': 0.5}],
        ),
    ])
