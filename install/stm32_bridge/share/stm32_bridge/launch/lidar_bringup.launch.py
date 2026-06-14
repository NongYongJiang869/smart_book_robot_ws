"""
LiDAR + 底盘 启动

用法:
  ros2 launch stm32_bridge lidar_bringup.launch.py
  ros2 launch stm32_bridge lidar_bringup.launch.py lidar_model:=X4
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    lidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    stm32_dir = get_package_share_directory('stm32_bridge')

    stm32_params = os.path.join(stm32_dir, 'config', 'stm32_params.yaml')
    lidar_params = os.path.join(stm32_dir, 'config', 'lidar_params.yaml')

    return LaunchDescription([
        DeclareLaunchArgument(
            'lidar_params',
            default_value=lidar_params,
            description='LiDAR 参数文件路径'),

        # ── STM32 底盘桥接 ──
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[stm32_params],
        ),

        # ── YDLidar 驱动 ──
        # 注意: 用普通 Node 而非 LifecycleNode,
        # 驱动内部在 on_configure 回调中自动完成初始化和激活
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            emulate_tty=True,
            parameters=[LaunchConfiguration('lidar_params')],
        ),

        # ── TF: base_link → laser_frame ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['--x', '0.12', '--y', '0', '--z', '0.15',
                       '--frame-id', 'base_link', '--child-frame-id', 'laser_frame'],
        ),
    ])
