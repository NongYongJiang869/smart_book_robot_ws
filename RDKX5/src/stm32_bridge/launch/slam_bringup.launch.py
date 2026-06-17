"""
SLAM 建图启动 — LiDAR + 底盘 + gmapping

用法:
  ros2 launch stm32_bridge slam_bringup.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stm32_dir = get_package_share_directory('stm32_bridge')
    lidar_dir = get_package_share_directory('ydlidar_ros2_driver')
    slam_dir  = get_package_share_directory('slam_gmapping')

    stm32_params = os.path.join(stm32_dir, 'config', 'stm32_params.yaml')
    lidar_params = os.path.join(stm32_dir, 'config', 'lidar_params.yaml')
    slam_params  = os.path.join(slam_dir, 'params', 'slam_gmapping.yaml')

    return LaunchDescription([
        # ── STM32 底盘桥接 (odometry) ──
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[stm32_params],
        ),

        # ── YDLidar 驱动 (/scan) ──
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[lidar_params],
        ),

        # ── slam_gmapping (map→odom TF + /map) ──
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[slam_params],
        ),

        # ── TF: base_footprint → base_link (slam_gmapping 需要) ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_footprint',
            arguments=['--x', '0', '--y', '0', '--z', '0.076',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'base_link'],
        ),

        # ── TF: base_link → laser_frame ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            arguments=['--x', '0.12', '--y', '0', '--z', '0.15',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'laser_frame'],
        ),

        # ── RViz2 (X11 转发到 MobaXterm) ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
        ),

        # ── 键盘遥控 (xterm 窗口) ──
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{'speed': 0.2, 'turn': 0.5}],
        ),
    ])
