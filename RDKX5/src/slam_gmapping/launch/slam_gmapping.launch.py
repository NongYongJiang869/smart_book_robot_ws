"""
slam_gmapping 独立启动 — 需要外部提供 /scan 和 /odom

用法:
  ros2 launch slam_gmapping slam_gmapping.launch.py
  ros2 launch slam_gmapping slam_gmapping.launch.py use_rviz:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    slam_dir = get_package_share_directory('slam_gmapping')
    slam_params = os.path.join(slam_dir, 'params', 'slam_gmapping.yaml')

    use_rviz = LaunchConfiguration('use_rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='是否启动 RViz2 (需要 X11 显示环境)'),

        # ── slam_gmapping ──
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[slam_params],
        ),

        # ── TF: base_footprint → base_link ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_footprint',
            output='screen',
            arguments=['--x', '0', '--y', '0', '--z', '0.076',
                       '--frame-id', 'base_footprint',
                       '--child-frame-id', 'base_link'],
        ),

        # ── TF: base_link → laser_frame ──
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_laser',
            output='screen',
            arguments=['--x', '0.12', '--y', '0', '--z', '0.15',
                       '--frame-id', 'base_link',
                       '--child-frame-id', 'laser_frame'],
        ),

        # ── RViz2 (可选, 需要 X11 DISPLAY) ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            condition=IfCondition(use_rviz),
        ),
    ])
