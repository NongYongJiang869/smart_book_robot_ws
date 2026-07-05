"""
导航 — AMCL + Nav2 路径规划

用法:
  ros2 launch stm32_bridge navigation.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stm32_dir = get_package_share_directory('stm32_bridge')
    nav2_params = os.path.join(stm32_dir, 'config', 'nav2_params.yaml')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='启动 RViz2'),

        # ── Map Server ──
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen', parameters=[{'yaml_filename': '/root/library_map.yaml'}]),

        # ── AMCL ──
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'amcl_params.yaml')]),

        # ── Planner ──
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             output='screen', parameters=[nav2_params]),

        # ── Controller (DWB) ──
        Node(package='nav2_controller', executable='controller_server', name='controller_server',
             output='screen', parameters=[nav2_params]),

        # ── Behavior Tree Navigator ──
        Node(package='nav2_bt_navigator', executable='bt_navigator', name='bt_navigator',
             output='screen', parameters=[nav2_params]),

        # ── Behavior Server ──
        Node(package='nav2_behaviors', executable='behavior_server', name='behavior_server',
             output='screen', parameters=[nav2_params]),

        # ── Lifecycle Manager ──
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'node_names': ['map_server', 'amcl', 'planner_server',
                          'controller_server', 'behavior_server', 'bt_navigator'],
                          'autostart': True,
                          'bond_timeout': 10.0}]),

        # ── RF2O ──
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom_rf2o',
                          'publish_tf': True, 'base_frame_id': 'base_footprint',
                          'odom_frame_id': 'odom_laser', 'freq': 12.0,
                          'init_pose_from_topic': ''}]),

        # ── LiDAR ──
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'lidar_params.yaml')]),

        # ── STM32 底盘 ──
        Node(package='stm32_bridge', executable='stm32_bridge_node',
             name='stm32_bridge', output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'stm32_params.yaml'),
                        {'publish_odom_tf': False}]),

        # ── 到位后原地旋转 ──
        Node(package='stm32_bridge', executable='rotate_to_goal',
             name='rotate_to_goal', output='screen'),

        # ── TF ──
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_footprint', output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0.076',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_laser', output='screen',
             arguments=['--x', '0.12', '--y', '0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),

        # ── RViz ──
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(stm32_dir, 'config', 'map_view.rviz')],
             condition=IfCondition(use_rviz)),
    ])
