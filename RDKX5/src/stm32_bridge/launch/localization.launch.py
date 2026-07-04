"""
AMCL 定位启动 — 基于已有地图 + 激光里程计

用法:
  ros2 launch stm32_bridge localization.launch.py use_rviz:=true
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stm32_dir = get_package_share_directory('stm32_bridge')

    map_file = LaunchConfiguration('map_file', default='/root/library_map.yaml')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='/root/library_map.yaml',
                              description='地图 yaml 文件路径'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='是否启动 RViz2'),

        LogInfo(msg='📍 AMCL 定位启动'),
        LogInfo(msg=f'   地图: /root/library_map.yaml'),

        # ── Map Server ──
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': '/root/library_map.yaml',
                          'frame_id': 'map'}]),

        # ── AMCL ──
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'amcl_params.yaml')],
             remappings=[('/scan', '/scan')]),

        # ── 自动激活 lifecycle 节点 ──
        ExecuteProcess(
            cmd=['bash', '-c',
                 'source /opt/ros/humble/setup.bash; sleep 8; '
                 'ros2 lifecycle set /map_server configure; sleep 1; '
                 'ros2 lifecycle set /map_server activate; sleep 1; '
                 'ros2 lifecycle set /amcl configure; sleep 1; '
                 'ros2 lifecycle set /amcl activate; '
                 'echo "Lifecycle done"'],
            output='screen'),

        # ── RF2O 激光里程计 ──
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom_rf2o',
                          'publish_tf': True, 'base_frame_id': 'base_footprint',
                          'odom_frame_id': 'odom_laser', 'freq': 12.0,
                          'init_pose_from_topic': ''}]),

        # ── YDLidar 驱动 ──
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'lidar_params.yaml')]),

        # ── STM32 底盘桥接 (接收 /cmd_vel, 不发TF) ──
        Node(package='stm32_bridge', executable='stm32_bridge_node',
             name='stm32_bridge', output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'stm32_params.yaml'),
                        {'publish_odom_tf': False}]),

        # ── TF: base_footprint → base_link ──
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_footprint', output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0.076',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']),

        # ── TF: base_link → laser_frame ──
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='static_tf_laser', output='screen',
             arguments=['--x', '0.12', '--y', '0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),

        # ── RViz2 (可选) ──
        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(stm32_dir, 'config', 'map_view.rviz')],
             condition=IfCondition(use_rviz)),
    ])
