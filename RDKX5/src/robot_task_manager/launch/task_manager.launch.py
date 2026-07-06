#!/usr/bin/env python3
"""
启动机器人任务管理器（含导航 + 定位 + RViz + 状态机）

用法:
    # 完整启动（导航 + RViz + 状态机）
    ros2 launch robot_task_manager task_manager.launch.py use_rviz:=true robot_name:=robot-01

    # 无 RViz 模式
    ros2 launch robot_task_manager task_manager.launch.py robot_name:=robot-02
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    stm32_dir = get_package_share_directory('stm32_bridge')
    task_dir = get_package_share_directory('robot_task_manager')

    # ── 参数声明 ──
    robot_name = LaunchConfiguration('robot_name', default='robot-01')
    server_url = LaunchConfiguration('server_url', default='39.105.113.176')
    docking_station = LaunchConfiguration('docking_station', default='1F-充电站')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    # ── Nav2 参数 ──
    nav2_params = os.path.join(stm32_dir, 'config', 'nav2_params.yaml')

    ld = LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot-01',
                              description='机器人唯一名称'),
        DeclareLaunchArgument('server_url', default_value='39.105.113.176',
                              description='服务器地址'),
        DeclareLaunchArgument('docking_station', default_value='1F-充电站',
                              description='停靠/充电站'),
        DeclareLaunchArgument('use_rviz', default_value='false',
                              description='启动 RViz2'),

        # ════════════════════════════════════════════════
        # 导航 & 定位
        # ════════════════════════════════════════════════

        # ── Map Server ──
        Node(package='nav2_map_server', executable='map_server', name='map_server',
             output='screen',
             parameters=[{'yaml_filename': '/root/library_map.yaml'}]),

        # ── AMCL 定位 ──
        Node(package='nav2_amcl', executable='amcl', name='amcl',
             output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'amcl_params.yaml')]),

        # ── Nav2 Planner ──
        Node(package='nav2_planner', executable='planner_server',
             name='planner_server', output='screen',
             parameters=[nav2_params]),

        # ── Nav2 Controller (DWB) ──
        Node(package='nav2_controller', executable='controller_server',
             name='controller_server', output='screen',
             parameters=[nav2_params]),

        # ── Behavior Tree Navigator ──
        Node(package='nav2_bt_navigator', executable='bt_navigator',
             name='bt_navigator', output='screen',
             parameters=[nav2_params]),

        # ── Behavior Server ──
        Node(package='nav2_behaviors', executable='behavior_server',
             name='behavior_server', output='screen',
             parameters=[nav2_params]),

        # ── Keepout Mask Publisher（禁区蒙版 + FilterInfo 一起发布） ──
        Node(package='stm32_bridge', executable='keepout_mask_publisher',
             name='keepout_mask_publisher', output='screen'),

        # ── Lifecycle Manager ──
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager', output='screen',
             parameters=[{'node_names': ['map_server', 'amcl', 'planner_server',
                          'controller_server', 'behavior_server', 'bt_navigator'],
                          'autostart': True,
                          'bond_timeout': 10.0}]),

        # ── RF2O 激光里程计 ──
        Node(package='rf2o_laser_odometry', executable='rf2o_laser_odometry_node',
             name='rf2o_laser_odometry', output='screen',
             arguments=['--ros-args', '--log-level', 'warn'],
             parameters=[{'laser_scan_topic': '/scan', 'odom_topic': '/odom_rf2o',
                          'publish_tf': True, 'base_frame_id': 'base_footprint',
                          'odom_frame_id': 'odom_laser', 'freq': 12.0,
                          'init_pose_from_topic': ''}]),

        # ── YDLidar ──
        Node(package='ydlidar_ros2_driver', executable='ydlidar_ros2_driver_node',
             name='ydlidar_ros2_driver_node', output='screen',
             arguments=['--ros-args', '--log-level', 'error'],
             parameters=[os.path.join(stm32_dir, 'config', 'lidar_params.yaml')]),

        # ── STM32 底盘（激光里程计模式，不发布轮式 odom TF） ──
        Node(package='stm32_bridge', executable='stm32_bridge_node',
             name='stm32_bridge', output='screen',
             parameters=[os.path.join(stm32_dir, 'config', 'stm32_params.yaml'),
                        {'publish_odom_tf': False}]),

        # ── 到位后原地旋转对准 ──
        Node(package='stm32_bridge', executable='rotate_to_goal',
             name='rotate_to_goal', output='screen'),

        # ── TF 静态变换 ──
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_footprint', output='screen',
             arguments=['--x', '0', '--y', '0', '--z', '0.076',
                        '--frame-id', 'base_footprint', '--child-frame-id', 'base_link']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             name='tf_laser', output='screen',
             arguments=['--x', '0.15', '--y', '0', '--z', '0.15',
                        '--frame-id', 'base_link', '--child-frame-id', 'laser_frame']),

        # ════════════════════════════════════════════════
        # 任务状态机
        # ════════════════════════════════════════════════

        Node(
            package='robot_task_manager',
            executable='task_manager_node',
            name='task_manager_node',
            output='screen',
            parameters=[
                os.path.join(task_dir, 'config', 'robot_config.yaml'),
                {
                    'robot_name': robot_name,
                    'server_url': server_url,
                    'docking_station': docking_station,
                },
            ],
        ),

        # ════════════════════════════════════════════════
        # RViz
        # ════════════════════════════════════════════════

        Node(package='rviz2', executable='rviz2', name='rviz2', output='screen',
             arguments=['-d', os.path.join(stm32_dir, 'config', 'map_view.rviz')],
             condition=IfCondition(use_rviz)),

        LogInfo(msg=f'🤖 机器人任务管理器已启动 ({robot_name})'),
    ])

    return ld
