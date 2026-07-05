#!/usr/bin/env python3
"""
启动机器人任务管理器

用法:
    # 基础启动（仅 stm32_bridge + task_manager）
    ros2 launch robot_task_manager task_manager.launch.py

    # 指定机器人名称
    ros2 launch robot_task_manager task_manager.launch.py robot_name:=robot-02

    # 完整导航模式（含 Nav2）
    ros2 launch robot_task_manager task_manager.launch.py use_nav:=true map:=/path/to/map.yaml
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('robot_task_manager')
    config_dir = os.path.join(pkg_share, 'config')

    # ── 参数声明 ──
    robot_name = LaunchConfiguration('robot_name', default='robot-01')
    server_url = LaunchConfiguration('server_url', default='39.105.113.176')
    docking_station = LaunchConfiguration('docking_station', default='1F-充电站')
    use_nav = LaunchConfiguration('use_nav', default='false')
    map_file = LaunchConfiguration('map', default='')

    # ── task_manager 节点 ──
    task_manager_node = Node(
        package='robot_task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        parameters=[
            os.path.join(config_dir, 'robot_config.yaml'),
            {
                'robot_name': robot_name,
                'server_url': server_url,
                'docking_station': docking_station,
            },
        ],
    )

    ld = LaunchDescription([
        DeclareLaunchArgument('robot_name', default_value='robot-01',
                              description='机器人唯一名称'),
        DeclareLaunchArgument('server_url', default_value='39.105.113.176',
                              description='服务器地址 (不含协议)'),
        DeclareLaunchArgument('docking_station', default_value='1F-充电站',
                              description='停靠/充电站名称'),
        DeclareLaunchArgument('use_nav', default_value='false',
                              description='是否启动 Nav2 导航'),
        DeclareLaunchArgument('map', default_value='',
                              description='地图文件路径 (use_nav=true 时需要)'),

        # 始终启动 stm32_bridge（底盘通信）
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('stm32_bridge'),
                    'config', 'stm32_params.yaml'
                ),
            ],
        ),

        task_manager_node,

        LogInfo(msg=f'🤖 机器人任务管理器已启动 ({robot_name})'),
    ])

    return ld
