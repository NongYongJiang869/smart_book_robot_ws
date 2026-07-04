"""
激光里程计建图 — 使用 rf2o 激光里程计替代轮式里程计

用法:
  手持建图 (不启动底盘):
    ros2 launch stm32_bridge slam_laser_odom_bringup.launch.py

  底盘驱动建图:
    ros2 launch stm32_bridge slam_laser_odom_bringup.launch.py use_chassis:=true

  启用 RViz2 (需要 X11 DISPLAY):
    ros2 launch stm32_bridge slam_laser_odom_bringup.launch.py use_rviz:=true

键盘遥控请在新终端手动运行:
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.6
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
    slam_dir  = get_package_share_directory('slam_gmapping')

    stm32_params = os.path.join(stm32_dir, 'config', 'stm32_params.yaml')
    lidar_params = os.path.join(stm32_dir, 'config', 'lidar_params.yaml')
    ekf_params   = os.path.join(stm32_dir, 'config', 'ekf_params.yaml')
    slam_params  = os.path.join(slam_dir, 'params', 'slam_gmapping_laser_odom.yaml')

    use_chassis = LaunchConfiguration('use_chassis', default='false')
    use_rviz = LaunchConfiguration('use_rviz', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_chassis',
            default_value='false',
            description='是否启动 STM32 底盘'),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='是否启动 RViz2 (需要 X11 显示环境)'),

        # ── 提示键盘遥控 ──
        LogInfo(msg='💡 建图请在【新终端】中运行键盘控制:'),
        LogInfo(msg='   ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.05 -p turn:=0.5'),
        LogInfo(msg='   按键: i=前进 j=左转 l=右转 k=停止 q/z=加减速 w/x=加减转向'),

        # ── YDLidar 驱动 (/scan) ──
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[lidar_params],
        ),

        # ── RF2O 激光里程计 (/odom_rf2o, 不发布 TF 交给 EKF) ──
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': False,          # ← 关闭, 由 EKF 接管 TF
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_laser',
                'init_pose_from_topic': '',
                'freq': 15.0,             # ≥ LiDAR 12Hz, 确保不丢扫描帧
            }],
        ),

        # ── EKF 融合 (rf2o + 陀螺 → /odom_fused + TF) ──
        Node(
            package='robot_localization',
            executable='ekf_node',
            name='ekf_filter_node',
            output='screen',
            parameters=[ekf_params],
        ),

        # ── slam_gmapping (map→odom TF + /map) ──
        Node(
            package='slam_gmapping',
            executable='slam_gmapping',
            name='slam_gmapping',
            output='screen',
            parameters=[slam_params],
            remappings=[
                ('/odom', '/odom_rf2o'),
            ],
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

        # ── STM32 底盘桥接 (可选, 不发TF避免与rf2o冲突) ──
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[stm32_params, {'publish_odom_tf': False}],
            condition=IfCondition(use_chassis),
        ),

        # ── RViz2 (可选, 需要 X11 DISPLAY) ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(stm32_dir, 'config', 'map_view.rviz')],
            condition=IfCondition(use_rviz),
        ),
    ])
