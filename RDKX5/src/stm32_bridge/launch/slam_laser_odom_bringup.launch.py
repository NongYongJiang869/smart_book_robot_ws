"""
激光里程计建图 — 使用 rf2o 激光里程计替代轮式里程计

用法:
  手持建图 (不启动底盘):
    ros2 launch stm32_bridge slam_laser_odom_bringup.launch.py

  底盘驱动建图 (可用键盘遥控):
    ros2 launch stm32_bridge slam_laser_odom_bringup.launch.py use_chassis:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    stm32_dir = get_package_share_directory('stm32_bridge')
    slam_dir  = get_package_share_directory('slam_gmapping')

    stm32_params = os.path.join(stm32_dir, 'config', 'stm32_params.yaml')
    lidar_params = os.path.join(stm32_dir, 'config', 'lidar_params.yaml')
    slam_params  = os.path.join(slam_dir, 'params', 'slam_gmapping_laser_odom.yaml')

    use_chassis = LaunchConfiguration('use_chassis', default='false')

    return LaunchDescription([
        DeclareLaunchArgument(
            'use_chassis',
            default_value='false',
            description='是否启动 STM32 底盘 (用于键盘遥控建图)'),

        # ── YDLidar 驱动 (/scan) ──
        Node(
            package='ydlidar_ros2_driver',
            executable='ydlidar_ros2_driver_node',
            name='ydlidar_ros2_driver_node',
            output='screen',
            parameters=[lidar_params],
        ),

        # ── RF2O 激光里程计 (/odom_rf2o + odom→base_footprint TF) ──
        Node(
            package='rf2o_laser_odometry',
            executable='rf2o_laser_odometry_node',
            name='rf2o_laser_odometry',
            output='screen',
            parameters=[{
                'laser_scan_topic': '/scan',
                'odom_topic': '/odom_rf2o',
                'publish_tf': True,
                'base_frame_id': 'base_footprint',
                'odom_frame_id': 'odom_laser',  # 独立帧, 不与stm32的odom冲突
                'init_pose_from_topic': '',
                'freq': 10.0,
            }],
        ),

        # ── slam_gmapping (map→odom TF + /map)
        #     从 /odom_rf2o 获取里程计, 避免与 STM32 的 /odom 冲突 ──
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

        # ── STM32 底盘桥接 (可选, 用于键盘遥控) ──
        Node(
            package='stm32_bridge',
            executable='stm32_bridge_node',
            name='stm32_bridge',
            output='screen',
            parameters=[stm32_params],
            condition=IfCondition(use_chassis),
        ),

        # ── 键盘遥控 (可选, 需要底盘) ──
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='teleop_keyboard',
            output='screen',
            prefix='xterm -e',
            parameters=[{
                'speed': 0.05,      # 初始线速度 0.05 m/s
                'turn': 0.6,        # 初始角速度 0.6 rad/s
            }],
            condition=IfCondition(use_chassis),
        ),

        # ── RViz2 ──
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', os.path.join(stm32_dir, 'config', 'map_view.rviz')],
        ),
    ])
