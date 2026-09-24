"""3D Tracking Pipeline (ROS 2 port of 3d_tracking_pipeline.launch)
Assumes YOLO and fusion are already running; starts 3D tracking + speed monitor."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

TRACKING_ARGS = [
    ('distance_threshold_3d', '10.0'),
    ('max_missed_frames', '30'),
    ('min_detections_for_tracking', '1'),
    ('min_lidar_points', '5'),
    ('min_speed_threshold', '0.1'),     # 0.1 m/s minimum to be considered "moving"
    ('max_tracking_distance', '250.0'),
    ('quiet_mode', 'false'),
    ('publish_all_tracks', 'true'),
    ('publish_high_quality_only', 'false'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in TRACKING_ARGS] + [
            # Speed Monitor Arguments
            DeclareLaunchArgument('enable_speed_monitor', default_value='true'),
            DeclareLaunchArgument('speed_limit_ms', default_value='15.0'),
            DeclareLaunchArgument('warn_threshold_ms', default_value='11.11'),  # 40 km/h
            DeclareLaunchArgument('monitor_classes', default_value='0,1,2,3,5,7'),
            DeclareLaunchArgument('use_rviz', default_value='false'),

            # 3D Object Tracking Node
            Node(
                package='drive_yolo',
                executable='drive_yolo_tracking_3d_node',
                name='tracking_3d_node',
                output='screen',
                parameters=[{name: LaunchConfiguration(name) for name, _ in TRACKING_ARGS}],
            ),

            # Speed Monitor Node
            Node(
                package='drive_yolo',
                executable='drive_yolo_speed_monitor_node',
                name='speed_monitor_node',
                output='screen',
                parameters=[{
                    'speed_limit_ms': LaunchConfiguration('speed_limit_ms'),
                    'warn_threshold_ms': LaunchConfiguration('warn_threshold_ms'),
                    'monitor_classes': LaunchConfiguration('monitor_classes'),
                    'monitor_all_classes': False,
                    'quiet_mode': LaunchConfiguration('quiet_mode'),
                }],
                condition=IfCondition(LaunchConfiguration('enable_speed_monitor')),
            ),

            # Optional: RViz for 3D visualization
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'tracking_visualization.rviz')],
                condition=IfCondition(LaunchConfiguration('use_rviz')),
            ),
        ])
