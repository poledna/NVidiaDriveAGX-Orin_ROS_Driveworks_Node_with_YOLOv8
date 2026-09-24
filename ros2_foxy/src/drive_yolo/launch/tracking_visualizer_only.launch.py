"""Standalone rviz2 visualization of tracked objects
(ROS 2 port of tracking_visualizer_only.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

VISUALIZER_ARGS = [
    ('show_velocity_vectors', 'true'),
    ('show_track_ids', 'true'),
    ('show_positions', 'true'),
    ('show_predicted_positions', 'true'),
    ('velocity_scale', '2.0'),         # Scale velocity arrows for better visibility
    ('min_velocity_display', '0.5'),   # Only show moving objects (m/s)
    ('frame_id', 'camera'),
    ('use_class_colors', 'true'),
]


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in VISUALIZER_ARGS] + [
            DeclareLaunchArgument('launch_rviz', default_value='true'),

            Node(
                package='drive_yolo',
                executable='tracking_rviz_visualizer.py',
                name='tracking_rviz_visualizer',
                output='screen',
                parameters=[{name: LaunchConfiguration(name) for name, _ in VISUALIZER_ARGS}],
            ),

            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', os.path.join(pkg_share, 'rviz', 'tracking_visualization.rviz')],
                condition=IfCondition(LaunchConfiguration('launch_rviz')),
            ),
        ])
