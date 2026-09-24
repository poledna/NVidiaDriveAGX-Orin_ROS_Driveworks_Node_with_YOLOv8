"""LiDAR-Camera Fusion with Synchronization Parameters
(ROS 2 port of lidar_camera_fusion_with_sync.launch)

USAGE:
1. First run sync_diagnostics.launch.py to measure the timing offset
2. Pass the measured offset: time_offset_ms:=<value>
3. Adjust sync_time_slop and sync_queue_size as needed
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        # Time tolerance for matching messages (seconds)
        DeclareLaunchArgument('sync_time_slop', default_value='0.15'),
        # Queue size for message buffering - increase if messages get dropped
        DeclareLaunchArgument('sync_queue_size', default_value='30'),
        # Manual time offset between camera and LiDAR (ms), positive = camera ahead
        DeclareLaunchArgument('time_offset_ms', default_value='0.0'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'lidar_camera_fusion.launch.py')),
            launch_arguments={
                'sync_time_slop': LaunchConfiguration('sync_time_slop'),
                'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                'time_offset_ms': LaunchConfiguration('time_offset_ms'),
            }.items(),
        ),
    ])
