"""LiDAR-Camera Fusion (ROS 2 port of lidar_camera_fusion.launch)

Parameters live in config/lidar_camera_fusion.yaml. The synchronization
parameters can be overridden from the command line, e.g.
  ros2 launch drive_yolo lidar_camera_fusion.launch.py sync_time_slop:=0.15 time_offset_ms:=45.3
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        DeclareLaunchArgument('params_file',
                              default_value=os.path.join(pkg_share, 'config', 'lidar_camera_fusion.yaml')),
        DeclareLaunchArgument('lidar_topic', default_value='/invz_summation_reflection'),
        # Synchronization (floats need the decimal point)
        DeclareLaunchArgument('sync_time_slop', default_value='0.1'),
        DeclareLaunchArgument('sync_queue_size', default_value='20'),
        DeclareLaunchArgument('time_offset_ms', default_value='0.0'),

        Node(
            package='drive_yolo',
            executable='drive_yolo_lidar_fusion_node',
            name='lidar_camera_fusion',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'sync_time_slop': LaunchConfiguration('sync_time_slop'),
                    'sync_queue_size': LaunchConfiguration('sync_queue_size'),
                    'time_offset_ms': LaunchConfiguration('time_offset_ms'),
                },
            ],
            # Remap LiDAR topic
            remappings=[('/lidar/points', LaunchConfiguration('lidar_topic'))],
        ),
    ])
