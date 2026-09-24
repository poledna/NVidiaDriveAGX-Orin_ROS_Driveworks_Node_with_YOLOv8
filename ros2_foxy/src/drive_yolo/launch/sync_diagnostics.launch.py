"""Synchronization Diagnostics (ROS 2 port of sync_diagnostics.launch)

Measures and reports timing differences between camera detections and LiDAR
point clouds. Reports every 2 seconds: mean/median/std dev of the offset and a
recommended time_offset_ms for the fusion node, e.g.
  ros2 launch drive_yolo lidar_camera_fusion_with_sync.launch.py time_offset_ms:=45.3
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('camera_topic', default_value='/detections'),
        DeclareLaunchArgument('lidar_topic', default_value='/invz_summation_reflection'),

        Node(
            package='drive_yolo',
            executable='drive_yolo_sync_diagnostics',
            name='sync_diagnostics',
            output='screen',
            parameters=[{
                # Topics to monitor
                'camera_topic': LaunchConfiguration('camera_topic'),
                'lidar_topic': LaunchConfiguration('lidar_topic'),
                # Queue size for statistics buffer (samples)
                'queue_size': 50,
                # Report interval in seconds
                'report_interval': 2.0,
            }],
        ),
    ])
