"""Pointcloud Delay Demonstration (ROS 2 port of pointcloud_delay_demo.launch)

Adds artificial delay to point clouds to demonstrate/test synchronization.
Workflow:
  1. ros2 launch drive_yolo pointcloud_delay_demo.launch.py delay_ms:=50.0
  2. Run fusion on the delayed cloud:
       ros2 launch drive_yolo lidar_camera_fusion.launch.py lidar_topic:=/invz_summation_reflection_delayed
  3. ros2 launch drive_yolo sync_diagnostics.launch.py lidar_topic:=/invz_summation_reflection_delayed
  4. Monitor the queue: ros2 topic echo /pointcloud_delay/queue_size
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/invz_summation_reflection'),
        DeclareLaunchArgument('output_topic', default_value='/invz_summation_reflection_delayed'),
        # Option 1: time based delay (used when frame_delay is 0)
        DeclareLaunchArgument('delay_ms', default_value='50.0'),
        # Option 2: frame based delay (at 30 Hz: 1 frame = 33ms)
        DeclareLaunchArgument('frame_delay', default_value='0'),
        # false = shift timestamps by the delay, true = keep original timestamps
        DeclareLaunchArgument('preserve_timestamps', default_value='false'),
        # Passthrough mode (no delay)
        DeclareLaunchArgument('passthrough', default_value='false'),

        Node(
            package='drive_yolo',
            executable='drive_yolo_pointcloud_delay',
            name='pointcloud_delay',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': LaunchConfiguration('output_topic'),
                'delay_ms': LaunchConfiguration('delay_ms'),
                'frame_delay': LaunchConfiguration('frame_delay'),
                'queue_size': 100,
                'preserve_timestamps': LaunchConfiguration('preserve_timestamps'),
                'passthrough': LaunchConfiguration('passthrough'),
                'report_interval': 5.0,
            }],
        ),
    ])
