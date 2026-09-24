"""Enhanced 3D Tracking with Low-Pass Filtering (Python tracker)
(ROS 2 port of motpy_tracking.launch)"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# (name, default) - floats keep their decimal point, ROS 2 parameters are typed
ARGS = [
    # 3D Tracking parameters
    ('distance_threshold_3d', '10.0'),
    ('max_missed_frames', '8'),
    ('min_detections_for_tracking', '1'),
    ('min_lidar_points', '0'),
    ('min_speed_threshold', '0.1'),
    ('max_tracking_distance', '200.0'),
    # Velocity filtering parameters
    ('velocity_filter_alpha', '0.05'),        # Lower = more smoothing (0.1-0.5 recommended)
    ('acceleration_filter_alpha', '0.05'),    # Acceleration filter
    ('max_reasonable_velocity', '30.0'),      # Max velocity in m/s (108 km/h)
    ('max_reasonable_acceleration', '5.0'),   # Max acceleration in m/s^2
    # System configuration
    ('quiet_mode', 'false'),
    ('publish_all_tracks', 'true'),
    ('publish_high_quality_only', 'false'),
    # RViz visualization parameters
    ('enable_rviz_visualization', 'true'),
    ('show_velocity_vectors', 'true'),        # Show velocity arrows
    ('show_bounding_boxes', 'true'),          # Show 3D bounding boxes
    ('show_track_ids', 'true'),               # Show track ID labels
    ('show_raw_vs_filtered', 'true'),         # Compare raw vs filtered velocity
]


def generate_launch_description():
    params = {name: LaunchConfiguration(name) for name, _ in ARGS}
    params.update({
        'publish_rate': 10.0,                  # 10 Hz publishing
        'enable_continuous_publishing': True,  # Enable heartbeat
    })

    return LaunchDescription(
        [DeclareLaunchArgument(name, default_value=default) for name, default in ARGS] + [
            DeclareLaunchArgument('use_python_tracker', default_value='true'),

            # Enhanced Python Tracking Node
            Node(
                package='drive_yolo',
                executable='tracking_3d_node.py',
                name='tracking_3d_python_node',
                output='screen',
                parameters=[params],
                condition=IfCondition(LaunchConfiguration('use_python_tracker')),
            ),
        ])
