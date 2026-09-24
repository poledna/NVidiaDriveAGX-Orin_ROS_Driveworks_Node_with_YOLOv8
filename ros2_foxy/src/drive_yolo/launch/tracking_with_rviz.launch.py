"""Python 3D tracker + rviz2 visualization (ROS 2 port of tracking_with_rviz.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('drive_yolo'), 'launch')

    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        # The existing motpy tracking launch
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'motpy_tracking.launch.py')),
            launch_arguments={
                'enable_rviz_visualization': 'true',
                'show_velocity_vectors': 'true',
            }.items(),
        ),

        # rviz2 marker publisher for tracked objects (+ optional rviz2)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(launch_dir, 'tracking_visualizer_only.launch.py')),
            launch_arguments={
                'launch_rviz': LaunchConfiguration('launch_rviz'),
            }.items(),
        ),
    ])
