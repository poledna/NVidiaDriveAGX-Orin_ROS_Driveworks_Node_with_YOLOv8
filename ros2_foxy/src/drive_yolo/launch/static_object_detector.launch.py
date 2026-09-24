"""Static Object Detector (ROS 2 port of static_object_detector.launch)
Detects static objects at known world coordinates using LiDAR.
Objects are defined in config/static_objects.yaml."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        DeclareLaunchArgument('world_frame', default_value='map'),
        DeclareLaunchArgument('lidar_frame', default_value='lidar'),
        DeclareLaunchArgument('min_points_threshold', default_value='5'),
        DeclareLaunchArgument('objects_file',
                              default_value=os.path.join(pkg_share, 'config', 'static_objects.yaml')),

        Node(
            package='drive_yolo',
            executable='drive_yolo_static_object_detector',
            name='static_object_detector',
            output='screen',
            parameters=[
                LaunchConfiguration('objects_file'),
                {
                    'world_frame': LaunchConfiguration('world_frame'),
                    'lidar_frame': LaunchConfiguration('lidar_frame'),
                    'min_points_threshold': LaunchConfiguration('min_points_threshold'),
                    'detection_timeout': 2.0,
                },
            ],
        ),
    ])
