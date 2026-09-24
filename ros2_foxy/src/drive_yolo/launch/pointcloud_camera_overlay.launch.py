"""Pointcloud Camera Overlay (ROS 2 port of pointcloud_camera_overlay.launch)

Starts the vehicle TFs and the camera info publisher needed to overlay the
LiDAR point cloud on the camera image in rviz2 (rviz/pointcloud_overlay.rviz).
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        DeclareLaunchArgument('launch_rviz', default_value='false'),

        # TF transformations for all sensors
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'M8VIL_TFs.launch.py'))),

        # Camera Info Publisher
        Node(
            package='drive_yolo',
            executable='drive_yolo_camera_info_publisher',
            name='camera_info_publisher',
            output='screen',
            parameters=[{
                'camera_frame': 'camera_entron',
                'image_width': 3848,
                'image_height': 2168,
                'publish_rate': 30.0,
                # Camera matrix (3x3 row-major: fx, 0, cx, 0, fy, cy, 0, 0, 1)
                'camera_matrix': [3145.24132, 0.0, 1887.68485, 0.0, 3166.08502, 990.62304, 0.0, 0.0, 1.0],
                # Distortion coefficients (k1, k2, p1, p2, k3)
                'distortion_coefficients': [-0.298671, 0.087807, 0.000383, 0.000551, 0.000000],
            }],
        ),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'rviz', 'pointcloud_overlay.rviz')],
            condition=IfCondition(LaunchConfiguration('launch_rviz')),
        ),
    ])
