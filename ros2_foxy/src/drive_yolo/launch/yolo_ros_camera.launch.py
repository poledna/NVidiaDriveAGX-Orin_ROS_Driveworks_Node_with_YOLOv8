"""YOLO with a ROS image topic as input (ROS 2 port of yolo_ros_camera.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.path.join(pkg_share, 'models', 'yolo11n.engine')),
        DeclareLaunchArgument('confidence_threshold', default_value='0.3'),
        DeclareLaunchArgument('detection_threshold', default_value='0.3'),
        DeclareLaunchArgument('quiet_mode', default_value='true'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_input'),

        # YOLO Detection Node with ROS camera
        Node(
            package='drive_yolo',
            executable='drive_yolo_node',
            name='drive_yolo_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'use_driveworks_camera': False,
                'image_topic': LaunchConfiguration('image_topic'),
                'enable_hood_filtering': True,
                'ignore_hood_pixels': 290,
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_threshold': LaunchConfiguration('detection_threshold'),
                'quiet_mode': LaunchConfiguration('quiet_mode'),
            }],
        ),
    ])
