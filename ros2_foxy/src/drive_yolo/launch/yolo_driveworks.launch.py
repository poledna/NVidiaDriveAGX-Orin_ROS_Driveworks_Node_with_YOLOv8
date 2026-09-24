"""YOLO with DriveWorks Camera (ROS 2 port of yolo_driveworks.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')

    return LaunchDescription([
        # Arguments for easy configuration
        DeclareLaunchArgument('model_path', default_value=os.path.join(pkg_share, 'models', 'yolov8n.engine')),
        DeclareLaunchArgument('confidence_threshold', default_value='0.1'),
        DeclareLaunchArgument('detection_threshold', default_value='0.1'),
        DeclareLaunchArgument('quiet_mode', default_value='true'),
        DeclareLaunchArgument('camera_params',
                              default_value='camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed'),

        # YOLO Detection Node with DriveWorks
        Node(
            package='drive_yolo',
            executable='drive_yolo_node',
            name='drive_yolo_node',
            output='screen',
            parameters=[{
                # Model configuration
                'model_path': LaunchConfiguration('model_path'),
                # Camera configuration
                'use_driveworks_camera': True,
                'camera_params': LaunchConfiguration('camera_params'),
                'enable_hood_filtering': True,
                'ignore_hood_pixels': 290,
                'vehicle_confidence_threshold': 0.5,     # 50% for vehicles
                'pedestrian_confidence_threshold': 0.2,  # 20% for pedestrians
                # Detection thresholds
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_threshold': LaunchConfiguration('detection_threshold'),
                'quiet_mode': LaunchConfiguration('quiet_mode'),
            }],
        ),
    ])
