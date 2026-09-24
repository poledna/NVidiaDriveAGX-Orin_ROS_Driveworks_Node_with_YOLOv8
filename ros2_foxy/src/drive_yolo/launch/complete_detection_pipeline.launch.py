"""Complete Detection Pipeline: YOLO + LiDAR fusion + 3D tracking (+ speed monitor)
(ROS 2 port of complete_detection_pipeline.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('drive_yolo')
    quiet_mode = LaunchConfiguration('quiet_mode')

    return LaunchDescription([
        DeclareLaunchArgument('model_path', default_value=os.path.join(pkg_share, 'models', 'yolov8n.engine')),
        DeclareLaunchArgument('use_driveworks', default_value='true'),
        DeclareLaunchArgument('confidence_threshold', default_value='0.3'),
        DeclareLaunchArgument('detection_threshold', default_value='0.3'),
        DeclareLaunchArgument('quiet_mode', default_value='true'),
        DeclareLaunchArgument('camera_params',
                              default_value='camera-name=F008A120RM0AV2,interface=csi-ab,link=1,output-format=processed'),
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('lidar_topic', default_value='/invz_summation_reflection'),
        DeclareLaunchArgument('fusion_params_file',
                              default_value=os.path.join(pkg_share, 'config', 'lidar_camera_fusion.yaml')),
        # 3D tracking
        DeclareLaunchArgument('distance_threshold_3d', default_value='2.0'),   # 2 meters for 3D association
        DeclareLaunchArgument('max_missed_frames', default_value='8'),
        DeclareLaunchArgument('min_detections_for_tracking', default_value='3'),
        DeclareLaunchArgument('min_lidar_points', default_value='5'),
        DeclareLaunchArgument('min_speed_threshold', default_value='0.1'),     # 0.1 m/s = "moving"
        DeclareLaunchArgument('max_tracking_distance', default_value='30.0'),  # Track objects up to 30m
        DeclareLaunchArgument('publish_all_tracks', default_value='true'),
        DeclareLaunchArgument('publish_high_quality_only', default_value='false'),
        # Speed monitor
        DeclareLaunchArgument('enable_speed_monitor', default_value='false'),
        DeclareLaunchArgument('speed_limit_ms', default_value='13.89'),        # 50 km/h
        DeclareLaunchArgument('warn_threshold_ms', default_value='11.11'),     # 40 km/h
        DeclareLaunchArgument('monitor_classes', default_value='0,1,2,3,5,7'),

        # YOLO Detection Node (DriveWorks camera or ROS image topic)
        Node(
            package='drive_yolo',
            executable='drive_yolo_node',
            name='drive_yolo_node',
            output='screen',
            parameters=[{
                'model_path': LaunchConfiguration('model_path'),
                'use_driveworks_camera': LaunchConfiguration('use_driveworks'),
                'camera_params': LaunchConfiguration('camera_params'),
                'image_topic': LaunchConfiguration('image_topic'),
                'confidence_threshold': LaunchConfiguration('confidence_threshold'),
                'detection_threshold': LaunchConfiguration('detection_threshold'),
                'quiet_mode': quiet_mode,
            }],
        ),

        # LiDAR-Camera Fusion Node. The ROS1 pipeline used different filtering
        # settings than lidar_camera_fusion.launch - they override the yaml here.
        Node(
            package='drive_yolo',
            executable='drive_yolo_lidar_fusion_node',
            name='lidar_camera_fusion',
            output='screen',
            parameters=[
                LaunchConfiguration('fusion_params_file'),
                {
                    'min_distance': 10.0,
                    'enable_point_clustering': False,
                    'max_point_distance_from_center': 3.0,
                    'min_points_for_object': 8,
                    'max_object_height': 4.0,
                    'max_object_width': 3.0,
                    'outlier_distance_threshold': 10.0,
                },
            ],
            remappings=[('/lidar/points', LaunchConfiguration('lidar_topic'))],
        ),

        # 3D Object Tracking Node
        Node(
            package='drive_yolo',
            executable='drive_yolo_tracking_3d_node',
            name='tracking_3d_node',
            output='screen',
            parameters=[{
                'distance_threshold_3d': LaunchConfiguration('distance_threshold_3d'),
                'max_missed_frames': LaunchConfiguration('max_missed_frames'),
                'min_detections_for_tracking': LaunchConfiguration('min_detections_for_tracking'),
                'min_lidar_points': LaunchConfiguration('min_lidar_points'),
                'min_speed_threshold': LaunchConfiguration('min_speed_threshold'),
                'max_tracking_distance': LaunchConfiguration('max_tracking_distance'),
                'quiet_mode': quiet_mode,
                'publish_all_tracks': LaunchConfiguration('publish_all_tracks'),
                'publish_high_quality_only': LaunchConfiguration('publish_high_quality_only'),
            }],
        ),

        # Optional: Speed Monitor Node
        Node(
            package='drive_yolo',
            executable='drive_yolo_speed_monitor_node',
            name='speed_monitor_node',
            output='screen',
            parameters=[{
                'speed_limit_ms': LaunchConfiguration('speed_limit_ms'),
                'warn_threshold_ms': LaunchConfiguration('warn_threshold_ms'),
                'monitor_classes': LaunchConfiguration('monitor_classes'),
                'monitor_all_classes': False,
                'quiet_mode': quiet_mode,
            }],
            condition=IfCondition(LaunchConfiguration('enable_speed_monitor')),
        ),
    ])
