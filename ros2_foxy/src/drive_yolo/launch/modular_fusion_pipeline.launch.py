"""Modular LiDAR-Camera Fusion Pipeline (ROS 2 port of modular_fusion_pipeline.launch)

Breaks the fusion down into separately debuggable nodes:
  1. Point Cloud Filter    - distance / ROI / ground filtering
  2. Point Cloud Projector - projects the filtered 3D points onto the image
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

CAMERA_MATRIX = [3145.24132, 0.0, 1887.68485, 0.0, 3166.08502, 990.62304, 0.0, 0.0, 1.0]
DISTORTION_COEFFICIENTS = [-0.298671, 0.087807, 0.000383, 0.000551, 0.000000]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('input_topic', default_value='/lidar/points'),

        # Step 1: Point Cloud Filter Node
        Node(
            package='drive_yolo',
            executable='drive_yolo_pointcloud_filter',
            name='pointcloud_filter',
            output='screen',
            parameters=[{
                'input_topic': LaunchConfiguration('input_topic'),
                'output_topic': '/lidar/points_filtered',
                # Distance filtering
                'min_distance': 0.5,
                'max_distance': 100.0,
                # ROI filtering (forward-facing LiDAR)
                'enable_roi_filtering': True,
                'roi_x_min': 0.0,    # Forward
                'roi_x_max': 50.0,
                'roi_y_min': -10.0,  # Right
                'roi_y_max': 10.0,   # Left
                'roi_z_min': -2.0,   # Below
                'roi_z_max': 10.0,   # Above
                # Ground filtering
                'enable_ground_filtering': False,
                'ground_z_threshold': -1.0,
                # Visualization
                'enable_visualization': True,
                'target_frame': 'lidar',
            }],
        ),

        # Step 2: Point Cloud Projector Node
        Node(
            package='drive_yolo',
            executable='drive_yolo_pointcloud_projector',
            name='pointcloud_projector',
            output='screen',
            parameters=[{
                'lidar_frame': 'lidar',
                'camera_frame': 'camera_entron',
                'transform_timeout': 0.1,
                'camera_matrix': CAMERA_MATRIX,
                'distortion_coefficients': DISTORTION_COEFFICIENTS,
                'image_width': 3848,
                'image_height': 2168,
                'debug_visualization': True,
            }],
        ),
    ])
