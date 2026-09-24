"""Detection Visualizer (ROS 2 port of detection_visualizer.launch)
Draws YOLO detections on the camera images -> /camera/image_with_detections"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drive_yolo',
            executable='detection_visualizer_node.py',
            name='detection_visualizer',
            output='screen',
            parameters=[{
                'bbox_thickness': 2,
                'text_scale': 0.6,
                'text_thickness': 2,
                'show_confidence': True,
                'show_class_name': True,
            }],
            # Remap topics here if needed:
            # remappings=[('/camera/image_raw', '/your/camera/topic'),
            #             ('/detections', '/your/detections/topic')],
        ),
    ])
