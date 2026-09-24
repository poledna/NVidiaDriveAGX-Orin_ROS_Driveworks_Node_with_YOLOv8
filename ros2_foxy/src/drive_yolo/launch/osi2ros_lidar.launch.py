"""OSI to ROS LiDAR Bridge (ROS 2 port of osi2ros_lidar.launch)
Receives OSI LiDAR data via HTTP and publishes it as a PointCloud2."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5002'),
        DeclareLaunchArgument('topic_name', default_value='/invz_summation_reflection'),

        Node(
            package='drive_yolo',
            executable='osi2ros_lidar.py',
            name='lidar_osi_http_receiver',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'topic_name': LaunchConfiguration('topic_name'),
            }],
        ),
    ])
