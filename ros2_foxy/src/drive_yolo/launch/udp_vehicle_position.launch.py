"""UDP Vehicle Position TF Publisher (ROS 2 port of udp_vehicle_position.launch)
Receives vehicle position via UDP (roll, pitch, yaw, x, y) and publishes it
as TF transform and odometry. Pairs with the ROS2 inssol_udp_sender node."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='5000'),
        DeclareLaunchArgument('world_frame', default_value='map'),
        DeclareLaunchArgument('vehicle_frame', default_value='base_link'),
        DeclareLaunchArgument('publish_rate', default_value='50.0'),
        DeclareLaunchArgument('z_offset', default_value='0.0'),

        Node(
            package='drive_yolo',
            executable='drive_yolo_udp_vehicle_tf',
            name='udp_vehicle_tf_publisher',
            output='screen',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'world_frame': LaunchConfiguration('world_frame'),
                'vehicle_frame': LaunchConfiguration('vehicle_frame'),
                'publish_rate': LaunchConfiguration('publish_rate'),
                'publish_odom': True,
                'z_offset': LaunchConfiguration('z_offset'),
            }],
        ),
    ])
