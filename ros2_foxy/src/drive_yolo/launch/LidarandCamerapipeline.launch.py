"""Full on-vehicle pipeline: Python tracker + DriveWorks YOLO + vehicle TFs + fusion
(ROS 2 port of LidarandCamerapipeline.launch)"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    launch_dir = os.path.join(get_package_share_directory('drive_yolo'), 'launch')

    def include(name):
        return IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(launch_dir, name)))

    return LaunchDescription([
        include('motpy_tracking.launch.py'),
        include('yolo_driveworks.launch.py'),
        include('M8VIL_TFs.launch.py'),
        include('lidar_camera_fusion.launch.py'),
    ])
