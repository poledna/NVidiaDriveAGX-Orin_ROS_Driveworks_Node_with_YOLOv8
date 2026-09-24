"""Static TFs of the M8VIL test vehicle (ROS 2 port of M8VIL_TFs.launch)"""
from launch import LaunchDescription
from launch_ros.actions import Node

# (node name, "x y z yaw pitch roll parent child") - same argument order as ROS1
STATIC_TFS = [
    # Car Settings
    # Car rear Axle
    ('CarrearAxle', '0 0 0.342 0 0 0 car_topographical_zero car_rear_axle'),
    # Car Wheels
    ('WheelFR', '2.825 -0.879 0 0 0 0 car_rear_axle Wheel_FR'),
    ('WheelBR', '0 -0.891 0 0 0 0 car_rear_axle Wheel_BR'),
    ('WheelBL', '0 0.891 0 0 0 0 car_rear_axle Wheel_BL'),
    ('WheelFL', '2.830 0.879 0 0 0 0 car_rear_axle Wheel_FL'),
    # Car Sensors Settings
    # LiDARs
    ('Innoviz_LiDAR_TF', '1.539 0.006 1.041 0 0 0 car_rear_axle lidar'),
    ('Ouster_LiDAR_TF', '1.509 0.002 1.133 0 0 0 car_rear_axle ouster_lidar'),
    # Cameras zz=0.752
    ('FrontCamera_Entron_TF', '1.706 -0.016 0.752 -1.55 0.015 -1.45 car_rear_axle camera_entron'),
    ('FrontCamera_Lucid_TF', '1.693 0.016 0.784 0 0 0 car_rear_axle camera_lucid'),
    # RADARs
    ('ZFRADAR_TF', '3.685 -0.269 -0.034 0 0 0 car_rear_axle zf_radar'),
    ('ARS408_RADAR_TF', '3.678 -0.005 -0.08 0 0 0 car_rear_axle ars_radar'),
    # IMU
    ('IMAR_TF', '0.761 0.048 0.230 0 0 0 car_rear_axle IMU'),
    ('GPS_Anttena', '1.083 0 1.055 0 0 0 car_rear_axle GPS'),
    # Fake Car Sensors TF
    ('fake_Camera_entronTF', '0 0 0 0 0 0 camera_entron camera'),
    ('fake_Innoviz_LiDAR_TF', '0 0 0 0 0 0 lidar base_link'),
    ('fake_Ouster_LiDAR_TF', '0 0 0 0 0 0 ouster_lidar base_link_ouster'),
    ('fake_radar_TF', '0 0 0 0 0 0 zf_radar radar_center'),
]


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=name,
            arguments=args.split(),
        )
        for name, args in STATIC_TFS
    ])
