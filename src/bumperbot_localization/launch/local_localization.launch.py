from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

import os

def generate_launch_description():
    # Publishes static transform between base_footprint_ekf and imu_ekf
    static_transform_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["--x", "0", "--y", "0", "--z", "0.103",
                   "--qx", "1", "--qy", "0", "--qz", "0", "--qw", "0",
                   "--frame-id", "base_footprint_ekf",
                   "--child-frame-id", "imu_link_ekf"]
    )

    # EKF to fuse IMU + odometry data into filtered odometry
    robot_localization = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        output="screen",
        parameters=[os.path.join(get_package_share_directory("bumperbot_localization"),
                                 "config", "ekf.yaml")]
    )

    # Needed for EKF node to change topic name
    imu_republisher_cpp = Node(
        package="bumperbot_localization",
        executable="imu_republisher"
    )

    return LaunchDescription([
        static_transform_publisher,
        robot_localization,
        imu_republisher_cpp
    ])
