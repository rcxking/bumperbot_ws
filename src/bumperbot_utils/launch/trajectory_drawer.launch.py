from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Odom topic
    odom_topic_arg = DeclareLaunchArgument(
        "odom_topic",
        default_value="/bumperbot_controller/odom"
    )

    # Extract runtime arguments
    odom_topic = LaunchConfiguration("odom_topic")

    # Spawn trajectory drawer Node
    traj_drawer = Node(
        package="bumperbot_utils",
        executable="trajectory_drawer",
        parameters=[{"odom_topic": odom_topic}]
    )

    return LaunchDescription([
        odom_topic_arg,
        traj_drawer
    ])
