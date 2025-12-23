from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Wheel radius argument
    wheel_radius_arg = DeclareLaunchArgument(
        "wheel_radius",
        default_value="0.033"
    )

    # Wheel separation argument
    wheel_separation_arg = DeclareLaunchArgument(
        "wheel_separation",
        default_value="0.17"
    )

    # Which controller to use?
    use_simple_controller_arg = DeclareLaunchArgument(
        "use_simple_controller",
        default_value="True"
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "/controller_manager"
        ]
    )

    # Extract runtime arguments
    wheel_radius = LaunchConfiguration("wheel_radius")
    wheel_separation = LaunchConfiguration("wheel_separation")
    use_simple_controller = LaunchConfiguration("use_simple_controller")

    wheel_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "bumperbot_controller",
            "--controller-manager",
            "/controller_manager"
        ],
        condition=UnlessCondition(use_simple_controller)
    )

    simple_controller = GroupAction(
        condition=IfCondition(use_simple_controller),
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    "simple_velocity_controller",
                    "--controller-manager",
                    "/controller_manager"
                ],
            ),

            # Simple velocity controller
            Node(
                package="bumperbot_controller",
                executable="simple_controller",
                parameters=[{"wheel_radius": wheel_radius,
                            "wheel_separation": wheel_separation}]
            )
        ]
    )

    return LaunchDescription([
        wheel_radius_arg,
        wheel_separation_arg,
        use_simple_controller_arg,
        joint_state_broadcaster_spawner,
        wheel_controller_spawner,
        simple_controller,
    ])
