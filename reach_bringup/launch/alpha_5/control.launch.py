from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_controller"),
        DeclareLaunchArgument("use_sim"),
        DeclareLaunchArgument("robot_description"),
    ]

    controllers_file = LaunchConfiguration("controllers_file")
    robot_controller = LaunchConfiguration("robot_controller")
    use_sim = LaunchConfiguration("use_sim")
    robot_description = LaunchConfiguration("robot_description")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[controllers_file, {"use_sim_time": use_sim}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim}],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager",
            "controller_manager",
        ],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            robot_controller,
            "--controller-manager",
            "controller_manager",
        ],
        parameters=[{"use_sim_time": use_sim}],
    )

    # Delay joint_state_broadcaster after controller_manager
    delay_jsb_spawner_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        ),
    )

    # Delay start of robot_controller after `joint_state_broadcaster`
    delay_robot_controller_spawners_after_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[robot_controller_spawner],
        ),
    )

    return LaunchDescription(
        args
        + [
            controller_manager,
            robot_state_publisher,
            delay_jsb_spawner_after_controller_manager,
            delay_robot_controller_spawners_after_jsb_spawner,
        ]
    )
