from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_controller"),
        DeclareLaunchArgument("tcp_controller"),
        DeclareLaunchArgument("robot_description"),
    ]

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": LaunchConfiguration("robot_description")}],
    )

    # ros2_control launch
    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[LaunchConfiguration("controllers_file")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("robot_controller")],
    )

    tcp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("tcp_controller")],
    )

    # Delay joint_state_broadcaster after controller_manager
    # This will not work if the controller_manager is not started (i.e., in simulation)
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

    # Delay start of tcp_controller after `joint_state_broadcaster`
    delay_tcp_controller_spawners_after_jsb_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[tcp_controller_spawner],
        ),
    )

    nodes = [
        controller_manager,
        robot_state_publisher,
        delay_jsb_spawner_after_controller_manager,
        delay_robot_controller_spawners_after_jsb_spawner,
        delay_tcp_controller_spawners_after_jsb_spawner,
    ]

    return LaunchDescription(args + nodes)
