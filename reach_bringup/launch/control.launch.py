from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_controller"),
        DeclareLaunchArgument("tcp_controller"),
        DeclareLaunchArgument("use_sim"),
        DeclareLaunchArgument("robot_description"),
    ]

    use_sim = LaunchConfiguration("use_sim")

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        output="both",
        parameters=[LaunchConfiguration("controllers_file"), {"use_sim_time": use_sim}],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
        condition=UnlessCondition(use_sim),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[
            {
                "robot_description": LaunchConfiguration("robot_description"),
                "use_sim_time": use_sim,
            }
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        parameters=[{"use_sim_time": use_sim}],
    )

    robot_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("robot_controller")],
        parameters=[{"use_sim_time": use_sim}],
    )

    tcp_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[LaunchConfiguration("tcp_controller")],
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
