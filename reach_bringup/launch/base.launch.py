import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit, OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("controllers_file"),
        DeclareLaunchArgument("robot_controller"),
        DeclareLaunchArgument("tcp_controller"),
        DeclareLaunchArgument("use_sim"),
        DeclareLaunchArgument("use_rviz"),
        DeclareLaunchArgument("robot_description"),
        DeclareLaunchArgument("gz_model_name"),
        DeclareLaunchArgument("gz_world_file"),
        DeclareLaunchArgument("rviz_config"),
    ]

    use_sim = LaunchConfiguration("use_sim")
    robot_description = LaunchConfiguration("robot_description")

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[{"robot_description": robot_description, "use_sim_time": use_sim}],
    )

    rviz_spawner = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[robot_description, {"use_sim_time": use_sim}],
        condition=IfCondition(LaunchConfiguration("use_rviz")),
    )

    # Gazebo launch
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=["/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock"],
        condition=IfCondition(use_sim),
    )

    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name",
            LaunchConfiguration("gz_model_name"),
            "-topic",
            "robot_description",
            "-allow_renaming",
            "true",
        ],
        output="screen",
        condition=IfCondition(use_sim),
    )

    gz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                os.path.join(
                    get_package_share_directory("ros_gz_sim"),
                    "launch",
                    "gz_sim.launch.py",
                ),
            ]
        ),
        launch_arguments=[
            (
                "gz_args",
                [
                    "-v 4 --physics-engine gz-physics-bullet-featherstone-plugin -r",
                    " ",  # No, you cannot remove this
                    LaunchConfiguration("gz_world_file"),
                ],
            )
        ],
        condition=IfCondition(use_sim),
    )

    # ros2_control launch
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
    # This will not work if the controller_manager is not started (i.e., in simulation)
    delay_jsb_spawner_after_controller_manager = RegisterEventHandler(
        event_handler=OnProcessStart(
            target_action=controller_manager,
            on_start=[joint_state_broadcaster_spawner],
        ),
    )

    # Delay the joint_state_broadcaster after the gz_ros2_control controller_manager
    # has been launched.
    delay_jsb_spawner_after_spawn_entity = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=gz_spawner,
            on_exit=[joint_state_broadcaster_spawner],
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
        gz_bridge,
        gz_spawner,
        gz_launch,
        rviz_spawner,
        delay_jsb_spawner_after_controller_manager,
        delay_jsb_spawner_after_spawn_entity,
        delay_robot_controller_spawners_after_jsb_spawner,
        delay_tcp_controller_spawners_after_jsb_spawner,
    ]

    return LaunchDescription(args + nodes)
