import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    declare_ns = DeclareLaunchArgument("ns", default_value="micro_ip")
    declare_parameters_file = DeclareLaunchArgument(
        "parameters_file",
        default_value=os.path.join(
            get_package_share_directory("reach_description"),
            "config",
            "micro_ip",
            "gstreamer_proxy.yaml",
        ),
    )

    gstreamer_proxy_node = ComposableNode(
        package="reach_ip_camera",
        plugin="reach::GStreamerProxy",
        namespace=LaunchConfiguration("ns"),
        name="gstreamer_proxy",
        parameters=[LaunchConfiguration("parameters_file")],
        extra_arguments=[{"use_intra_process_comms": True}],
    )

    rectify_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::RectifyNode",
        name="rectify_node",
        namespace=LaunchConfiguration("ns"),
        remappings=[("image", "image_raw")],
    )

    debayer_node = ComposableNode(
        package="image_proc",
        plugin="image_proc::DebayerNode",
        namespace=LaunchConfiguration("ns"),
        name="debayer_node",
        remappings=[("image_raw", "image_raw")],
    )

    container = ComposableNodeContainer(
        name="micro_ip_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            gstreamer_proxy_node,
            rectify_node,
            debayer_node,
        ],
    )

    return LaunchDescription(
        [
            declare_parameters_file,
            declare_ns,
            container,
        ]
    )
