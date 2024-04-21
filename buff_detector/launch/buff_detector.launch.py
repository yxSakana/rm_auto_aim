from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import LoadComposableNodes, Node
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = Node(
        name="image_container",
        package="rclcpp_components",
        executable="component_container",
        output="both"
    )

    load_composable_nodes = LoadComposableNodes(
        target_container="image_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_camera",
                plugin="hik_camera::HikCameraNode",
                name="hik_camera_node",
            ),
            ComposableNode(
                package="buff_detector",
                plugin="armor_auto_aim::BuffDetectorNode",
                name="buff_detector_node"
            )
        ]
    )

    return LaunchDescription([container, load_composable_nodes])
