from launch import LaunchDescription
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
                name="hik_camera_node"
            )
        ]
    )

    return LaunchDescription([
        Node(
            package="hik_camera",
            executable="hik_camera_node",
            output="screen",
            emulate_tty=True
        )
    ])

    return LaunchDescription([
        Node(
            package="hik_camera",
            executable="hik_camera_node",
            output="screen",
            emulate_tty=True
        )
    ])

    return LaunchDescription([container, load_composable_nodes])
