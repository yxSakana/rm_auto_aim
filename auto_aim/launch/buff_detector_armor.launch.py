import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, TimerAction
from launch.substitutions import Command
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    node_params = os.path.join(
        get_package_share_directory("auto_aim"), "config", "node_params.yaml"
    )
    container = Node(
        name="image_container",
        package="rclcpp_components",
        executable="component_container",
        output="both"
    )


    hik_camera_node = ComposableNode(
        package="hik_camera",
        plugin="hik_camera::HikCameraNode",
        name="hik_camera_node",
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True
        }])
    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="armor_auto_aim::ArmorDetectorNode",
        name="armor_detector_node",
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True
        }])
    cam_detector_container = ComposableNodeContainer(
        name="camera_detecotr_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            hik_camera_node,
            armor_detector_node
        ],
        output="both",
        emulate_tty=True,
        on_exit=Shutdown()  
    )

    buff_detector_node = Node(
        package="buff_detector",
        executable="buff_detector_node",
        name="buff_detector_node",
        parameters=[node_params],
        output="both",
        on_exit=Shutdown())

    return LaunchDescription([cam_detector_container, buff_detector_node])
