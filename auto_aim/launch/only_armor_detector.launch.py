import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))


def generate_launch_description():
    from common import node_params
    
    armor_detector_ns = LaunchConfiguration("armor_detector_ns")
    armor_detector_ns_arg = DeclareLaunchArgument(
        "armor_detector_ns",
        default_value=""
    )
    hik_camera_node = ComposableNode(
        package="hik_camera",
        plugin="hik_camera::HikCameraNode",
        name="hik_camera_node",
        namespace=armor_detector_ns,
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True}])
    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="armor_auto_aim::ArmorDetectorNode",
        name="armor_detector_node",
        namespace=armor_detector_ns,
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True}])
    return LaunchDescription([
        armor_detector_ns_arg,
        ComposableNodeContainer(
            name="camera_detector_container",
            namespace=armor_detector_ns,
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
    ])
