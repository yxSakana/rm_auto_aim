import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown, DeclareLaunchArgument, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from launch import LaunchDescription

sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))


def generate_launch_description():
    from common import node_params
    
    armor_auto_aim_ns = LaunchConfiguration("armor_auto_aim_ns")
    armor_auto_aim_ns_arg = DeclareLaunchArgument(
        "armor_auto_aim_ns",
        default_value=""
    )
    hik_camera_node = ComposableNode(
        package="hik_camera",
        plugin="hik_camera::HikCameraNode",
        name="hik_camera_node",
        namespace=armor_auto_aim_ns,
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True}])
    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="armor_auto_aim::ArmorDetectorNode",
        name="armor_detector_node",
        namespace=armor_auto_aim_ns,
        parameters=[node_params],
        extra_arguments=[{
            "user_intra_process_comms": True}])
    armor_tracker_node = Node(
        package="armor_tracker",
        executable="armor_tracker_node",
        name="armor_tracker_node",
        namespace=armor_auto_aim_ns,
        parameters=[node_params],
        output="both",
        on_exit=Shutdown())
    delay_armor_tracker_node = TimerAction(
        period=1.5,
        actions=[armor_tracker_node]
    )
    return LaunchDescription([
        armor_auto_aim_ns_arg,
        ComposableNodeContainer(
            name="camera_detector_container",
            namespace=armor_auto_aim_ns,
            package="rclcpp_components",
            executable="component_container",
            composable_node_descriptions=[
                hik_camera_node,
                armor_detector_node
            ],
            output="both",
            emulate_tty=True,
            on_exit=Shutdown()
        ),
        delay_armor_tracker_node
    ])
