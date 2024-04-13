import os
import sys

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))


def generate_launch_description():

    from common import node_params
    from launch_ros.descriptions import ComposableNode
    from launch_ros.actions import ComposableNodeContainer
    from launch import LaunchDescription

    cam_dect = ComposableNodeContainer(
        name="camera_detector_container",
        namespace="",
        package="rclcpp_components",
        executable="component_container",
        composable_node_descriptions=[
            ComposableNode(
                package="hik_camera",
                plugin="hik_camera::HikCameraNode",
                name='camera_node',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            ),
            ComposableNode(
                package='armor_detector',
                plugin='armor_auto_aim::ArmorDetectorNode',
                name='armor_detector',
                parameters=[node_params],
                extra_arguments=[{'use_intra_process_comms': True}]
            )
        ]
    )

    return LaunchDescription([
        cam_dect
    ])

