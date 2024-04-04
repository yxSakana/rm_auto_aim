import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, TimerAction
from launch.substitutions import Command
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    robot_description = Command([
        "xacro ", os.path.join(
            get_package_share_directory("self_state"), "urdf", "self_state.urdf.xacro"),
        " xyz:=", "\"0 0 0\"",
        " rpy:=", "\"0 0 0\""
    ])

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 1000.0
        }],
        on_exit=Shutdown()
    )

    hik_camera_node = Node(
        package="hik_camera",
        executable="hik_camera_node",
        output="both",
        on_exit=Shutdown())

    armor_detector_node = Node(
        package="armor_detector",
        executable="armor_detector_node",
        output="both",
        on_exit=Shutdown())

    armor_tracker_node = Node(
        package="armor_tracker",
        executable="armor_tracker_node",
        output="both",
        on_exit=Shutdown())

    serial_node = Node(
        package="custom_serial_driver",
        executable="custom_serial_driver_node",
        output="both",
        on_exit=Shutdown())

    controller_io_node = Node(
        package="controller_io",
        executable="controller_io_node",
        output="both",
        on_exit=Shutdown())

    delay_serial_node = TimerAction(
        period=2.5,
        actions=[serial_node]
    )

    delay_armor_tracker_node = TimerAction(
        period=2.5,
        actions=[armor_tracker_node]
    )

    return LaunchDescription([
        robot_state_publisher_node,
        hik_camera_node,
        armor_detector_node,
        delay_armor_tracker_node,
        delay_serial_node,
        controller_io_node,
    ])
    container = Node(
        name="image_container",
        package="rclcpp_components",
        executable="component_container_mt",
        output="both"
    )
    hik_camera_node = ComposableNode(
        package="hik_camera",
        plugin="hik_camera::HikCameraNode",
        name="hik_camera_node",
        extra_arguments=[{
            "user_intra_process_comms": True
        }])
    armor_detector_node = ComposableNode(
        package="armor_detector",
        plugin="armor_auto_aim::ArmorDetectorNode",
        name="armor_detector_node",
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
    # armor_tracker_node = ComposableNode(
    #     package="armor_tracker",
    #     plugin="armor_auto_aim::ArmorTrackerNode",
    #     name="armor_tracker_node",
    #     extra_arguments=[{
    #         "user_intra_process_comms": True
    #     }])

    # load_composable_nodes = LoadComposableNodes(
    #     target_container="image_container",
    #     composable_node_descriptions=[
    #         armor_tracker_node
    #     ]
    # )
    return LaunchDescription([
        robot_state_publisher_node,
        container,
        cam_detector_container,
        armor_tracker_node,
        delay_serial_node,
        controller_io_node,
    ])
