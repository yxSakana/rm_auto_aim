import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

node_params = os.path.join(
    get_package_share_directory("auto_aim"), "config", "node_params.yaml"
)

robot_description = Command([
    "xacro ", os.path.join(
        get_package_share_directory("self_state"), "urdf", "self_state.urdf.xacro"),
    # " xyz:=", "\"0.10 -0.1 0.00\"",
    " xyz:=", "\"0.00 -0.06 0.04\"",
    " rpy:=", "\"0.0 -0.0 -0.01\""
])

# robot descript
robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{
        "robot_description": robot_description,
        "publish_frequency": 1000.0
    }],
    on_exit=Shutdown()
)
# camera && armor_detector
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
    name="camera_detector_container",
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
# armor tracker
armor_tracker_node = Node(
    package="armor_tracker",
    executable="armor_tracker_node",
    name="armor_tracker_node",
    parameters=[node_params],
    output="both",
    on_exit=Shutdown())
# serial
serial_node = ComposableNode(
    package="custom_serial_driver",
    plugin="custom_serial::SerialDriverNode",
    name="custom_serial_node",
    parameters=[node_params],
    extra_arguments=[{
        "user_intra_process_comms": True
    }])
controller_io_node = ComposableNode(
    package="controller_io",
    plugin="armor_auto_aim::ControllerIONode",
    name="controller_io_node",
    extra_arguments=[{
        "user_intra_process_comms": True
    }]
)
serial_container = ComposableNodeContainer(
    name="serial_container",
    namespace="",
    package="rclcpp_components",
    executable="component_container",
    composable_node_descriptions=[
        serial_node,
        controller_io_node,
    ],
    output="both",
    emulate_tty=True,
    on_exit=Shutdown()  
)
