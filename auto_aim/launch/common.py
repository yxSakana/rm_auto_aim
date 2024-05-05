import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown, TimerAction
from launch.substitutions import Command
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

pkg_dir = get_package_share_directory("auto_aim")
with open(os.path.join(pkg_dir, "config", "robot_type"), "r") as f:
    robot_type = f.read().strip()
node_params = os.path.join(
    get_package_share_directory("auto_aim"), "config", robot_type + ".yaml")

# robot state publisher
camera_offset = yaml.safe_load(open(os.path.join(
    get_package_share_directory("auto_aim"), "config", "camera_offset.yaml")))

if robot_type == "sentry":
    robot_description = Command([
        "xacro ", os.path.join(
            get_package_share_directory("robot_descript"), "urdf", "sentry_robot_descript.urdf.xacro"),
        " gimbal2camera_xyz:="      , camera_offset[robot_type]["gimbal2camera_xyz"],
        " gimbal2camera_rpy:="      , camera_offset[robot_type]["gimbal2camera_rpy"],
        " gimbal2slave_camera_xyz:=", camera_offset[robot_type]["gimbal2slave_camera_xyz"],
        " gimbal2slave_camera_rpy:=", camera_offset[robot_type]["gimbal2slave_camera_rpy"],
    ])
else:
    robot_description = Command([
        "xacro ", os.path.join(
            get_package_share_directory("robot_descript"), "urdf", "robot_descript.urdf.xacro"),
        " xyz:=", camera_offset[robot_type]["xyz"],
        " rpy:=", camera_offset[robot_type]["rpy"]])

robot_state_publisher_node = Node(
    package="robot_state_publisher",
    executable="robot_state_publisher",
    parameters=[{
        "robot_description": robot_description,
        "publish_frequency": 1000.0
    }],
    on_exit=Shutdown()
)

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
    parameters=[node_params],
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
delay_serial_node = TimerAction(
    period=2.5,
    actions=[serial_container]
)
