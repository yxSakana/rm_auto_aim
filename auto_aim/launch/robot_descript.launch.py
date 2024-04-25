import os
import yaml

from ament_index_python.packages import get_package_share_directory
from launch.actions import Shutdown
from launch.substitutions import Command
from launch_ros.actions import Node
from launch import LaunchDescription


def generate_launch_description():
    pkg_dir = get_package_share_directory("auto_aim")
    # robot type
    with open(os.path.join(pkg_dir, "config", "robot_type"), "r") as f:
        robot_type = f.read().strip()
    # camera offset
    camera_offset = yaml.safe_load(open(os.path.join(
        get_package_share_directory("auto_aim"), "config", "camera_offset.yaml")))
    robot_description = Command([
        "xacro ", os.path.join(
            get_package_share_directory("robot_descript"), "urdf", "robot_descript.urdf.xacro"),
        " xyz:=", camera_offset["infantry_CS016"]["xyz"],
        " rpy:=", camera_offset["infantry_CS016"]["rpy"]])
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=robot_type,
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 1000.0
        }],
        on_exit=Shutdown()
    )

    return LaunchDescription([
        robot_state_publisher_node
    ])
