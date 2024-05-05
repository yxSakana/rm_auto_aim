import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))

from common import robot_type, \
    robot_state_publisher_node, delay_serial_node

def generate_launch_description():
    print("robot type: ", robot_type)
    return LaunchDescription([
        robot_state_publisher_node,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("auto_aim"),
                    "launch", "armor_auto_aim.launch.py"
                ])
            ]),
            launch_arguments={
                "armor_auto_aim_ns": ""
            }.items()
        ),
        delay_serial_node,
    ])
