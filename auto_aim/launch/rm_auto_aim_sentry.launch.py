import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))

from common import \
    robot_state_publisher_node, armors_filter, \
    delay_armor_tracker_node, delay_serial_node

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("auto_aim"),
                    "launch", "cam_detector.launch.py"
                ])
            ]),
            launch_arguments={
                "armor_detector_ns": "sentry_master"
            }.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("auto_aim"),
                    "launch", "cam_detector.launch.py"
                ])
            ]),
            launch_arguments={
                "armor_detector_ns": "sentry_slave"
            }.items()
        ),
        armors_filter,
        robot_state_publisher_node,
        delay_armor_tracker_node,
        delay_serial_node,
    ])
