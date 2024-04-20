import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown, TimerAction
from launch.substitutions import Command
from launch_ros.actions import LoadComposableNodes, Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))

from common import robot_state_publisher_node, \
        cam_detector_container, delay_armor_tracker_node, \
        delay_serial_node

def generate_launch_description():
    return LaunchDescription([
        robot_state_publisher_node,
        cam_detector_container,
        delay_armor_tracker_node,
        delay_serial_node,
    ])
