import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))

from common import robot_type, \
    robot_state_publisher_node, delay_serial_node

def generate_launch_description():
    print("robot type: ", robot_type)
    return LaunchDescription([
        robot_state_publisher_node,
        delay_serial_node,
    ])

