import os
import sys

from ament_index_python.packages import get_package_share_directory
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))


def generate_launch_description():

    from common import cam_detector_container
    from launch import LaunchDescription

    return LaunchDescription([
        cam_detector_container
    ])

