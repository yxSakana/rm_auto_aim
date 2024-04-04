import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
sys.path.append(os.path.join(get_package_share_directory('auto_aim'), 'launch'))

from common import cam_detector_container

def generate_launch_description():
    return LaunchDescription([
        cam_detector_container,
    ])
