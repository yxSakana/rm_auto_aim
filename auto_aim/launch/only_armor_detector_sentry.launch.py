from launch import LaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("auto_aim"),
                    "launch", "only_armor_detector.launch.py"
                ])
            ]),
            launch_arguments={
                "armor_detector_ns": "master"
            }.items()
        ),
        TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare("auto_aim"),
                            "launch", "only_armor_detector.launch.py"
                        ])
                    ]),
                    launch_arguments={
                        "armor_detector_ns": "slave"
                    }.items()
                ),
            ]
        )
    ])
