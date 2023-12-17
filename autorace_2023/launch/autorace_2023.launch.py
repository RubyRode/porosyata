import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('robot_bringup'), 'launch'),
                                       '/autorace_2023.launch.py']),
    )

    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('autorace_camera'), 'launch'),
                                        "/extrinsic_camera_calibration.launch.py"]),
    )


    return LaunchDescription([
        bringup,
        camera,
        Node(
            package='autorace_2023',
            executable='detect_lane',
            name='lane_detection',
        ),
        Node(
            package="autorace_2023",
            executable="pid_lane",
            name="PID",
        ),
        Node(
            package="referee_console",
            executable="mission_autorace_2023_referee",
            name="referee"
        ),
        Node(
            package="detect_signs",
            executable="detect",
            name="sign_detection",
        ),
        Node(
            package="autorace_2023",
            executable="mov_ctrl",
            name="mov_ctrl"
            ),
    ])
