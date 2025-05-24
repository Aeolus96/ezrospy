import os

from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def launch_gui():
    return Node(
        name="ezrospy_gui",
        package="ezrospy",
        executable="ezrospy_gui",
        output="screen",
    )


def generate_launch_description():
    ld = LaunchDescription()
    ld.add_action(launch_gui())
    return ld
