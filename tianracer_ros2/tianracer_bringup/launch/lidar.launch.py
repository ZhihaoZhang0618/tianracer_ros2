import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    lidar = os.environ.get("TIANRACER_LIDAR", "ltme")
    model = os.environ.get("TIANRACER_LIDAR_MODEL", "a2")
    ld = LaunchDescription()
    if "ltme" in lidar:
        ld.add_action(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("ltme_node"),
                'launch','ltme-02a.launch.py')),
            ))
    elif "rplidar" in lidar:
        ld.add_action(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("tianracer_bringup"),
            'launch','includes','lidar', 'rplidar.launch.py')),
            launch_arguments={'model': model}.items(),
        ))

    return ld