import os
import launch
import launch_ros
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    ld =  launch.LaunchDescription()

    robot_localization_node = launch_ros.actions.Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(get_package_share_directory("tianracer_bringup"), 'launch/config/ekf.yaml')]
    )

    
    
    ld.add_action(robot_localization_node)                                           
    return ld