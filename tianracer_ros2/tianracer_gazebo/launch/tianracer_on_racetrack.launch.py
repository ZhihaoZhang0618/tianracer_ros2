import os
import launch
import launch_ros
import xacro
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import ExecuteProcess

def generate_launch_description():

    ld = launch.LaunchDescription()
    ld.add_action(DeclareLaunchArgument('paused', default_value='false'))
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true'))
    ld.add_action(DeclareLaunchArgument('gui', default_value='true'))
    ld.add_action(DeclareLaunchArgument('headless', default_value='false'))
    ld.add_action(DeclareLaunchArgument('debug', default_value='false'))
    ld.add_action(DeclareLaunchArgument('x_pos', default_value='0'))
    ld.add_action(DeclareLaunchArgument('y_pos', default_value='0'))
    ld.add_action(DeclareLaunchArgument('z_pos', default_value='0'))
    ld.add_action(DeclareLaunchArgument('R_pos', default_value='0'))
    ld.add_action(DeclareLaunchArgument('P_pos', default_value='0'))
    ld.add_action(DeclareLaunchArgument('Y_pos', default_value='1.5708'))


    start_gazebo_cmd = ExecuteProcess(
        cmd=['gazebo', '--verbose','-s', 'libgazebo_ros_init.so', '-s', 'libgazebo_ros_factory.so', os.path.join(get_package_share_directory("tianracer_gazebo"), 'worlds/tianracer_racetrack.world')],
        output='screen')
    ld.add_action(start_gazebo_cmd)

    # xarco
    xacro_file = os.path.join(get_package_share_directory("tianracer_gazebo"), 'urdf/tianracer.xacro')
    doc = xacro.parse(open(xacro_file))
    xacro.process_doc(doc)
    params = {'robot_description': doc.toxml()}
	# 启动机器人状态发布节点
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[params]
    )
    ld.add_action(node_robot_state_publisher)


    robot_des = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        arguments=['-topic', 'robot_description','-entity', 'tianracer'],
        #    args="-urdf -model tianracer -param robot_description -x $(arg x_pos) -y $(arg y_pos) -z $(arg z_pos) -R $(arg R_pos) -P $(arg P_pos) -Y $(arg Y_pos)" />
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},
                    {'x': LaunchConfiguration('x_pos')},
                    {'y': LaunchConfiguration('y_pos')},
                     {'z': LaunchConfiguration('z_pos')},
                     {'R': LaunchConfiguration('R_pos')},
                     {'P': LaunchConfiguration('P_pos')},
                     {'Y': LaunchConfiguration('Y_pos')},
                     

                    ],
        output='screen')

    ld.add_action(robot_des)



    return ld

        # # Load robot model description parameters
        # Node(
        #     package='xacro',
        #     executable='xacro',
        #     name='xacro',
        #     output='screen',
        #     arguments=[
        #         'inorder',
        #         LaunchConfiguration('robot_description')
        #     ],
        #     parameters=[
        #         {'robot_description': FindExecutable(name='xacro') + ' --inorder $(find tianracer_gazebo)/urdf/tianracer.xacro'}
        #     ]
        # ),

        # # Spawn robot model in Gazebo
        # Node(
        #     package='gazebo_ros',
        #     executable='spawn_model',
        #     name='urdf_spawner',
        #     output='screen',
        #     arguments=[
        #         '-urdf',
        #         '-model', 'tianracer',
        #         '-param', 'robot_description',
        #         '-x', LaunchConfiguration('x_pos'),
        #         '-y', LaunchConfiguration('y_pos'),
        #         '-z', LaunchConfiguration('z_pos'),
        #         '-R', LaunchConfiguration('R_pos'),
        #         '-P', LaunchConfiguration('P_pos'),
        #         '-Y', LaunchConfiguration('Y_pos')
        #     ]
        # ),

        # Launch the simulation joystick control
        # Node(
        #     package='tianracer_gazebo',
        #     executable='keyboard_teleop.py',
        #     name='keyboard_teleop'
        # )

