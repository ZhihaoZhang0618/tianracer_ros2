from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import Command
from rclpy.parameter import ParameterValue
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
import os
from ament_index_python.packages import get_package_share_directory
import xacro
def generate_launch_description():
    # 设置launch文件的参数
    paused = LaunchConfiguration('paused', default='false')
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    gui = LaunchConfiguration('gui', default='true')
    headless = LaunchConfiguration('headless', default='false')
    debug = LaunchConfiguration('debug', default='false')
    world = LaunchConfiguration('world', default=os.path.join(get_package_share_directory('tianracer_gazebo'), 'worlds', 'tianracer_racetrack.world'))
    x_pos = LaunchConfiguration('x_pos', default='0')
    y_pos = LaunchConfiguration('y_pos', default='0')
    z_pos = LaunchConfiguration('z_pos', default='0')
    R_pos = LaunchConfiguration('R_pos', default='0')
    P_pos = LaunchConfiguration('P_pos', default='0')
    Y_pos = LaunchConfiguration('Y_pos', default='1.5708')
    # 获取gazebo_ros包的路径
    gazebo_ros_pkg = FindPackageShare(package='gazebo_ros').find('gazebo_ros')

    # 获取tianracer_gazebo包的路径
    tianracer_gazebo_pkg = FindPackageShare(package='tianracer_gazebo').find('tianracer_gazebo')

    # 运行gazebo仿真环境


    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory("gazebo_ros"),
            'launch','gazebo.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'paused': paused,
                'debug': debug,
                'gui': gui,
                'headless': headless,
                'world': world
                            }.items()
    )


    robot_des = xacro.process_file(os.path.join(tianracer_gazebo_pkg, 'urdf', 'tianracer.xacro'))
    robot_des = robot_des.toxml()

    robot_description = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_des
        }]
    )


    # 在gazebo中加载机器人模型
    urdf_spawner = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='urdf_spawner',
        output='screen',
        arguments=['-topic', 'robot_description','-entity', 'tianracer',
                   '-x', x_pos,
                   '-y', y_pos,
                   '-z', z_pos,
                   '-R', R_pos,
                   '-P', P_pos,
                   '-Y', Y_pos]
    )


    return LaunchDescription([
        # 运行gazebo仿真环境
        world_launch,
        robot_description,
        # urdf_spawner
    ])
