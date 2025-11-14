from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg = 'my_robot'
    share = get_package_share_directory(pkg)

    # نمرّر اسم الملف كاملاً (مع .world)
    world_file_arg = DeclareLaunchArgument('world_file', default_value='maze_big.world',
        description='World file name inside my_robot/worlds (e.g., maze_big.world | indoor.world | outdoor.world)')
    x_arg = DeclareLaunchArgument('x', default_value='-9.0')
    y_arg = DeclareLaunchArgument('y', default_value='-9.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.15')
    Y_arg = DeclareLaunchArgument('Y', default_value='0.0')

    world_path = PathJoinSubstitution([share, 'worlds', LaunchConfiguration('world_file')])

    xacro_file = os.path.join(share, 'urdf', 'myrobot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_path}.items()
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )

    spawn = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','my_robot',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('Y')],
        output='screen'
    )

    return LaunchDescription([world_file_arg, x_arg, y_arg, z_arg, Y_arg, gazebo, rsp, spawn])

