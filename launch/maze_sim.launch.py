from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg = 'my_robot'
    share = get_package_share_directory(pkg)
    world = os.path.join(share, 'worlds', 'maze_big.world')
    xacro_file = os.path.join(share, 'urdf', 'myrobot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('gazebo_ros'),'launch','gazebo.launch.py')),
        launch_arguments={'world': world}.items()
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
                   '-x','-9.0','-y','-9.0','-z','0.15','-Y','0.0'],
        output='screen'
    )

    return LaunchDescription([gazebo, rsp, spawn])
