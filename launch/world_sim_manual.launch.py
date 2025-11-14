from launch import LaunchDescription
from launch.actions import ExecuteProcess, SetEnvironmentVariable, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os, xacro

def generate_launch_description():
    pkg = 'my_robot'
    share = get_package_share_directory(pkg)

    # Args
    world_file = DeclareLaunchArgument('world_file', default_value='indoor.world')
    x_arg = DeclareLaunchArgument('x', default_value='-9.0')
    y_arg = DeclareLaunchArgument('y', default_value='-9.0')
    z_arg = DeclareLaunchArgument('z', default_value='0.15')
    Y_arg = DeclareLaunchArgument('Y', default_value='0.0')

    world_path = PathJoinSubstitution([share, 'worlds', LaunchConfiguration('world_file')])

    # Robot description
    xacro_file = os.path.join(share, 'urdf', 'myrobot.xacro')
    robot_description = xacro.process_file(xacro_file).toxml()

    # Env so gzserver finds ROS plugins/resources
    env1 = SetEnvironmentVariable('GAZEBO_PLUGIN_PATH', '/opt/ros/humble/lib')
    env2 = SetEnvironmentVariable('GAZEBO_MODEL_PATH',
                                  os.environ.get('GAZEBO_MODEL_PATH','') + ':/usr/share/gazebo-11/models')
    env3 = SetEnvironmentVariable('GAZEBO_RESOURCE_PATH',
                                  os.environ.get('GAZEBO_RESOURCE_PATH','') + ':/usr/share/gazebo-11')

    # Run gzserver/gzclient manually WITHOUT libgazebo_ros_force_system.so
    gzserver = ExecuteProcess(
        cmd=['gzserver', '--verbose', world_path,
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )
    gzclient = ExecuteProcess(cmd=['gzclient'], output='screen')

    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
        output='screen'
    )
    spawn = Node(
        package='gazebo_ros', executable='spawn_entity.py',
        arguments=['-topic','robot_description','-entity','my_robot',
                   '-x', LaunchConfiguration('x'),
                   '-y', LaunchConfiguration('y'),
                   '-z', LaunchConfiguration('z'),
                   '-Y', LaunchConfiguration('Y')],
        output='screen'
    )

    return LaunchDescription([world_file, x_arg, y_arg, z_arg, Y_arg,
                              env1, env2, env3, gzserver, gzclient, rsp, spawn])
