from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess,
                            IncludeLaunchDescription, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('lab1_bringup')
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([share, 'config', 'speed_params.yaml']))

    system = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([share, 'launch', 'system.launch.py'])),
        launch_arguments={'params_file': LaunchConfiguration('params_file')}.items())

    trigger = TimerAction(period=4.0, actions=[
        ExecuteProcess(
            cmd=['ros2', 'action', 'send_goal', '--feedback', '/draw_square',
                 'lab1_interfaces/action/DrawSquare', '{side_length: 3.0}'],
            output='screen')])

    return LaunchDescription([params_file, system, trigger])
