from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution(
            [FindPackageShare('lab1_bringup'), 'config', 'speed_params.yaml']),
        description='YAML file with max_linear_speed / max_angular_speed')

    return LaunchDescription([
        params_file,
        Node(package='turtlesim', executable='turtlesim_node', name='turtlesim'),
        Node(package='lab1_turtle_control', executable='turtle_controller',
             name='turtle_controller', output='screen',
             parameters=[LaunchConfiguration('params_file')]),
        Node(package='lab1_turtle_control', executable='reset_service',
             name='reset_service', output='screen'),
        Node(package='lab1_turtle_control', executable='square_server',
             name='square_server', output='screen'),
    ])
