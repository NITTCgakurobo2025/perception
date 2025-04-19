from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='R1')

    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        Node(
            package='perception',
            executable='obstacle_detector',
            name=[name, '_obstacle_detector'],
            output='screen',
            parameters=[{
                'input_topic': ['/', name, '/transformed_scan'],
                'output_topic': ['/', name, '/obstacles'],
                'hull_topic': ['/', name, '/obstacles_hull'],
            }]
        )
    ])
