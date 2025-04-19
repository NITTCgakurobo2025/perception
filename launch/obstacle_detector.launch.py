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
            name='obstacle_detector',
            namespace=name,
            output='screen',
            parameters=[{
                'input_topic': 'transformed_scan',
                'output_topic': 'obstacles',
                'hull_topic': 'obstacles_hull',
            }]
        )
    ])
