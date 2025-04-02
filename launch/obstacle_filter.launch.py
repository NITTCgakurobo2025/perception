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
            executable='obstacle_filter',
            name=[name, '_obstacle_filter'],
            output='screen',
            parameters=[{
                'scan_topic': ['/', name, '/merged_scan'],
                'transform_topic': ['/', name, '/scan_transform'],
                'output_topic': ['/', name, '/transformed_scan'],
                'wall_topic': ['/', name, '/detected_wall'],
            }]
        )
    ])
