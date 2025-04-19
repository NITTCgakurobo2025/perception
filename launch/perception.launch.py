import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription


def generate_launch_description():
    name_parameter = DeclareLaunchArgument(
        'robot_name',
        default_value='R1')
    
    name = LaunchConfiguration('robot_name')

    return LaunchDescription([
        name_parameter,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('perception'),
                    'launch', 'obstacle_filter.launch.py'
                ),
            ),
            launch_arguments={'robot_name': name}.items()
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('perception'),
                    'launch', 'obstacle_detector.launch.py'
                ),
            ),
            launch_arguments={'robot_name': name}.items()
        ),
    ])
