from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Paths to the welder and positioner launch files
    welder_launch_path = os.path.join(
        get_package_share_directory('welder'), 'launch', 'demo.launch.py')
    positioner_launch_path = os.path.join(
        get_package_share_directory('positioner'), 'launch', 'demo.launch.py')

    # RViz configuration
    rviz_config_path = os.path.join(
        get_package_share_directory('multirobot_sim'), 'config', 'multirobot.rviz')

    return LaunchDescription([
        # Include the welder launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(welder_launch_path),
            launch_arguments={'use_sim_time': 'true', 'rviz': 'false', 'namespace': 'welder'}.items(),
        ),
        # Include the positioner launch file
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(positioner_launch_path),
            launch_arguments={'use_sim_time': 'true', 'rviz': 'false', 'namespace': 'positioner'}.items(),
        ),
        # Launch a single instance of RViz
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_path],
        ),
    ])
