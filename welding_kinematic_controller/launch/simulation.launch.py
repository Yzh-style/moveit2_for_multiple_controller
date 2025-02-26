from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory, get_package_share_path
from launch.substitutions import Command
import os

def generate_launch_description():
    package_share_path = get_package_share_path('welding_kinematic_controller')
    welder_urdf_file = os.path.join(get_package_share_directory('welding_kinematic_controller'),'urdf','welder.urdf.xacro')
    positioner_urdf_file = os.path.join(get_package_share_directory('welding_kinematic_controller'),'urdf','positioner.urdf.xacro')

    return LaunchDescription([
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(package_share_path, 'launch', 'welder.launch.py')
        #     )
        # ),

        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource(
        #         os.path.join(package_share_path, 'launch', 'positioner.launch.py')
        #     )
        # ),

        # # Welder Joint State Publisher
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='welder_joint_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': Command(['xacro ', welder_urdf_file])}]
        # ),

        # # Positioner Joint State Publisher
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='positioner_joint_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': Command(['xacro ', positioner_urdf_file])}]
        # ),
        
        # Welder Robot State
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='welder_state_publisher',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', welder_urdf_file])}],
            remappings=[('/joint_states', '/welder_joint_states')]  
        ),

        # Positioner Robot State
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='positioner_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', positioner_urdf_file])}],
            remappings=[('/joint_states', '/positioner_joint_states')] 
        ),
        
        # Welder Joint State GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='welder_joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', welder_urdf_file]), 'rate': 70}],
            remappings=[('/joint_states', '/welder_joint_states')]  
        ),

        # Positioner Joint State GUI
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='positioner_joint_state_publisher_gui',
            output='screen',
            parameters=[{'robot_description': Command(['xacro ', positioner_urdf_file]), 'rate': 70}],
            remappings=[('/joint_states', '/positioner_joint_states')]
        ),

        #RViz2 Publisher
        Node(
            package='rviz2',
            executable='rviz2',
            arguments=['-d', os.path.join(package_share_path, 'config', 'config.rviz')],
            output='screen',
        ),
    ])

