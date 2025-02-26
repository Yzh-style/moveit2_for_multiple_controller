from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_path
import os

def generate_launch_description():
    package_share_path = get_package_share_path('welding_kinematic_controller')
    positioner_description = Command(['xacro ', os.path.join(package_share_path, 'urdf', 'positioner.urdf.xacro')])

    return LaunchDescription([
        # Node(
        #     package='robot_state_publisher',
        #     executable='robot_state_publisher',
        #     namespace='positioner',
        #     parameters=[{'robot_description': positioner_description}],
        #     output='screen',
        # ),
        # Node(
        #     package='joint_state_publisher',
        #     executable='joint_state_publisher',
        #     name='welder_joint_state_publisher',
        #     output='screen',
        #     parameters=[{'robot_description': Command(['xacro ', positioner_description])}]
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     arguments=['1', '0', '0', '0', '0', '0', 'base_link'],
        #     output='screen',
        # ),
    ])
