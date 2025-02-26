from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation clock'),
    moveit_config = (
        MoveItConfigsBuilder(robot_name="multirobot", package_name="positioner_welder_sim")
        .robot_description(file_path="config/multirobot.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )
    # MoveGroupInterface demo executable
    multirobot_move_group_demo = Node(
        name="multi_robot_plan_node",
        package="positioner_welder_sim",
        executable="multi_robot_plan_node",
        output="screen",
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    return LaunchDescription([multirobot_move_group_demo])
