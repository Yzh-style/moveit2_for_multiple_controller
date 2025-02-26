import os
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Configure MoveIt for positioner
    positioner_config = (
        MoveItConfigsBuilder(robot_name="positioner", package_name="positioner")
        .robot_description(file_path="config/positioner.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Configure MoveIt for welder
    welder_config = (
        MoveItConfigsBuilder(robot_name="welder", package_name="welder")
        .robot_description(file_path="config/welder.urdf.xacro")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .to_moveit_configs()
    )

    # Positioner Nodes
    positioner_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace="positioner_",
        parameters=[positioner_config.to_dict()],
    )

    positioner_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="positioner_",
        output="both",
        parameters=[positioner_config.robot_description],
    )

    positioner_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="positioner_",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "positioner_base_link"],
    )

    # Welder Nodes
    welder_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        namespace="welder_",
        parameters=[welder_config.to_dict()],
    )

    welder_robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace="welder_",
        output="both",
        parameters=[welder_config.robot_description],
    )

    welder_static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        namespace="welder_",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "welder_base_link"],
    )

    # Shared RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("multirobot_sim"), "config", "multirobot_name.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        namespace="multirobot",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            positioner_config.robot_description,
            positioner_config.robot_description_semantic,
            positioner_config.planning_pipelines,
            welder_config.robot_description,
            welder_config.robot_description_semantic,
            welder_config.planning_pipelines,
        ],
    )

    return LaunchDescription(
        [
            rviz_node,
            positioner_move_group_node,
            positioner_robot_state_publisher,
            positioner_static_tf,
            welder_move_group_node,
            welder_robot_state_publisher,
            welder_static_tf,
        ]
    )
