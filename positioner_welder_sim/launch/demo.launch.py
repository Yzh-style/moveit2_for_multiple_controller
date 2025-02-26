import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder
from launch.actions import DeclareLaunchArgument

def generate_launch_description():

   # 添加 use_sim_time 参数声明
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='true', description='Use simulation clock'
    )

    package_name = "positioner_welder_sim"
    config_directory = os.path.join(get_package_share_directory(package_name), "config")

    urdf_xacro_path = os.path.join(config_directory, "multirobot.urdf.xacro")
    srdf_path = os.path.join(config_directory, "multirobot.srdf")
    controllers_yaml_path = os.path.join(config_directory, "moveit_controllers.yaml")

    moveit_config = (
        MoveItConfigsBuilder(robot_name="multirobot", package_name=package_name)
        .robot_description(file_path=urdf_xacro_path)
        .robot_description_semantic(file_path=srdf_path)
        .trajectory_execution(file_path=controllers_yaml_path)
        .to_moveit_configs()
    )
        
    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    # RViz
    rviz_config_file = os.path.join(
        get_package_share_directory("positioner_welder_sim"), "config", "moveit.rviz"
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
        ],
    )

    # Static TF
    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["0", "0", "0", "0", "0", "0", "world", "multirobot_base_link"],
    )

    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("positioner_welder_sim"), "config", "ros2_controllers.yaml"
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="both",
    )

    # Load controllers
    load_controllers = []
    for controller in [
        "joint_state_broadcaster",
        "positioner_planning_group_controller",
        "welder_planning_group_controller"         # add a controller
    ]:
        load_controllers.append(
            ExecuteProcess(
                cmd=["ros2 run controller_manager spawner", controller],
                shell=True,
                output="screen",
            )
        )

    return LaunchDescription(
        [
            rviz_node,
            static_tf,
            robot_state_publisher,
            run_move_group_node,
            ros2_control_node,
        ] + load_controllers
    )
