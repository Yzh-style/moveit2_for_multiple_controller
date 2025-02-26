import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        "namespace", default_value="positioner", description="Namespace for the robot"
    )   

    db_arg = DeclareLaunchArgument(
        "db", default_value="False", description="Enable database functionality"
    )

    use_fake_hardware = DeclareLaunchArgument(
        "fake_hardware", default_value="true", description="Use fake hardware"
    )

    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value=os.path.join(
            get_package_share_directory("positioner"),
            "config",
            "moveit.rviz",
        ),
        description="Path to the RViz configuration file",
    )

    rviz_arg = DeclareLaunchArgument(
        "rviz", default_value="true", description="Launch RViz"
    )

    # Configure MoveIt
    moveit_config = (
        MoveItConfigsBuilder(robot_name="positioner", package_name="positioner")
        .robot_description(
            file_path="config/positioner.urdf.xacro",
            mappings={"ros2_control_hardware_type": LaunchConfiguration("fake_hardware")},
        )
        .robot_description_semantic(file_path="config/positioner.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .planning_scene_monitor(publish_robot_description=True, publish_robot_description_semantic=True)
        .planning_pipelines(pipelines=["ompl", "chomp", "pilz_industrial_motion_planner"])
        .to_moveit_configs()
    )

    # Define nodes
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[moveit_config.robot_description],
    )

    static_tf_publisher = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "positioner_base_link"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", LaunchConfiguration("rviz_config")],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
        ],
        condition=IfCondition(LaunchConfiguration("rviz")),
    )


    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[os.path.join(get_package_share_directory("positioner"), "config", "ros2_controllers.yaml")],
        remappings=[
            ("/controller_manager/robot_description", "/robot_description"),
        ],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
#########################################################################################################################################
#########################################################################################################################################
    positioner_planning_group_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["positioner_planning_group_controller", "--controller-manager", "/controller_manager"],
    )


    

    mongodb_server_node = Node(
        package="warehouse_ros_mongo",
        executable="mongo_wrapper_ros.py",
        parameters=[
            {"warehouse_port": 33829},
            {"warehouse_host": "localhost"},
            {"warehouse_plugin": "warehouse_ros_mongo::MongoDatabaseConnection"},
        ],
        output="screen",
        condition=IfCondition(LaunchConfiguration("db")),
    )

    return LaunchDescription([
        namespace_arg,
        use_fake_hardware,
        db_arg,
        rviz_arg,
        rviz_config_arg,
        move_group_node,
        robot_state_publisher,
        static_tf_publisher,
        ros2_control_node,
        joint_state_broadcaster_spawner,
        positioner_planning_group_controller_spawner, ##############################################################
        mongodb_server_node,
        rviz_node,  # This will now launch only if `rviz` is set to `true`.
    ])

