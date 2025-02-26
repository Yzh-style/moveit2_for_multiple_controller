#include <pluginlib/class_loader.hpp>

// MoveIt
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/planning_scene.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <moveit/move_group_interface/move_group_interface.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("path_planning_with_contraints");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    std::shared_ptr<rclcpp::Node> path_planning_with_contraints_node =
         rclcpp::Node::make_shared("path_planning_with_contraints_node", node_options);

    // 定义规划组名称 
    const std::string PLANNING_GROUP =  "welder_planning_group";
    robot_model_loader::RobotModelLoader robot_model_loader(path_planning_with_contraints_node, "robot_description"); //或者是demo的node名字不知道
    const moveit::core::RobotModelPtr& robot_model = robot_model_loader.getModel();
    moveit::core::RobotStatePtr robot_state(new moveit::core::RobotState(robot_model));
    const moveit::core::JointModelGroup* joint_model_group = robot_state->getJointModelGroup(PLANNING_GROUP);

    // 初始化MoveIt接口
    planning_scene::PlanningScenePtr planning_scene(new planning_scene::PlanningScene(robot_model));
    planning_scene->getCurrentStateNonConst();  //planning_scene->getCurrentStateNonConst().setToDefaultValues(joint_model_group, "ready");

    std::unique_ptr<pluginlib::ClassLoader<planning_interface::PlannerManager>> planner_plugin_loader;
    planning_interface::PlannerManagerPtr planner_instance;
    std::string planner_plugin_name;



    //get the name of planning plugin we want to load from the ROS parameter server, and then load the planner making sure to catch all exceptions.
    if (!path_planning_with_contraints_node->get_parameter("planning_plugin", planner_plugin_name))
        RCLCPP_FATAL(LOGGER, "Could not find planner plugin name");
    try
    {
        planner_plugin_loader.reset(new pluginlib::ClassLoader<planning_interface::PlannerManager>(
            "moveit_core", "planning_interface::PlannerManager"));
    }
    catch (pluginlib::PluginlibException& ex)
    {
        RCLCPP_FATAL(LOGGER, "Exception while creating planning plugin loader %s", ex.what());
    }
    try
    {
        planner_instance.reset(planner_plugin_loader->createUnmanagedInstance(planner_plugin_name));
        if (!planner_instance->initialize(robot_model, path_planning_with_contraints_node,
                                        path_planning_with_contraints_node->get_namespace()))
        RCLCPP_FATAL(LOGGER, "Could not initialize planner instance");
        RCLCPP_INFO(LOGGER, "Using planning interface '%s'", planner_instance->getDescription().c_str());
    }
    catch (pluginlib::PluginlibException& ex)
    {
        const std::vector<std::string>& classes = planner_plugin_loader->getDeclaredClasses();
        std::stringstream ss;
        for (const auto& cls : classes)
        ss << cls << " ";
        RCLCPP_ERROR(LOGGER, "Exception while loading planner '%s': %s\nAvailable plugins: %s", planner_plugin_name.c_str(),
                    ex.what(), ss.str().c_str());
    }
    moveit::planning_interface::MoveGroupInterface move_group(path_planning_with_contraints_node, PLANNING_GROUP);

  
    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools(path_planning_with_contraints_node, "multirobot_base_link",
                                                        "Yzh_maker", move_group.getRobotModel());                 // "move_group_tutorial"
    visual_tools.enableBatchPublishing();
    visual_tools.deleteAllMarkers();  // clear all old markers
    visual_tools.trigger();

    /* Remote control is an introspection tool that allows users to step through a high level script
        via buttons and keyboard shortcuts in RViz */
    visual_tools.loadRemoteControl();

    /* RViz provides many types of markers, in this demo we will use text, cylinders, and spheres*/
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.75;
    visual_tools.publishText(text_pose, "Path_planning_with_constraints", rvt::WHITE, rvt::XLARGE);

    /* Batch publishing is used to reduce the number of messages being sent to RViz for large visualizations */
    visual_tools.trigger();

    /* We can also use visual_tools to wait for user input */
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to start the demo");

   // Pose Goal


    visual_tools.trigger();
    planning_interface::MotionPlanRequest req;
    planning_interface::MotionPlanResponse res;
    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = "multirobot_base_link";
    pose.pose.position.x = 0.3;
    pose.pose.position.y = 0.4;
    pose.pose.position.z = 0.75;
    pose.pose.orientation.w = 1.0;

    // A tolerance of 0.01 m is specified in position
    // and 0.01 radians in orientation
    std::vector<double> tolerance_pose(3, 0.01);
    std::vector<double> tolerance_angle(3, 0.01);
    moveit_msgs::msg::Constraints pose_goal =
    kinematic_constraints::constructGoalConstraints("welding_tool", pose, tolerance_pose, tolerance_angle);    // w_link6
    req.group_name = PLANNING_GROUP;
    req.goal_constraints.push_back(pose_goal);
   
   // planning context
    planning_interface::PlanningContextPtr context =
       planner_instance->getPlanningContext(planning_scene, req, res.error_code_);
    context->solve(res);
    if (res.error_code_.val != res.error_code_.SUCCESS)
    {
        RCLCPP_ERROR(LOGGER, "Could not compute plan successfully");
        return 0;
    }
  // Visualize the result
  // ^^^^^^^^^^^^^^^^^^^^
    std::shared_ptr<rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>> display_publisher =
        path_planning_with_contraints_node->create_publisher<moveit_msgs::msg::DisplayTrajectory>("/display_planned_path",
                                                                                                1);
    moveit_msgs::msg::DisplayTrajectory display_trajectory;
    /* Visualize the trajectory */
    moveit_msgs::msg::MotionPlanResponse response;
    res.getMessage(response);

    display_trajectory.trajectory_start = response.trajectory_start;
    display_trajectory.trajectory.push_back(response.trajectory);
    visual_tools.publishTrajectoryLine(display_trajectory.trajectory.back(), joint_model_group);
    visual_tools.trigger();
    display_publisher->publish(display_trajectory);

    /* Set the state in the planning scene to the final state of the last plan */
    robot_state->setJointGroupPositions(joint_model_group, response.trajectory.joint_trajectory.points.back().positions);
    planning_scene->setCurrentState(*robot_state.get());

    // Display the goal state
    visual_tools.publishAxisLabeled(pose.pose, "goal_1");
    visual_tools.publishText(text_pose, "Pose Goal (1)", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();
    visual_tools.prompt("Press 'next' in the RvizVisualToolsGui window to continue the demo");
    /*目标关节状态 
    std::vector<double> target_joint_positions = { 
        76.0 * M_PI / 180.0,   // w_manip_bas
        -38.0 * M_PI / 180.0,  // w_link1_to_link2
        11.0 * M_PI / 180.0,   // w_link2_to_link3
        27.0 * M_PI / 180.0,   // w_link3_to_link4
        -76.0 * M_PI / 180.0,  // w_link4_to_link5
        0.0 * M_PI / 180.0     // w_link5_to_tool
    };
    */
    /* 设置目标关节角度
    move_group.setJointValueTarget(target_joint_positions);

    // 进行运动规划
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    bool success = (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS);

    if (success)
    {
        RCLCPP_INFO(LOGGER, "Planning successful, executing plan...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Planning failed, please check configuration.");
        move_group.execute(my_plan);
    }

    // 取消约束，恢复自由运动
    move_group.clearPathConstraints();

    RCLCPP_INFO(LOGGER, "Motion complete, path constraints cleared.");
*/
    rclcpp::shutdown();
    return 0;
}
