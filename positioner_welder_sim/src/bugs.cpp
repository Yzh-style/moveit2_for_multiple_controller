// #include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("path_planning_with_constraints");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("path_planning_with_constraints_node");

    // 定义规划组名称，需与MoveIt配置中的group名称一致
    static const std::string PLANNING_GROUP = "welder_planning_group";

    // 初始化MoveIt接口
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
    planning_scene_monitor::PlanningSceneMonitorPtr psm(
      new planning_scene_monitor::PlanningSceneMonitor(robot_model_loader));
    // 获取机器人当前状态
    moveit::core::RobotStatePtr robot_state(
    new moveit::core::RobotState(planning_scene_monitor::LockedPlanningSceneRO(psm)->getCurrentState()));

    geometry_msgs::msg::Pose start_pose = move_group.getCurrentPose().pose;

    RCLCPP_INFO(LOGGER, "Current pose: x=%.2f, y=%.2f, z=%.2f", start_pose.position.x, start_pose.position.y, start_pose.position.z);

    /* 设置末端执行器约束（保持welding_tool与Z轴成45度角）
    moveit_msgs::msg::OrientationConstraint ocm;
    ocm.link_name = "welding_tool";  // 确保此值与URDF/Xacro中工具末端一致
    ocm.header.frame_id = "world";  // 参考launch文件中的tf发布
    ocm.orientation.x = 0.382;  // 45° 旋转（四元数 x 分量）
    ocm.orientation.y = 0.0;
    ocm.orientation.z = 0.0;
    ocm.orientation.w = 0.923;  // 45° 旋转（四元数 w 分量）
    ocm.absolute_x_axis_tolerance = 0.1;
    ocm.absolute_y_axis_tolerance = 0.1;
    ocm.absolute_z_axis_tolerance = 0.1;
    ocm.weight = 1.0;

    // 应用约束
    moveit_msgs::msg::Constraints path_constraints;
    path_constraints.orientation_constraints.push_back(ocm);
    move_group.setPathConstraints(path_constraints);
    */
    // 设置路径点
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(start_pose);

    // 向下运动 0.3m
    geometry_msgs::msg::Pose pose_down = start_pose;
    pose_down.position.z -= -0.3;
     pose_down.position.x  -= 0.3;
    waypoints.push_back(pose_down);

    // 向右运动 0.3m
    geometry_msgs::msg::Pose pose_right = pose_down;
    pose_right.position.x += 0.3;
    waypoints.push_back(pose_right);

    // 执行笛卡尔路径规划
    moveit_msgs::msg::RobotTrajectory trajectory;
    const double eef_step = 0.01;  // 1 cm 步长
    const double jump_threshold = 0.0;  // 禁止跳跃
    double fraction = move_group.computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

    RCLCPP_INFO(LOGGER, "Cartesian path (%.2f%% achieved)", fraction * 100.0);

    if (fraction > 0.95)  // 确保路径成功规划
    {
        RCLCPP_INFO(LOGGER, "Executing Cartesian path...");
        move_group.execute(trajectory);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Path planning failed, please adjust waypoints or constraints.");
    }

    // 清除路径约束
    move_group.clearPathConstraints();

    rclcpp::shutdown();
    return 0;
}
