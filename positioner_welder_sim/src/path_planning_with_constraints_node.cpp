#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("path_planning_with_contraints");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("path_planning_with_contraints_node");

    // 定义规划组名称
    static const std::string PLANNING_GROUP = "welder_planning_group";

    // 初始化MoveIt接口
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 目标关节状态 (从图像获取的目标状态)
    std::vector<double> target_joint_positions = { 
        76.0 * M_PI / 180.0,   // w_manip_bas
        -38.0 * M_PI / 180.0,  // w_link1_to_link2
        11.0 * M_PI / 180.0,   // w_link2_to_link3
        27.0 * M_PI / 180.0,   // w_link3_to_link4
        -76.0 * M_PI / 180.0,  // w_link4_to_link5
        0.0 * M_PI / 180.0     // w_link5_to_tool
    };

    // 设置目标关节角度
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
    }

    // 取消约束，恢复自由运动
    move_group.clearPathConstraints();

    RCLCPP_INFO(LOGGER, "Motion complete, path constraints cleared.");

    rclcpp::shutdown();
    return 0;
}
