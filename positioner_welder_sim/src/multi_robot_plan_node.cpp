#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_robot_state.hpp>
#include <moveit_msgs/msg/display_trajectory.hpp>

#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/collision_object.hpp>

#include <moveit_visual_tools/moveit_visual_tools.h>

static const rclcpp::Logger LOGGER = rclcpp::get_logger("multi_robot_plan_node");

void execute_positioner_path(const std::shared_ptr<rclcpp::Node>& node)
{
    static const std::string POSITIONER_PLANNING_GROUP = "positioner_planning_group";
    moveit::planning_interface::MoveGroupInterface move_group(node, POSITIONER_PLANNING_GROUP);

    std::vector<double> target_joint_positions = {0, 85 * M_PI / 180.0, -41.0 * M_PI / 180.0, 0, 49.0 * M_PI / 180.0};
    std::vector<double> home_joint_positions = {0.0, 63 * M_PI / 180, -38 * M_PI / 180, 0.0, -76 * M_PI / 180};
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    move_group.setJointValueTarget(target_joint_positions);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "Positioner moving to target position...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Positioner planning to target failed.");
        return;
    }
    rclcpp::sleep_for(std::chrono::seconds(4));

    move_group.setJointValueTarget(home_joint_positions);
    if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
    {
        RCLCPP_INFO(LOGGER, "Returning positioner to home position...");
        move_group.execute(my_plan);
    }
    else
    {
        RCLCPP_WARN(LOGGER, "Positioner failed to return home.");
    }
    rclcpp::sleep_for(std::chrono::seconds(4));
}

void execute_welder_path(const std::shared_ptr<rclcpp::Node>& node)
{
    static const std::string WELDER_PLANNING_GROUP = "welder_planning_group";
    moveit::planning_interface::MoveGroupInterface move_group(node, WELDER_PLANNING_GROUP);

    std::vector<std::vector<double>> joint_positions = {
        {-69 * M_PI / 180.0, -8 * M_PI / 180.0, 77 * M_PI / 180.0, -55 * M_PI / 180.0, 33 * M_PI / 180.0, -8 * M_PI / 180.0},
        {-72 * M_PI / 180.0, 19 * M_PI / 180.0, 9 * M_PI / 180.0, -15 * M_PI / 180.0, 35 * M_PI / 180.0, -7 * M_PI / 180.0}
    };

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    for (const auto& position : joint_positions)
    {
        move_group.setJointValueTarget(position);
        if (move_group.plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS)
        {
            RCLCPP_INFO(LOGGER, "Welder moving to target joint position...");
            move_group.execute(my_plan);
            rclcpp::sleep_for(std::chrono::seconds(2));
        }
        else
        {
            RCLCPP_WARN(LOGGER, "Welder failed to reach target position.");
        }
    }
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("multi_robot_plan_node");

    for (int cycle = 0; cycle < 2; ++cycle)
    {
        RCLCPP_INFO(LOGGER, "Cycle %d starting...", cycle + 1);
        execute_positioner_path(node);
        execute_welder_path(node);
        RCLCPP_INFO(LOGGER, "Cycle %d complete.", cycle + 1);
    }

    RCLCPP_INFO(LOGGER, "All motion cycles complete.");
    rclcpp::shutdown();
    return 0;
}
