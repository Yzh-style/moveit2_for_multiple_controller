#include <rclcpp/rclcpp.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_visual_tools/moveit_visual_tools.h>

using namespace std::chrono_literals;

static const rclcpp::Logger LOGGER = rclcpp::get_logger("welder_path_planning");

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("welder_path_planning_node");

    static const std::string PLANNING_GROUP = "welder_planning_group";
    moveit::planning_interface::MoveGroupInterface move_group(node, PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // 定义路径点
    std::vector<std::vector<geometry_msgs::msg::Pose>> waypoint_sets;

    // 第一段路径
    std::vector<geometry_msgs::msg::Pose> waypoints1;
    geometry_msgs::msg::Pose target_pose;
    target_pose.position.x = 1.2375;
    target_pose.position.y = 0.1278;
    target_pose.position.z = 2.5243;
    target_pose.orientation.w = 1.0;
    waypoints1.push_back(target_pose);

    target_pose.position.z = 3.5243;
    waypoints1.push_back(target_pose);
    waypoint_sets.push_back(waypoints1);

    // 第二段路径
    std::vector<geometry_msgs::msg::Pose> waypoints2;
    target_pose.position.x = 0;
    waypoints2.push_back(target_pose);
    waypoint_sets.push_back(waypoints2);

    // 第三段路径
    std::vector<geometry_msgs::msg::Pose> waypoints3;
    target_pose.position.x = -0.9375;
    target_pose.position.y = 1.1278;
    target_pose.position.z = 3.5243;
    waypoints3.push_back(target_pose);
    waypoint_sets.push_back(waypoints3);
    // 第四段路径
    std::vector<geometry_msgs::msg::Pose> waypoints4;
    target_pose.position.x = 0;
    target_pose.position.y = 1.1;
    target_pose.position.z = 3;
    waypoints4.push_back(target_pose);
    waypoint_sets.push_back(waypoints4);

    moveit_msgs::msg::RobotTrajectory trajectory;
    double eef_step = 0.005;  // 端部步长 (提高精度)
    double jump_threshold = 0.0; // 避免突变

    for (size_t i = 0; i < waypoint_sets.size(); ++i)
    {
        double fraction = move_group.computeCartesianPath(waypoint_sets[i], eef_step, jump_threshold, trajectory);

        if (fraction > 0.9)
        {
            RCLCPP_INFO(LOGGER, "Path segment %ld successfully computed. Executing...", i+1);
            move_group.execute(trajectory);
            RCLCPP_INFO(LOGGER, "Execution completed. Waiting for 4 seconds before next segment...");
            rclcpp::sleep_for(4s);
        }
        else
        {
            RCLCPP_WARN(LOGGER, "Failed to compute full path for segment %ld. Success rate: %.2f%%", i+1, fraction * 100.0);
            move_group.execute(trajectory);
        }
    }

    RCLCPP_INFO(LOGGER, "All motion segments complete.");
    rclcpp::shutdown();
    return 0;
}
