cmake_minimum_required(VERSION 3.22)
project(positioner_welder_sim)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(moveit_visual_tools REQUIRED)


set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  moveit_ros_planning_interface
  Eigen3
)

add_executable(path_planning_with_constraints_node
  src/path_planning_with_constraints_node.cpp
)

add_executable(path_planning_follow_line_node
  src/path_planning_follow_line_node.cpp
)

add_executable(multi_robot_plan_node
  src/multi_robot_plan_node.cpp
)
target_include_directories(path_planning_with_constraints_node PUBLIC
  ${Eigen3_INCLUDE_DIRS}
  ${rclcpp_INCLUDE_DIRS}
  ${moveit_ros_planning_interface_INCLUDE_DIRS}
)

ament_target_dependencies(path_planning_with_constraints_node
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  graph_msgs
  Eigen3
)

ament_target_dependencies(path_planning_follow_line_node
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  graph_msgs
  Eigen3
)

ament_target_dependencies(multi_robot_plan_node
  rclcpp
  moveit_ros_planning_interface
  moveit_visual_tools
  moveit_msgs
  graph_msgs
  Eigen3
)

# 安装可执行文件到lib/positioner_welder_sim
install(TARGETS 
  path_planning_with_constraints_node
  path_planning_follow_line_node
  multi_robot_plan_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)
ament_package()
