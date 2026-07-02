cmake_minimum_required(VERSION 3.8)
project(ik_arm)

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic -Werror)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# ── ROS2 action interface generation ──
find_package(rosidl_default_generators REQUIRED)

# The target name MUST match ${PROJECT_NAME} so the generated headers use
# the namespace "ik_arm::action::ComputeIK" matching our #include paths.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/ComputeIK.action"
  DEPENDENCIES geometry_msgs
)

# Include headers (for both library and executable targets)
include_directories(include)

# ── Robot model library (pure Eigen math, no ROS deps) ──
add_library(robot_model
  src/robot_model.cpp
)
target_link_libraries(robot_model Eigen3::Eigen)

# ── Main node executable ──
add_executable(ik_arm_node
  src/ik_arm_node.cpp
)
target_link_libraries(ik_arm_node robot_model)
target_link_libraries(ik_arm_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_target_dependencies(ik_arm_node rclcpp rclcpp_action sensor_msgs geometry_msgs std_msgs)
add_dependencies(ik_arm_node ${PROJECT_NAME})

# ── Demo node (sends IK action goals) ──
add_executable(ik_demo
  src/ik_demo.cpp
)
target_link_libraries(ik_demo ${PROJECT_NAME}__rosidl_typesupport_cpp)
ament_target_dependencies(ik_demo rclcpp rclcpp_action geometry_msgs)
add_dependencies(ik_demo ${PROJECT_NAME})

# Install executables
install(
  TARGETS
  ik_arm_node
  ik_demo
  robot_model
  DESTINATION
  lib/${PROJECT_NAME}
)

# Install headers
install(
  DIRECTORY include/
  DESTINATION include
)

# Install config
install(
  DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  # suppress lint checks that don't work in this environment
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_xmllint_FOUND TRUE)

  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_gtest REQUIRED)

  # Unit test for RobotModel (pure math, no ROS init needed)
  ament_add_gtest(test_robot_model test/test_robot_model.cpp src/robot_model.cpp)
  target_include_directories(test_robot_model PRIVATE include)
  target_link_libraries(test_robot_model Eigen3::Eigen)

  # Unit test for IKArmNode (ROS node, via NodeOptions)
  ament_add_gtest(test_ik_arm_node test/test_ik_arm_node.cpp src/ik_arm_node.cpp)
  target_include_directories(test_ik_arm_node PRIVATE include)
  target_compile_definitions(test_ik_arm_node PRIVATE BUILD_TESTING=1)
  ament_target_dependencies(test_ik_arm_node rclcpp rclcpp_action sensor_msgs geometry_msgs std_msgs)
  target_link_libraries(test_ik_arm_node robot_model)
  target_link_libraries(test_ik_arm_node ${PROJECT_NAME}__rosidl_typesupport_cpp)
  add_dependencies(test_ik_arm_node ${PROJECT_NAME})
endif()

ament_package()
