cmake_minimum_required(VERSION 3.8)
project(ib2_data_replay)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

# Component library
ament_auto_add_library(bag_pose_replay
  src/bag_pose_replay.cpp
)
rclcpp_components_register_nodes(bag_pose_replay "bag_pose_replay::BagPoseReplay")

# Standalone executable
ament_auto_add_executable(bag_pose_replay_node
  src/bag_pose_replay_node.cpp
)
target_link_libraries(bag_pose_replay_node bag_pose_replay)

# Install launch and headers
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}/
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_cpplint_FOUND TRUE)
  set(ament_cmake_copyright_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package()
