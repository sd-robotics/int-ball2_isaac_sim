cmake_minimum_required(VERSION 3.8)
project(int-ball2_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(joy REQUIRED)
find_package(xacro REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  config
  rviz
  launch
  scripts
  description
  DESTINATION share/${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/bag_tf_publisher.py
  scripts/bag_wrench_multiplier.py
  scripts/direct_effort_control.py
  scripts/direct_velocity_control.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
