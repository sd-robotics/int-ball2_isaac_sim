cmake_minimum_required(VERSION 3.10.2)
project(ib2_nav)
# cmake_policy(SET CMP0054 OLD) 

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# option(NAV_DEBUG "execute in nav debug mode" OFF)
# if (NAV_DEBUG)
#   add_definitions(-DNAV_DEBUG)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(ib2_interfaces REQUIRED)


###########
## Build ##
###########

# Find libraries
include_directories(
 include
)

## Declare a C++ library
add_library(${PROJECT_NAME} SHARED
  src/nav.cpp
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "ib2::Nav"
  EXECUTABLE ib2_nav_node)

ament_target_dependencies(${PROJECT_NAME}
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "tf2_ros"
  "tf2"
  "tf2_geometry_msgs"
  "nav_msgs"
  "sensor_msgs"
  "geometry_msgs"
  "example_interfaces"
  "ib2_interfaces"
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(DIRECTORY
  include/${PROJECT_NAME}
  DESTINATION include
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)


# if(BUILD_TESTING)
#   find_package(ament_lint_auto REQUIRED)
#   # the following line skips the linter which checks for copyrights
#   # comment the line when a copyright and license is added to all source files
#   set(ament_cmake_copyright_FOUND TRUE)
#   # the following line skips cpplint (only works in a git repo)
#   # comment the line when this package is in a git repo and when
#   # a copyright and license is added to all source files
#   set(ament_cmake_cpplint_FOUND TRUE)
#   ament_lint_auto_find_test_dependencies()
# endif()

ament_package()
