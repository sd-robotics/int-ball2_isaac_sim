cmake_minimum_required(VERSION 3.8)
project(ib2_isaac_sim)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(isaac_ros_nitros REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(DIRECTORY
  assets
  launch
  scripts
  DESTINATION share/${PROJECT_NAME})

# install Python executables
install(
  PROGRAMS
    scripts/create_isaacsim_stage.py
    scripts/open_isaacsim_stage.py
    scripts/run_isaacsim.py
  DESTINATION
    lib/${PROJECT_NAME}
)

ament_package()
